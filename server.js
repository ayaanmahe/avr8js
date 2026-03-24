/**
 * avr8js REST API Server
 * Simulates: LEDs, Servo, Stepper Motor, Buttons, OLED (I2C), UART Serial (TX+RX), PWM, ADC, NeoPixels
 * Deploy on Render, trigger via GitHub Actions
 */

import express from 'express';
import {
  CPU,
  AVRIOPort,
  portBConfig,
  portCConfig,
  portDConfig,
  AVRTimer,
  timer0Config,
  timer1Config,
  timer2Config,
  AVRUSART,
  usart0Config,
  AVRSPI,
  spiConfig,
  AVRI2C,
  i2cConfig,
  AVRADC,
  adcConfig,
  avrInstruction,
  PinState,
} from 'avr8js';

const app = express();
app.use(express.json({ limit: '2mb' }));

// ─── Helpers ────────────────────────────────────────────────────────────────

/**
 * Load a flat byte array (Intel HEX decoded) into a Uint16Array program memory.
 * The client sends raw bytes as a plain JS number array.
 */
function loadProgram(bytes) {
  const program = new Uint16Array(32768); // 64KB flash = 32K words
  for (let i = 0; i < bytes.length - 1; i += 2) {
    program[i / 2] = (bytes[i + 1] << 8) | bytes[i];
  }
  return program;
}

/**
 * Decode Intel HEX string into a flat byte array.
 */
function decodeIntelHex(hexStr) {
  const bytes = new Array(32768).fill(0);
  const lines = hexStr.replace(/\r/g, '').split('\n');
  for (const line of lines) {
    if (!line.startsWith(':')) continue;
    const byteCount = parseInt(line.substring(1, 3), 16);
    const address = parseInt(line.substring(3, 7), 16);
    const recordType = parseInt(line.substring(7, 9), 16);
    if (recordType === 0) {
      for (let i = 0; i < byteCount; i++) {
        bytes[address + i] = parseInt(line.substring(9 + i * 2, 11 + i * 2), 16);
      }
    }
  }
  return bytes;
}

// ─── Component Simulators ────────────────────────────────────────────────────

/**
 * LED Simulator
 * Tracks ON/OFF state of individual GPIO pins.
 */
class LEDSimulator {
  constructor(port, pinMask, label) {
    this.label = label;
    this.pinMask = pinMask;
    this.state = false;
    this.events = [];
    port.addListener(() => {
      const newState = !!(port.pinState(Math.log2(pinMask)) === PinState.High);
      if (newState !== this.state) {
        this.state = newState;
        this.events.push({ cycle: null, state: newState ? 'ON' : 'OFF' });
      }
    });
  }

  result() {
    return { label: this.label, currentState: this.state ? 'ON' : 'OFF', events: this.events.slice(-50) };
  }
}

/**
 * Button Simulator
 * Injects a LOW signal on a pin after `triggerAfterCycles` cycles to simulate a press.
 */
class ButtonSimulator {
  constructor(cpu, port, pin, label, triggerAfterCycles = 100000) {
    this.label = label;
    this.pin = pin;
    this.pressed = false;
    this.triggerAt = triggerAfterCycles;
    this._cpu = cpu;
    this._port = port;
  }

  tick() {
    if (!this.pressed && this._cpu.cycles >= this.triggerAt) {
      // Simulate button press: pull pin LOW (active-low buttons)
      this._port.setPin(this.pin, PinState.Low);
      this.pressed = true;
    }
  }

  result() {
    return { label: this.label, pin: this.pin, pressedAtCycle: this.triggerAt, wasPressed: this.pressed };
  }
}

/**
 * Servo Simulator
 * Reads PWM pulse width on a pin and converts to angle (500µs=0°, 2500µs=180°).
 * Works by tracking Timer1 PWM output on OC1A (pin PB1).
 */
class ServoSimulator {
  constructor() {
    this.pulseWidthUs = 0;
    this.angle = 90;
    this.events = [];
    this._lastHigh = 0;
    this._isHigh = false;
  }

  /**
   * Call this each CPU cycle with the current pin state and cpu.cycles.
   */
  onPinChange(isHigh, cycleCount, cpuFreq = 16e6) {
    const usPerCycle = 1e6 / cpuFreq;
    if (isHigh && !this._isHigh) {
      this._lastHigh = cycleCount;
      this._isHigh = true;
    } else if (!isHigh && this._isHigh) {
      const pulseUs = (cycleCount - this._lastHigh) * usPerCycle;
      this._isHigh = false;
      if (pulseUs >= 400 && pulseUs <= 2700) {
        this.pulseWidthUs = pulseUs;
        // Map 500µs→0°, 2500µs→180°
        this.angle = Math.round(((pulseUs - 500) / 2000) * 180);
        this.angle = Math.max(0, Math.min(180, this.angle));
        this.events.push({ cycleCount, pulseUs: Math.round(pulseUs), angle: this.angle });
      }
    }
  }

  result() {
    return {
      currentAngle: this.angle,
      lastPulseUs: Math.round(this.pulseWidthUs),
      events: this.events.slice(-50),
    };
  }
}

/**
 * Stepper Motor Simulator (4-wire, full-step)
 * Tracks the step sequence on 4 GPIO pins and counts steps + direction.
 * Full-step sequence: 1010 → 0110 → 0101 → 1001
 */
const FULL_STEP_SEQUENCE = [0b1010, 0b0110, 0b0101, 0b1001];

class StepperSimulator {
  constructor() {
    this.steps = 0;
    this.direction = 'CW';
    this.position = 0;
    this.lastPattern = -1;
    this.events = [];
    this._seqIndex = 0;
  }

  /**
   * Call with current 4-bit pattern of pins IN1,IN2,IN3,IN4.
   */
  onPattern(pattern, cycleCount) {
    if (pattern === this.lastPattern) return;
    const idx = FULL_STEP_SEQUENCE.indexOf(pattern);
    if (idx === -1) return;

    if (this.lastPattern !== -1) {
      const lastIdx = FULL_STEP_SEQUENCE.indexOf(this.lastPattern);
      const diff = (idx - lastIdx + 4) % 4;
      if (diff === 1) {
        this.direction = 'CW';
        this.position++;
        this.steps++;
        this.events.push({ cycleCount, step: this.steps, direction: 'CW', position: this.position });
      } else if (diff === 3) {
        this.direction = 'CCW';
        this.position--;
        this.steps++;
        this.events.push({ cycleCount, step: this.steps, direction: 'CCW', position: this.position });
      }
    }

    this.lastPattern = pattern;
  }

  result() {
    return {
      totalSteps: this.steps,
      finalPosition: this.position,
      lastDirection: this.direction,
      angleApprox: Math.round((this.position % 200) * 1.8), // 200 steps/rev
      events: this.events.slice(-100),
    };
  }
}

/**
 * OLED Simulator (I2C SSD1306-style)
 * Intercepts I2C writes and records the framebuffer commands.
 * Decodes basic SSD1306 display commands and page/column writes.
 */
class OLEDSimulator {
  constructor() {
    this.i2cAddress = 0x3c;
    this.framebuffer = new Uint8Array(128 * 8); // 128x64 as 8 pages of 128 bytes
    this.commands = [];
    this.dataBuffer = [];
    this._isData = false;
    this._page = 0;
    this._col = 0;
    this._receiveBuffer = [];
    this._state = 'idle'; // 'idle' | 'command' | 'data'
  }

  /**
   * Feed raw I2C bytes from AVRI2C onReceive callback.
   */
  onI2CData(address, bytes) {
    if (address !== this.i2cAddress) return;
    if (bytes.length < 1) return;

    const controlByte = bytes[0];
    const isData = !!(controlByte & 0x40);

    for (let i = 1; i < bytes.length; i++) {
      const b = bytes[i];
      if (!isData) {
        this._parseCommand(b);
      } else {
        // Write to framebuffer
        if (this._col < 128 && this._page < 8) {
          this.framebuffer[this._page * 128 + this._col] = b;
          this._col++;
          if (this._col >= 128) {
            this._col = 0;
            this._page = (this._page + 1) % 8;
          }
        }
      }
    }
  }

  _parseCommand(cmd) {
    this.commands.push(`0x${cmd.toString(16).padStart(2, '0')}`);
    if (cmd === 0xb0 || (cmd >= 0xb0 && cmd <= 0xb7)) {
      this._page = cmd & 0x07;
      this._col = 0;
    }
    // More SSD1306 commands can be decoded here
  }

  /**
   * Convert framebuffer to a simple 128x64 bit array for visualization.
   */
  getPixels() {
    const pixels = [];
    for (let page = 0; page < 8; page++) {
      for (let bit = 0; bit < 8; bit++) {
        const row = [];
        for (let col = 0; col < 128; col++) {
          row.push((this.framebuffer[page * 128 + col] >> bit) & 1);
        }
        pixels.push(row);
      }
    }
    return pixels;
  }

  result() {
    return {
      i2cAddress: `0x${this.i2cAddress.toString(16)}`,
      commandsReceived: this.commands.slice(-30),
      framebufferSummary: {
        nonZeroBytes: this.framebuffer.filter((b) => b !== 0).length,
        totalBytes: this.framebuffer.length,
      },
      // pixels: this.getPixels(), // Uncomment for full 128x64 bitmap (large!)
    };
  }
}

/**
 * NeoPixel / WS2812B Simulator
 * Detects timing-based bit encoding on a data pin (800kHz protocol).
 * T0H ≈ 0.4µs, T1H ≈ 0.8µs at 16MHz.
 */
class NeoPixelSimulator {
  constructor(numPixels = 8) {
    this.numPixels = numPixels;
    this.pixels = Array(numPixels).fill(null).map(() => ({ r: 0, g: 0, b: 0 }));
    this.events = [];
    this._bitBuffer = [];
    this._lastEdge = 0;
    this._isHigh = false;
    this._T0H = 6;  // cycles at 16MHz ≈ 0.4µs
    this._T1H = 13; // cycles at 16MHz ≈ 0.8µs
  }

  onPinChange(isHigh, cycle) {
    if (isHigh && !this._isHigh) {
      this._lastEdge = cycle;
      this._isHigh = true;
    } else if (!isHigh && this._isHigh) {
      const highCycles = cycle - this._lastEdge;
      this._isHigh = false;
      // Classify bit: short pulse = 0, long pulse = 1
      if (highCycles <= this._T0H + 3) {
        this._bitBuffer.push(0);
      } else if (highCycles >= this._T1H - 3) {
        this._bitBuffer.push(1);
      }

      // Every 24 bits = 1 GRB pixel
      if (this._bitBuffer.length >= 24) {
        const bits = this._bitBuffer.splice(0, 24);
        const g = this._bitsToInt(bits, 0, 8);
        const r = this._bitsToInt(bits, 8, 16);
        const b = this._bitsToInt(bits, 16, 24);
        const pixelIdx = this.events.length % this.numPixels;
        this.pixels[pixelIdx] = { r, g, b };
        this.events.push({ cycle, pixel: pixelIdx, r, g, b, hex: `#${r.toString(16).padStart(2,'0')}${g.toString(16).padStart(2,'0')}${b.toString(16).padStart(2,'0')}` });
      }
    }
  }

  _bitsToInt(bits, start, end) {
    let val = 0;
    for (let i = start; i < end; i++) val = (val << 1) | bits[i];
    return val;
  }

  result() {
    return {
      numPixels: this.numPixels,
      pixels: this.pixels,
      events: this.events.slice(-50),
    };
  }
}

/**
 * Two-Way Serial Simulator
 *
 * TX (Arduino → Host): captures every byte the AVR transmits via UART.
 * RX (Host → Arduino): injects bytes into the UART receive buffer at
 *   specified cycle timestamps, simulating what a host would send.
 *
 * RX input format (serialInput array):
 *   [
 *     { text: "hello\n", atCycle: 200000 },    // send a string at cycle 200000
 *     { bytes: [0x01, 0xff], atCycle: 500000 } // send raw bytes at cycle 500000
 *   ]
 */
class SerialSimulator {
  constructor(usart, cpu) {
    this._usart = usart;
    this._cpu = cpu;

    // TX state
    this.txOutput = '';
    this.txEvents = [];

    // RX queue — flat list of { atCycle, byte }, sorted ascending
    this._rxQueue = [];
    this._rxQueueIndex = 0;

    // Wire up TX listener
    usart.onByteTransmit = (byte) => {
      const ch = String.fromCharCode(byte);
      this.txOutput += ch;
      this.txEvents.push({ cycle: cpu.cycles, char: ch, byte });
    };
  }

  /**
   * Load RX schedule from user config.
   * Each entry: { text?, bytes?, atCycle }
   */
  loadRxSchedule(serialInput = []) {
    const queue = [];
    for (const entry of serialInput) {
      const atCycle = entry.atCycle ?? 0;
      if (entry.text) {
        for (const ch of entry.text) {
          queue.push({ atCycle, byte: ch.charCodeAt(0) });
        }
      } else if (Array.isArray(entry.bytes)) {
        for (const b of entry.bytes) {
          queue.push({ atCycle, byte: b & 0xff });
        }
      }
    }
    this._rxQueue = queue.sort((a, b) => a.atCycle - b.atCycle);
    this._rxQueueIndex = 0;
  }

  /**
   * Call once per simulation step.
   * Injects any RX bytes whose atCycle has been reached.
   */
  tick() {
    while (
      this._rxQueueIndex < this._rxQueue.length &&
      this._cpu.cycles >= this._rxQueue[this._rxQueueIndex].atCycle
    ) {
      const { byte } = this._rxQueue[this._rxQueueIndex++];
      // Push byte into UART receive register
      if (typeof this._usart.onRxComplete === 'function') {
        this._usart.onRxComplete(byte);
      }
    }
  }

  get rxDelivered() { return this._rxQueueIndex; }
  get rxPending()   { return this._rxQueue.length - this._rxQueueIndex; }

  result() {
    return {
      tx: {
        output: this.txOutput,
        byteCount: this.txEvents.length,
        events: this.txEvents.slice(-200),
      },
      rx: {
        scheduled: this._rxQueue.length,
        delivered: this.rxDelivered,
        pending: this.rxPending,
        schedule: this._rxQueue.slice(0, 100).map(e => ({
          atCycle: e.atCycle,
          char: e.byte >= 32 && e.byte < 127 ? String.fromCharCode(e.byte) : null,
          byte: e.byte,
        })),
      },
    };
  }
}

// ─── Main Simulation Runner ──────────────────────────────────────────────────

function runSimulation(bytes, config) {
  const {
    steps = 2000000,
    cpuFreq = 16e6,
    buttons = [],        // [{ pin: 'D2', triggerAfterCycles: 100000 }]
    leds = [],           // [{ pin: 'B5', label: 'LED13' }]
    servoPin = 'B1',     // OC1A
    stepperPins = null,  // { in1:'D8', in2:'D9', in3:'D10', in4:'D11' }
    oledEnabled = false,
    neoPixels = 0,       // number of NeoPixels on pin D6
    adcInputs = {},      // { 'A0': 512, 'A1': 300 } — analog values (0–1023)
    serialInput = [],    // [{ text:'hello\n', atCycle:200000 }, { bytes:[0x01], atCycle:500000 }]
  } = config;

  const program = loadProgram(bytes);
  const cpu = new CPU(program, cpuFreq);

  // ── Timers ──
  const timer0 = new AVRTimer(cpu, timer0Config);
  const timer1 = new AVRTimer(cpu, timer1Config);
  const timer2 = new AVRTimer(cpu, timer2Config);

  // ── Ports ──
  const portB = new AVRIOPort(cpu, portBConfig);
  const portC = new AVRIOPort(cpu, portCConfig);
  const portD = new AVRIOPort(cpu, portDConfig);

  function resolvePort(pinStr) {
    const port = pinStr[0].toUpperCase();
    const pin = parseInt(pinStr[1]);
    const portObj = port === 'B' ? portB : port === 'C' ? portC : portD;
    return { portObj, pin };
  }

  // ── UART (two-way) ──
  const usart = new AVRUSART(cpu, usart0Config, cpuFreq);
  const serialSim = new SerialSimulator(usart, cpu);
  serialSim.loadRxSchedule(serialInput);

  // ── ADC ──
  const adc = new AVRADC(cpu, adcConfig);
  const adcPinMap = { A0: 0, A1: 1, A2: 2, A3: 3, A4: 4, A5: 5 };
  for (const [pinName, value] of Object.entries(adcInputs)) {
    const channel = adcPinMap[pinName];
    if (channel !== undefined) {
      adc.channelValues[channel] = value;
    }
  }

  // ── I2C ──
  const i2c = new AVRI2C(cpu, i2cConfig);
  const i2cLog = [];
  const oled = oledEnabled ? new OLEDSimulator() : null;
  i2c.onStart = () => {};
  i2c.onConnected = (address, write) => {
    i2cLog.push({ event: 'connect', address: `0x${address.toString(16)}`, write });
  };
  i2c.onData = (data) => {
    i2cLog.push({ event: 'data', byte: data });
  };

  // ── SPI ──
  const spi = new AVRSPI(cpu, spiConfig, cpuFreq);
  const spiLog = [];
  spi.onTransfer = (byte) => {
    spiLog.push({ cycle: cpu.cycles, byte });
    return 0xff; // MISO = 0xFF (no slave responding)
  };

  // ── LEDs ──
  const ledSims = leds.map(({ pin, label }) => {
    const { portObj, pin: pinNum } = resolvePort(pin);
    return new LEDSimulator(portObj, 1 << pinNum, label || `LED_${pin}`);
  });

  // ── Buttons ──
  const buttonSims = buttons.map(({ pin, label, triggerAfterCycles }) => {
    const { portObj, pin: pinNum } = resolvePort(pin);
    return new ButtonSimulator(cpu, portObj, pinNum, label || `BTN_${pin}`, triggerAfterCycles);
  });

  // ── Servo ──
  const servo = new ServoSimulator();
  const servoPinInfo = resolvePort(servoPin);
  let lastServoState = false;
  servoPinInfo.portObj.addListener(() => {
    const isHigh = servoPinInfo.portObj.pinState(servoPinInfo.pin) === PinState.High;
    if (isHigh !== lastServoState) {
      servo.onPinChange(isHigh, cpu.cycles, cpuFreq);
      lastServoState = isHigh;
    }
  });

  // ── Stepper ──
  let stepperSim = null;
  let stepperPortPins = null;
  if (stepperPins) {
    stepperSim = new StepperSimulator();
    stepperPortPins = Object.entries(stepperPins).map(([key, pin]) => resolvePort(pin));
    // Listen on all 4 stepper ports
    const allPorts = [...new Set(stepperPortPins.map(p => p.portObj))];
    allPorts.forEach(port => {
      port.addListener(() => {
        const pattern = stepperPortPins.reduce((acc, { portObj, pin }, i) => {
          const isHigh = portObj.pinState(pin) === PinState.High ? 1 : 0;
          return acc | (isHigh << (3 - i));
        }, 0);
        stepperSim.onPattern(pattern, cpu.cycles);
      });
    });
  }

  // ── NeoPixels ──
  let neoPixelSim = null;
  let lastNeoState = false;
  if (neoPixels > 0) {
    neoPixelSim = new NeoPixelSimulator(neoPixels);
    const neoPort = portD;
    const neoPin = 6; // D6
    neoPort.addListener(() => {
      const isHigh = neoPort.pinState(neoPin) === PinState.High;
      if (isHigh !== lastNeoState) {
        neoPixelSim.onPinChange(isHigh, cpu.cycles);
        lastNeoState = isHigh;
      }
    });
  }

  // ── Pin state snapshot ──
  const pinSnapshots = [];
  const snapshotInterval = Math.floor(steps / 20);

  // ── Run loop ──
  for (let i = 0; i < steps; i++) {
    avrInstruction(cpu);
    timer0.tick();
    timer1.tick();
    timer2.tick();
    usart.tick();
    serialSim.tick();
    adc.tick();

    buttonSims.forEach(b => b.tick());

    if (i % snapshotInterval === 0) {
      pinSnapshots.push({
        cycle: cpu.cycles,
        portB: portB.pinState(0) + '' + portB.pinState(1) + portB.pinState(2) + portB.pinState(3) + portB.pinState(4) + portB.pinState(5),
        portD: portD.pinState(0) + '' + portD.pinState(1) + portD.pinState(2),
      });
    }
  }

  // ── Build result ──
  return {
    cycles: cpu.cycles,
    simulatedTimeMs: ((cpu.cycles / cpuFreq) * 1000).toFixed(2),
    serial: serialSim.result(),
    leds: ledSims.map(l => l.result()),
    buttons: buttonSims.map(b => b.result()),
    servo: servo.result(),
    stepper: stepperSim ? stepperSim.result() : null,
    oled: oled ? oled.result() : null,
    neoPixels: neoPixelSim ? neoPixelSim.result() : null,
    i2c: { events: i2cLog.slice(-50) },
    spi: { transfers: spiLog.slice(-50) },
    pinSnapshots,
  };
}

// ─── Routes ──────────────────────────────────────────────────────────────────

/**
 * POST /simulate
 *
 * Body (JSON):
 * {
 *   "hexFile": "...",          // Intel HEX string (preferred)
 *   "hexBytes": [0x0e, ...],   // OR raw byte array
 *   "steps": 2000000,
 *   "cpuFreq": 16000000,
 *   "leds": [{ "pin": "B5", "label": "LED13" }],
 *   "buttons": [{ "pin": "D2", "label": "BTN1", "triggerAfterCycles": 500000 }],
 *   "servoPin": "B1",
 *   "stepperPins": { "in1": "D8", "in2": "D9", "in3": "D10", "in4": "D11" },
 *   "oledEnabled": true,
 *   "neoPixels": 8,
 *   "adcInputs": { "A0": 512, "A1": 300 },
 *   "serialInput": [
 *     { "text": "hello\n", "atCycle": 200000 },
 *     { "bytes": [0x01, 0x0a], "atCycle": 500000 }
 *   ]
 * }
 */
app.post('/simulate', (req, res) => {
  try {
    const { hexFile, hexBytes, ...config } = req.body;

    let bytes;
    if (hexFile) {
      bytes = decodeIntelHex(hexFile);
    } else if (hexBytes && Array.isArray(hexBytes)) {
      bytes = hexBytes;
    } else {
      return res.status(400).json({ error: 'Provide either hexFile (Intel HEX string) or hexBytes (byte array).' });
    }

    const result = runSimulation(bytes, config);
    res.json({ ok: true, result });
  } catch (err) {
    res.status(500).json({ ok: false, error: err.message, stack: err.stack });
  }
});

/**
 * GET /health
 */
app.get('/health', (req, res) => {
  res.json({ status: 'ok', version: '3.0.0', components: ['LED', 'Button', 'Servo', 'Stepper', 'OLED', 'NeoPixel', 'Serial-TX', 'Serial-RX', 'SPI', 'I2C', 'ADC'] });
});

/**
 * GET /docs  — quick API reference
 */
app.get('/docs', (req, res) => {
  res.json({
    endpoints: {
      'POST /simulate': {
        description: 'Run AVR simulation with component support',
        body: {
          hexFile: 'Intel HEX string (use this OR hexBytes)',
          hexBytes: 'Array of byte numbers',
          steps: 'Number of CPU instructions to run (default: 2000000)',
          cpuFreq: 'CPU frequency in Hz (default: 16000000)',
          leds: '[{ pin: "B5", label: "LED13" }] — B0-B7, C0-C5, D0-D7',
          buttons: '[{ pin: "D2", label: "BTN1", triggerAfterCycles: 500000 }]',
          servoPin: '"B1" (OC1A PWM pin, default)',
          stepperPins: '{ in1:"D8", in2:"D9", in3:"D10", in4:"D11" }',
          oledEnabled: 'true — simulate SSD1306 on I2C address 0x3C',
          neoPixels: '8 — number of WS2812B pixels on D6',
          adcInputs: '{ "A0": 512 } — analog values 0–1023',
          serialInput: '[{ text:"hello\\n", atCycle:200000 }, { bytes:[0x01], atCycle:500000 }] — bytes injected into Arduino RX at given cycle',
        },
      },
    },
    pinMapping: {
      'B0-B7': 'Arduino pins 8-13 + crystal',
      'C0-C5': 'Arduino pins A0-A5',
      'D0-D7': 'Arduino pins 0-7',
      'B1 (OC1A)': 'Servo PWM pin (pin 9)',
      'D6 (OC0A)': 'NeoPixel data pin (pin 6)',
    },
  });
});

/**
 * POST /serial
 *
 * Lightweight endpoint focused purely on two-way serial interaction.
 * Useful for testing sketches that do Serial.read() / Serial.write().
 *
 * Body:
 * {
 *   "hexFile": "...",          // Intel HEX string
 *   "hexBytes": [...],         // OR raw bytes
 *   "steps": 2000000,
 *   "serialInput": [
 *     { "text": "ping\n",  "atCycle": 100000 },
 *     { "text": "status\n","atCycle": 800000 }
 *   ]
 * }
 *
 * Response:
 * {
 *   "ok": true,
 *   "cycles": 2000000,
 *   "simulatedTimeMs": "125.00",
 *   "serial": {
 *     "tx": { "output": "pong\nok\n", "byteCount": 8, "events": [...] },
 *     "rx": { "scheduled": 10, "delivered": 10, "pending": 0, "schedule": [...] }
 *   }
 * }
 */
app.post('/serial', (req, res) => {
  try {
    const { hexFile, hexBytes, steps = 2000000, cpuFreq = 16e6, serialInput = [] } = req.body;

    let bytes;
    if (hexFile) {
      bytes = decodeIntelHex(hexFile);
    } else if (hexBytes && Array.isArray(hexBytes)) {
      bytes = hexBytes;
    } else {
      return res.status(400).json({ error: 'Provide hexFile or hexBytes.' });
    }

    const program = loadProgram(bytes);
    const cpu = new CPU(program, cpuFreq);

    const timer0 = new AVRTimer(cpu, timer0Config);
    const timer1 = new AVRTimer(cpu, timer1Config);
    const timer2 = new AVRTimer(cpu, timer2Config);

    const usart = new AVRUSART(cpu, usart0Config, cpuFreq);
    const serialSim = new SerialSimulator(usart, cpu);
    serialSim.loadRxSchedule(serialInput);

    for (let i = 0; i < steps; i++) {
      avrInstruction(cpu);
      timer0.tick();
      timer1.tick();
      timer2.tick();
      usart.tick();
      serialSim.tick();
    }

    res.json({
      ok: true,
      cycles: cpu.cycles,
      simulatedTimeMs: ((cpu.cycles / cpuFreq) * 1000).toFixed(2),
      serial: serialSim.result(),
    });
  } catch (err) {
    res.status(500).json({ ok: false, error: err.message });
  }
});

// ─── Start ────────────────────────────────────────────────────────────────────

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`✅ avr8js API running on port ${PORT}`);
  console.log(`📖 Docs: http://localhost:${PORT}/docs`);
});
