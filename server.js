/**
 * avr8js REST API Server v4.0
 * Only uses confirmed avr8js exports to avoid startup crashes.
 * Confirmed real exports: CPU, avrInstruction, AVRTimer, timer0Config,
 *   timer1Config, timer2Config, AVRIOPort, portBConfig, portCConfig,
 *   portDConfig, AVRUSART, usart0Config
 *
 * I2C/SPI/ADC are simulated via CPU memory write hooks (no bad imports).
 */

import express from 'express';
import {
  CPU,
  avrInstruction,
  AVRTimer,
  timer0Config,
  timer1Config,
  timer2Config,
  AVRIOPort,
  portBConfig,
  portCConfig,
  portDConfig,
  AVRUSART,
  usart0Config,
} from 'avr8js';

const app = express();
app.use(express.json({ limit: '2mb' }));

// ─── Helpers ─────────────────────────────────────────────────────────────────

function loadProgram(bytes) {
  const program = new Uint16Array(32768);
  for (let i = 0; i < bytes.length - 1; i += 2) {
    program[i / 2] = (bytes[i + 1] << 8) | bytes[i];
  }
  return program;
}

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

// ─── Component Simulators ─────────────────────────────────────────────────────

/**
 * LED Simulator — tracks pin HIGH/LOW via port write hooks
 */
class LEDSimulator {
  constructor(label) {
    this.label = label;
    this.state = false;
    this.events = [];
  }
  update(isHigh, cycle) {
    if (isHigh !== this.state) {
      this.state = isHigh;
      this.events.push({ cycle, state: isHigh ? 'ON' : 'OFF' });
    }
  }
  result() {
    return { label: this.label, currentState: this.state ? 'ON' : 'OFF', events: this.events.slice(-50) };
  }
}

/**
 * Button Simulator — injects LOW on a pin at a given cycle
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
      // Pull pin LOW by manipulating port input register via writeData
      const pinr = this._port.pinState ? null : null; // we write directly via pinRegister
      // Use port's internal PIN register to simulate external pull-low
      this._port.setPin?.(this.pin, 0) ?? (this._port.pinState = 0);
      this.pressed = true;
    }
  }
  result() {
    return { label: this.label, pin: this.pin, triggerAtCycle: this.triggerAt, wasPressed: this.pressed };
  }
}

/**
 * Servo Simulator — decodes PWM pulse width on a pin → angle
 */
class ServoSimulator {
  constructor() {
    this.pulseWidthUs = 0;
    this.angle = 90;
    this.events = [];
    this._lastHigh = 0;
    this._isHigh = false;
  }
  onPinChange(isHigh, cycle, cpuFreq) {
    if (isHigh && !this._isHigh) {
      this._lastHigh = cycle;
      this._isHigh = true;
    } else if (!isHigh && this._isHigh) {
      const pulseUs = ((cycle - this._lastHigh) / cpuFreq) * 1e6;
      this._isHigh = false;
      if (pulseUs >= 400 && pulseUs <= 2700) {
        this.pulseWidthUs = pulseUs;
        this.angle = Math.max(0, Math.min(180, Math.round(((pulseUs - 500) / 2000) * 180)));
        this.events.push({ cycle, pulseUs: Math.round(pulseUs), angle: this.angle });
      }
    }
  }
  result() {
    return { currentAngle: this.angle, lastPulseUs: Math.round(this.pulseWidthUs), events: this.events.slice(-50) };
  }
}

/**
 * Stepper Motor Simulator — 4-wire full-step
 */
const FULL_STEP_SEQ = [0b1010, 0b0110, 0b0101, 0b1001];
class StepperSimulator {
  constructor() {
    this.steps = 0;
    this.direction = 'CW';
    this.position = 0;
    this.lastPattern = -1;
    this.events = [];
  }
  onPattern(pattern, cycle) {
    if (pattern === this.lastPattern) return;
    const idx = FULL_STEP_SEQ.indexOf(pattern);
    if (idx === -1) return;
    if (this.lastPattern !== -1) {
      const lastIdx = FULL_STEP_SEQ.indexOf(this.lastPattern);
      const diff = (idx - lastIdx + 4) % 4;
      if (diff === 1 || diff === 3) {
        this.direction = diff === 1 ? 'CW' : 'CCW';
        this.position += diff === 1 ? 1 : -1;
        this.steps++;
        this.events.push({ cycle, step: this.steps, direction: this.direction, position: this.position });
      }
    }
    this.lastPattern = pattern;
  }
  result() {
    return {
      totalSteps: this.steps,
      finalPosition: this.position,
      lastDirection: this.direction,
      angleApprox: Math.round((this.position % 200) * 1.8),
      events: this.events.slice(-100),
    };
  }
}

/**
 * OLED Simulator (SSD1306 I2C) — intercepts TWI register writes
 */
class OLEDSimulator {
  constructor() {
    this.framebuffer = new Uint8Array(128 * 8);
    this.commands = [];
    this._page = 0;
    this._col = 0;
  }
  onI2CByte(isData, byte) {
    if (!isData) {
      this.commands.push(`0x${byte.toString(16).padStart(2, '0')}`);
      if (byte >= 0xb0 && byte <= 0xb7) { this._page = byte & 0x07; this._col = 0; }
    } else if (this._col < 128 && this._page < 8) {
      this.framebuffer[this._page * 128 + this._col++] = byte;
      if (this._col >= 128) { this._col = 0; this._page = (this._page + 1) % 8; }
    }
  }
  result() {
    return {
      i2cAddress: '0x3c',
      commandsReceived: this.commands.slice(-30),
      framebufferNonZeroBytes: this.framebuffer.filter(b => b !== 0).length,
    };
  }
}

/**
 * NeoPixel Simulator (WS2812B) — decodes timing-based bit protocol
 */
class NeoPixelSimulator {
  constructor(numPixels) {
    this.numPixels = numPixels;
    this.pixels = Array(numPixels).fill(null).map(() => ({ r: 0, g: 0, b: 0 }));
    this.events = [];
    this._bits = [];
    this._lastEdge = 0;
    this._isHigh = false;
  }
  onPinChange(isHigh, cycle) {
    if (isHigh && !this._isHigh) { this._lastHigh = cycle; this._isHigh = true; }
    else if (!isHigh && this._isHigh) {
      const d = cycle - this._lastHigh;
      this._isHigh = false;
      this._bits.push(d >= 10 ? 1 : 0); // ~10 cycles = 0.6µs threshold at 16MHz
      if (this._bits.length >= 24) {
        const bits = this._bits.splice(0, 24);
        const g = bits.slice(0,8).reduce((a,b,i) => a|(b<<(7-i)),0);
        const r = bits.slice(8,16).reduce((a,b,i) => a|(b<<(7-i)),0);
        const b = bits.slice(16,24).reduce((a,b,i) => a|(b<<(7-i)),0);
        const idx = this.events.length % this.numPixels;
        this.pixels[idx] = { r, g, b };
        this.events.push({ cycle, pixel: idx, r, g, b, hex: `#${r.toString(16).padStart(2,'0')}${g.toString(16).padStart(2,'0')}${b.toString(16).padStart(2,'0')}` });
      }
    }
  }
  result() {
    return { numPixels: this.numPixels, pixels: this.pixels, events: this.events.slice(-50) };
  }
}

/**
 * Two-Way Serial Simulator
 * TX: captures every byte AVR transmits
 * RX: injects bytes into UART at scheduled cycles
 *
 * serialInput: [{ text: 'hello\n', atCycle: 200000 }, { bytes: [0x01], atCycle: 500000 }]
 */
class SerialSimulator {
  constructor(usart, cpu) {
    this._usart = usart;
    this._cpu = cpu;
    this.txOutput = '';
    this.txEvents = [];
    this._rxQueue = [];
    this._rxIndex = 0;

    usart.onByteTransmit = (byte) => {
      const ch = String.fromCharCode(byte);
      this.txOutput += ch;
      this.txEvents.push({ cycle: cpu.cycles, char: ch, byte });
    };
  }

  loadRxSchedule(serialInput = []) {
    const queue = [];
    for (const entry of serialInput) {
      const atCycle = entry.atCycle ?? 0;
      if (entry.text) {
        for (const ch of String(entry.text)) queue.push({ atCycle, byte: ch.charCodeAt(0) });
      } else if (Array.isArray(entry.bytes)) {
        for (const b of entry.bytes) queue.push({ atCycle, byte: b & 0xff });
      }
    }
    this._rxQueue = queue.sort((a, b) => a.atCycle - b.atCycle);
    this._rxIndex = 0;
  }

  tick() {
    while (
      this._rxIndex < this._rxQueue.length &&
      this._cpu.cycles >= this._rxQueue[this._rxIndex].atCycle
    ) {
      const { byte } = this._rxQueue[this._rxIndex++];
      if (typeof this._usart.onRxComplete === 'function') {
        this._usart.onRxComplete(byte);
      }
    }
  }

  result() {
    return {
      tx: {
        output: this.txOutput,
        byteCount: this.txEvents.length,
        events: this.txEvents.slice(-200),
      },
      rx: {
        scheduled: this._rxQueue.length,
        delivered: this._rxIndex,
        pending: this._rxQueue.length - this._rxIndex,
        schedule: this._rxQueue.slice(0, 100).map(e => ({
          atCycle: e.atCycle,
          char: e.byte >= 32 && e.byte < 127 ? String.fromCharCode(e.byte) : null,
          byte: e.byte,
        })),
      },
    };
  }
}

// ─── Main Simulation Runner ───────────────────────────────────────────────────

function resolvePin(pinStr) {
  // pinStr like "B5", "D2", "C1"
  const port = pinStr[0].toUpperCase();
  const pin = parseInt(pinStr[1]);
  return { port, pin };
}

function runSimulation(bytes, config) {
  const {
    steps = 2000000,
    cpuFreq = 16e6,
    leds = [],           // [{ pin: 'B5', label: 'LED13' }]
    buttons = [],        // [{ pin: 'D2', label: 'BTN1', triggerAfterCycles: 500000 }]
    servoPin = 'B1',
    stepperPins = null,  // { in1:'D8', in2:'D9', in3:'D10', in4:'D11' }
    oledEnabled = false,
    neoPixels = 0,       // count of WS2812B on pin D6
    adcInputs = {},      // { A0: 512, A1: 300 } — written to ADC data registers directly
    serialInput = [],    // [{ text:'hello\n', atCycle:200000 }]
  } = config;

  const program = loadProgram(bytes);
  const cpu = new CPU(program);

  // ── Timers ──
  const timer0 = new AVRTimer(cpu, timer0Config);
  const timer1 = new AVRTimer(cpu, timer1Config);
  const timer2 = new AVRTimer(cpu, timer2Config);

  // ── GPIO Ports ──
  const portB = new AVRIOPort(cpu, portBConfig);
  const portC = new AVRIOPort(cpu, portCConfig);
  const portD = new AVRIOPort(cpu, portDConfig);

  function getPort(portLetter) {
    return portLetter === 'B' ? portB : portLetter === 'C' ? portC : portD;
  }

  // ── Two-Way Serial ──
  const usart = new AVRUSART(cpu, usart0Config, cpuFreq);
  const serialSim = new SerialSimulator(usart, cpu);
  serialSim.loadRxSchedule(serialInput);

  // ── ADC — inject values via data registers ──
  // ATmega328p ADC data registers: ADCL=0x78, ADCH=0x79
  // Channel selection is in ADMUX (0x7C). We intercept reads per channel.
  const ADC_CHANNEL_MAP = { A0: 0, A1: 1, A2: 2, A3: 3, A4: 4, A5: 5 };
  const adcValues = {};
  for (const [pin, val] of Object.entries(adcInputs)) {
    const ch = ADC_CHANNEL_MAP[pin];
    if (ch !== undefined) adcValues[ch] = Math.max(0, Math.min(1023, val));
  }
  // Hook ADC data register reads to return injected values
  cpu.readHooks[0x78] = () => { // ADCL
    const ch = cpu.data[0x7c] & 0x0f;
    return (adcValues[ch] ?? 512) & 0xff;
  };
  cpu.readHooks[0x79] = () => { // ADCH
    const ch = cpu.data[0x7c] & 0x0f;
    return ((adcValues[ch] ?? 512) >> 8) & 0x03;
  };
  // Also mark ADC as always complete (ADIF=1, ADSC=0 in ADCSRA=0x7A)
  cpu.writeHooks[0x7a] = (val) => {
    cpu.data[0x7a] = (val | 0x10) & ~0x40; // set ADIF, clear ADSC
  };

  // ── I2C log (intercept TWI registers) ──
  const i2cLog = [];
  const oled = oledEnabled ? new OLEDSimulator() : null;
  // TWDR = 0xBB (TWI data register), TWCR = 0xBC
  let twiIsData = false;
  cpu.writeHooks[0xbb] = (val) => { // TWDR write
    cpu.data[0xbb] = val;
    i2cLog.push({ type: 'write', byte: val });
    if (oled) oled.onI2CByte(twiIsData, val);
  };
  cpu.writeHooks[0xbc] = (val) => { // TWCR write
    cpu.data[0xbc] = val | 0x80; // TWINT=1 (always acknowledge)
    twiIsData = !!(val & 0x04);  // rough: check EA bit to guess data vs address
  };

  // ── SPI log ──
  const spiLog = [];
  // SPDR = 0x4E
  cpu.writeHooks[0x4e] = (val) => {
    cpu.data[0x4e] = val;
    spiLog.push({ cycle: cpu.cycles, byte: val });
    // SPSR = 0x4D — set SPIF (transfer complete)
    cpu.data[0x4d] = cpu.data[0x4d] | 0x80;
  };

  // ── LEDs ──
  const ledSims = {};
  for (const { pin, label } of leds) {
    const { port, pin: pinNum } = resolvePin(pin);
    const key = `${port}${pinNum}`;
    const sim = new LEDSimulator(label || `LED_${pin}`);
    ledSims[key] = { sim, pinNum };
    getPort(port).addListener(() => {
      const isHigh = (cpu.data[portBConfig.portRegister + (port === 'B' ? 0 : port === 'C' ? -2 : -4)] >> pinNum) & 1;
      sim.update(!!isHigh, cpu.cycles);
    });
  }

  // Better LED listener using port objects
  for (const { port, pin: pinNum, sim } of Object.values(ledSims).map((v, i) => ({ ...v, port: leds[i]?.pin[0]?.toUpperCase() }))) {
    // (already wired above)
  }

  // ── Buttons ──
  const buttonSims = buttons.map(({ pin, label, triggerAfterCycles }) => {
    const { port, pin: pinNum } = resolvePin(pin);
    return new ButtonSimulator(cpu, getPort(port), pinNum, label || `BTN_${pin}`, triggerAfterCycles);
  });

  // ── Servo ──
  const servo = new ServoSimulator();
  const { port: servoPort, pin: servoPin2 } = resolvePin(servoPin);
  let lastServo = false;
  getPort(servoPort).addListener(() => {
    const portReg = servoPort === 'B' ? portBConfig.portRegister : servoPort === 'C' ? portCConfig.portRegister : portDConfig.portRegister;
    const isHigh = !!((cpu.data[portReg] >> servoPin2) & 1);
    if (isHigh !== lastServo) { servo.onPinChange(isHigh, cpu.cycles, cpuFreq); lastServo = isHigh; }
  });

  // ── Stepper ──
  let stepperSim = null;
  if (stepperPins) {
    stepperSim = new StepperSimulator();
    const pins = Object.values(stepperPins).map(p => resolvePin(p));
    const listenPorts = [...new Set(pins.map(p => p.port))];
    listenPorts.forEach(portLetter => {
      getPort(portLetter).addListener(() => {
        const pattern = pins.reduce((acc, { port, pin: pinNum }, i) => {
          const reg = port === 'B' ? portBConfig.portRegister : port === 'C' ? portCConfig.portRegister : portDConfig.portRegister;
          return acc | (((cpu.data[reg] >> pinNum) & 1) << (3 - i));
        }, 0);
        stepperSim.onPattern(pattern, cpu.cycles);
      });
    });
  }

  // ── NeoPixels on D6 ──
  let neoSim = null;
  if (neoPixels > 0) {
    neoSim = new NeoPixelSimulator(neoPixels);
    let lastNeo = false;
    portD.addListener(() => {
      const isHigh = !!((cpu.data[portDConfig.portRegister] >> 6) & 1);
      if (isHigh !== lastNeo) { neoSim.onPinChange(isHigh, cpu.cycles); lastNeo = isHigh; }
    });
  }

  // ── Run loop ──
  const snapshotInterval = Math.max(1, Math.floor(steps / 20));
  const pinSnapshots = [];

  for (let i = 0; i < steps; i++) {
    avrInstruction(cpu);
    timer0.tick();
    timer1.tick();
    timer2.tick();
    usart.tick();
    serialSim.tick();
    buttonSims.forEach(b => b.tick());

    if (i % snapshotInterval === 0) {
      pinSnapshots.push({
        cycle: cpu.cycles,
        portB: cpu.data[portBConfig.portRegister].toString(2).padStart(8, '0'),
        portC: cpu.data[portCConfig.portRegister].toString(2).padStart(8, '0'),
        portD: cpu.data[portDConfig.portRegister].toString(2).padStart(8, '0'),
      });
    }
  }

  return {
    cycles: cpu.cycles,
    simulatedTimeMs: ((cpu.cycles / cpuFreq) * 1000).toFixed(2),
    serial: serialSim.result(),
    leds: Object.values(ledSims).map(({ sim }) => sim.result()),
    buttons: buttonSims.map(b => b.result()),
    servo: servo.result(),
    stepper: stepperSim?.result() ?? null,
    oled: oled?.result() ?? null,
    neoPixels: neoSim?.result() ?? null,
    i2c: { events: i2cLog.slice(-50) },
    spi: { transfers: spiLog.slice(-50) },
    pinSnapshots,
  };
}

// ─── Routes ───────────────────────────────────────────────────────────────────

/**
 * POST /simulate — full simulation with all components
 */
app.post('/simulate', (req, res) => {
  try {
    const { hexFile, hexBytes, ...config } = req.body;
    let bytes;
    if (hexFile) bytes = decodeIntelHex(hexFile);
    else if (Array.isArray(hexBytes)) bytes = hexBytes;
    else return res.status(400).json({ error: 'Provide hexFile (Intel HEX string) or hexBytes (array).' });

    res.json({ ok: true, result: runSimulation(bytes, config) });
  } catch (err) {
    res.status(500).json({ ok: false, error: err.message, stack: err.stack });
  }
});

/**
 * POST /serial — lightweight two-way serial only endpoint
 *
 * Body: { hexFile|hexBytes, steps, cpuFreq,
 *   serialInput: [{ text:'ping\n', atCycle:100000 }, { bytes:[0x01], atCycle:500000 }] }
 */
app.post('/serial', (req, res) => {
  try {
    const { hexFile, hexBytes, steps = 2000000, cpuFreq = 16e6, serialInput = [] } = req.body;
    let bytes;
    if (hexFile) bytes = decodeIntelHex(hexFile);
    else if (Array.isArray(hexBytes)) bytes = hexBytes;
    else return res.status(400).json({ error: 'Provide hexFile or hexBytes.' });

    const program = loadProgram(bytes);
    const cpu = new CPU(program);
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

/**
 * GET /health
 */
app.get('/health', (_req, res) => {
  res.json({
    status: 'ok',
    version: '4.0.0',
    components: ['LED', 'Button', 'Servo', 'Stepper', 'OLED', 'NeoPixel', 'Serial-TX', 'Serial-RX', 'SPI', 'I2C', 'ADC'],
  });
});

/**
 * GET /docs
 */
app.get('/docs', (_req, res) => {
  res.json({
    endpoints: {
      'POST /simulate': {
        hexFile: 'Intel HEX string',
        hexBytes: 'Array of byte numbers (alternative to hexFile)',
        steps: 'CPU instructions to run (default: 2000000)',
        cpuFreq: 'Hz (default: 16000000)',
        leds: '[{ pin:"B5", label:"LED13" }]',
        buttons: '[{ pin:"D2", label:"BTN1", triggerAfterCycles:500000 }]',
        servoPin: '"B1" (OC1A, Arduino pin 9)',
        stepperPins: '{ in1:"D8", in2:"D9", in3:"D10", in4:"D11" }',
        oledEnabled: 'true — SSD1306 on I2C 0x3C',
        neoPixels: '8 — WS2812B count on D6',
        adcInputs: '{ "A0": 512, "A1": 300 } — values 0–1023',
        serialInput: '[{ text:"hello\\n", atCycle:200000 }, { bytes:[0x01], atCycle:500000 }]',
      },
      'POST /serial': {
        description: 'Lightweight two-way serial only (no GPIO overhead)',
        hexFile: 'Intel HEX string',
        hexBytes: 'Byte array',
        steps: 'default 2000000',
        serialInput: '[{ text:"ping\\n", atCycle:100000 }]',
      },
    },
    pinMapping: {
      'B0-B7': 'Arduino pins 8-13 + crystal',
      'C0-C5': 'Arduino analog pins A0-A5',
      'D0-D7': 'Arduino digital pins 0-7',
      'B1 (OC1A)': 'Servo PWM (pin 9)',
      'D6 (OC0A)': 'NeoPixel data (pin 6)',
    },
  });
});

// ─── Start ────────────────────────────────────────────────────────────────────

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`avr8js API running on port ${PORT}`);
});
