from flask import Flask, request, jsonify, render_template_string
import serial
import time
import threading
from typing import Optional

SERIAL_PORT = "COM12"
BAUD_RATE = 115200

# -----------------------------
# Machine setup
# -----------------------------

MOTOR_FULL_STEPS_PER_REV = 200  # Normal 1.8° NEMA 17

# Current hardware setup:
# X axis has MS1 + MS2 + MS3 = 1/16
# Y axis has MS1 + MS2 + MS3 = 1/16
X_MICROSTEPS = 16
Y_MICROSTEPS = 16

X_STEPS_PER_DEGREE = (MOTOR_FULL_STEPS_PER_REV * X_MICROSTEPS) / 360.0
Y_STEPS_PER_DEGREE = (MOTOR_FULL_STEPS_PER_REV * Y_MICROSTEPS) / 360.0

# Safer defaults while testing
DEFAULT_X_MAX_FEED = 6000       # degrees/min
DEFAULT_Y_MAX_FEED = 6000       # degrees/min
DEFAULT_X_ACCELERATION = 100    # degrees/sec²
DEFAULT_Y_ACCELERATION = 100    # degrees/sec²

DEFAULT_SPEED_PERCENT = 40

# -----------------------------
# Servo setup for grbl_v1.1h_config_B.hex
# -----------------------------
# Servo is controlled by spindle PWM using M3 S...
# With $30=1000.
# Signal should be on CNC Shield Z-Endstop signal pin.
DEFAULT_PEN_UP_S = 400
DEFAULT_PEN_DOWN_S = 900
DEFAULT_SERVO_DWELL = 0.25

MIN_SERVO_S = 0
MAX_SERVO_S = 1000

app = Flask(__name__)

serial_lock = threading.Lock()
grbl: Optional[serial.Serial] = None


HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>GRBL Golf Ball Plotter Controller</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background: #111827;
      color: #f9fafb;
      margin: 0;
      padding: 24px;
    }

    .container {
      max-width: 1280px;
      margin: 0 auto;
    }

    h1 {
      margin-bottom: 8px;
    }

    h2 {
      margin-top: 0;
    }

    .subtitle {
      color: #9ca3af;
      margin-bottom: 24px;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(285px, 1fr));
      gap: 16px;
    }

    .card {
      background: #1f2937;
      border: 1px solid #374151;
      border-radius: 14px;
      padding: 18px;
    }

    label {
      display: block;
      margin-top: 12px;
      margin-bottom: 6px;
      color: #d1d5db;
    }

    input, select {
      width: 100%;
      padding: 10px;
      border-radius: 8px;
      border: 1px solid #4b5563;
      background: #111827;
      color: #f9fafb;
      box-sizing: border-box;
    }

    input[type="range"] {
      padding: 0;
    }

    button {
      padding: 10px 14px;
      margin: 6px 4px 6px 0;
      border: none;
      border-radius: 8px;
      background: #2563eb;
      color: white;
      cursor: pointer;
      font-weight: bold;
    }

    button:hover {
      background: #1d4ed8;
    }

    button.danger {
      background: #dc2626;
    }

    button.danger:hover {
      background: #b91c1c;
    }

    button.secondary {
      background: #4b5563;
    }

    button.secondary:hover {
      background: #374151;
    }

    button.success {
      background: #16a34a;
    }

    button.success:hover {
      background: #15803d;
    }

    button.warning-btn {
      background: #d97706;
    }

    button.warning-btn:hover {
      background: #b45309;
    }

    pre {
      background: #030712;
      border: 1px solid #374151;
      border-radius: 12px;
      padding: 14px;
      min-height: 320px;
      overflow: auto;
      white-space: pre-wrap;
    }

    .row {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }

    .small {
      color: #9ca3af;
      font-size: 0.9rem;
      line-height: 1.4;
    }

    .warning {
      color: #fbbf24;
    }

    .value-pill {
      display: inline-block;
      background: #111827;
      border: 1px solid #374151;
      padding: 6px 9px;
      border-radius: 999px;
      margin-top: 6px;
      font-size: 0.9rem;
      color: #e5e7eb;
    }

    .two-col {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>GRBL Golf Ball Plotter Controller</h1>
    <div class="subtitle">
      Python web UI → GRBL servo firmware → Arduino Uno → CNC Shield V3 → X ball rotation + Y arm tilt + M3 S servo pen
    </div>

    <div class="grid">
      <div class="card">
        <h2>Connection / Status</h2>
        <button onclick="sendCommand('?')">Get Status</button>
        <button onclick="sendCommand('$$')">Show Settings</button>
        <button onclick="sendCommand('$I')">Show Firmware Info</button>
        <button onclick="sendCommand('$G')">Show G-code State</button>
        <button onclick="sendCommand('$X')" class="success">Unlock</button>
        <button onclick="sendCommand('$C')" class="warning-btn">Toggle Check Mode</button>
        <button onclick="softReset()" class="danger">Soft Reset</button>
        <p class="small">
          Use $I to confirm firmware config B. If GRBL is in Check mode, motors will not move.
        </p>
      </div>

      <div class="card">
        <h2>GRBL Setup</h2>

        <p class="small">
          X axis: 1/{{ x_microsteps }} microstepping, $100 = {{ x_steps_per_degree }} steps/degree<br>
          Y axis: 1/{{ y_microsteps }} microstepping, $101 = {{ y_steps_per_degree }} steps/degree<br>
          Servo: spindle PWM with M3 S-value, signal on CNC Shield Z-Endstop signal pin
        </p>

        <div class="two-col">
          <div>
            <label>X max feed, degrees/min</label>
            <input id="xMaxFeed" type="number" value="{{ default_x_max_feed }}" step="100" />
          </div>

          <div>
            <label>Y max feed, degrees/min</label>
            <input id="yMaxFeed" type="number" value="{{ default_y_max_feed }}" step="100" />
          </div>
        </div>

        <div class="two-col">
          <div>
            <label>X acceleration, degrees/sec²</label>
            <input id="xAcceleration" type="number" value="{{ default_x_acceleration }}" step="10" />
          </div>

          <div>
            <label>Y acceleration, degrees/sec²</label>
            <input id="yAcceleration" type="number" value="{{ default_y_acceleration }}" step="10" />
          </div>
        </div>

        <p>
          <span class="value-pill">X steps/degree: {{ x_steps_per_degree }}</span>
          <span class="value-pill">Y steps/degree: {{ y_steps_per_degree }}</span>
          <span class="value-pill">$30: 1000</span>
        </p>

        <button onclick="applyMachineConfig()" class="success">Apply GRBL Settings</button>

        <p class="small warning">
          This disables soft limits, hard limits, and homing while testing.
        </p>
      </div>

      <div class="card">
        <h2>X Speed Control</h2>

        <label>Speed percentage</label>
        <input id="speedPercent" type="range" min="1" max="100" value="{{ default_speed_percent }}" oninput="updateComputedValues()" />

        <p>
          <span class="value-pill"><span id="speedPercentLabel">{{ default_speed_percent }}</span>%</span>
          <span class="value-pill">Feed: <span id="computedFeedLabel">0</span> deg/min</span>
          <span class="value-pill">RPM: <span id="rpmLabel">0.0</span></span>
        </p>

        <button onclick="setSpeedPercent(10)" class="secondary">10%</button>
        <button onclick="setSpeedPercent(25)" class="secondary">25%</button>
        <button onclick="setSpeedPercent(40)" class="secondary">40%</button>
        <button onclick="setSpeedPercent(60)" class="secondary">60%</button>
        <button onclick="setSpeedPercent(80)" class="secondary">80%</button>
        <button onclick="setSpeedPercent(100)" class="secondary">100%</button>
      </div>

      <div class="card">
        <h2>Pen Servo Control</h2>

        <div class="two-col">
          <div>
            <label>Pen up S-value</label>
            <input id="penUpS" type="number" value="{{ default_pen_up_s }}" step="10" min="0" max="1000" />
          </div>

          <div>
            <label>Pen down S-value</label>
            <input id="penDownS" type="number" value="{{ default_pen_down_s }}" step="10" min="0" max="1000" />
          </div>
        </div>

        <label>Servo dwell, seconds</label>
        <input id="servoDwell" type="number" value="{{ default_servo_dwell }}" step="0.05" min="0" max="5" />

        <div class="row">
          <button onclick="penUp()" class="success">Pen Up</button>
          <button onclick="penDown()" class="warning-btn">Pen Down</button>
          <button onclick="penTest()" class="secondary">Test Pen</button>
          <button onclick="servoOff()" class="danger">Servo Off M5</button>
        </div>

        <p class="small">
          Uses M3 S... instead of Z movement. Tune S-values if the servo moves too far or not far enough.
        </p>
      </div>

      <div class="card">
        <h2>X Relative Rotation</h2>

        <label>Degrees to rotate</label>
        <input id="relativeDegrees" type="number" value="90" step="0.1" />

        <div class="row">
          <button onclick="rotateRelativePositive()">Rotate +</button>
          <button onclick="rotateRelativeNegative()">Rotate -</button>
        </div>

        <div class="row">
          <button class="secondary" onclick="rotateRelative(1)">+1°</button>
          <button class="secondary" onclick="rotateRelative(5)">+5°</button>
          <button class="secondary" onclick="rotateRelative(10)">+10°</button>
          <button class="secondary" onclick="rotateRelative(45)">+45°</button>
          <button class="secondary" onclick="rotateRelative(90)">+90°</button>
          <button class="secondary" onclick="rotateRelative(180)">+180°</button>
          <button class="secondary" onclick="rotateRelative(360)">+360°</button>
        </div>

        <div class="row">
          <button class="secondary" onclick="rotateRelative(-1)">-1°</button>
          <button class="secondary" onclick="rotateRelative(-5)">-5°</button>
          <button class="secondary" onclick="rotateRelative(-10)">-10°</button>
          <button class="secondary" onclick="rotateRelative(-45)">-45°</button>
          <button class="secondary" onclick="rotateRelative(-90)">-90°</button>
          <button class="secondary" onclick="rotateRelative(-180)">-180°</button>
          <button class="secondary" onclick="rotateRelative(-360)">-360°</button>
        </div>
      </div>

      <div class="card">
        <h2>X Go To Absolute Angle</h2>

        <label>Target X angle</label>
        <input id="absoluteAngle" type="number" value="0" step="0.1" />

        <div class="row">
          <button onclick="goToAngle()">Go To X Angle</button>
          <button onclick="zeroCurrentPosition()" class="warning-btn">Zero X/Y</button>
        </div>

        <div class="row">
          <button class="secondary" onclick="goToQuick(0)">X 0°</button>
          <button class="secondary" onclick="goToQuick(90)">X 90°</button>
          <button class="secondary" onclick="goToQuick(180)">X 180°</button>
          <button class="secondary" onclick="goToQuick(270)">X 270°</button>
          <button class="secondary" onclick="goToQuick(360)">X 360°</button>
        </div>
      </div>

      <div class="card">
        <h2>Y Arm Tilt</h2>

        <p class="small">
          Y is now also 1/{{ y_microsteps }} microstepping.
          Lower Y feed/acceleration if the arm shakes, skips, or sounds harsh.
        </p>

        <div class="row">
          <button class="secondary" onclick="moveYRelative(-1)">Y -1°</button>
          <button class="secondary" onclick="moveYRelative(1)">Y +1°</button>
          <button class="secondary" onclick="moveYRelative(-5)">Y -5°</button>
          <button class="secondary" onclick="moveYRelative(5)">Y +5°</button>
          <button class="secondary" onclick="moveYRelative(-10)">Y -10°</button>
          <button class="secondary" onclick="moveYRelative(10)">Y +10°</button>
        </div>

        <div class="row">
          <button onclick="goToY(-45)">Y -45°</button>
          <button onclick="goToY(-25)">Y -25°</button>
          <button onclick="goToY(0)">Y 0°</button>
          <button onclick="goToY(25)">Y +25°</button>
          <button onclick="goToY(45)">Y +45°</button>
        </div>
      </div>

      <div class="card">
        <h2>Preset Tests</h2>

        <button onclick="runPreset()">Run X Rotation Preset</button>
        <button onclick="runArmSwingPreset()">Run Y Arm Swing Preset</button>
        <button onclick="runDrawTest()" class="success">Run Pen Draw Test</button>

        <div class="row">
          <button onclick="rotateRelative(1440)" class="secondary">4 Rotations +</button>
          <button onclick="rotateRelative(-1440)" class="secondary">4 Rotations -</button>
        </div>
      </div>

      <div class="card">
        <h2>Fine Movement</h2>

        <label>X fine degree step</label>
        <input id="fineStep" type="number" value="0.1125" step="0.0001" />

        <button onclick="finePositive()">X Fine +</button>
        <button onclick="fineNegative()">X Fine -</button>

        <p class="small">
          At X 1/16 microstepping, one theoretical microstep is 0.1125°.
        </p>
      </div>

      <div class="card">
        <h2>Raw G-code</h2>

        <label>Command</label>
        <input id="rawCommand" type="text" value="M3 S{{ default_pen_up_s }}" />

        <button onclick="sendRaw()">Send</button>

        <p class="small warning">
          X test: G91 G1 X10 F300<br>
          Y test: G91 G1 Y5 F300<br>
          Pen up/down test: M3 S{{ default_pen_up_s }} / M3 S{{ default_pen_down_s }}<br>
          Servo off: M5
        </p>
      </div>
    </div>

    <h2>Log</h2>
    <pre id="log"></pre>
  </div>

  <script>
    const DEFAULT_X_MAX_FEED = {{ default_x_max_feed }};
    const DEFAULT_Y_MAX_FEED = {{ default_y_max_feed }};
    const DEFAULT_X_ACCELERATION = {{ default_x_acceleration }};
    const DEFAULT_Y_ACCELERATION = {{ default_y_acceleration }};
    const DEFAULT_PEN_UP_S = {{ default_pen_up_s }};
    const DEFAULT_PEN_DOWN_S = {{ default_pen_down_s }};
    const DEFAULT_SERVO_DWELL = {{ default_servo_dwell }};

    function appendLog(text) {
      const log = document.getElementById("log");
      const now = new Date().toLocaleTimeString();
      log.textContent += `[${now}] ${text}\\n`;
      log.scrollTop = log.scrollHeight;
    }

    async function postJson(url, data = {}) {
      try {
        const response = await fetch(url, {
          method: "POST",
          headers: {
            "Content-Type": "application/json"
          },
          body: JSON.stringify(data)
        });

        const result = await response.json();

        if (result.ok) {
          appendLog(`> ${result.command || url}`);
          appendLog(`< ${result.response}`);
        } else {
          appendLog(`ERROR: ${result.error}`);
        }

        return result;
      } catch (error) {
        appendLog(`FETCH ERROR: ${error}`);
      }
    }

    function getXMaxFeed() {
      return parseFloat(document.getElementById("xMaxFeed").value || DEFAULT_X_MAX_FEED);
    }

    function getYMaxFeed() {
      return parseFloat(document.getElementById("yMaxFeed").value || DEFAULT_Y_MAX_FEED);
    }

    function getXAcceleration() {
      return parseFloat(document.getElementById("xAcceleration").value || DEFAULT_X_ACCELERATION);
    }

    function getYAcceleration() {
      return parseFloat(document.getElementById("yAcceleration").value || DEFAULT_Y_ACCELERATION);
    }

    function getSpeedPercent() {
      return parseFloat(document.getElementById("speedPercent").value || "{{ default_speed_percent }}");
    }

    function getComputedFeed() {
      return Math.max(1, getXMaxFeed() * (getSpeedPercent() / 100));
    }

    function getPenUpS() {
      return parseInt(document.getElementById("penUpS").value || DEFAULT_PEN_UP_S);
    }

    function getPenDownS() {
      return parseInt(document.getElementById("penDownS").value || DEFAULT_PEN_DOWN_S);
    }

    function getServoDwell() {
      return parseFloat(document.getElementById("servoDwell").value || DEFAULT_SERVO_DWELL);
    }

    function updateComputedValues() {
      const feed = getComputedFeed();
      const rpm = feed / 360;
      const speedPercent = getSpeedPercent();

      document.getElementById("computedFeedLabel").textContent = Math.round(feed).toString();
      document.getElementById("rpmLabel").textContent = rpm.toFixed(1);
      document.getElementById("speedPercentLabel").textContent = Math.round(speedPercent).toString();
    }

    function setSpeedPercent(value) {
      document.getElementById("speedPercent").value = value;
      updateComputedValues();
    }

    function sendCommand(command) {
      return postJson("/command", { command });
    }

    function sendRaw() {
      const command = document.getElementById("rawCommand").value;
      sendCommand(command);
    }

    function softReset() {
      postJson("/reset", {});
    }

    function applyMachineConfig() {
      postJson("/apply-config", {
        x_max_feed: getXMaxFeed(),
        y_max_feed: getYMaxFeed(),
        x_acceleration: getXAcceleration(),
        y_acceleration: getYAcceleration()
      });
    }

    function penUp() {
      postJson("/pen-up", {
        s: getPenUpS(),
        dwell: getServoDwell()
      });
    }

    function penDown() {
      postJson("/pen-down", {
        s: getPenDownS(),
        dwell: getServoDwell()
      });
    }

    function penTest() {
      postJson("/pen-test", {
        up_s: getPenUpS(),
        down_s: getPenDownS(),
        dwell: getServoDwell()
      });
    }

    function servoOff() {
      postJson("/servo-off", {});
    }

    function runPreset() {
      postJson("/preset-test", {
        feed: getComputedFeed()
      });
    }

    function runArmSwingPreset() {
      postJson("/arm-swing-preset", {
        feed: getYMaxFeed()
      });
    }

    function runDrawTest() {
      postJson("/draw-test", {
        rotation_feed: getComputedFeed(),
        up_s: getPenUpS(),
        down_s: getPenDownS(),
        dwell: getServoDwell()
      });
    }

    function rotateRelative(degrees) {
      return postJson("/rotate-relative", {
        degrees: degrees,
        feed: getComputedFeed()
      });
    }

    function rotateRelativePositive() {
      const d = Math.abs(parseFloat(document.getElementById("relativeDegrees").value || "0"));
      rotateRelative(d);
    }

    function rotateRelativeNegative() {
      const d = Math.abs(parseFloat(document.getElementById("relativeDegrees").value || "0"));
      rotateRelative(-d);
    }

    function goToAngle() {
      const angle = parseFloat(document.getElementById("absoluteAngle").value || "0");
      postJson("/go-to-angle", {
        angle: angle,
        feed: getComputedFeed()
      });
    }

    function goToQuick(angle) {
      document.getElementById("absoluteAngle").value = angle;
      postJson("/go-to-angle", {
        angle: angle,
        feed: getComputedFeed()
      });
    }

    function moveYRelative(degrees) {
      return postJson("/move-y-relative", {
        degrees: degrees,
        feed: getYMaxFeed()
      });
    }

    function goToY(angle) {
      return postJson("/go-to-y-angle", {
        angle: angle,
        feed: getYMaxFeed()
      });
    }

    function zeroCurrentPosition() {
      postJson("/zero-position", {});
    }

    function finePositive() {
      const d = Math.abs(parseFloat(document.getElementById("fineStep").value || "0.1125"));
      rotateRelative(d);
    }

    function fineNegative() {
      const d = Math.abs(parseFloat(document.getElementById("fineStep").value || "0.1125"));
      rotateRelative(-d);
    }

    updateComputedValues();
    appendLog("GRBL golf ball plotter controller loaded.");
  </script>
</body>
</html>
"""


# -----------------------------
# Serial helpers
# -----------------------------

def connect_grbl() -> serial.Serial:
    global grbl

    if grbl and grbl.is_open:
        return grbl

    grbl = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3)
    time.sleep(2)

    # Wake up GRBL
    grbl.write(b"\r\n\r\n")
    time.sleep(1)

    # Clear startup messages
    while grbl.in_waiting:
        line = grbl.readline().decode(errors="ignore").strip()
        if line:
            print(line)

    return grbl


def read_available_lines(ser: serial.Serial) -> list[str]:
    lines = []

    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            lines.append(line)

    return lines


def read_until_ok_or_error(ser: serial.Serial, timeout: float = 15) -> str:
    end_time = time.time() + timeout
    lines = []

    while time.time() < end_time:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        lines.append(line)

        if line == "ok" or line.startswith("error:") or line.startswith("ALARM:"):
            break

    return "\n".join(lines) if lines else "NO RESPONSE"


def wait_until_idle_unlocked(ser: serial.Serial, timeout: float = 20) -> bool:
    end_time = time.time() + timeout

    while time.time() < end_time:
        ser.write(b"?")
        time.sleep(0.15)

        lines = read_available_lines(ser)

        for line in lines:
            if line.startswith("<Idle"):
                return True
            if line.startswith("<Alarm") or line.startswith("ALARM:"):
                return False

        time.sleep(0.05)

    return False


def send_to_grbl_unlocked(ser: serial.Serial, command: str, timeout: float = 15) -> str:
    command = command.strip()

    if not command:
        raise ValueError("Empty command")

    if command == "?":
        ser.write(b"?")
        time.sleep(0.2)
        lines = read_available_lines(ser)
        return "\n".join(lines) if lines else "NO STATUS RESPONSE"

    ser.write((command + "\n").encode("utf-8"))
    return read_until_ok_or_error(ser, timeout=timeout)


def send_to_grbl(command: str, timeout: float = 15) -> str:
    with serial_lock:
        ser = connect_grbl()
        return send_to_grbl_unlocked(ser, command, timeout=timeout)


def send_many(commands: list[str], delay: float = 0.04, wait_idle_between: bool = True) -> str:
    results = []

    with serial_lock:
        ser = connect_grbl()

        for cmd in commands:
            if wait_idle_between:
                wait_until_idle_unlocked(ser)

            response = send_to_grbl_unlocked(ser, cmd)
            results.append(f"{cmd} -> {response}")
            time.sleep(delay)

    return "\n".join(results)


# -----------------------------
# Validation helpers
# -----------------------------

def validate_feed(feed: float) -> float:
    feed = float(feed)

    if feed <= 0:
        raise ValueError("Feed rate must be greater than 0")

    if feed > 100000:
        raise ValueError("Feed rate is too high")

    return feed


def validate_degrees(degrees: float) -> float:
    degrees = float(degrees)

    if abs(degrees) > 100000:
        raise ValueError("Degree value is too large")

    return degrees


def validate_y_degrees(degrees: float) -> float:
    degrees = float(degrees)

    if degrees < -45 or degrees > 45:
        raise ValueError("Y angle must be between -45 and +45 degrees")

    return degrees


def validate_servo_s(s_value: int) -> int:
    s_value = int(s_value)

    if s_value < MIN_SERVO_S or s_value > MAX_SERVO_S:
        raise ValueError(f"Servo S value must be between {MIN_SERVO_S} and {MAX_SERVO_S}")

    return s_value


def validate_dwell(dwell: float) -> float:
    dwell = float(dwell)

    if dwell < 0:
        raise ValueError("Dwell must not be negative")

    if dwell > 5:
        raise ValueError("Dwell is too long")

    return dwell


# -----------------------------
# Routes
# -----------------------------

@app.route("/")
def index():
    return render_template_string(
        HTML,
        x_microsteps=X_MICROSTEPS,
        y_microsteps=Y_MICROSTEPS,
        x_steps_per_degree=f"{X_STEPS_PER_DEGREE:.6f}",
        y_steps_per_degree=f"{Y_STEPS_PER_DEGREE:.6f}",
        default_x_max_feed=DEFAULT_X_MAX_FEED,
        default_y_max_feed=DEFAULT_Y_MAX_FEED,
        default_x_acceleration=DEFAULT_X_ACCELERATION,
        default_y_acceleration=DEFAULT_Y_ACCELERATION,
        default_speed_percent=DEFAULT_SPEED_PERCENT,
        default_pen_up_s=DEFAULT_PEN_UP_S,
        default_pen_down_s=DEFAULT_PEN_DOWN_S,
        default_servo_dwell=DEFAULT_SERVO_DWELL,
    )


@app.route("/command", methods=["POST"])
def command():
    data = request.get_json(force=True)
    cmd = data.get("command", "").strip()

    try:
        response = send_to_grbl(cmd)

        return jsonify({
            "ok": True,
            "command": cmd,
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/reset", methods=["POST"])
def reset():
    try:
        with serial_lock:
            ser = connect_grbl()
            ser.write(b"\x18")
            time.sleep(1)
            lines = read_available_lines(ser)

        return jsonify({
            "ok": True,
            "command": "CTRL-X RESET",
            "response": "\n".join(lines) if lines else "RESET SENT"
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/apply-config", methods=["POST"])
def apply_config():
    data = request.get_json(force=True)

    try:
        x_max_feed = validate_feed(data.get("x_max_feed", DEFAULT_X_MAX_FEED))
        y_max_feed = validate_feed(data.get("y_max_feed", DEFAULT_Y_MAX_FEED))

        x_acceleration = float(data.get("x_acceleration", DEFAULT_X_ACCELERATION))
        y_acceleration = float(data.get("y_acceleration", DEFAULT_Y_ACCELERATION))

        if x_acceleration <= 0 or y_acceleration <= 0:
            raise ValueError("Acceleration must be greater than 0")

        if x_acceleration > 10000 or y_acceleration > 10000:
            raise ValueError("Acceleration is too high")

        commands = [
            "$X",

            # Servo/spindle range for grbl-servo
            "$30=1000",
            "$31=0",
            "$32=0",

            # Disable homing/limits while testing
            "$22=0",
            "$20=0",
            "$21=0",

            # X axis = ball rotation
            f"$100={X_STEPS_PER_DEGREE:.6f}",
            f"$110={x_max_feed:.3f}",
            f"$120={x_acceleration:.3f}",
            "$130=100000",

            # Y axis = arm tilt
            f"$101={Y_STEPS_PER_DEGREE:.6f}",
            f"$111={y_max_feed:.3f}",
            f"$121={y_acceleration:.3f}",
            "$131=90",

            # Z is not used for servo anymore.
            # Keep safe low defaults.
            "$102=80.000",
            "$112=500.000",
            "$122=50.000",
            "$132=10",

            "G21",
            "G90",
        ]

        response = send_many(commands, wait_idle_between=False)

        return jsonify({
            "ok": True,
            "command": (
                f"APPLY CONFIG | "
                f"X $100={X_STEPS_PER_DEGREE:.6f}, "
                f"Y $101={Y_STEPS_PER_DEGREE:.6f}, "
                f"Servo via M3 S-value"
            ),
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


# -----------------------------
# Pen servo with M3 S...
# -----------------------------

@app.route("/pen-up", methods=["POST"])
def pen_up():
    data = request.get_json(force=True)

    try:
        s_value = validate_servo_s(data.get("s", DEFAULT_PEN_UP_S))
        dwell = validate_dwell(data.get("dwell", DEFAULT_SERVO_DWELL))

        commands = [
            "$X",
            f"M3 S{s_value}",
            f"G4 P{dwell:.3f}",
        ]

        response = send_many(commands, delay=0.04, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"PEN UP M3 S{s_value}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/pen-down", methods=["POST"])
def pen_down():
    data = request.get_json(force=True)

    try:
        s_value = validate_servo_s(data.get("s", DEFAULT_PEN_DOWN_S))
        dwell = validate_dwell(data.get("dwell", DEFAULT_SERVO_DWELL))

        commands = [
            "$X",
            f"M3 S{s_value}",
            f"G4 P{dwell:.3f}",
        ]

        response = send_many(commands, delay=0.04, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"PEN DOWN M3 S{s_value}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/pen-test", methods=["POST"])
def pen_test():
    data = request.get_json(force=True)

    try:
        up_s = validate_servo_s(data.get("up_s", DEFAULT_PEN_UP_S))
        down_s = validate_servo_s(data.get("down_s", DEFAULT_PEN_DOWN_S))
        dwell = validate_dwell(data.get("dwell", DEFAULT_SERVO_DWELL))

        commands = [
            "$X",

            f"M3 S{up_s}",
            f"G4 P{dwell:.3f}",

            f"M3 S{down_s}",
            f"G4 P{dwell:.3f}",

            f"M3 S{up_s}",
            f"G4 P{dwell:.3f}",

            f"M3 S{down_s}",
            f"G4 P{dwell:.3f}",

            f"M3 S{up_s}",
            f"G4 P{dwell:.3f}",
        ]

        response = send_many(commands, delay=0.04, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"PEN TEST M3 S{up_s}/S{down_s}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/servo-off", methods=["POST"])
def servo_off():
    try:
        commands = [
            "$X",
            "M5",
        ]

        response = send_many(commands, delay=0.04, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": "SERVO OFF M5",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


# -----------------------------
# X movement
# -----------------------------

@app.route("/rotate-relative", methods=["POST"])
def rotate_relative():
    data = request.get_json(force=True)

    try:
        degrees = validate_degrees(data.get("degrees", 0))
        feed = validate_feed(data.get("feed", DEFAULT_X_MAX_FEED * 0.4))

        commands = [
            "$X",
            "G21",
            "G91",
            f"G1 X{degrees:.6f} F{feed:.3f}",
            "G4 P0.01",
            "G90",
        ]

        response = send_many(commands, delay=0.03, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"RELATIVE X ROTATE {degrees:.3f}° @ F{feed:.0f}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/go-to-angle", methods=["POST"])
def go_to_angle():
    data = request.get_json(force=True)

    try:
        angle = validate_degrees(data.get("angle", 0))
        feed = validate_feed(data.get("feed", DEFAULT_X_MAX_FEED * 0.4))

        commands = [
            "$X",
            "G21",
            "G90",
            f"G1 X{angle:.6f} F{feed:.3f}",
            "G4 P0.01",
        ]

        response = send_many(commands, delay=0.03, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"GO TO X {angle:.3f}° @ F{feed:.0f}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


# -----------------------------
# Y movement
# -----------------------------

@app.route("/move-y-relative", methods=["POST"])
def move_y_relative():
    data = request.get_json(force=True)

    try:
        degrees = validate_degrees(data.get("degrees", 0))
        feed = validate_feed(data.get("feed", DEFAULT_Y_MAX_FEED))

        commands = [
            "$X",
            "G21",
            "G91",
            f"G1 Y{degrees:.6f} F{feed:.3f}",
            "G4 P0.01",
            "G90",
        ]

        response = send_many(commands, delay=0.03, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"RELATIVE Y MOVE {degrees:.3f}° @ F{feed:.0f}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/go-to-y-angle", methods=["POST"])
def go_to_y_angle():
    data = request.get_json(force=True)

    try:
        angle = validate_y_degrees(data.get("angle", 0))
        feed = validate_feed(data.get("feed", DEFAULT_Y_MAX_FEED))

        commands = [
            "$X",
            "G21",
            "G90",
            f"G1 Y{angle:.6f} F{feed:.3f}",
            "G4 P0.01",
        ]

        response = send_many(commands, delay=0.03, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"GO TO Y {angle:.3f}° @ F{feed:.0f}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


# -----------------------------
# Zeroing
# -----------------------------

@app.route("/zero-position", methods=["POST"])
def zero_position():
    try:
        commands = [
            "$X",
            "G21",
            "G92 X0 Y0",
            "G90",
        ]

        response = send_many(commands, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": "SET CURRENT X/Y POSITION AS ZERO",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


# -----------------------------
# Presets
# -----------------------------

@app.route("/preset-test", methods=["POST"])
def preset_test():
    data = request.get_json(force=True)

    try:
        feed = validate_feed(data.get("feed", DEFAULT_X_MAX_FEED * 0.4))
        slow_feed = max(100, feed * 0.25)

        commands = [
            "$X",
            "G21",
            "G91",

            f"G1 X90 F{feed:.3f}",
            "G4 P0.3",

            f"G1 X-90 F{feed:.3f}",
            "G4 P0.3",

            f"G1 X360 F{feed:.3f}",
            "G4 P0.3",

            f"G1 X-360 F{feed:.3f}",
            "G4 P0.3",

            f"G1 X10 F{slow_feed:.3f}",
            "G4 P0.2",

            f"G1 X-10 F{slow_feed:.3f}",
            "G4 P0.2",

            "G90",
        ]

        response = send_many(commands, delay=0.05, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"X ROTATION PRESET @ F{feed:.0f}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/arm-swing-preset", methods=["POST"])
def arm_swing_preset():
    data = request.get_json(force=True)

    try:
        feed = validate_feed(data.get("feed", DEFAULT_Y_MAX_FEED))
        swing_feed = min(feed, DEFAULT_Y_MAX_FEED)

        commands = [
            "$X",
            "G21",
            "G90",

            "G92 Y0",

            f"G1 Y-10 F{swing_feed:.3f}",
            "G4 P0.2",
            f"G1 Y10 F{swing_feed:.3f}",
            "G4 P0.2",
            f"G1 Y0 F{swing_feed:.3f}",
            "G4 P0.3",

            f"G1 Y-25 F{swing_feed:.3f}",
            "G4 P0.2",
            f"G1 Y25 F{swing_feed:.3f}",
            "G4 P0.2",
            f"G1 Y0 F{swing_feed:.3f}",
            "G4 P0.3",

            f"G1 Y-45 F{swing_feed:.3f}",
            "G4 P0.2",
            f"G1 Y45 F{swing_feed:.3f}",
            "G4 P0.2",
            f"G1 Y0 F{swing_feed:.3f}",
            "G4 P0.3",
        ]

        response = send_many(commands, delay=0.05, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"Y ARM SWING PRESET @ F{swing_feed:.0f}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


@app.route("/draw-test", methods=["POST"])
def draw_test():
    data = request.get_json(force=True)

    try:
        rotation_feed = validate_feed(data.get("rotation_feed", DEFAULT_X_MAX_FEED * 0.4))
        up_s = validate_servo_s(data.get("up_s", DEFAULT_PEN_UP_S))
        down_s = validate_servo_s(data.get("down_s", DEFAULT_PEN_DOWN_S))
        dwell = validate_dwell(data.get("dwell", DEFAULT_SERVO_DWELL))

        commands = [
            "$X",
            "G21",
            "G90",

            # Pen up
            f"M3 S{up_s}",
            f"G4 P{dwell:.3f}",

            # Move with pen up
            "G91",
            f"G1 X45 F{rotation_feed:.3f}",
            "G4 P0.2",

            # Pen down
            f"M3 S{down_s}",
            f"G4 P{dwell:.3f}",

            # Draw rotation
            f"G1 X180 F{rotation_feed:.3f}",
            "G4 P0.2",

            # Pen up
            f"M3 S{up_s}",
            f"G4 P{dwell:.3f}",

            # Return with pen up
            f"G1 X-225 F{rotation_feed:.3f}",
            "G4 P0.2",

            "G90",
        ]

        response = send_many(commands, delay=0.05, wait_idle_between=True)

        return jsonify({
            "ok": True,
            "command": f"PEN DRAW TEST M3 S{up_s}/S{down_s}",
            "response": response
        })

    except Exception as e:
        return jsonify({
            "ok": False,
            "error": str(e)
        }), 500


# -----------------------------
# Main
# -----------------------------

if __name__ == "__main__":
    print("Starting GRBL golf ball plotter controller...")
    print(f"Serial port: {SERIAL_PORT}")
    print(f"X microsteps: {X_MICROSTEPS}")
    print(f"Y microsteps: {Y_MICROSTEPS}")
    print(f"X steps/degree: {X_STEPS_PER_DEGREE:.6f}")
    print(f"Y steps/degree: {Y_STEPS_PER_DEGREE:.6f}")
    print(f"Default X feed: {DEFAULT_X_MAX_FEED}")
    print(f"Default Y feed: {DEFAULT_Y_MAX_FEED}")
    print(f"Default pen up/down: S{DEFAULT_PEN_UP_S}/S{DEFAULT_PEN_DOWN_S}")
    print("Servo mode: M3 S-value through grbl_v1.1h_config_B.hex")
    print("Open this in your browser:")
    print("http://127.0.0.1:5000")

    app.run(host="127.0.0.1", port=5000, debug=False)