/*
 * ELEC3204 - Stairlift Motor Controller
 * Application: Home stairlift for elderly/mobility-impaired users
 *
 * Features:
 * - 4-quadrant H-bridge motor control (L298N)
 * - Smooth ramping for passenger comfort
 * - PID closed-loop speed regulation
 * - Emergency stop function
 * - Latching direction control (press Up/Down to start, Stop to stop)
 * - Direction reversal while moving
 * - Position tracking via encoder
 * - Auto-home via 3-second Stop button hold
 * - DEBUG_MODE: manual press-and-hold positioning (5-sec DOWN hold at bottom)
 * - Visual status indicators (LEDs)
 *
 * Hardware:
 * - Arduino Uno
 * - L298N Dual H-Bridge Motor Driver
 * - DC Motor with 48 CPR encoder
 * - 3 push buttons (Up, Down, Stop)
 * - 3 status LEDs (Green, Yellow, Red)
 */

// ============================================================================
// OUTPUT MODE
// ============================================================================
// Define HUMAN_OUTPUT to see readable event messages (button presses, state
// transitions, "DEBUG_MODE: ready for input", etc.) on the Serial Monitor.
// Comment it out to emit only CSV telemetry for the Python plotter.


//#define HUMAN_OUTPUT
//#define OPEN_LOOP_MODE   // Uncomment for open-loop demonstration


// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Encoder pins (hardware interrupts)
#define ENCODER_A_PIN 2     // INT0 - Yellow wire
#define ENCODER_B_PIN 3     // INT1 - White wire

// Button pins (with internal pull-up)
#define BUTTON_UP_PIN 6     // Up direction
#define BUTTON_DOWN_PIN 4   // Down direction
#define BUTTON_STOP_PIN 5   // Emergency stop

// L298N Motor Driver pins
#define MOTOR_IN1_PIN 8     // Direction control 1
#define MOTOR_ENA_PIN 9     // PWM speed control (must be PWM-capable)
#define MOTOR_IN2_PIN 10    // Direction control 2

// Status LED pins
#define LED_UP_PIN 11       // Green  - Moving up
#define LED_DOWN_PIN 12     // Yellow - Moving down
#define LED_STOPPED_PIN 13  // Red    - Stopped / Emergency stop / Homing

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================

enum State {
  IDLE,
  RAMP_UP,
  RUNNING,
  RAMP_DOWN,
  REVERSING,  // Ramping down to switch direction
  HOMING,     // Returning to bottom position
  BRAKING,
  E_STOP,
  DEBUG_MODE  // Manual press-and-hold positioning (no PID, no limits)
};

enum Direction {
  DIR_STOP,
  DIR_UP,
  DIR_DOWN
};

// ============================================================================
// MECHANICAL GEOMETRY
// ============================================================================

// Triangle dimensions (cm)
const float BASE_LENGTH_CM            = 17;  // horizontal base
const float ELEVATION_CM              = 10.0;  // vertical rise

// Pulley dimensions (cm)
const float MOTOR_PULLEY_DIAMETER_CM    = 5.0;
const float REDIRECT_PULLEY_DIAMETER_CM = 3.0;  // not used in math, for reference

// Encoder / gearbox
const int   ENCODER_CPR = 12;    // counts per revolution at motor shaft
const float GEAR_RATIO  = 99.0;  // CHANGE THIS to match your motor label

// ============================================================================
// DERIVED CONSTANTS
// ============================================================================

const float BOARD_LENGTH_CM      = sqrt(BASE_LENGTH_CM * BASE_LENGTH_CM +
                                        ELEVATION_CM * ELEVATION_CM);
const float MOTOR_PULLEY_CIRC_CM = PI * MOTOR_PULLEY_DIAMETER_CM;
const long  PULSES_PER_CM        = (long)((ENCODER_CPR * GEAR_RATIO) /
                                          MOTOR_PULLEY_CIRC_CM);
const long  PULSES_FULL_TRAVEL   = (long)(BOARD_LENGTH_CM * PULSES_PER_CM);

// Safety margins — stop slightly before mechanical ends
const long  TOP_LIMIT_PULSES    = PULSES_FULL_TRAVEL - (1 * PULSES_PER_CM);
const long  BOTTOM_LIMIT_PULSES = 0.5 * PULSES_PER_CM;
const long  HOMING_TOLERANCE_PULSES = 4 * PULSES_PER_CM;   // 1 cm tolerance around home

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// State machine
State     currentState     = IDLE;
Direction currentDirection = DIR_STOP;
Direction pendingDirection = DIR_STOP;  // target direction after REVERSING

// PWM control
int pwmValue       = 0;    // current PWM duty cycle (0-255)
int targetPwmValue = 50;  // target PWM for steady-state operation
const int UP_PWM   = 100;
const int DOWN_PWM = 75; // closed loop 75

const int UP_BREAKAWAY_PWM   = 170;
const int UP_CRUISE_PWM      = 55;
const int DOWN_BREAKAWAY_PWM = 120;
const int DOWN_CRUISE_PWM    = 50;
const float MOTION_THRESHOLD_RPM = 5.0;   // we're definitely moving above this

// Ramp control
const int           RAMP_STEP     = 5;    // PWM increment per step
const int           RAMP_INTERVAL = 50;   // ms between ramp steps
const int           E_STOP_STEP   = 15;   // faster decrement for E_STOP
unsigned long       lastRampTime  = 0;

// Active electrical braking — used at the bottom limit only, where gravity
// would otherwise accelerate the cart during PWM ramp-down.
const unsigned long BRAKE_HOLD_MS  = 500;
unsigned long       brakeStartTime = 0;

// Homing
const unsigned long HOME_HOLD_MS = 3000;  // Stop hold duration to trigger homing
const int           HOME_PWM     = 100;   // PWM for homing
unsigned long       stopHoldStart = 0;    // when Stop was first pressed
bool                stopHeldDown  = false;// whether Stop is currently held

// Debug mode (manual positioning)
const unsigned long DEBUG_HOLD_MS        = 5000; // DOWN hold at bottom to enter
const int           DEBUG_PWM            = 120;  // fixed PWM while button held
unsigned long       downHoldStart        = 0;
bool                downHeldDown         = false;
bool                debugAwaitingRelease = false; // wait for fresh press after entry

// Encoder (volatile for ISR access)
volatile long pulseCount       = 0;
volatile bool encoderDirection = true;  // true = up, false = down

// Speed measurement
float         measuredRPM        = 0.0;
float         lastValidRPM       = 0.0;
const float   MAX_PLAUSIBLE_RPM  = 200.0;  // physical max ~165 RPM at 12V
unsigned long lastSpeedCalcTime  = 0;
const int     SPEED_CALC_INTERVAL = 50;    // ms
long          lastPulseCount     = 0;

// PID controller — tune Kp/Ki independently per direction
const float KP_UP   = 1.5;
const float KI_UP   = 0.3 / 25;
const float KP_DOWN = 1.2;
const float KI_DOWN = 0.2 / 25;
const float TARGET_RPM_UP   = 27.0;  // active setpoint going up
const float TARGET_RPM_DOWN = 40.0;  // gravity helps; same RPM needs less PWM, keeps PID linear
float       targetRPM       = TARGET_RPM_UP;  // current setpoint (updated by PID)

float       pidError         = 0.0;
float       pidIntegral      = 0.0;
const float PID_INTEGRAL_MAX = 200.0;

// Button debouncing
unsigned long lastButtonChangeTime[3] = {0, 0, 0};
bool          buttonState[3]          = {HIGH, HIGH, HIGH};
bool          lastButtonReading[3]    = {HIGH, HIGH, HIGH};
const int     DEBOUNCE_DELAY          = 15;  // ms

// Emergency stop LED flashing
unsigned long lastEStopFlashTime    = 0;
bool          eStopLedState         = false;
const int     E_STOP_FLASH_INTERVAL = 250;  // ms

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("# Stairlift Controller - Starting");

  // Encoder
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  // Buttons with internal pull-up
  pinMode(BUTTON_UP_PIN,   INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

  // Motor driver
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_ENA_PIN, OUTPUT);

  // LEDs
  pinMode(LED_UP_PIN,      OUTPUT);
  pinMode(LED_DOWN_PIN,    OUTPUT);
  pinMode(LED_STOPPED_PIN, OUTPUT);

  // Initialise motor stopped
  setMotorDirection(DIR_STOP);
  analogWrite(MOTOR_ENA_PIN, 0);
  updateLEDs();

  // Encoder interrupt on rising edge of A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  Serial.print("# Board length: ");
  Serial.print(BOARD_LENGTH_CM, 2);
  Serial.print(" cm | Full travel pulses: ");
  Serial.println(PULSES_FULL_TRAVEL);
  Serial.print("# CONFIG TARGET_RPM_UP=");
  Serial.print(TARGET_RPM_UP, 1);
  Serial.print(" TARGET_RPM_DOWN=");
  Serial.println(TARGET_RPM_DOWN, 1);
  Serial.println("# Setup complete - System ready");
  Serial.println("# millis,state,targetRPM,measuredRPM,pwm");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  updateButtons();
  calculateSpeed();
  handleStateMachine();
  updateMotorPWM();
  updateLEDs();
  plotterPrint();

  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500) {
    //debugPrint();
    //plotData();
    lastDebugTime = millis();
  }
}

// ============================================================================
// ENCODER ISR
// ============================================================================

void encoderISR() {
  // Read B on rising edge of A to determine direction
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    pulseCount++;
    encoderDirection = true;
  } else {
    pulseCount--;
    encoderDirection = false;
  }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void updateButtons() {
  int buttonPins[3] = {BUTTON_UP_PIN, BUTTON_DOWN_PIN, BUTTON_STOP_PIN};

  for (int i = 0; i < 3; i++) {
    bool reading = digitalRead(buttonPins[i]);

    if (reading != lastButtonReading[i]) {
      lastButtonChangeTime[i] = millis();
    }

    if ((millis() - lastButtonChangeTime[i]) > DEBOUNCE_DELAY) {
      if (reading != buttonState[i]) {
        buttonState[i] = reading;
        if (buttonState[i] == LOW) {
          handleButtonPress(i);
        } else {
          handleButtonRelease(i);
        }
      }
    }

    lastButtonReading[i] = reading;
  }
}

void handleButtonPress(int button) {
  switch (button) {

    case 0: // Up
      Serial.println("UP button pressed");
      if (atTopLimit()) {
        Serial.println("Already at top — ignoring");
        break;
      }
      if (currentState == IDLE) {
        currentDirection = DIR_UP;
        currentState     = RAMP_UP;
        targetPwmValue   = targetPwmForDirection(DIR_UP);
      } else if (currentDirection == DIR_DOWN &&
                 (currentState == RAMP_UP || currentState == RUNNING)) {
        Serial.println("Reversing: DOWN -> UP");
        pendingDirection = DIR_UP;
        currentState     = REVERSING;
      }
      break;

    case 1: // Down
      Serial.println("DOWN button pressed");

      // At bottom + IDLE → start debug-mode arming timer instead of motion
      if (currentState == IDLE && atBottomLimit()) {
        downHeldDown  = true;
        downHoldStart = millis();
        Serial.println("DOWN held at bottom — arming debug mode (hold 5s)");
        break;
      }
      if (atBottomLimit()) {
        Serial.println("Already at bottom — ignoring");
        break;
      }
      if (currentState == IDLE) {
        currentDirection = DIR_DOWN;
        currentState     = RAMP_UP;
        targetPwmValue   = targetPwmForDirection(DIR_DOWN);;
      } else if (currentDirection == DIR_UP &&
                 (currentState == RAMP_UP || currentState == RUNNING)) {
        Serial.println("Reversing: UP -> DOWN");
        pendingDirection = DIR_DOWN;
        currentState     = REVERSING;
      }
      break;

    case 2: // Stop — exits DEBUG_MODE cleanly; otherwise normal E_STOP / arm-home
      stopHeldDown  = true;
      stopHoldStart = millis();
      Serial.println("STOP button pressed");

      if (currentState == DEBUG_MODE) {
        Serial.println("Exiting DEBUG_MODE — re-anchoring bottom to current position");
        pwmValue             = 0;
        currentDirection     = DIR_STOP;
        debugAwaitingRelease = false;

        // Use the current cart position as the new "bottom" reference
        noInterrupts();
        pulseCount     = 0;
        lastPulseCount = 0;
        interrupts();

        currentState = IDLE;
        break;
      }
      if (currentState != IDLE && currentState != HOMING) {
        currentState = E_STOP;
      }
      break;
  }
}

void handleButtonRelease(int button) {
  switch (button) {
    case 0:
      Serial.println("UP button released (motor continues)");
      break;
    case 1:
      if (downHeldDown) {
        downHeldDown = false;
        Serial.println("DOWN released — debug-mode arming cancelled");
      } else {
        Serial.println("DOWN button released (motor continues)");
      }
      break;
    case 2:
      stopHeldDown = false;
      Serial.println("STOP button released");
      break;
  }
}

// ============================================================================
// SPEED CALCULATION
// ============================================================================

void calculateSpeed() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastSpeedCalcTime;

  if (elapsedTime >= SPEED_CALC_INTERVAL) {
    long pulseDelta = pulseCount - lastPulseCount;

    float rawRPM = (float)(abs(pulseDelta) * 60000.0) /
                   ((float)ENCODER_CPR * GEAR_RATIO * elapsedTime);

    // Reject noise spikes above physical maximum
    if (rawRPM > MAX_PLAUSIBLE_RPM) {
      measuredRPM = lastValidRPM;
    } else {
      measuredRPM  = rawRPM;
      lastValidRPM = rawRPM;
    }

    lastPulseCount    = pulseCount;
    lastSpeedCalcTime = currentTime;
  }
}

// ============================================================================
// SERIAL PLOTTER OUTPUT — STACKED TIMING DIAGRAM
// ============================================================================

const int PLOT_INTERVAL_MS = 20;   // 50 Hz update rate
const int LANE_HIGH        = 60;   // "on/pressed" offset within a lane

void plotData() {
  const int laneBtnUp   = 800;
  const int laneBtnDown = 700;
  const int laneBtnStop = 600;
  const int laneLedU    = 500;
  const int laneLedD    = 400;
  const int laneLedR    = 300;
  const int lanePWM     = 150;
  const int lanePos     = 0;

  int btnU = laneBtnUp   + ((buttonState[0] == LOW) ? LANE_HIGH : 0);
  int btnD = laneBtnDown + ((buttonState[1] == LOW) ? LANE_HIGH : 0);
  int btnS = laneBtnStop + ((buttonState[2] == LOW) ? LANE_HIGH : 0);
  int ledU = laneLedU    + (digitalRead(LED_UP_PIN)      ? LANE_HIGH : 0);
  int ledD = laneLedD    + (digitalRead(LED_DOWN_PIN)    ? LANE_HIGH : 0);
  int ledR = laneLedR    + (digitalRead(LED_STOPPED_PIN) ? LANE_HIGH : 0);

  int pwmPlot = lanePWM + (pwmValue * 90) / 255;
  int posPlot = lanePos + (int)((getChairPositionCM() / BOARD_LENGTH_CM) * 90.0);

  Serial.print("BtnUp:");    Serial.print(btnU);
  Serial.print("\tBtnDown:");Serial.print(btnD);
  Serial.print("\tBtnStop:");Serial.print(btnS);
  Serial.print("\tLedU:");   Serial.print(ledU);
  Serial.print("\tLedD:");   Serial.print(ledD);
  Serial.print("\tLedR:");   Serial.print(ledR);
  Serial.print("\tPWM:");    Serial.print(pwmPlot);
  Serial.print("\tPos:");    Serial.println(posPlot);
}

// ============================================================================
// STATE MACHINE
// ============================================================================

void handleStateMachine() {
  unsigned long currentTime = millis();

  switch (currentState) {

    case IDLE:
      pwmValue     = 0;
      pidIntegral  = 0;
      lastValidRPM = 0.0;

      // 5-second DOWN hold at bottom → enter DEBUG_MODE
      if (downHeldDown && atBottomLimit() &&
          (millis() - downHoldStart >= DEBUG_HOLD_MS)) {
        Serial.println("5-second DOWN hold — entering DEBUG_MODE");
        downHeldDown         = false;
        currentDirection     = DIR_STOP;
        pwmValue             = 0;
        pidIntegral          = 0;
        debugAwaitingRelease = true;   // wait for fresh button press
        currentState         = DEBUG_MODE;
        break;
      }

      // 3-second Stop hold triggers homing (only if not already at bottom)
      if (stopHeldDown && !atBottomLimit() &&
          (millis() - stopHoldStart >= HOME_HOLD_MS)) {
        Serial.println("3-second Stop hold — homing to bottom");
        stopHeldDown     = false;
        currentDirection = DIR_DOWN;
        currentState     = HOMING;
        pwmValue         = 0;
        pidIntegral      = 0;
      }
      break;

    case RAMP_UP:
      if (currentDirection == DIR_UP && atTopLimit()) {
        Serial.println("Top limit reached — stopping");
        currentState = RAMP_DOWN;
        break;
      }
      if (currentDirection == DIR_DOWN && atBottomLimit()) {
        Serial.println("Bottom limit reached — engaging brake");
        brakeStartTime = currentTime;
        currentState   = BRAKING;
        break;
      }

        if (currentTime - lastRampTime >= RAMP_INTERVAL) {
          int breakawayPwm = (currentDirection == DIR_UP) ? UP_BREAKAWAY_PWM
                                                          : DOWN_BREAKAWAY_PWM;
          int cruisePwm    = (currentDirection == DIR_UP) ? UP_CRUISE_PWM
                                                          : DOWN_CRUISE_PWM;

          // Ramp aggressively toward breakaway
          if (pwmValue < breakawayPwm) {
            pwmValue += RAMP_STEP;
            if (pwmValue > breakawayPwm) pwmValue = breakawayPwm;
          }

          // The moment the motor is actually moving, snap to cruise and hand off
          if (measuredRPM > MOTION_THRESHOLD_RPM) {
            Serial.println("Motion detected — snapping to cruise PWM, entering RUNNING");
            pwmValue       = cruisePwm;
            targetPwmValue = cruisePwm;
            pidIntegral    = 0;
            currentState   = RUNNING;
          }
          lastRampTime = currentTime;
        }
        break;

    case RUNNING:
      if (currentDirection == DIR_UP && atTopLimit()) {
        Serial.println("Top limit reached — stopping");
        currentState = RAMP_DOWN;
        break;
      }
      if (currentDirection == DIR_DOWN && atBottomLimit()) {
        Serial.println("Bottom limit reached — engaging brake");
        brakeStartTime = currentTime;
        currentState   = BRAKING;
        break;
      }

      runPIDControl();
      break;

    case RAMP_DOWN:
      if (currentTime - lastRampTime >= RAMP_INTERVAL) {
        if (pwmValue > 0) {
          pwmValue -= RAMP_STEP;
          if (pwmValue < 0) pwmValue = 0;
        }
        if (pwmValue <= 0) {
          Serial.println("Stopped — entering IDLE");
          currentState     = IDLE;
          currentDirection = DIR_STOP;
        }
        lastRampTime = currentTime;
      }
      break;

    case REVERSING:
      if (currentTime - lastRampTime >= RAMP_INTERVAL) {
        if (pwmValue > 0) {
          pwmValue -= RAMP_STEP;
          if (pwmValue < 0) pwmValue = 0;
        }
        if (pwmValue <= 0) {
          Serial.print("Direction reversed to: ");
          Serial.println(pendingDirection == DIR_UP ? "UP" : "DOWN");
          currentDirection = pendingDirection;
          targetPwmValue   = targetPwmForDirection(currentDirection);
          pendingDirection = DIR_STOP;
          pidIntegral      = 0;
          currentState     = RAMP_UP;
        }
        lastRampTime = currentTime;
      }
      break;

    case HOMING:
      // Drive down until we're within tolerance of the calibrated home
      // (pulseCount == 0). Do NOT re-anchor pulseCount here — the home
      // reference was set at boot or on DEBUG_MODE exit and must persist.
      if (pulseCount <= HOMING_TOLERANCE_PULSES) {
        Serial.println("Homing complete — at home position");
        pwmValue         = 0;
        currentState     = IDLE;
        currentDirection = DIR_STOP;
        stopHoldStart    = millis();
        break;
      }

      if (currentTime - lastRampTime >= RAMP_INTERVAL) {
        if (pwmValue < HOME_PWM) {
          pwmValue += 5;
          if (pwmValue > HOME_PWM) pwmValue = HOME_PWM;
        }
        lastRampTime = currentTime;
      }
      break;

    case E_STOP:
      if (currentTime - lastRampTime >= RAMP_INTERVAL) {
        if (pwmValue > 0) {
          pwmValue -= E_STOP_STEP;
          if (pwmValue < 0) pwmValue = 0;
        }

        if (currentTime - lastEStopFlashTime >= E_STOP_FLASH_INTERVAL) {
          eStopLedState      = !eStopLedState;
          lastEStopFlashTime = currentTime;
        }

        if (pwmValue <= 0) {
          Serial.println("Emergency stop complete — entering IDLE");
          currentState     = IDLE;
          currentDirection = DIR_STOP;
          eStopLedState    = true;
          if (stopHeldDown) stopHoldStart = millis();
        }

        lastRampTime = currentTime;
      }
      break;

    case BRAKING:
  // H-bridge in brake mode (IN1=IN2=HIGH) with ENA full duty shorts the
  // motor terminals through the bridge, dumping back-EMF as heat in the
  // windings. This is much stronger than freewheel and stops the cart
  // against gravity within a few centimetres.
      currentDirection = DIR_STOP;
      pwmValue         = 255;

      if (currentTime - brakeStartTime >= BRAKE_HOLD_MS) {
        Serial.println("Brake complete — entering IDLE");
        pwmValue     = 0;
        currentState = IDLE;
        pidIntegral  = 0;
      }
      break;


    case DEBUG_MODE: {
      // Manual press-and-hold positioning. No PID, no limit checks, no ramping.
      // Exit only via STOP press (handled in handleButtonPress).
      bool upPressed   = (buttonState[0] == LOW);
      bool downPressed = (buttonState[1] == LOW);

      // Wait for both buttons released after entry so we don't immediately
      // drive into the mechanical stop with DOWN still held from arming.
      if (debugAwaitingRelease) {
        pwmValue = 0;
        if (!upPressed && !downPressed) {
          debugAwaitingRelease = false;
          Serial.println("DEBUG_MODE: ready for input");
        }
        break;
      }

      Direction desiredDir = DIR_STOP;
      if (upPressed && !downPressed)      desiredDir = DIR_UP;
      else if (downPressed && !upPressed) desiredDir = DIR_DOWN;
      // Both pressed or neither → stop (safe default)

      if (desiredDir == DIR_STOP) {
        pwmValue = 0;                    // direction held — safer for H-bridge
      } else if (desiredDir == currentDirection) {
        pwmValue = DEBUG_PWM;
      } else {
        // Direction change requested — drop PWM first, flip dir next iteration
        if (pwmValue > 0) {
          pwmValue = 0;
        } else {
          currentDirection = desiredDir;
          pwmValue = DEBUG_PWM;
        }
      }
      break;
    }
  }
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

void runPIDControl() {

  // Direction-specific setpoint and gains
  targetRPM = (currentDirection == DIR_UP) ? TARGET_RPM_UP : TARGET_RPM_DOWN;

  #ifdef OPEN_LOOP_MODE
    pwmValue    = targetPwmValue;   // hold feedforward, no correction
    pidError    = 0;
    pidIntegral = 0;
    return;
  #endif


  pidError = targetRPM - measuredRPM;

  float kp = (currentDirection == DIR_UP) ? KP_UP : KP_DOWN;
  float ki = (currentDirection == DIR_UP) ? KI_UP : KI_DOWN;

  pidIntegral += pidError;
  if (pidIntegral >  PID_INTEGRAL_MAX) pidIntegral =  PID_INTEGRAL_MAX;
  if (pidIntegral < -PID_INTEGRAL_MAX) pidIntegral = -PID_INTEGRAL_MAX;

  float pidOutput = (kp * pidError) + (ki * pidIntegral);

  pwmValue = targetPwmValue + (int)pidOutput;
  if (pwmValue > 255) pwmValue = 255;
  if (pwmValue < 0)   pwmValue = 0;
}

int targetPwmForDirection(Direction dir) {
  return (dir == DIR_UP) ? UP_PWM : DOWN_PWM;
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setMotorDirection(Direction dir) {
  switch (dir) {
    case DIR_UP:
      digitalWrite(MOTOR_IN1_PIN, HIGH);
      digitalWrite(MOTOR_IN2_PIN, LOW);
      break;
    case DIR_DOWN:
      digitalWrite(MOTOR_IN1_PIN, LOW);
      digitalWrite(MOTOR_IN2_PIN, HIGH);
      break;
    case DIR_STOP:
      digitalWrite(MOTOR_IN1_PIN, HIGH);
      digitalWrite(MOTOR_IN2_PIN, HIGH);
      break;
  }
}

void updateMotorPWM() {
  static Direction lastDirection = DIR_STOP;
  if (currentDirection != lastDirection) {
    setMotorDirection(currentDirection);
    lastDirection = currentDirection;
  }
  analogWrite(MOTOR_ENA_PIN, pwmValue);
}

// ============================================================================
// LED INDICATORS
// ============================================================================

void updateLEDs() {
  if (currentState == DEBUG_MODE) {
    // Direction LEDs follow motion; red blinks fast to flag the abnormal mode
    digitalWrite(LED_UP_PIN,      (currentDirection == DIR_UP)   && pwmValue > 0);
    digitalWrite(LED_DOWN_PIN,    (currentDirection == DIR_DOWN) && pwmValue > 0);
    digitalWrite(LED_STOPPED_PIN, (millis() / 250) % 2);

  } else if (currentState == IDLE && downHeldDown && atBottomLimit()) {
    // Fast yellow blink while DOWN is held toward debug-mode entry
    digitalWrite(LED_UP_PIN,      LOW);
    digitalWrite(LED_DOWN_PIN,    (millis() / 200) % 2);
    digitalWrite(LED_STOPPED_PIN, LOW);

  } else if (currentState == E_STOP) {
    // Flash red during emergency stop
    digitalWrite(LED_UP_PIN,      LOW);
    digitalWrite(LED_DOWN_PIN,    LOW);
    digitalWrite(LED_STOPPED_PIN, eStopLedState);

  } else if (currentState == HOMING) {
    // Slow blink red during homing
    digitalWrite(LED_UP_PIN,      LOW);
    digitalWrite(LED_DOWN_PIN,    LOW);
    digitalWrite(LED_STOPPED_PIN, (millis() / 500) % 2);

  } else if (currentState == IDLE && stopHeldDown) {
    // Rapid blink red while Stop hold is building toward home trigger
    digitalWrite(LED_UP_PIN,      LOW);
    digitalWrite(LED_DOWN_PIN,    LOW);
    digitalWrite(LED_STOPPED_PIN, (millis() / 200) % 2);

  } else if (currentState == IDLE) {
    // Solid red when stopped
    digitalWrite(LED_UP_PIN,      LOW);
    digitalWrite(LED_DOWN_PIN,    LOW);
    digitalWrite(LED_STOPPED_PIN, HIGH);

  } else if (currentDirection == DIR_UP) {
    digitalWrite(LED_UP_PIN,      HIGH);
    digitalWrite(LED_DOWN_PIN,    LOW);
    digitalWrite(LED_STOPPED_PIN, LOW);

  } else if (currentDirection == DIR_DOWN) {
    digitalWrite(LED_UP_PIN,      LOW);
    digitalWrite(LED_DOWN_PIN,    HIGH);
    digitalWrite(LED_STOPPED_PIN, LOW);
  }
}

// ============================================================================
// POSITION TRACKING
// ============================================================================

float getChairPositionCM() {
  return (float)pulseCount / (float)PULSES_PER_CM;
}

bool atTopLimit() {
  return pulseCount >= TOP_LIMIT_PULSES;
}

bool atBottomLimit() {
  return pulseCount <= BOTTOM_LIMIT_PULSES;
}

// ============================================================================
// TELEMETRY OUTPUT FOR PYTHON PLOTTER
// ============================================================================
// Emits one CSV line per sample at ~50 Hz, format:
//   millis,state,targetRPM,measuredRPM,pwm
// where state is the integer value of the State enum:
//   0=IDLE 1=RAMP_UP 2=RUNNING 3=RAMP_DOWN 4=REVERSING
//   5=HOMING 6=BRAKING 7=E_STOP 8=DEBUG_MODE
//
// All other Serial.print() lines in the sketch start with '#' or are
// non-numeric strings, so the Python parser silently drops them.

void plotterPrint() {
#ifdef HUMAN_OUTPUT
  return;   // human-readable mode: suppress CSV stream
#else
  static unsigned long lastTelemMs = 0;
  const unsigned long TELEM_INTERVAL_MS = 20;   // 50 Hz

  if (millis() - lastTelemMs < TELEM_INTERVAL_MS) return;
  lastTelemMs = millis();

  Serial.print(millis());           Serial.print(',');
  Serial.print((int)currentState);  Serial.print(',');
  Serial.print(targetRPM, 1);       Serial.print(',');
  Serial.print(measuredRPM, 1);     Serial.print(',');
  Serial.println(pwmValue);
#endif
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void debugPrint() {
  Serial.print("State: ");
  switch (currentState) {
    case IDLE:       Serial.print("IDLE");       break;
    case RAMP_UP:    Serial.print("RAMP_UP");    break;
    case RUNNING:    Serial.print("RUNNING");    break;
    case RAMP_DOWN:  Serial.print("RAMP_DOWN");  break;
    case REVERSING:  Serial.print("REVERSING"); break;
    case HOMING:     Serial.print("HOMING");     break;
    case E_STOP:     Serial.print("E_STOP");     break;
    case DEBUG_MODE: Serial.print("DEBUG_MODE"); break;
    default:         Serial.print("?");          break;
  }

  Serial.print(" | Dir: ");
  switch (currentDirection) {
    case DIR_STOP: Serial.print("STOP"); break;
    case DIR_UP:   Serial.print("UP");   break;
    case DIR_DOWN: Serial.print("DOWN"); break;
  }

  Serial.print(" | PWM: ");
  Serial.print(pwmValue);

  Serial.print(" | Target PWM: ");
  Serial.print(targetPwmValue);

  Serial.print(" | RPM: ");
  Serial.print(measuredRPM, 2);

  Serial.print(" | Pos: ");
  Serial.print(getChairPositionCM(), 1);
  Serial.print("cm");

  Serial.print(" | Pulses: ");
  Serial.print(pulseCount);

  if (currentState == RUNNING) {
    Serial.print(" | Err: ");
    Serial.print(pidError, 2);
    Serial.print(" | Int: ");
    Serial.print(pidIntegral, 2);
  }

  if (stopHeldDown) {
    Serial.print(" | StopHeld: ");
    Serial.print((millis() - stopHoldStart) / 1000.0, 1);
    Serial.print("s");
  }

  if (downHeldDown) {
    Serial.print(" | DownHeld: ");
    Serial.print((millis() - downHoldStart) / 1000.0, 1);
    Serial.print("s");
  }

  Serial.println();
}
