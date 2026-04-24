// ========= SENSOR PINS =========
int S[5] = {A1, A2, A3, A4, A5};

// ========= MOTOR PINS =========
#define AIN1 4
#define AIN2 3
#define PWMA 9
#define BIN1 6
#define BIN2 7
#define PWMB 10
#define STBY 5

// ========= SETTINGS =========
int baseSpeed    = 160;
int maxSpeed     = 220;
int minSpeed     = 75;    // dead-zone floor (active wheel only)
int minTurnSpeed = 90;    // floor for dynamic base during sharp turns
int sensorThreshold[5] = {500, 500, 500, 500, 500};

#define CALIBRATE 0
#define DEBUG_SERIAL 0
#define SERIAL_BAUD 115200

int dirLeft  = 1;
int dirRight = 1;

// ========= PID TUNING =========
float Kp = 35.0;
float Ki = 0.0;
float Kd = 20.0;

// ========= DYNAMIC SPEED =========
// Scales baseSpeed down proportionally to |error|.
// 0.0 = no reduction (fixed speed), 1.0 = full reduction to minTurnSpeed at max error.
// Start at 0.5 and increase until corners feel controlled.
#define SPEED_REDUCTION_FACTOR  0.5f

// ========= INTERSECTION NAVIGATION =========
// Define your track layout here — one command per junction in order.
// CMD_STRAIGHT = drive through, CMD_LEFT = turn left 90°, CMD_RIGHT = turn right 90°
#define CMD_STRAIGHT  0
#define CMD_LEFT      1
#define CMD_RIGHT     2

int navSequence[] = { CMD_STRAIGHT, CMD_LEFT, CMD_STRAIGHT, CMD_RIGHT };
int navTotal      = 4;   // must match array length above
int navIndex      = 0;

// How long to hold a 90° turn spin — TUNE THIS for your robot.
// Too short: robot exits at an angle. Too long: robot overshoots the new line.
// Typical range: 280–450ms depending on baseSpeed and wheel spacing.
#define TURN_EXECUTE_MS     350
#define TURN_SPIN_SPEED     130
#define JUNCTION_DEBOUNCE_MS 300   // ignore re-trigger while still on same junction

bool executingTurn       = false;
unsigned long turnStartTime   = 0;
int  pendingTurn         = CMD_STRAIGHT;
unsigned long lastJunctionTime = 0;

// ========= LOST LINE / U-TURN RECOVERY =========
// Phase 1: coast forward  — bridges short gaps
// Phase 2: U-turn (180°) — robot now faces back along the track
// Phase 3: backtrack      — drive forward; PID picks up the line
// Phase 4: wide spin      — last resort if still lost

#define LOST_COAST_MS      120
// Time to spin 180°. Depends on motor speed and robot geometry.
// Too little: partial turn. Too much: robot faces wrong direction.
// Start at 700ms, adjust 50ms at a time until heading is reliably reversed.
#define LOST_UTURN_MS      700
#define LOST_BACKTRACK_MS  1200

unsigned long lostTime   = 0;
bool   lineWasLost       = false;

// ========= INTERNAL STATE =========
float  lastError = 0;
float  integral  = 0;
int    lastPos   = 0;
int    lastDir   = 1;   // -1=left, +1=right

#if CALIBRATE
void calibrate() {
  int sMin[5], sMax[5];
  for (int i = 0; i < 5; i++) { sMin[i] = 1023; sMax[i] = 0; }

  // Signal start — blink STBY or just use a delay so you can position the robot
  // Slowly drag the robot across the full line width for ~3 seconds
  unsigned long start = millis();
  while (millis() - start < 3000) {
    for (int i = 0; i < 5; i++) {
      int v = analogRead(S[i]);
      sMin[i] = min(sMin[i], v);
      sMax[i] = max(sMax[i], v);
    }
  }

  // Threshold = midpoint between darkest (on line) and brightest (off line)
  for (int i = 0; i < 5; i++) {
    sensorThreshold[i] = (sMin[i] + sMax[i]) / 2;
  }
}
#endif

// ========================================================
void setup() {
#if DEBUG_SERIAL
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

#if CALIBRATE
  delay(1000)
  calibrate()
#endif
}

// ========= MOTOR DRIVER =========
void setMotor(int left, int right) {
  left  *= dirLeft;
  right *= dirRight;

  if (left >= 0)  { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); left = -left; }

  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); right = -right; }

  if (left  > 0) left  = max(left,  minSpeed);
  if (right > 0) right = max(right, minSpeed);

  analogWrite(PWMA, constrain(left,  0, 255));
  analogWrite(PWMB, constrain(right, 0, 255));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ========= WEIGHTED POSITION =========
int readPosition(bool *allBlack, bool *allWhite, int *sensorCount) {
  const int weight[5] = {-4, -2, 0, 2, 4};
  int b[5];
  int sum = 0, count = 0;

  for (int i = 0; i < 5; i++) {
    b[i] = (analogRead(S[i]) < sensorThreshold[i]) ? 1 : 0;
    if (b[i]) { sum += weight[i]; count++; }
  }

#if DEBUG_SERIAL
  for (int i = 0; i < 5; i++) { Serial.print(b[i]); Serial.print(" "); }
#endif

  *allBlack = (count == 5);
  *allWhite = (count == 0);
  *sensorCount = count;

  if (count == 0) {
#if DEBUG_SERIAL
    Serial.println("| LOST");
#endif
    return lastPos;
  }

  int pos = sum / count;
#if DEBUG_SERIAL
  Serial.print("| pos="); Serial.println(pos);
#endif

  lastPos = pos;
  if (pos < 0) lastDir = -1;
  else if (pos > 0) lastDir = 1;
  return pos;
}

// ========= DYNAMIC BASE SPEED =========
// Returns a speed between baseSpeed (error=0) and minTurnSpeed (error=4).
// Large error = approaching or in a sharp turn = slow down.
int getDynamicSpeed(float error) {
  float t = constrain(abs(error) / 4.0f, 0.0f, 1.0f);
  int dynSpeed = baseSpeed - (int)(t * SPEED_REDUCTION_FACTOR * (baseSpeed - minTurnSpeed));
  return max(dynSpeed, (int)minTurnSpeed);
}

// ========= INTERSECTION HANDLER =========
// Called once when allBlack is detected. Pops the next nav command
// and starts a timed turn if required.
void handleIntersection() {
  unsigned long now = millis();

  // Debounce: ignore if we just handled a junction
  if (now - lastJunctionTime < JUNCTION_DEBOUNCE_MS) return;

  // If we've exhausted the sequence, just go straight (default)
  if (navIndex >= navTotal) return;

  int cmd = navSequence[navIndex];
  navIndex++;
  lastJunctionTime = now;

  if (cmd == CMD_STRAIGHT) {
    // No turn needed — PID takes over immediately after this junction
    return;
  }

  // Start timed 90° turn
  executingTurn = true;
  turnStartTime = now;
  pendingTurn   = cmd;
}

// ========================================================
void loop() {

  // -------- EXECUTING INTERSECTION TURN --------
  // This runs until TURN_EXECUTE_MS elapses, blocking all other logic.
  if (executingTurn) {
    unsigned long elapsed = millis() - turnStartTime;
    if (elapsed < TURN_EXECUTE_MS) {
      if (pendingTurn == CMD_LEFT)
        setMotor(-TURN_SPIN_SPEED, TURN_SPIN_SPEED);
      else
        setMotor(TURN_SPIN_SPEED, -TURN_SPIN_SPEED);
      return;
    }
    executingTurn = false;   // turn complete — resume PID
  }

  bool allBlack, allWhite;
  int  sensorCount;
  int  pos = readPosition(&allBlack, &allWhite, &sensorCount);

  // -------- LINE LOST --------
  if (allWhite) {
    if (!lineWasLost) {
      lostTime    = millis();
      lineWasLost = true;
    }

    unsigned long elapsed = millis() - lostTime;

    if (elapsed < LOST_COAST_MS) {
      // Phase 1: coast forward — bridges short gaps in the line
      setMotor(baseSpeed - 30, baseSpeed - 30);

    } else if (elapsed < LOST_COAST_MS + LOST_UTURN_MS) {
      // Phase 2: U-turn — spin 180° to reverse heading back onto the track.
      // We spin in lastDir so the pivot matches the geometry of the missed turn.
      int searchDir = (lastDir == 0) ? 1 : lastDir;
      setMotor(100 * searchDir, -100 * searchDir);

    } else if (elapsed < LOST_COAST_MS + LOST_UTURN_MS + LOST_BACKTRACK_MS) {
      // Phase 3: backtrack — drive forward (now facing the way we came).
      // Sensors are live here: if they detect the line mid-backtrack,
      // lineWasLost resets on the next loop and PID takes over cleanly.
      setMotor(baseSpeed - 30, baseSpeed - 30);

    } else {
      // Phase 4: still lost after full backtrack — widen spin as last resort
      int searchDir = (lastDir == 0) ? 1 : lastDir;
      setMotor(110 * searchDir, -110 * searchDir);
    }
    return;
  }

  // -------- LINE FOUND --------
  lineWasLost = false;

  // -------- ALL BLACK = junction --------
  if (allBlack) {
    handleIntersection();
    if (!executingTurn) {
      // CMD_STRAIGHT or sequence exhausted — just drive through
      setMotor(baseSpeed, baseSpeed);
    }
    return;
  }

  // -------- PID WITH DYNAMIC SPEED --------
  float error      = (float)pos;
  integral        += error;
  integral         = constrain(integral, -100, 100);

  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // Dynamic base: slows down proportionally as error grows
  int dynBase = getDynamicSpeed(error);

  int leftSpeed  = (int)(dynBase + correction);
  int rightSpeed = (int)(dynBase - correction);

  leftSpeed  = constrain(leftSpeed,  -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  setMotor(leftSpeed, rightSpeed);
}
