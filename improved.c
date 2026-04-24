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
int baseSpeed   = 160;   // increased from 110
int maxSpeed    = 220;   // hard cap
int minSpeed    = 75;    // dead-zone floor (only for active wheel)
int threshold   = 500;

#define DEBUG_SERIAL 0
#define SERIAL_BAUD 115200

int dirLeft  = 1;
int dirRight = 1;

// ========= PID TUNING =========
// Start with only Kp, then add Kd once it tracks well.
// Raise Kp until it starts to oscillate, then back off 20%.
// Add Kd (~10x less than Kp) to dampen oscillation.
float Kp = 35.0;
float Ki = 0.0;   // keep 0 for line followers – integral winds up fast
float Kd = 20.0;

// ========= BROKEN LINE / RECOVERY =========
// How long (ms) to keep driving on last known position before spinning
#define LOST_COAST_MS   120   // coast straight briefly (crosses short gaps)
#define LOST_SPIN_MS    600   // then spin-search for the line

// ========= INTERNAL STATE =========
float  lastError   = 0;
float  integral    = 0;
int    lastPos     = 0;  // last valid weighted position (-4 … +4 scale)
int    lastDir     = 1;  // -1=left, +1=right; keep last non-zero for recovery
unsigned long lostTime = 0;
bool   lineWasLost = false;

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
}

// ========= MOTOR DRIVER =========
void setMotor(int left, int right) {
  left  *= dirLeft;
  right *= dirRight;

  // LEFT motor direction
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else           { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); left = -left; }

  // RIGHT motor direction
  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); right = -right; }

  // Apply dead-zone floor ONLY to the active (non-zero) wheel
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
// Returns a position value from -4 (hard left) to +4 (hard right).
// Returns lastPos and sets *allBlack / *allWhite flags.
int readPosition(bool *allBlack, bool *allWhite, int *sensorCount) {
  // Sensor weights: S0=far-left … S4=far-right
  const int weight[5] = {-4, -2, 0, 2, 4};
  int b[5];
  int sum = 0, count = 0;

  for (int i = 0; i < 5; i++) {
    b[i] = (analogRead(S[i]) < threshold) ? 1 : 0;
    if (b[i]) { sum += weight[i]; count++; }
  }

  // Debug
#if DEBUG_SERIAL
  for (int i = 0; i < 5; i++) { Serial.print(b[i]); Serial.print(" "); }
#endif

  *allBlack = (count == 5);   // junction / solid block
  *allWhite = (count == 0);   // nothing seen
  *sensorCount = count;

  if (count == 0) {
#if DEBUG_SERIAL
    Serial.println("| LOST");
#endif
    return lastPos;            // return last known position
  }

  int pos = sum / count;       // average weighted position
#if DEBUG_SERIAL
  Serial.print("| pos="); Serial.println(pos);
#endif

  lastPos = pos;
  if (pos < 0) lastDir = -1;
  else if (pos > 0) lastDir = 1;
  return pos;
}

// ========================================================
void loop() {
  bool allBlack, allWhite;
  int sensorCount;
  int pos = readPosition(&allBlack, &allWhite, &sensorCount);

  // -------- LINE LOST --------
  if (allWhite) {
    if (!lineWasLost) {
      lostTime    = millis();
      lineWasLost = true;
    }

    unsigned long elapsed = millis() - lostTime;

    if (elapsed < LOST_COAST_MS) {
      // Phase 1: coast forward — bridges short broken-line gaps
      setMotor(baseSpeed - 30, baseSpeed - 30);
    } else if (elapsed < LOST_COAST_MS + LOST_SPIN_MS) {
      // Phase 2: spin toward last known direction
      int spin = 90;
      int searchDir = (lastDir == 0) ? 1 : lastDir;
      setMotor(spin * searchDir, -spin * searchDir);
    } else {
      // Phase 3: widen search — increase spin speed
      int spin = 110;
      int searchDir = (lastDir == 0) ? 1 : lastDir;
      setMotor(spin * searchDir, -spin * searchDir);
    }
    return;
  }

  // -------- LINE FOUND --------
  lineWasLost = false;

  // -------- ALL BLACK = junction, just go straight --------
  if (allBlack) {
    setMotor(baseSpeed, baseSpeed);
    return;
  }

  // -------- PID --------
  float error = (float)pos;
  integral   += error;
  integral    = constrain(integral, -100, 100);  // anti-windup

  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftSpeed  = (int)(baseSpeed + correction);
  int rightSpeed = (int)(baseSpeed - correction);

  // Clamp to maxSpeed without reducing the other wheel below baseSpeed
  leftSpeed  = constrain(leftSpeed,  -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  setMotor(leftSpeed, rightSpeed);
}