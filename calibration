// ========= SENSOR PINS =========
int S[5] = {A1, A2, A3, A4, A5};

// ========= MODE =========
// 0 = normal line following, 1 = autonomous calibration/tuning mode
#define CALIBRATION_MODE 0

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
int sensorThreshold[5] = {500, 500, 500, 500, 500};
bool usePerSensorThresholds = false;

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

#if CALIBRATION_MODE
// ========= CALIBRATION SETTINGS =========
#define CAL_TIMEOUT_MS           30000UL
#define CAL_THRESHOLD_SCAN_MS     9000UL
#define CAL_SETTLE_MS              400UL
#define CAL_TRIAL_MS              1400UL
#define CAL_TEST_BASE_SPEED         120
#define CAL_MIN_SENSOR_SPREAD        90
#define CAL_LARGE_SCORE        1000000.0f

enum CalibrationPhase {
  CAL_PHASE_BOOT = 0,
  CAL_PHASE_SCAN_THRESHOLD,
  CAL_PHASE_SWEEP_KP,
  CAL_PHASE_SWEEP_KD,
  CAL_PHASE_DONE,
  CAL_PHASE_FAIL
};

struct TuneStats {
  unsigned int samples;
  float absErrorSum;
  unsigned int signFlips;
  unsigned int saturations;
  unsigned int lostDetections;
  float prevError;
};

const float kpCandidates[] = {10.0f, 14.0f, 18.0f, 22.0f, 26.0f, 30.0f, 34.0f, 38.0f, 42.0f, 46.0f};
const float kdCandidates[] = {0.0f, 4.0f, 8.0f, 12.0f, 16.0f, 20.0f, 24.0f, 28.0f};
const int kpCandidateCount = sizeof(kpCandidates) / sizeof(kpCandidates[0]);
const int kdCandidateCount = sizeof(kdCandidates) / sizeof(kdCandidates[0]);

CalibrationPhase calPhase = CAL_PHASE_BOOT;
TuneStats calStats;

bool calStarted = false;
bool calTrialActive = false;
bool calPrintedFinal = false;
bool calPrintedFail = false;

unsigned long calStartMs = 0;
unsigned long calPhaseStartMs = 0;
unsigned long calTrialStartMs = 0;

int calRawMin[5] = {1023, 1023, 1023, 1023, 1023};
int calRawMax[5] = {0, 0, 0, 0, 0};

int calKpIndex = 0;
int calKdIndex = 0;

float calBestKp = 35.0f;
float calBestKd = 20.0f;
float calBestKpScore = CAL_LARGE_SCORE;
float calBestKdScore = CAL_LARGE_SCORE;

float calFallbackKp = 35.0f;
float calFallbackKd = 20.0f;
float calFallbackKpScore = CAL_LARGE_SCORE;
float calFallbackKdScore = CAL_LARGE_SCORE;

const char *calFailReason = "unknown";
#endif

// ========================================================
int getThresholdForSensor(int i) {
  if (usePerSensorThresholds) return sensorThreshold[i];
  return threshold;
}

void readRawSensors(int raw[5]) {
  for (int i = 0; i < 5; i++) raw[i] = analogRead(S[i]);
}

// ========================================================
void setup() {
#if DEBUG_SERIAL || CALIBRATION_MODE
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

#if CALIBRATION_MODE
  Serial.println("=== CALIBRATION MODE ENABLED ===");
  Serial.println("Robot will scan thresholds and tune Kp/Kd (Ki fixed at 0).");
#endif
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
int readPosition(bool *allBlack, bool *allWhite) {
  // Sensor weights: S0=far-left … S4=far-right
  const int weight[5] = {-4, -2, 0, 2, 4};
  int raw[5];
  int b[5];
  int sum = 0, count = 0;

  readRawSensors(raw);

  for (int i = 0; i < 5; i++) {
    b[i] = (raw[i] < getThresholdForSensor(i)) ? 1 : 0;
    if (b[i]) { sum += weight[i]; count++; }
  }

  // Debug
#if DEBUG_SERIAL
  for (int i = 0; i < 5; i++) { Serial.print(b[i]); Serial.print(" "); }
#endif

  *allBlack = (count == 5);   // junction / solid block
  *allWhite = (count == 0);   // nothing seen

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

void runNormalFollowMode() {
  bool allBlack, allWhite;
  int pos = readPosition(&allBlack, &allWhite);

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

#if CALIBRATION_MODE
float absFloat(float v) {
  return (v < 0.0f) ? -v : v;
}

void resetControllerState() {
  lastError = 0;
  integral = 0;
  lastPos = 0;
  lastDir = 1;
  lineWasLost = false;
  lostTime = 0;
}

void resetTuneStats(TuneStats *stats) {
  stats->samples = 0;
  stats->absErrorSum = 0.0f;
  stats->signFlips = 0;
  stats->saturations = 0;
  stats->lostDetections = 0;
  stats->prevError = 0.0f;
}

float computeTuneScore(const TuneStats *stats) {
  if (stats->samples == 0) return CAL_LARGE_SCORE;

  float sampleCount = (float)stats->samples;
  float avgError = stats->absErrorSum / sampleCount;
  float flipRate = (float)stats->signFlips / sampleCount;
  float satRate = (float)stats->saturations / sampleCount;
  float lostRate = (float)stats->lostDetections / sampleCount;

  return avgError + (flipRate * 30.0f) + (satRate * 45.0f) + (lostRate * 250.0f);
}

bool isTuneStable(const TuneStats *stats) {
  if (stats->samples == 0) return false;

  float sampleCount = (float)stats->samples;
  float satRate = (float)stats->saturations / sampleCount;
  float lostRate = (float)stats->lostDetections / sampleCount;

  return (satRate < 0.80f) && (lostRate < 0.35f);
}

void updateTuneStats(TuneStats *stats, float error, bool allWhite, int leftCmd, int rightCmd) {
  stats->samples++;
  stats->absErrorSum += absFloat(error);

  if (((error > 0.6f) && (stats->prevError < -0.6f)) ||
      ((error < -0.6f) && (stats->prevError > 0.6f))) {
    stats->signFlips++;
  }

  if ((leftCmd >= maxSpeed) || (leftCmd <= -maxSpeed) ||
      (rightCmd >= maxSpeed) || (rightCmd <= -maxSpeed)) {
    stats->saturations++;
  }

  if (allWhite) stats->lostDetections++;
  stats->prevError = error;
}

void beginTuneTrial() {
  resetControllerState();
  resetTuneStats(&calStats);
  calTrialStartMs = millis();
  calTrialActive = true;
}

void runThresholdScanMotion(unsigned long elapsed) {
  int slow = 85;
  int fast = 125;
  int spin = 90;
  unsigned long block = (elapsed / 1400UL) % 4UL;

  if (block == 0UL) {
    setMotor(slow, fast);      // arc right
  } else if (block == 1UL) {
    setMotor(fast, slow);      // arc left
  } else if (block == 2UL) {
    setMotor(-spin, spin);     // spin left
  } else {
    setMotor(spin, -spin);     // spin right
  }
}

void runCalibrationControlStep(float activeKp, float activeKd, float *errorOut, bool *allWhiteOut, int *leftCmdOut, int *rightCmdOut) {
  bool allBlack = false;
  bool allWhite = false;
  int pos = readPosition(&allBlack, &allWhite);

  int leftCmd = 0;
  int rightCmd = 0;
  float error = (float)pos;

  if (allWhite) {
    int searchDir = (lastDir == 0) ? 1 : lastDir;
    int spin = 85;
    leftCmd = spin * searchDir;
    rightCmd = -spin * searchDir;
  } else if (allBlack) {
    leftCmd = CAL_TEST_BASE_SPEED;
    rightCmd = CAL_TEST_BASE_SPEED;
  } else {
    float derivative = error - lastError;
    float correction = (activeKp * error) + (activeKd * derivative);
    lastError = error;

    leftCmd = constrain((int)(CAL_TEST_BASE_SPEED + correction), -maxSpeed, maxSpeed);
    rightCmd = constrain((int)(CAL_TEST_BASE_SPEED - correction), -maxSpeed, maxSpeed);
  }

  setMotor(leftCmd, rightCmd);

  if (errorOut) *errorOut = error;
  if (allWhiteOut) *allWhiteOut = allWhite;
  if (leftCmdOut) *leftCmdOut = leftCmd;
  if (rightCmdOut) *rightCmdOut = rightCmd;
}

void printThresholdSummary() {
  Serial.println("Threshold scan complete:");
  for (int i = 0; i < 5; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" min="); Serial.print(calRawMin[i]);
    Serial.print(" max="); Serial.print(calRawMax[i]);
    Serial.print(" threshold="); Serial.println(sensorThreshold[i]);
  }
  Serial.print("Global threshold = ");
  Serial.println(threshold);
}

void printCalibrationResults() {
  Serial.println("=== CALIBRATION COMPLETE ===");

  Serial.print("int sensorThreshold[5] = {");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensorThreshold[i]);
    if (i < 4) Serial.print(", ");
  }
  Serial.println("};");

  Serial.print("int threshold = ");
  Serial.print(threshold);
  Serial.println(";");

  Serial.print("float Kp = ");
  Serial.print(Kp, 2);
  Serial.println(";");

  Serial.println("float Ki = 0.00;");

  Serial.print("float Kd = ");
  Serial.print(Kd, 2);
  Serial.println(";");

  Serial.println("bool usePerSensorThresholds = true;  // optional");

  Serial.println("Set CALIBRATION_MODE to 0 for normal operation.");
}

void runCalibrationMode() {
  unsigned long now = millis();

  if (!calStarted) {
    calStarted = true;
    calPhase = CAL_PHASE_BOOT;
    calStartMs = now;
    calPhaseStartMs = now;
    calTrialActive = false;
    calPrintedFinal = false;
    calPrintedFail = false;
    calFailReason = "unknown";
  }

  if ((calPhase != CAL_PHASE_DONE) && (calPhase != CAL_PHASE_FAIL) && ((now - calStartMs) > CAL_TIMEOUT_MS)) {
    calFailReason = "timeout reached";
    calPhase = CAL_PHASE_FAIL;
    calPhaseStartMs = now;
  }

  switch (calPhase) {
    case CAL_PHASE_BOOT: {
      for (int i = 0; i < 5; i++) {
        calRawMin[i] = 1023;
        calRawMax[i] = 0;
      }

      calKpIndex = 0;
      calKdIndex = 0;
      calBestKp = Kp;
      calBestKd = Kd;
      calBestKpScore = CAL_LARGE_SCORE;
      calBestKdScore = CAL_LARGE_SCORE;
      calFallbackKp = Kp;
      calFallbackKd = Kd;
      calFallbackKpScore = CAL_LARGE_SCORE;
      calFallbackKdScore = CAL_LARGE_SCORE;
      usePerSensorThresholds = false;

      resetControllerState();

      Serial.println("Phase 1/3: scanning thresholds.");
      calPhase = CAL_PHASE_SCAN_THRESHOLD;
      calPhaseStartMs = now;
      break;
    }

    case CAL_PHASE_SCAN_THRESHOLD: {
      int raw[5];
      readRawSensors(raw);

      for (int i = 0; i < 5; i++) {
        if (raw[i] < calRawMin[i]) calRawMin[i] = raw[i];
        if (raw[i] > calRawMax[i]) calRawMax[i] = raw[i];
      }

      runThresholdScanMotion(now - calPhaseStartMs);

      if ((now - calPhaseStartMs) >= CAL_THRESHOLD_SCAN_MS) {
        stopMotors();

        bool valid = true;
        long thresholdSum = 0;

        for (int i = 0; i < 5; i++) {
          int spread = calRawMax[i] - calRawMin[i];
          sensorThreshold[i] = (calRawMin[i] + calRawMax[i]) / 2;
          thresholdSum += sensorThreshold[i];

          if (spread < CAL_MIN_SENSOR_SPREAD) valid = false;
        }

        threshold = (int)(thresholdSum / 5L);
        usePerSensorThresholds = true;
        printThresholdSummary();

        if (!valid) {
          calFailReason = "insufficient sensor contrast";
          calPhase = CAL_PHASE_FAIL;
          calPhaseStartMs = now;
        } else {
          Serial.println("Phase 2/3: tuning Kp (Ki fixed at 0).");
          calPhase = CAL_PHASE_SWEEP_KP;
          calPhaseStartMs = now;
          calTrialActive = false;
          calKpIndex = 0;
        }
      }
      break;
    }

    case CAL_PHASE_SWEEP_KP: {
      if (calKpIndex >= kpCandidateCount) {
        if (calBestKpScore >= CAL_LARGE_SCORE) {
          calBestKp = calFallbackKp;
          Serial.print("No stable Kp found, fallback Kp=");
          Serial.println(calBestKp, 2);
        }

        Serial.print("Selected Kp=");
        Serial.println(calBestKp, 2);

        Serial.println("Phase 3/3: tuning Kd.");
        calPhase = CAL_PHASE_SWEEP_KD;
        calPhaseStartMs = now;
        calTrialActive = false;
        calKdIndex = 0;
        resetControllerState();
        break;
      }

      float candidateKp = kpCandidates[calKpIndex];

      if (!calTrialActive) {
        beginTuneTrial();
        Serial.print("Testing Kp=");
        Serial.println(candidateKp, 2);
      }

      unsigned long trialElapsed = now - calTrialStartMs;

      if (trialElapsed < CAL_SETTLE_MS) {
        float tempError;
        bool tempAllWhite;
        int tempLeft;
        int tempRight;
        runCalibrationControlStep(candidateKp, 0.0f, &tempError, &tempAllWhite, &tempLeft, &tempRight);
        break;
      }

      if (trialElapsed < (CAL_SETTLE_MS + CAL_TRIAL_MS)) {
        float error;
        bool allWhite;
        int leftCmd;
        int rightCmd;

        runCalibrationControlStep(candidateKp, 0.0f, &error, &allWhite, &leftCmd, &rightCmd);
        updateTuneStats(&calStats, error, allWhite, leftCmd, rightCmd);
        break;
      }

      stopMotors();
      float score = computeTuneScore(&calStats);
      bool stable = isTuneStable(&calStats);

      if (score < calFallbackKpScore) {
        calFallbackKpScore = score;
        calFallbackKp = candidateKp;
      }

      if (stable && (score < calBestKpScore)) {
        calBestKpScore = score;
        calBestKp = candidateKp;
      }

      Serial.print("Kp result: ");
      Serial.print(candidateKp, 2);
      Serial.print(" score=");
      Serial.print(score, 3);
      Serial.print(" stable=");
      Serial.println(stable ? "yes" : "no");

      calKpIndex++;
      calTrialActive = false;
      break;
    }

    case CAL_PHASE_SWEEP_KD: {
      if (calKdIndex >= kdCandidateCount) {
        if (calBestKdScore >= CAL_LARGE_SCORE) {
          calBestKd = calFallbackKd;
          Serial.print("No stable Kd found, fallback Kd=");
          Serial.println(calBestKd, 2);
        }

        Kp = calBestKp;
        Ki = 0.0f;
        Kd = calBestKd;

        calPhase = CAL_PHASE_DONE;
        calPhaseStartMs = now;
        calPrintedFinal = false;
        stopMotors();
        break;
      }

      float candidateKd = kdCandidates[calKdIndex];

      if (!calTrialActive) {
        beginTuneTrial();
        Serial.print("Testing Kd=");
        Serial.println(candidateKd, 2);
      }

      unsigned long trialElapsed = now - calTrialStartMs;

      if (trialElapsed < CAL_SETTLE_MS) {
        float tempError;
        bool tempAllWhite;
        int tempLeft;
        int tempRight;
        runCalibrationControlStep(calBestKp, candidateKd, &tempError, &tempAllWhite, &tempLeft, &tempRight);
        break;
      }

      if (trialElapsed < (CAL_SETTLE_MS + CAL_TRIAL_MS)) {
        float error;
        bool allWhite;
        int leftCmd;
        int rightCmd;

        runCalibrationControlStep(calBestKp, candidateKd, &error, &allWhite, &leftCmd, &rightCmd);
        updateTuneStats(&calStats, error, allWhite, leftCmd, rightCmd);
        break;
      }

      stopMotors();
      float score = computeTuneScore(&calStats);
      bool stable = isTuneStable(&calStats);

      if (score < calFallbackKdScore) {
        calFallbackKdScore = score;
        calFallbackKd = candidateKd;
      }

      if (stable && (score < calBestKdScore)) {
        calBestKdScore = score;
        calBestKd = candidateKd;
      }

      Serial.print("Kd result: ");
      Serial.print(candidateKd, 2);
      Serial.print(" score=");
      Serial.print(score, 3);
      Serial.print(" stable=");
      Serial.println(stable ? "yes" : "no");

      calKdIndex++;
      calTrialActive = false;
      break;
    }

    case CAL_PHASE_DONE:
      stopMotors();
      if (!calPrintedFinal) {
        printCalibrationResults();
        calPrintedFinal = true;
      }
      break;

    case CAL_PHASE_FAIL:
      stopMotors();
      if (!calPrintedFail) {
        Serial.println("=== CALIBRATION FAILED ===");
        Serial.print("Reason: ");
        Serial.println(calFailReason);
        Serial.println("Check lighting/contrast, sensor height, and track placement.");
        calPrintedFail = true;
      }
      break;
  }
}
#endif

// ========================================================
void loop() {
#if CALIBRATION_MODE
  runCalibrationMode();
#else
  runNormalFollowMode();
#endif
}
