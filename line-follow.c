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
int baseSpeed = 110;
int turnSpeed = 70;     // turning strength
int minSpeed = 90;      // TT motor dead-zone

int threshold = 500;    // adjust if needed

int dirLeft = 1;
int dirRight = 1;

void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
}

// ========= MOTOR =========
void setMotor(int left, int right) {

  left *= dirLeft;
  right *= dirRight;

  // LEFT
  if (left >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    left = -left;
  }

  // RIGHT
  if (right >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    right = -right;
  }

  // prevent motor stop
  if (left > 0) left = max(left, minSpeed);
  if (right > 0) right = max(right, minSpeed);

  analogWrite(PWMA, constrain(left, 0, 255));
  analogWrite(PWMB, constrain(right, 0, 255));
}

// ========= LOOP =========
void loop() {

  int b[5];

  for (int i = 0; i < 5; i++) {
    int val = analogRead(S[i]);
    b[i] = (val < threshold) ? 1 : 0;
  }

  // DEBUG
  Serial.print(b[0]); Serial.print(" ");
  Serial.print(b[1]); Serial.print(" ");
  Serial.print(b[2]); Serial.print(" ");
  Serial.print(b[3]); Serial.print(" ");
  Serial.println(b[4]);

  // ========= BASIC LOGIC =========

  // CENTER
  if (b[2]) {
    setMotor(baseSpeed, baseSpeed);
  }

  // LEFT
  else if (b[1]) {
    setMotor(baseSpeed - turnSpeed, baseSpeed + turnSpeed);
  }
  else if (b[0]) {
    setMotor(baseSpeed - 2*turnSpeed, baseSpeed + 2*turnSpeed);
  }

  // RIGHT
  else if (b[3]) {
    setMotor(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
  }
  else if (b[4]) {
    setMotor(baseSpeed + 2*turnSpeed, baseSpeed - 2*turnSpeed);
  }

  // LOST LINE
  else {
    setMotor(80, -80);
  }
}