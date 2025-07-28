#include <Arduino.h>

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

int lastStateA1 = LOW;
int lastStateA2 = LOW;

// === ขา Encoder ===
#define ENCODER1_A 32
#define ENCODER1_B 33
#define ENCODER2_A 25
#define ENCODER2_B 26

// === ขามอเตอร์ ===
#define MOTOR_A_DIR 14
#define MOTOR_A_PWM 12
#define MOTOR_B_DIR 27
#define MOTOR_B_PWM 13
#define MOTOR_C_DIR 4     // เพิ่มมอเตอร์ C
#define MOTOR_C_PWM 2

// === Buzzer ===
#define BUZZER_PIN 21

// === ความเร็วเริ่มต้น ===
int pwmSpeed = 70;

// === ค่าคงที่ทางฟิสิกส์ของหุ่นยนต์ ===
#define WHEEL_DIAMETER 0.1
#define ENCODER_CPR 400
#define GEAR_RATIO 1.0
#define WHEEL_BASE 0.455

// === ระยะทางล้อ ===
float distance1 = 0;
float distance2 = 0;

// === ตำแหน่งหุ่นยนต์ ===
float posX = 0.0;
float posY = 0.0;
float theta = 0.0;

// === PID state ===
long previousError = 0;
long integral = 0;

// === ฟังก์ชัน Buzzer ===
void beep(int duration) {
  tone(BUZZER_PIN, 1000, duration);
}

// === ฟังก์ชันคำนวณระยะทาง ===
float calculateDistance(long counts) {
  float circumference = WHEEL_DIAMETER * 3.14159;
  return (counts / (ENCODER_CPR * GEAR_RATIO)) * circumference;
}

// === PID ===
int calculatePID(long error) {
  float Kp = 0.5, Ki = 0.001, Kd = 0.001;
  long derivative = error - previousError;
  previousError = error;
  integral += error;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return constrain(output, -50, 50);
}

// === Interrupt Encoder ===
void IRAM_ATTR updateEncoder1() {
  int stateA = digitalRead(ENCODER1_A);
  int stateB = digitalRead(ENCODER1_B);
  if (stateA != lastStateA1) {
    encoder1Count += (stateA == stateB) ? -1 : 1;
    lastStateA1 = stateA;
  }
}

void IRAM_ATTR updateEncoder2() {
  int stateA = digitalRead(ENCODER2_A);
  int stateB = digitalRead(ENCODER2_B);
  if (stateA != lastStateA2) {
    encoder2Count += (stateA == stateB) ? 1 : -1;
    lastStateA2 = stateA;
  }
}

// === ควบคุมมอเตอร์ ===
void driveMotor(int dirPin, int pwmPin, int speed, bool reverse = false) {
  if (reverse) speed = -speed;
  if (speed > 0) digitalWrite(dirPin, HIGH);
  else if (speed < 0) {
    digitalWrite(dirPin, LOW);
    speed = -speed;
  } else digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, constrain(speed, 0, 255));
}

void stopMotors() {
  driveMotor(MOTOR_A_DIR, MOTOR_A_PWM, 0);
  driveMotor(MOTOR_B_DIR, MOTOR_B_PWM, 0);
}

// === Odometry ===
void updateOdometry() {
  static long lastEncoder1 = 0;
  static long lastEncoder2 = 0;

  long delta1 = encoder1Count - lastEncoder1;
  long delta2 = encoder2Count - lastEncoder2;

  lastEncoder1 = encoder1Count;
  lastEncoder2 = encoder2Count;

  float dist1 = calculateDistance(delta1);
  float dist2 = calculateDistance(delta2);

  float d_center = (dist1 + dist2) / 2.0;
  float d_theta = (dist2 - dist1) / WHEEL_BASE;

  theta += d_theta;
  posX += d_center * cos(theta);
  posY += d_center * sin(theta);
}

// === setup() ===
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("ESP32 Joystick, Encoder, and Motor Control");

  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_C_DIR, OUTPUT);  // เพิ่มมอเตอร์ C
  pinMode(MOTOR_C_PWM, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoder2, CHANGE);

  beep(200);
}
  int pwmSpeedC = 100;  // ความเร็วเริ่มต้นของมอเตอร์ C

// === loop() ===
void loop() {
  static char prevCommand = 0;
  char command = 0;

  if (Serial2.available()) {
    command = Serial2.read();
    Serial.print("Joystick Command (Serial2): ");
    Serial.println(command);
  }
  if (Serial.available()) {
    command = Serial.read();
    Serial.print("Manual Command (Serial): ");
    Serial.println(command);
  }

  if (command != 0 && command != prevCommand) {
    if ((prevCommand == 'C' || prevCommand == 'D' || prevCommand == 'B') && command == 'A') {
      encoder1Count = 0;
      encoder2Count = 0;
      previousError = 0;
      integral = 0;
      delay(100);
    }
    prevCommand = command;
  }

  int speedA = pwmSpeed;
  int speedB = pwmSpeed;
  int speedB_back = 77;
  int adjustment = 0;

  long error;

  if (pwmSpeed > 0) {
    error = encoder2Count - encoder1Count;
    adjustment = calculatePID(error);
    speedB = pwmSpeed + adjustment;
  }

  switch (command) {
    case 'I':  // Beep
      for (int i = 0; i < 3; i++) {
        beep(300);
        delay(100);
      }
      break;
      case 'F':  // เพิ่มความเร็วมอเตอร์ C
        pwmSpeedC = min(pwmSpeedC + 10, 255);
        Serial.print("Motor C Speed Increased: ");
        Serial.println(pwmSpeedC);
        break;

      case 'N':  // ลดความเร็วมอเตอร์ C
        pwmSpeedC = max(pwmSpeedC - 10, 0);
        Serial.print("Motor C Speed Decreased: ");
        Serial.println(pwmSpeedC);
        break;

    case 'E':  // เปิดมอเตอร์ C
      driveMotor(MOTOR_C_DIR, MOTOR_C_PWM, -pwmSpeedC);
      break;

    case 'M':  // ปิดมอเตอร์ C
      driveMotor(MOTOR_C_DIR, MOTOR_C_PWM, 0);
      break;

    case 'A':
      driveMotor(MOTOR_A_DIR, MOTOR_A_PWM, speedA);
      driveMotor(MOTOR_B_DIR, MOTOR_B_PWM, speedB, true);
      break;

    case 'B':
      driveMotor(MOTOR_A_DIR, MOTOR_A_PWM, -speedA);
      driveMotor(MOTOR_B_DIR, MOTOR_B_PWM, -speedB_back, true);
      break;

    case 'L':
      pwmSpeed = min(pwmSpeed + 13, 255);
      break;

    case 'K':
      pwmSpeed = max(pwmSpeed - 13, 0);
      break;

    case 'D':
      driveMotor(MOTOR_A_DIR, MOTOR_A_PWM, speedA);
      driveMotor(MOTOR_B_DIR, MOTOR_B_PWM, -speedB_back, true);
      break;

    case 'C':
      driveMotor(MOTOR_A_DIR, MOTOR_A_PWM, -speedA);
      driveMotor(MOTOR_B_DIR, MOTOR_B_PWM, speedB_back, true);
      break;

    default:
      stopMotors();
      break;
  }

  distance1 = calculateDistance(encoder1Count);
  distance2 = calculateDistance(encoder2Count);
  updateOdometry();

  Serial.print("Encoder 1: ");
  Serial.println(encoder1Count);
  Serial.print("Encoder 2: ");
  Serial.println(encoder2Count);

  delay(50);
}
