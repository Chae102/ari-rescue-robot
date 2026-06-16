#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Servo.h>
#include <MPU6050_tockn.h>

#define DHTPIN 2
#define DHTTYPE DHT11

const int ENA = 3;
const int WHITE_LED = 4;
const int IN1 = 5;
const int IN2 = 6;
const int TRIG = 7;
const int ECHO = 8;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 11;
const int SERVO_PIN = 12;
const int BUZZER_PIN = 13;

const int LIGHT_SEN = A0;
const int FLAME_SEN = A1;
const int STATE_LED = A2;

const int SPEED_LEFT = 115;
const int SPEED_RIGHT = 115;

const float GYRO_LIMIT = 40.0;
const int FLAME_LIMIT = 300;

const float TEMP_ON = 40.0;
const float TEMP_OFF = 38.0;
const float HUMID_ON = 80.0;
const float HUMID_OFF = 75.0;
const int DARK_ON = 350;
const int DARK_OFF = 430;

const long OBSTACLE_DISTANCE = 25;
const long TURN_CLEAR_DISTANCE = 30;
const unsigned long DHT_INTERVAL = 2000;

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);
Servo myServo;
MPU6050 mpu6050(Wire);

int currentState = 0;
int lastState = -1;

unsigned long lastBuzzerTime = 0;
bool buzzerToggle = false;

float lastTemp = 25.0;
float lastHumid = 50.0;
unsigned long lastDhtReadTime = 0;

bool tempAlarm = false;
bool humidAlarm = false;
bool darkAlarm = false;

void playToneManual(int pin, int frequency, int durationMs) {
  if (frequency <= 0) return;

  long halfPeriod = 1000000L / frequency / 2;
  long cycles = (long)durationMs * 1000L / (halfPeriod * 2);

  for (long i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(pin, LOW);
    delayMicroseconds(halfPeriod);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(WHITE_LED, OUTPUT);
  pinMode(LIGHT_SEN, INPUT);
  pinMode(FLAME_SEN, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(STATE_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  dht.begin();
  myServo.attach(SERVO_PIN);
  myServo.write(90);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print(F("CALIBRATING MPU"));
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(F("RESCUE ROBOT V20"));
  lcd.setCursor(0, 1);
  lcd.print(F("SYSTEM READY"));
  delay(1500);
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();

  updateSensors(currentMillis);
  currentState = decideState();

  updateLcd();
  updateLeds();
  updateBuzzer(currentMillis);
  updateDrive();
}

void updateSensors(unsigned long currentMillis) {
  mpu6050.update();

  if (currentMillis - lastDhtReadTime >= DHT_INTERVAL || lastDhtReadTime == 0) {
    lastDhtReadTime = currentMillis;

    float temp = dht.readTemperature();
    float humid = dht.readHumidity();

    if (!isnan(temp)) lastTemp = temp;
    if (!isnan(humid)) lastHumid = humid;
  }
}

int decideState() {
  float gyroX = mpu6050.getAngleX();
  float gyroY = mpu6050.getAngleY();

  int flameValue = analogRead(FLAME_SEN);
  int lightValue = analogRead(LIGHT_SEN);

  bool isGyroCritical =
    (gyroX > GYRO_LIMIT || gyroX < -GYRO_LIMIT ||
     gyroY > GYRO_LIMIT || gyroY < -GYRO_LIMIT);

  bool isFlameDetected = (flameValue < FLAME_LIMIT);

  if (lastTemp > TEMP_ON) tempAlarm = true;
  else if (lastTemp < TEMP_OFF) tempAlarm = false;

  if (lastHumid > HUMID_ON) humidAlarm = true;
  else if (lastHumid < HUMID_OFF) humidAlarm = false;

  if (lightValue < DARK_ON) darkAlarm = true;
  else if (lightValue > DARK_OFF) darkAlarm = false;

  if (isGyroCritical) return 5;

  if (isFlameDetected) return 4;
  if (tempAlarm) return 2;
  if (humidAlarm) return 3;
  if (darkAlarm) return 1;

  return 0;
}

void updateLcd() {
  if (currentState == lastState) return;

  lcd.clear();

  switch (currentState) {
    case 5:
      lcd.setCursor(0, 0);
      lcd.print(F("GYRO CRITICAL"));
      lcd.setCursor(0, 1);
      lcd.print(F("ROBOT TILTED"));
      break;

    case 4:
      lcd.setCursor(0, 0);
      lcd.print(F("DANGER"));
      lcd.setCursor(0, 1);
      lcd.print(F("FIRE DETECTED"));
      break;

    case 2:
      lcd.setCursor(0, 0);
      lcd.print(F("HIGH TEMP WARN"));
      lcd.setCursor(0, 1);
      lcd.print(F("CHECK AREA"));
      break;

    case 3:
      lcd.setCursor(0, 0);
      lcd.print(F("HUMID WARNING"));
      lcd.setCursor(0, 1);
      lcd.print(F("CHECK AREA"));
      break;

    case 1:
      lcd.setCursor(0, 0);
      lcd.print(F("DARK ZONE"));
      lcd.setCursor(0, 1);
      lcd.print(F("LIGHTS ON"));
      break;

    default:
      lcd.setCursor(0, 0);
      lcd.print(F("MODE: AUTO"));
      lcd.setCursor(0, 1);
      lcd.print(F("EXPLORING AREA"));
      break;
  }

  lastState = currentState;
}

void updateLeds() {
  digitalWrite(STATE_LED, (currentState == 0 || currentState == 1) ? HIGH : LOW);
  digitalWrite(WHITE_LED, darkAlarm ? HIGH : LOW);
}

void updateBuzzer(unsigned long currentMillis) {
  if (currentState == 5) {
    if (currentMillis - lastBuzzerTime >= 80) {
      lastBuzzerTime = currentMillis;
      buzzerToggle = !buzzerToggle;
      playToneManual(BUZZER_PIN, buzzerToggle ? 2300 : 1900, 40);
    }
  } else if (currentState == 4) {
    if (currentMillis - lastBuzzerTime >= 150) {
      lastBuzzerTime = currentMillis;
      buzzerToggle = !buzzerToggle;
      playToneManual(BUZZER_PIN, buzzerToggle ? 1500 : 1000, 60);
    }
  } else if (currentState == 2 || currentState == 3) {
    if (buzzerToggle && currentMillis - lastBuzzerTime >= 200) {
      buzzerToggle = false;
      lastBuzzerTime = currentMillis;
    } else if (!buzzerToggle && currentMillis - lastBuzzerTime >= 600) {
      buzzerToggle = true;
      lastBuzzerTime = currentMillis;
      playToneManual(BUZZER_PIN, 1000, 100);
    }
  } else {
    buzzerToggle = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void updateDrive() {
  if (currentState == 5 || currentState == 4) {
    stopMotors();
    return;
  }

  long distance = getFilteredDistance();

  if (distance < OBSTACLE_DISTANCE) {
    avoidObstacle();
  } else {
    moveForward(SPEED_LEFT, SPEED_RIGHT);
  }
}

void avoidObstacle() {
  for (int i = 0; i <= 5; i++) {
    int decelL = SPEED_LEFT - (i * 22);
    int decelR = SPEED_RIGHT - (i * 22);

    moveForward(clampSpeed(decelL), clampSpeed(decelR));
    if (!safeDelay(40)) return;
  }

  stopMotors();
  if (!safeDelay(200)) return;

  moveBackward(SPEED_LEFT, SPEED_RIGHT);
  if (!safeDelay(220)) return;

  stopMotors();
  if (!safeDelay(300)) return;

  myServo.write(150);
  if (!safeDelay(600)) return;
  long leftDist = getFilteredDistance();

  myServo.write(30);
  if (!safeDelay(600)) return;
  long rightDist = getFilteredDistance();

  myServo.write(90);
  if (!safeDelay(400)) return;

  if (leftDist >= TURN_CLEAR_DISTANCE && rightDist >= TURN_CLEAR_DISTANCE) {
    turnRight(140, 120);
    safeDelay(400);
  } else if (leftDist > rightDist) {
    turnLeft(140, 120);
    safeDelay(420);
  } else {
    turnRight(140, 120);
    safeDelay(400);
  }

  stopMotors();
  safeDelay(200);
}

bool safeDelay(unsigned long durationMs) {
  unsigned long startTime = millis();

  while (millis() - startTime < durationMs) {
    if (isEmergencyNow()) {
      stopMotors();
      currentState = decideState();
      lastState = -1;
      updateLcd();
      return false;
    }

    delay(20);
  }

  return true;
}

bool isEmergencyNow() {
  mpu6050.update();

  float gyroX = mpu6050.getAngleX();
  float gyroY = mpu6050.getAngleY();

  int flameValue = analogRead(FLAME_SEN);

  bool isGyroCritical =
    (gyroX > GYRO_LIMIT || gyroX < -GYRO_LIMIT ||
     gyroY > GYRO_LIMIT || gyroY < -GYRO_LIMIT);

  bool isFlameDetected = (flameValue < FLAME_LIMIT);

  return isGyroCritical || isFlameDetected;
}

long getFilteredDistance() {
  long a = getDistance();
  delay(10);

  long b = getDistance();
  delay(10);

  long c = getDistance();

  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;

  return c;
}

long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 6000);

  if (duration == 0) return 999;

  return duration * 0.034 / 2;
}

int clampSpeed(int speedValue) {
  if (speedValue < 0) return 0;
  if (speedValue > 255) return 255;
  return speedValue;
}

void moveForward(int speedL, int speedR) {
  analogWrite(ENA, clampSpeed(speedL));
  analogWrite(ENB, clampSpeed(speedR));

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveBackward(int speedL, int speedR) {
  analogWrite(ENA, clampSpeed(speedL));
  analogWrite(ENB, clampSpeed(speedR));

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft(int speedL, int speedR) {
  analogWrite(ENA, clampSpeed(speedL));
  analogWrite(ENB, clampSpeed(speedR));

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight(int speedL, int speedR) {
  analogWrite(ENA, clampSpeed(speedL));
  analogWrite(ENB, clampSpeed(speedR));

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}