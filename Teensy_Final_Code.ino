// Teensy 4.1 — PID code + AS5600 angle logging to SD
// Using Pololu TB6612FNG (item #713)

// ==================== PIN LAYOUT ====================
// Motor Driver (TB6612FNG, Channel A only)
//   VM    -> Motor supply (e.g., 6–12 V battery) [NOT from Teensy 5V]
//   GND   -> Common ground with Teensy
//   VCC   -> Teensy 5V (logic)
//   AO1/AO2 -> Motor leads
//   PWMA  -> Teensy pin 6  (pwmPin)
//   AIN1  -> Teensy pin 7  (dir1Pin)
//   AIN2  -> Teensy pin 8  (dir2Pin)
//   STBY  -> Teensy pin 9  (stbyPin, set HIGH to enable)
//
// LDR
//   LDR1 -> A0 (Teensy pin 14)
//   LDR2 -> A1 (Teensy pin 15)
//
// Encoder (AS5600, I2C0 on Teensy 4.1)
//   SDA -> pin 18
//   SCL -> pin 19
// ====================================================

#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <SD.h>
#include <SPI.h>

double dt, last_time;
double intergral, previous, output = 0;
double kp, ki, kd;

int LDR1 = A0;
int LDR2 = A1;

// TB6612FNG control pins (Channel A)
int dir1Pin = 7;   // AIN1
int dir2Pin = 8;   // AIN2
int stbyPin = 9;   // STBY (HIGH to enable)
int pwmPin  = 6;   // PWMA (PWM)

Adafruit_AS5600 as5600;
File logFile;
const char* LOGNAME = "angle_log.csv";

unsigned long last_log_flush = 0;
const unsigned long flush_interval_ms = 500;

// throttle logging/serial/encoder to avoid blocking PID
unsigned long last_log_ms    = 0;
unsigned long last_serial_ms = 0;
unsigned long last_enc_ms    = 0;
const unsigned long LOG_PERIOD_MS     = 1000;
const unsigned long SERIAL_PERIOD_MS  = 100;
const unsigned long ENCODER_PERIOD_MS = 20;

float angle_deg_cached = 0.0f;

void setup() {
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);

  pinMode(dir1Pin, OUTPUT);
  pinMode(dir2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);
  pinMode(pwmPin,  OUTPUT);

  // Enable TB6612FNG
  digitalWrite(stbyPin, HIGH);

  kp = 0.6;
  ki = 0.0;
  kd = 0.0;
  last_time = 0;

  Serial.begin(9600);

  Wire.begin();
  as5600.begin();

  if (!SD.begin(BUILTIN_SDCARD)) {
    while (1) { Serial.println("SD init failed"); delay(1000); }
  }

  logFile = SD.open(LOGNAME, FILE_WRITE);
  if (!logFile) {
    while (1) { Serial.println("Could not open log file"); delay(1000); }
  }
  if (logFile.size() == 0) {
    logFile.println("millis,angle_deg,error,duty");
    logFile.flush();
  }
}

void loop() {
  unsigned long now_ms = millis();
  dt = max((now_ms - last_time) / 1000.0, 0.001);
  last_time = now_ms;

  int R1 = analogRead(LDR1);
  int R2 = analogRead(LDR2);
  double error = R1 - (R2 + 2);

  // PID
  double outPID = pid(error);

  // TB6612FNG direction (AIN1 / AIN2)
  if (outPID >= 0) {
    digitalWrite(dir1Pin, HIGH); // forward
    digitalWrite(dir2Pin, LOW);
  } else {
    digitalWrite(dir1Pin, LOW);  // reverse
    digitalWrite(dir2Pin, HIGH);
  }

  // Speed (0..255)
  int duty = constrain((int)abs(outPID), 0, 255);
  analogWrite(pwmPin, duty);

  // Throttled encoder read
  if (now_ms - last_enc_ms >= ENCODER_PERIOD_MS) {
    last_enc_ms = now_ms;
    angle_deg_cached = as5600.getAngle();  // 0..360
  }

  // Throttled Serial print
  if (now_ms - last_serial_ms >= SERIAL_PERIOD_MS) {
    last_serial_ms = now_ms;
    Serial.println(error);
  }

  // Throttled SD logging
  if (now_ms - last_log_ms >= LOG_PERIOD_MS) {
    last_log_ms = now_ms;
    if (logFile) {
      logFile.print(now_ms);
      logFile.print(',');
      logFile.print(angle_deg_cached, 3);
      logFile.print(',');
      logFile.print(error, 3);
      logFile.print(',');
      logFile.println(duty);
    }
  }

  // Occasional flush
  if (now_ms - last_log_flush >= flush_interval_ms) {
    last_log_flush = now_ms;
    if (logFile) logFile.flush();
  }
}

double pid(double error) {
  double proportional = error;
  intergral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;

  double out = (kp * proportional) + (ki * intergral) + (kd * derivative);
  return out;
}
