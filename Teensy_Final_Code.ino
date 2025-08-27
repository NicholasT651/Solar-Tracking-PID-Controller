// Motor Driver
// PWM  Pin 6
// DIR  Pin 7

// LDR 
// A0   Pin 14
// A1   Pin 15

// Encoder (I2C0 on Teensy 4.1)
// SDA  Pin 18
// SCL  Pin 19

// Teensy 4.1 — your PID code + AS5600 angle logging to SD

#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <SD.h>
#include <SPI.h>

double dt, last_time;
double intergral, previous, output = 0;
double kp, ki, kd;

int LDR1 = A0;
int LDR2 = A1;
int dirPin = 7;
int pwmPin = 6;

Adafruit_AS5600 as5600;
File logFile;
const char* LOGNAME = "angle_log.csv";

unsigned long last_log_flush = 0;
const unsigned long flush_interval_ms = 500;

// NEW: throttle logging/serial/encoder to avoid blocking PID
unsigned long last_log_ms    = 0;
unsigned long last_serial_ms = 0;
unsigned long last_enc_ms    = 0;
const unsigned long LOG_PERIOD_MS    = 1000;   // 50 Hz logging
const unsigned long SERIAL_PERIOD_MS = 100;  // 10 Hz serial
const unsigned long ENCODER_PERIOD_MS= 20;   // 50 Hz encoder read

// hold latest encoder angle without blocking the loop each time
float angle_deg_cached = 0.0f;

void setup() {
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

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
  double error = R1 - R2;

  // PID
  double outPID = pid(error);

  // Direction
  if (outPID >= 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Speed
  int duty = constrain((int)abs(outPID), 0, 255);
  analogWrite(pwmPin, duty);

  // Throttled encoder read (I2C can block)
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

  // Occasional flush (kept from your code, already throttled)
  if (now_ms - last_log_flush >= flush_interval_ms) {
    last_log_flush = now_ms;
    if (logFile) logFile.flush();
  }

  // Removed delay(10);  <-- this was causing control lag
}

double pid(double error)
{
  double proportional = error;
  intergral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;

  double out = (kp * proportional) + (ki * intergral) + (kd * derivative);
  return out;
}
