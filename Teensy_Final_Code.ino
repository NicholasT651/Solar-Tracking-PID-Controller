#include <Adafruit_AS5600.h>
#include <Wire.h>
#include <SD.h>

// Teensy 4.1 built-in microSD uses SDIO; no external SPI pins/CS required.
// Just call SD.begin(BUILTIN_SDCARD).

// Motor Driver
// PWM  Pin 6
// DIR  Pin 7

// LDR 
// A0   Pin 14
// A1   Pin 15

// Encoder (I2C0 on Teensy 4.1)
// SDA  Pin 18
// SCL  Pin 19

// ------------------ PID CONFIG ------------------
double dt, last_time;
double intergral = 0, previous = 0, output = 0;
double kp = 0.5, ki = 0.0, kd = 0.0;

int LDR1 = A0;
int LDR2 = A1;
int dirPin = 7;
int pwmPin = 6;

double current_error = 0;

// ------------------ TIMING ------------------
unsigned long last_pid_time = 0;
unsigned long last_log_time = 0;
const unsigned long pid_interval = 10;     // ms
const unsigned long log_interval = 1000;   // ms

// ------------------ SD CONFIG ------------------
File file;

// ------------------ AS5600 CONFIG ------------------
Adafruit_AS5600 as5600;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // ------------------ Pins ------------------
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  // ------------------ I2C Init ------------------
  Wire.begin(); // Teensy 4.1 I2C0 on pins 18(SDA)/19(SCL)

  // ------------------ SD Init (Teensy 4.1 built-in) ------------------
  Serial.println("Initializing built-in microSD...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Built-in SD init failed! Check card.");
    while (1) { delay(1000); }
  }
  Serial.println("Built-in SD initialized.");
  file = SD.open("log.txt", FILE_WRITE);
  if (file) {
    file.println("Time(ms),Error,Angle(deg)");
    file.close();
  } else {
    Serial.println("Failed to create log.txt");
  }

  // ------------------ AS5600 Init ------------------
  Serial.println("Initializing AS5600...");
  if (!as5600.begin()) {
    Serial.println("Could not find AS5600 sensor, check wiring!");
    while (1) { delay(1000); }
  }
  Serial.println("AS5600 found!");

  as5600.enableWatchdog(false);
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
  as5600.setZPosition(0);
  as5600.setMPosition(4095);
  as5600.setMaxAngle(4095);

  Serial.println("Setup complete. Waiting for magnet detection...");
  last_time = millis();
}

// ------------------ PID FUNCTION ------------------
double pid(double error) {
  double proportional = error;
  intergral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  return (kp * proportional) + (ki * intergral) + (kd * derivative);
}

// ------------------ MAIN LOOP ------------------
void loop() {
  unsigned long now = millis();

  // --- PID Control ---
  if (now - last_pid_time >= pid_interval) {
    dt = max((now - last_time) / 1000.0, 0.001);  // seconds
    last_time = now;
    last_pid_time = now;

    int R1 = analogRead(LDR1);
    int R2 = analogRead(LDR2);
    current_error = R1 - R2;

    output = pid(current_error);

    digitalWrite(dirPin, output >= 0 ? HIGH : LOW);
    analogWrite(pwmPin, constrain((int)abs(output), 0, 255));

    Serial.print("Error: ");
    Serial.print(current_error);
    Serial.print(" | ");
  }

  // --- AS5600 Angle Reading ---
  float angleDeg = 0.0f;
  if (as5600.isMagnetDetected()) {
    uint16_t angle = as5600.getAngle();
    angleDeg = angle * 360.0f / 4096.0f;

    Serial.print("Angle: ");
    Serial.print(angleDeg, 2);
    Serial.println(" deg");
  } else {
    Serial.println("Magnet not detected.");
  }

  // --- Logging ---
  if (now - last_log_time >= log_interval) {
    last_log_time = now;
    file = SD.open("log.txt", FILE_WRITE);
    if (file) {
      file.print(now);
      file.print(",");
      file.print(current_error);
      file.print(",");
      file.println(angleDeg, 2);
      file.close();
    } else {
      Serial.println("Failed to open log.txt");
    }
  }
}
