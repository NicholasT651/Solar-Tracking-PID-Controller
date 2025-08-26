
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

// ------------------ YOUR ORIGINAL VARS ------------------
double dt, last_time;
double intergral, previous, output = 0;
double kp, ki, kd;
// double setpoint = 75.00;

int LDR1 = A0;  // photolight resistor 1 (input pin)
int LDR2 = A1;  // photolight resistor 2 (input pin)
int dirPin = 7; // motor DIR
int pwmPin = 6; // motor PWM (use 0..255)

// ------------------ NEW: ENCODER + SD ------------------
Adafruit_AS5600 as5600;     // AS5600 over I2C
File logFile;               // SD logfile
const char* LOGNAME = "angle_log.csv";

// optional: simple throttling for SD flush
unsigned long last_log_flush = 0;
const unsigned long flush_interval_ms = 500;

// ------------------ SETUP ------------------
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

  // --- I2C + AS5600 ---
  Wire.begin();           // Teensy 4.1 default SDA=18, SCL=19
  as5600.begin();         // uses default I2C addr 0x36

  // --- SD init (built-in microSD on Teensy 4.1) ---
  // If you're using an external SPI SD module, replace BUILTIN_SDCARD with your CS pin (e.g., 10)
  if (!SD.begin(BUILTIN_SDCARD)) {
    // fallback example (uncomment if using external SD on pin 10):
    // if (!SD.begin(10)) {
    //   while (1) { Serial.println("SD init failed"); delay(1000); }
    // }
    while (1) { Serial.println("SD init failed"); delay(1000); }
  }

  // open (append) log file
  logFile = SD.open(LOGNAME, FILE_WRITE);
  if (!logFile) {
    while (1) { Serial.println("Could not open log file"); delay(1000); }
  }

  // write header if file was just created (size == 0)
  if (logFile.size() == 0) {
    logFile.println("millis,angle_deg,error,duty");
    logFile.flush();
  }

  // (kept from your original sketch)
  // for(int i = 0; i < 50; i++);
  {
    // Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
}

// ------------------ LOOP ------------------
void loop() {
  double now = millis();
  dt = max((now - last_time) / 1000.0, 0.001);
  last_time = now;

  int R1 = analogRead(LDR1);
  int R2 = analogRead(LDR2);
  double error = R1 - R2;  // signed difference

  double output = pid(error);  // get PID output

  // Set motor direction
  if (output >= 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Set motor speed using PID result (0..255)
  int duty = constrain((int)abs(output), 0, 255);
  analogWrite(pwmPin, duty);

  // Read AS5600 angle in degrees (0..360)
  // Adafruit_AS5600::getAngle() returns float degrees
  float angle_deg = as5600.getAngle();

  // Serial monitor (kept simple like your original)
  Serial.println(error);

  // Log to SD: millis, angle_deg, error, duty
  if (logFile) {
    logFile.print((unsigned long)now);
    logFile.print(',');
    logFile.print(angle_deg, 3);
    logFile.print(',');
    logFile.print(error, 3);
    logFile.print(',');
    logFile.println(duty);
  }

  // occasionally flush to ensure data is written
  if (millis() - last_log_flush >= flush_interval_ms) {
    last_log_flush = millis();
    if (logFile) logFile.flush();
  }

  delay(10);
}

// ------------------ PID FUNCTION ------------------
double pid(double error) // pid error calculation
{
  double proportional = error;                // kp term
  intergral += error * dt;                    // ki term
  double derivative = (error - previous) / dt; // kd term
  previous = error;

  double output = (kp * proportional) + (ki * intergral) + (kd * derivative);
  return output;
}
