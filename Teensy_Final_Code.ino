//#include <Adafruit_AS5600.h>
//#include <Wire.h>

// Motor Driver
// PWM  Pin 6
// DIR  Pin 7

// LDR 
// A0   Pin 14
// A1   Pin 15

// Encoder (I2C0 on Teensy 4.1)
// SDA  Pin 18
// SCL  Pin 19

// Teensy 4.1 — direct port of your Arduino Uno code (no extra features)

double dt, last_time;
double intergral, previous, output = 0;
double kp, ki, kd;
// double setpoint = 75.00;

int LDR1 = A0;  // photolight resistor 1 (input pin)
int LDR2 = A1;  // photolight resistor 2 (input pin)
int dirPin = 7; // motor DIR
int pwmPin = 6; // motor PWM (use 0..255)

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

  // for(int i = 0; i < 50; i++);
  {
    // Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
}

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
  analogWrite(pwmPin, constrain(abs(output), 0, 255));

  Serial.println(error);

  delay(10);
}

double pid(double error) // pid error calculation
{
  double proportional = error;              // kp term
  intergral += error * dt;                  // ki term
  double derivative = (error - previous) / dt; // kd term
  previous = error;

  double output = (kp * proportional) + (ki * intergral) + (kd * derivative);
  return output;
}
