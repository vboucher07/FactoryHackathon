#include <Arduino.h>
#include <math.h>

#define MOTOR_A_1A_PIN 2
#define MOTOR_A_1B_PIN 3
#define MOTOR_B_1A_PIN 4
#define MOTOR_B_1B_PIN 5
#define WHEEL_RADIUS 0.335 # meters
#define INTRA_WHEEL_DIST 0.11 # meters

#define COLOR_SENSOR1_ANALOG_PIN A1
#define COLOR_SENSOR1_DIGITAL_PIN 3
#define COLOR_SENSOR2_ANALOG_PIN A2
#define COLOR_SENSOR2_DIGITAL_PIN 5
#define DISTANCE_TRIG_PIN 10
#define DISTANCE_ECHO_PIN 9
#define LINE_THRESHOLD 60

int Line1_Color = 0;
int Line2_Color = 0;
boolean Line1_OnLine = false;
boolean Line2_OnLine = false;
boolean onLine = false;
float distance = 0;
long duration = 0;

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_A_1A_PIN, OUTPUT);
  pinMode(MOTOR_A_1B_PIN, OUTPUT);
  pinMode(MOTOR_B_1A_PIN, OUTPUT);
  pinMode(MOTOR_B_1B_PIN, OUTPUT);

  pinMode(COLOR_SENSOR1_ANALOG_PIN, INPUT);
  pinMode(COLOR_SENSOR1_DIGITAL_PIN, INPUT);
  pinMode(COLOR_SENSOR2_ANALOG_PIN, INPUT);
  pinMode(COLOR_SENSOR2_DIGITAL_PIN, INPUT);

  Serial.println("Setup Done");
}

void loop() {
  isOnLine();
}

boolean isOnLine(){
  Line1_Color = analogRead(COLOR_SENSOR1_ANALOG_PIN);
  Line2_Color = analogRead(COLOR_SENSOR1_ANALOG_PIN);
  Line1_OnLine = (Line1_Color < LINE_THRESHOLD) ? false : true;
  Line2_OnLine = (Line2_Color < LINE_THRESHOLD) ? false : true;
  return false; //TODO
}

float getDistance(){
  digitalWrite(DISTANCE_ECHO_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(DISTANCE_ECHO_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_ECHO_PIN, LOW);
}

void motorControl(float vel_linear, float vel_angular){
  float wheelVelA = vel_linear;  
  float wheelVelB = vel_linear;

  wheelVelA += M_PI*INTRA_WHEEL_DIST/2*-vel_angular/2;
  wheelVelB += M_PI*INTRA_WHEEL_DIST/2*vel_angular/2;

  pwmA = velToPWM(wheelVelA);
  pwmB = velToPWM(wheelVelB);

  if(pwmA >= 0)
    # forwards
    analogWrite(MOTOR_A_1A_PIN, pwmA);
    digitalWrite(MOTOR_A_1B_PIN, LOW);
  else
    # backwards
    analogWrite(MOTOR_A_1A_PIN, LOW);
    analogWrite(MOTOR_A_1B_PIN, -pwmA);

  if(pwmB >= 0)
    # forwards
    analogWrite(MOTOR_B_1A_PIN, pwmA);
    digitalWrite(MOTOR_B_1B_PIN, LOW);
  else
    # backwards
    analogWrite(MOTOR_B_1A_PIN, LOW);
    analogWrite(MOTOR_B_1B_PIN, -pwmA);
}


int velToPWM(float vel){
  return floor(vel*some_const);
}
