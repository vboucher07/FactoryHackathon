#include <Arduino.h>

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