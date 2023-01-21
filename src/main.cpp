#include <Arduino.h>
#include <HCSR04.h>
#include <TimerOne.h>

#define COLOR_SENSOR1_ANALOG_PIN A1
#define COLOR_SENSOR1_DIGITAL_PIN 3
#define COLOR_SENSOR2_ANALOG_PIN A2
#define COLOR_SENSOR2_DIGITAL_PIN 5
#define DISTANCE_TRIG_PIN 6
#define DISTANCE_ECHO_PIN 5
#define LINE_THRESHOLD 60
int Line1_Color = 0;
int Line2_Color = 0;
boolean Line1_OnLine = false;
boolean Line2_OnLine = false;
boolean onLine = false;
float distance = 0;
volatile int timerFlag = 0;
UltraSonicDistanceSensor distanceSensor(DISTANCE_TRIG_PIN, DISTANCE_ECHO_PIN);

boolean isOnLine();
float getDistance();
void timerISR();

void setup() {
  Serial.begin(9600);
  Timer1.initialize(50000);
  Timer1.attachInterrupt(timerISR);

  pinMode(COLOR_SENSOR1_ANALOG_PIN, INPUT);
  pinMode(COLOR_SENSOR1_DIGITAL_PIN, INPUT);
  pinMode(COLOR_SENSOR2_ANALOG_PIN, INPUT);
  pinMode(COLOR_SENSOR2_DIGITAL_PIN, INPUT);

  Serial.println("Setup Done");
}

void loop() {
  isOnLine();
  if(timerFlag){
    distance = distanceSensor.measureDistanceCm();
    timerFlag = 0;
  }
  Serial.print("OnLine: ");
  Serial.print(Line1_OnLine);
  Serial.print(" || Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

boolean isOnLine(){
  Line1_Color = analogRead(COLOR_SENSOR1_ANALOG_PIN);
  Line2_Color = analogRead(COLOR_SENSOR1_ANALOG_PIN);
  Line1_OnLine = (Line1_Color < LINE_THRESHOLD) ? false : true;
  Line2_OnLine = (Line2_Color < LINE_THRESHOLD) ? false : true;
  return false; //TODO
}

void timerISR(){
  timerFlag = 1;
}