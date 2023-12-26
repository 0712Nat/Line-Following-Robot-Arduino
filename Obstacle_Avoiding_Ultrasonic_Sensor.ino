#include <LiquidCrystal.h>

//LCD
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, 4, 5, 6, 7);

// Ultrasonic sensor
const int trigPin = 2;
const int echoPin = 10;
double duration;
double distance;

//Motor Control Pins
int ENA = 11;
int IN1 = A0; // RIGHT BACKWARD
int IN2 = A1; // RIGHT FORWARD

int ENB = 3;
int IN3 = A2; // LEFT BACKWARD
int IN4 = A3; // LEFT FORWARD

void setup()
{
  //Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  //Motor
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //LCD
  lcd.begin(16, 2);
}

double readUltrasonic()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.0343 / 2;

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HRSr04: ");
  lcd.print(distance);

  return(distance);
}

void moveForward()
{
  analogWrite(ENA, 75);
  analogWrite(ENB, 75);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMoving()
{
  unsigned long start = millis();
  while (millis()-start<=1000)
  {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

void turnRight()
{
  lcd.setCursor(0, 1);
  lcd.print("turning");
  
  analogWrite(ENA, 255);
  analogWrite(ENB, 80);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void loop()
{
  distance=readUltrasonic();

  if (distance<=20.0)
  {
    turnRight();
  }

  else
  {
    moveForward();
  }
}