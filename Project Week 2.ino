#include <LiquidCrystal.h>

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, 4, 5, 6, 7); // Initialize the LCD with your pin configuration

// Motor control pins
int ENA = 11;
int IN1 = A0; // RIGHT BACKWARD
int IN2 = A1; // RIGHT FORWARD

int ENB = 3;
int IN3 = A2; // LEFT BACKWARD
int IN4 = A3; // LEFT FORWARD

int RIGHT_SENSOR = 12;
int LEFT_SENSOR = 13;

// Encoder sensor pins
int encoderA = 10;
int encoderB = 2;
int stateA;
int lastStateA;
int stateB;
int lastStateB;
volatile unsigned long encoderCountA = 0;
volatile unsigned long encoderCountB = 0;
volatile unsigned long totalEncoderCount = 0;

volatile unsigned long encoderCountA1= 0;

float circumference = 21.2; //cm

// Accelerate at the beginning
unsigned long startTime;
unsigned long duration = 500; // 1000 milliseconds = 1 second

//Distance measurement
float distance = 0;
float distance1 = 0;
float distanceForward = 0;
float distanceForwardA = 0;
float distanceForwardB = 0;

void setup() {
  Serial.begin(9600);

  startTime = millis();

  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  lastStateA=digitalRead(encoderA);
  lastStateB=digitalRead(encoderB);

  lcd.begin(16, 2); // Initialize a 16x2 LCD
  for(int i= 0; i<= 2; i++)
  { 
    lcd.clear(); 
    lcd.setCursor(i,0); 
    lcd.print("Press SELECT");
    delay(300); 
  } 
  lcd.setCursor(4,1);
  lcd.print("to start"); 
}

float moveForward() 
{ 
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  stateA=digitalRead(encoderA);
  if (stateA!=lastStateA)
  {
    encoderCountA++;
  }
  stateA=lastStateA;
  
  distanceForwardA=circumference*(float)(encoderCountA)/40.0;
  Serial.print("Encoder A: ");
  Serial.println(encoderCountA);

  stateB=digitalRead(encoderB);
  if (stateB!=lastStateB)
  {
    encoderCountB++;
  }
  stateB=lastStateB;
  
  distanceForwardB=circumference*(float)(encoderCountB)/40.0;
  Serial.print("Encoder B: ");
  Serial.println(encoderCountB);
  
  return ((distanceForwardA + distanceFowardB)/2);
}

float turnRight() 
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

float turnLeft() 
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMoving() 
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() 
{
  int x=1;
  unsigned long currentTime = millis();
  unsigned long accelerationElapsedTime = currentTime - startTime;

  //  Check if the acceleration phase is still ongoing
  if (accelerationElapsedTime < duration)
  {
    // Accelerate the robot
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
      
    distance1=circumference*(float)(encoderCountA1)/40.0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.setCursor(0, 1);
    lcd.print(distance1);
    lcd.print("cm");

    Serial.print("acce: ");
    Serial.println(distance1);

    stateA=digitalRead(encoderA);
    if (stateA!=lastStateA)
    {
      encoderCountA1++;
    }
    stateA=lastStateA;
  } 
    
  else
  {
    while (x==1)
    {
      distance=distance1+distanceForward;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Distance: ");
      lcd.setCursor(0, 1);
      lcd.print(distance);
      lcd.print("cm");
  
      // Regular movement logic
      int rightSensorValue = digitalRead(RIGHT_SENSOR);
      int leftSensorValue = digitalRead(LEFT_SENSOR);

      Serial.print("Right Sensor: ");
      Serial.println(rightSensorValue);
      Serial.print(" | Left Sensor: ");
      Serial.println(leftSensorValue);
      
      Serial.print("Encoder A: ");
      Serial.println(encoderCountA);
      
      Serial.print("Encoder B: ");
      Serial.println(encoderCountB);
      
      Serial.print("Encoder count: ");
      Serial.println(totalEncoderCount);
      
      if (rightSensorValue == HIGH && leftSensorValue == HIGH) 
      {
        distanceForward=moveForward();
      } 
      
      else if (rightSensorValue == HIGH && leftSensorValue == LOW) 
      {
        distanceLeft = turnLeft();
      } 
     
      else if (rightSensorValue == LOW && leftSensorValue == HIGH)           
      {
        distanceRight = turnRight();
      } 
      
      else 
      {
        stopMoving();
        x=0;
      }
    }
  }

  while(x==0)
  {
    distance=distance1+distanceForward+distanceRight+distanceLeft;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.setCursor(0, 1);
    lcd.print(distance);
    lcd.print("cm");   
  }   
}
