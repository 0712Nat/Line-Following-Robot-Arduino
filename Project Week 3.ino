#include <LiquidCrystal.h>
#include<Wire.h>

int rightSensorValue;
int leftSensorValue; 

int b=1;
int once=1;
int end = 1;

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, 4, 5, 6, 7); // Initialize the LCD with your pin configuration

// MPU-6050
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;

// Motor control pins
int RIGHT_SENSOR = 12;
int LEFT_SENSOR = 13;

int ENA = 11;
int IN1 = A0; // RIGHT BACKWARD
int IN2 = A1; // RIGHT FORWARD

int ENB = 3;
int IN3 = A2; // LEFT BACKWARD
int IN4 = A3; // LEFT FORWARD

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
float distanceForward = 0;
float distanceForward1 = 0;
float distanceForward2 = 0;
float distanceMoveUpRamp = 0;
float distanceAccelerate = 0;
float distanceTask2 = 0;
float totalDistance = 0;

// Accelerate at the beginning
unsigned long startTime;
unsigned long accelerationDuration = 500; // 1000 milliseconds = 1 second

void setup()
{
  //MPU
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  startTime = millis();

    //Motor
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Encoder
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  lastStateA=digitalRead(encoderA);
  lastStateB=digitalRead(encoderB);

  lcd.begin(16, 2); // Initialize a 16x2 LCD
}

float moveForward(int once) 
{ 
  analogWrite(ENA, 65);
  analogWrite(ENB, 65);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if (once==0) //Already pass the ramp
  {
    stateA=digitalRead(encoderA);
    if (stateA!=lastStateA)
    {
      encoderCountA++;
    }
    stateA=lastStateA;
  
    distanceForward1=circumference*(float)(encoderCountA)/40.0;

    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB++;
    }
    stateB=lastStateB;
  
    distanceForward2=circumference*(float)(encoderCountB)/40.0;
    
    distanceForward= (distanceFoward1+distanceFoward2)/2;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance travelled: ");
    lcd.setCursor(0,1);
    lcd.print(distanceForward);

    if(readMPU()<10.0)
    {
      if (distanceForward >= 59.5 && distanceForward <= 60.5)
      {
        unsigned long startTask2 = millis();
        while (millis() - startTask2 <= 5000) 
        {
          stopMoving();
        }
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Distance travelled: ");
        lcd.setCursor(0,1);
        lcd.print(distanceForward);
      }
    }
  }
  
  else
  {
    stateA=digitalRead(encoderA);
    if (stateA!=lastStateA)
    {
      encoderCountA++;
    }
    stateA=lastStateA;
    distanceForward1=circumference*(float)(encoderCountA)/40.0;

    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB++;
    }
    stateB=lastStateB;
    distanceForward2=circumference*(float)(encoderCountB)/40.0;
    
    distanceForward= (distanceFoward1+distanceFoward2)/2;
    
    return (distanceForward);
  }
}

float moveUpRamp(){
  unsigned long hillStart = millis();
  while (millis() - hillStart <= 1300) {
    analogWrite(ENA, 255);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    double angleX=readMPU();
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Ramp angle: ");
    lcd.print(angleX);

    stateA=digitalRead(encoderA);
    if (stateA!=lastStateA)
    {
      encoderCountA++;
    }
    stateA=lastStateA;
    distanceForward1=circumference*(float)(encoderCountA)/40.0;

    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB++;
    }
    stateB=lastStateB;
    distanceForward2=circumference*(float)(encoderCountB)/40.0;
    
    distanceMoveUpRamp= (distanceFoward1+distanceFoward2)/2;
  }
  return (distanceMoveUpRamp);
}

void turnRight() 
{
  analogWrite(ENA, 100);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() 
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 100);
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

void rotate()
{
  unsigned long rotateStart = millis();
  while (millis()-rotateStart<=3200)
  {
    analogWrite(ENA, 200);
    analogWrite(ENB, 50);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  unsigned long rotateStop = millis();
  while (millis() - rotateStop <= 1000) 
  {
    stopMoving();
  }
}

double readMPU()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);

  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  int xAng = map(AcX,minVal,maxVal,-90,90); 
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);  
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  Serial.print("AngleX= ");
  Serial.println(x);

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Angle x: ");
  lcd.print(x);

  return (x);
}

void stop4s()
{
  unsigned long stopStart = millis();
  while (millis()-stopStart<=4000)
  {
    stopMoving();
  }

}

void loop()
{
  // Check if the acceleration phase is still ongoing
  if (millis()-startTime < 500)
  {
    // Accelerate the robot
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
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
    distanceForward1=circumference*(float)(encoderCountA)/40.0;

    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB++;
    }
    stateB=lastStateB;
    distanceForward2=circumference*(float)(encoderCountB)/40.0;
    
    distanceAccelerate= (distanceFoward1+distanceFoward2)/2;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance: ");
    lcd.print(distanceAccelerate);
  }
  
  else
  {
    while (b==1)
    {    
      totalDistance = distanceForward + distanceMoveUpRamp + distanceAccelerate;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Distance: ");
      lcd.print(totalDistance);
      
      rightSensorValue = digitalRead(RIGHT_SENSOR);
      leftSensorValue = digitalRead(LEFT_SENSOR); 
      
      double x1=readMPU();

      if (rightSensorValue == LOW && leftSensorValue == LOW) 
      {
        if ((x1>13.0)&&(x1<25.0)&&(once==1))
        {
          lcd.clear();
          lcd.print("Speed up");

          distanceMoveUpRamp=moveUpRamp();
          
          stop4s(); //stop for 4s
          rotate(); 

          unsigned long start1=millis();
          while ((millis()-start1)<500)
          {
            double x1=readMPU();
            analogWrite(ENA, 200);
            analogWrite(ENB, 200);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
          }
          once=0;
        }
        
        else
        {
          distanceForward=moveForward(once);
        }           
      } 
      
      else if (rightSensorValue == LOW && leftSensorValue == HIGH) 
      {
        turnLeft();
      } 
     
      else if (rightSensorValue == HIGH && leftSensorValue == LOW)           
      {
        turnRight();
      } 
      
      else 
      {
        double x1=readMPU();

        if (end==1) //reaches the end of the top of the ramp
        {
          unsigned long start=millis();
          while (millis()-start<300)
          {
            double x1=readMPU();
            analogWrite(ENA, 200);
            analogWrite(ENB, 200);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            end=0;
          }
          distanceForward=moveForward(once); //slowing down when going down the ramp
        }

        else
        {
          stopMoving();
          b=0;
        }
        
      }
    }
  }

  while(b==0)
  {
    totalDistance = distanceForward + distanceMoveUpRamp + distanceAccelerate;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance: ");
    lcd.print(totalDistance);
    lcd.print("cm");   
  } 
}
