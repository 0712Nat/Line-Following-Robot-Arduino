#include <LiquidCrystal.h>
#include<Wire.h>

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, 4, 5, 6, 7); // Initialize the LCD with your pin configuration

// Motor control pins
int RIGHT_SENSOR = 12;
int LEFT_SENSOR = 13;

int ENA = 11;
int IN1 = A0; // RIGHT BACKWARD
int IN2 = A1; // RIGHT FORWARD

int ENB = 3;
int IN3 = A2; // LEFT BACKWARD
int IN4 = A3; // LEFT FORWARD

// Ultrasonic sensor
const int trigPin = 2;
const int echoPin = 10;
const int buzzer = 12;
double duration;
double distance;
char t;

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
  return(distance);
}

void stop(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Jeff stop");
}

void setup()
{
  //Motor
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(buzzer, OUTPUT);

  Serial.begin(9600);
  
  lcd.begin(16, 2); // Initialize a 16x2 LCD
  lcd.setCursor(0, 0);
  lcd.print("Hello I am Jeff");
}

void loop() {
  distance = readUltrasonic();
  
  if(Serial.available()){
    t = Serial.read();
    Serial.println(t);
  }

  if(t == 'F'){            //move  forward(all motors rotate in forward direction)
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Jeff foward");
  }
 
  else if(t == 'B'){      //move reverse (all  motors rotate in reverse direction)
    if (distance>=20) {
      analogWrite(ENA, 150);
      analogWrite(ENB, 150);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Jeff backward");
    }
    else{
      stop();
      tone(buzzer, 2500);
      delay(500);
      tone (buzzer, 2500); 
      noTone (buzzer);
    }


  }
  
  else if(t == 'L'){      //turn right (left side motors rotate in forward direction,  right side motors doesn't rotate)
    analogWrite(ENA, 220);
    analogWrite(ENB, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Jeff left");
  }
 
    else  if(t == 'R'){      //turn left (right side motors rotate in forward direction, left  side motors doesn't rotate)
    analogWrite(ENA, 100);
    analogWrite(ENB, 220);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Jeff right");
  }

  else  if(t == 'G'){      //Foward left 
    analogWrite(ENA, 255);
    analogWrite(ENB, 85);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  else  if(t == 'I'){      //Foward right 
    analogWrite(ENA, 85);
    analogWrite(ENB, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  else  if(t == 'H'){      //Backward left 
    analogWrite(ENA, 255);
    analogWrite(ENB, 85);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  else  if(t == 'J'){      //Backward right
    analogWrite(ENA, 85);
    analogWrite(ENB, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  else  if(t == 'V'){      //Horn on
    tone(buzzer, 2500);
    delay(500);
  }

  else  if(t == 'v'){      //Horn off
    noTone (buzzer);
  }

  else if(t == 'S'){      //STOP (all motors stop)
    stop();
  }
  //delay(100);
}