//Team - Robin Singh & Jose Alex Colon
#include <LiquidCrystal.h>   //LCD

const int rs = 12, en = 13, d4 = 11, d5 = 10, d6 = 50, d7 = 51;  //LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                     //LCD


#include <DHT11.h>  //Temp & Humidity

DHT11 dht11(42);

//int speedPin = 5; //DC Motor
int dir1 = 52;    //DC Motor
int dir2 = 53;    //DC Motor

#include <Stepper.h>  //Stepper
const int stepsPerRevolution = 2038;  //Stepper
Stepper myStepper = Stepper(stepsPerRevolution, 49,48,47,46);   //Stepper

#include <Wire.h>   //RTC
#include <DS3231.h> //RTC
DS3231 clock;       //RTC
RTCDateTime dt;     //RTC



#define POWER_PIN 7     //Water Level
#define SIGNAL_PIN A5   //Water Level
int value = 0; //Water Level

#define RED 43  //LED
#define GREEN 44   //LED
#define BLUE 45   //LED

const byte interruptPin = 19;   //ISR
volatile byte state = LOW;     //ISR
// bool state = false;      //ISR

const byte interruptStopPin = 18;   //ISR

int resetButton = 6;    //Reset button
volatile byte resetVal = HIGH;      //Reset Button

void setup() {
  Serial.begin(9600);
  
  pinMode(dir1, OUTPUT);      //DC Motor
  pinMode(dir2, OUTPUT);      //DC Motor
  //pinMode(dcEnable, OUTPUT);  //DC Motor
  //Serial.begin(9600);         //DC Motor

  lcd.begin(16, 2);  //LCD
 
  clock.begin();                       //RTC
  clock.setDateTime(2023, 12, 15, 4, 40, 00);  //RTC

  pinMode(POWER_PIN, OUTPUT); //Water Level
  digitalWrite (POWER_PIN, LOW); //Water Level

  pinMode(RED, OUTPUT);   //LED
  pinMode(GREEN, OUTPUT);   //LED
  pinMode(BLUE, OUTPUT);   //LED

  pinMode(resetButton, INPUT_PULLUP);
  
  pinMode(interruptPin, INPUT_PULLUP);   //ISR
  attachInterrupt(digitalPinToInterrupt(interruptPin), start, RISING);    //ISR

  pinMode(interruptStopPin, INPUT_PULLUP);   //ISR
  attachInterrupt(digitalPinToInterrupt(interruptStopPin), stop, RISING);    //ISR
}

void loop() {
  
  if(state == HIGH){
    double temperature = temp();
    int waterLev = waterLevel();
    if(waterLev <= 100 && state == HIGH){
      lcd.clear();
      combinedMotors(0, 0);
      temp();
      lcd.setCursor(11, 0);
      lcd.print("Water");
      lcd.setCursor(13, 1);
      lcd.print("Low");
      error();
    }

    if(idle() == true && state == HIGH){
      realTimeClock("Running");
      lcd.clear();
      temp();
      analogWrite(RED,   0);
      analogWrite(GREEN, 0);
      analogWrite(BLUE,  252);
      combinedMotors(1, 10);
    }
    else if(state == HIGH){
      realTimeClock("Idle");
      analogWrite(RED,   0);
      analogWrite(GREEN, 255);
      analogWrite(BLUE,  0);
      combinedMotors(0, 0);
    }
  }
  else{
    realTimeClock("Disabled");
    analogWrite(RED,   255);
    analogWrite(GREEN, 50);
    analogWrite(BLUE,  0);
  }
}

void combinedMotors(int enable, int stepSpeed){
  dcMotor(enable);
  if(enable == 1){
    stepper(stepSpeed);
  }
  
}
void dcMotor(int enable){
  digitalWrite(dir1, enable);
  digitalWrite(dir2, 0);
  //analogWrite(dcEnable, enable);
}

//RTC function
void realTimeClock(String s){
  dt = clock.getDateTime();
  
  Serial.print(dt.year);   Serial.print("-");
  Serial.print(dt.month);  Serial.print("-");
  Serial.print(dt.day);    Serial.print(" ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.print("  :");
  Serial.println(s);
  
  delay(1000);
}

//Stepper function
void stepper(int speed){
  myStepper.setSpeed(5);
  myStepper.step(stepsPerRevolution);

  // step one revolution in the other direction:
  myStepper.step(-stepsPerRevolution);
  
}

//Water Level function
int waterLevel(){
  digitalWrite (POWER_PIN, HIGH); // turn the sensor ON
  delay(10); // wait 10 milliseconds
  value = analogRead (SIGNAL_PIN); // read the analog value from sensor
  digitalWrite (POWER_PIN, LOW); // turn the sensor OFF

  delay(1000);
  return value;
}


//Temp function
int temp(){
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.setCursor(6, 0);
    lcd.print(dht11.readTemperature(), 1);

    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.setCursor(5, 1);
    lcd.print(dht11.readHumidity(), 1);

    lcd.setCursor(7, 1);
    lcd.print("%");
    
  
  return dht11.readTemperature();
}

bool idle(){
  if(temp() >= 15 && waterLevel() > 100){
    return true;
  }
  return false;
}

void start(){
  state = HIGH;
}

void stop(){
  state = LOW;
  combinedMotors(0, 0);
  lcd.clear();
  analogWrite(RED,   255);
  analogWrite(GREEN, 50);
  analogWrite(BLUE,  0);
}

void error(){
  realTimeClock("Error");
  resetVal = HIGH;
  while(resetVal == HIGH){
    if(digitalRead(resetButton) == LOW){
      analogWrite(RED,   255);
      analogWrite(GREEN, 0);
      analogWrite(BLUE,  0);
      resetVal = LOW;
    }
    else{
      analogWrite(RED,   255);
      analogWrite(GREEN, 0);
      analogWrite(BLUE,  0);
    }
    
  }
}
