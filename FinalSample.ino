//Team- Robin Pratap Singh & Jose Alex Colon
// Define Port D Register Pointers
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29; 

// Define Port H Register Pointers
volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h  = (unsigned char*) 0x101; 
volatile unsigned char* pin_h  = (unsigned char*) 0x100; 

// Define Port B Register Pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

// Define Port F Register Pointers
volatile unsigned char* port_f = (unsigned char*) 0x31; 
volatile unsigned char* ddr_f  = (unsigned char*) 0x30; 
volatile unsigned char* pin_f  = (unsigned char*) 0x2F; 

// Define Port L Register Pointers
volatile unsigned char* port_l = (unsigned char*) 0x10B; 
volatile unsigned char* ddr_l  = (unsigned char*) 0x10A; 
volatile unsigned char* pin_l  = (unsigned char*) 0x109; 

//Delay
unsigned char *myTCCR1A = (unsigned char *) 0x80;
unsigned char *myTCCR1B = (unsigned char *) 0x81;
unsigned char *myTCCR1C = (unsigned char *) 0x82;
unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
unsigned int *myTCNT1 = (unsigned int *) 0x84;
unsigned char *myTIFR1 = (unsigned char *) 0x36;

#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

#include <LiquidCrystal.h>   //LCD

const int rs = 12, en = 13, d4 = 11, d5 = 10, d6 = 50, d7 = 51;  //LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                     //LCD


#include <DHT11.h>  //Temp & Humidity

DHT11 dht11(42);

#include <Stepper.h>  //Stepper
const int stepsPerRevolution = 500;  //Stepper
Stepper myStepper = Stepper(stepsPerRevolution, 49, 48, 47, 46);   //Stepper

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
  U0init(9600);
  adc_init();

  *ddr_b |= 0x01 << 0;   //DC MOTOR   PB0
  *ddr_b |= 0x01 << 1;   //DC MOTOR.  PB1

  lcd.begin(16, 2);  //LCD
 
  clock.begin();                       //RTC
  clock.setDateTime(2023, 11, 28, 10, 45, 00);  //RTC


  *ddr_h |= 0x01 << 4;   //Water Level PH4 

  *port_h |= (0x01 << 4); //Water Level. PH4

  *ddr_l |= 0x01 << 6;   //LED Red

  *ddr_l |= 0x01 << 5;   //LED Green

  *ddr_l |= 0x01 << 4;   //LED Blue

  *ddr_d |= 0x01 << 7;   //LED Yellow

  *ddr_h &= ~(0x01 << 3);     //Reset PH3
  *port_h |= (0x01 << 3);     //Reset PH3


  *ddr_d &= ~(0x01 << 2);     //ISR. Start PD2
  *port_d |= (0x01 << 2);     //ISR. Start PD2
  attachInterrupt(digitalPinToInterrupt(interruptPin), start, RISING);    //ISR


  *ddr_d &= ~(0x01 << 3);     //ISR. Stop PD3
  *port_d |= (0x01 << 3);     //ISR. Stop PD3
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
      *port_l &= ~(0x01 << 6);   //LED Red
      *port_l &= ~(0x01 << 5);   //LED Green
      *port_l |= (0x01 << 4);   //LED Blue
      *port_d &= ~(0x01 << 7);   //LED Yellow

      combinedMotors(1, 10);
    }
    else if(state == HIGH){
      realTimeClock("Idle");
      *port_l &= ~(0x01 << 6);   //LED Red
      *port_l |= (0x01 << 5);   //LED Green
      *port_l &= ~(0x01 << 4);   //LED Blue
      *port_d &= ~(0x01 << 7);   //LED Yellow
      combinedMotors(0, 0);
    }
  }
  else{
    realTimeClock("Disabled");
    *port_l &= ~(0x01 << 6);   //LED Red
    *port_l &= ~(0x01 << 5);   //LED Green
    *port_l &= ~(0x01 << 4);   //LED Blue
    *port_d |= (0x01 << 7);   //LED Yellow

  }
}

void combinedMotors(int enable, int stepSpeed){
  dcMotor(enable);
  if(enable == 1){
    stepper(stepSpeed);
  }
  
}
void dcMotor(int enable){
  if(enable == 1){
    *port_b |= (0x01 << 1);
  }
  else{
    *port_b &= ~(0x01 << 1);
  }
  *port_b &= ~(0x01 << 0);
}

//RTC function
void realTimeClock(String s){
  dt = clock.getDateTime();
  String str;
  str += dt.year; str += "-"; 
  str += dt.month; str += "-"; 
  str += dt.day; str += " "; 
  str += dt.hour; str += ":"; 
  str += dt.minute; str += ":"; 
  str += dt.second; str += "  :"; 
  str += s; 
  for(int i = 0; i < str.length(); i++){
    U0putchar(str[i]);
  }
  U0putchar('\n');
  my_delay(1024);
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
  *port_h |= (0x01 << 4); // turn the sensor ON
  my_delay(256); 
  value = adc_read(5); // read the analog value from sensor
  *port_h |= (0x01 << 4); // turn the sensor OFF
  my_delay(256);
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
  *port_l &= ~(0x01 << 6);   //LED Red
  *port_l &= ~(0x01 << 5);   //LED Green
  *port_l &= ~(0x01 << 4);   //LED Blue
  *port_d |= (0x01 << 7);   //LED Yellow
}

void error(){
  resetVal = HIGH;
  while(resetVal == HIGH){
    if(!(*pin_p & 0x08)){
      *port_l |= (0x01 << 6);   //LED Red
      *port_l |= (0x01 << 5);   //LED Green
      *port_l |= (0x01 << 4);   //LED Blue
      *port_d |= (0x01 << 7);   //LED Yellow
      resetVal = LOW;
    }
    else{
      *port_l |= (0x01 << 6);   //LED Red
      *port_l &= ~(0x01 << 5);   //LED Green
      *port_l &= ~(0x01 << 4);   //LED Blue
      *port_d &= ~(0x01 << 7);   //LED Yellow
    }
    
  }
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

//Serial.begin()
void U0init(int U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

//Serial.available()
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

//Serial.read()
unsigned char U0getchar()
{
  return *myUDR0;
}

//Serial.write()
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void my_delay(unsigned int ticks){
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);

  *myTCCR1B |= 0x01;
  while((*myTIFR1 & 0x01)==0);

  *myTCCR1B &= 0xF8;
  *myTIFR1 |= 0x01;
}

