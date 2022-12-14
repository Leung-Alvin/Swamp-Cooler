//Physical Circuit Equipment
//LCD 16x2
//DHT11 Sensor
//Water Level Detector
//10K Potentiometer
//10K Ohms Resistor
//Fan Blades & Motor
//PN2222 Transistor
//1N4001 Diode
//270 Ohms Resistor

#include <DHT.h>
#include <LiquidCrystal.h>
#include <DS3231.h>
#include <Wire.h>
#include "Stepper.h"
#include "IRremote.h"
#define RDA 0x80
#define TBE 0x20  

#define WRITE_HIGH_PB(pin_num)  *port_b |= (0x01 << pin_num);
#define WRITE_LOW_PB(pin_num)  *port_b &= ~(0x01 << pin_num);

#define WRITE_HIGH_PH(pin_num)  *port_h |= (0x01 << pin_num);
#define WRITE_LOW_PH(pin_num)  *port_h &= ~(0x01 << pin_num);

#define WRITE_HIGH_PE(pin_num) *port_e |= (0x01 << pin_num);
#define WRITE_LOW_PE(pin_num)  *port_e &= ~(0x01 << pin_num);

#define WRITE_HIGH_PG(pin_num) *port_g |= (0x01 << pin_num);
#define WRITE_LOW_PG(pin_num)  *port_g &= ~(0x01 << pin_num);

#define WRITE_HIGH_PA(pin_num) *port_a |= (0x01 << pin_num);
#define WRITE_LOW_PA(pin_num)  *port_a &= ~(0x01 << pin_num);
#define ALL_LIGHTS_DOWN()      *port_a &= ~(0x55);

volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20;

volatile unsigned char* port_g = (unsigned char*) 0x34; 
volatile unsigned char* ddr_g  = (unsigned char*) 0x33; 
volatile unsigned char* pin_g  = (unsigned char*) 0x32; 

volatile unsigned char* port_e = (unsigned char*) 0x2E; 
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D; 
volatile unsigned char* pin_e  = (unsigned char*) 0x2C; 


volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h  = (unsigned char*) 0x101; 
volatile unsigned char* pin_h  = (unsigned char*) 0x100; 

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

LiquidCrystal lcd(53,51,49,47,45,43);
#define DHTTYPE DHT11
#define WLDPIN 0 //Analog pin connected to water detection sensor
#define DHTPIN 52 //Digital pin connected to the DHT sensor
#define FANPIN 2
#define BLUELEDPIN 22
#define REDLEDPIN 24
#define GREENLEDPIN 26
#define YELLOWLEDPIN 28
#define BLUELED 0
#define REDLED 2
#define GREENLED 4
#define YELLOWLED 6

int curState = 0; // 0 - Disabled, 1 - Idle, 2 - Running, 3 - Error

DHT dht(DHTPIN, DHTTYPE);
DS3231 clock;
RTCDateTime dt;

//IRReceiver and Stepper
#define STEPS 32   // Number of steps per revolution of Internal shaft
int stepSize;  // 2048 = 1 Revolution
Stepper small_stepper(STEPS, 8, 10, 9, 11);
int irPort = 12;
IRrecv irrecv(irPort);  //12 -> PB_6

//PB7 = 13 0x80
//PH6 = 9 0010 0000 -> 0x20

int water_level_threshold = 100; //If below then water level is too low
float temp_threshold = 100; // High Fahrenheit
int curWat; //current water level
unsigned long previousMillis = 0;
void setup(){
  U0init(9600); //Serial Begin
  lcd.begin(16,2); //Turn on LCD
  *ddr_b |= 0x80; //Turn on BUILT_IN_LED
  *ddr_e |= 0x08; //Turn on Fan Motor 0000 1000
  *ddr_a |= 0x55; //Turn on DDR A (Lights)
  dht.begin(); //Start DHT
  adc_init(); //Start ADC and Water Level Sampling
  clock.begin();
  irrecv.enableIRIn();
  
}



void loop() 
{ 
  if(irrecv.decode()){
    processIR();
  }


  state_check(curState);
}

void printLogTime(){
  dt = clock.getDateTime();
  char timeStr[22];
  sprintf(timeStr, "%02d:%02d:%02d %02d/%02d/%02d: ",  dt.hour, dt.minute, dt.second, dt.month, dt.day, dt.year);
  U0putstr(timeStr);
}

void state_check(int state){
  if(state == 1 || state == 2){
    float curHum = dht.readHumidity(); //reads humidity
    float curTemp = dht.readTemperature(true); //reads temperature as fahrenheit
    curWat = adc_read(WLDPIN);
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 4000) { //if a minute has passed
      previousMillis = currentMillis;

    }
    lcd.setCursor(0, 0); //First line in LCD Display

    if (isnan(curHum) || isnan(curTemp)) { //Checks if DHT reads wrong input and to try again
      U0putstr("Failed to read from DHT sensor!");
    }
    lcd.print(curTemp);
    lcd.print("F ");
    lcd.print(curHum);
    lcd.print("% ");
    

    if(state == 1){ //idle
      if(curTemp > temp_threshold){
        curState = 2; //set to running
        set_state(curState);
      }
      if(curWat <= water_level_threshold){
        curState = 3; //set to error
        set_state(curState);
      }
    }

    if(state == 2){ //running
      if(curTemp <= temp_threshold){
        curState = 1; //set to idle
        set_state(curState);
      }
      if(curWat < water_level_threshold){
        curState = 3; //set to error
        set_state(curState);
      }
    }
  } 
  if(state == 3){
    curWat = adc_read(WLDPIN);
  }
}

void set_state(int state){
  switch (state){
  //Disabled State, no monitoring of temp/water. Start button monitored with ISR
  case 0:
    turnFanOff();
    printLogTime();
    U0putstr("Changing to Disabled\n");
    turnLightOn(YELLOWLED);
    break;
  //Idle State - water/temp should be monitored
  case 1:
    turnFanOff();
    printLogTime();
    U0putstr("Changing to Idle\n");
    turnLightOn(GREENLED);
    break;
  //Running State - Fan on
  case 2:

    turnFanOn();
    printLogTime();
    U0putstr("Changing to Running\n");
    turnLightOn(BLUELED);
    break;
  //Error State - Regardless of temperature, fan is off
  case 3:
    turnFanOff();
    printLogTime();
    U0putstr("Changing to Error\n");
    lcd.setCursor(0, 1);
    lcd.print("Water Low!");
    turnLightOn(REDLED);
    break;
  }
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000;  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 5 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &=  0b11110111;// clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &=  0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &=  0b01111111;// clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |=  0b01000000;// set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &=  0b11011111;// clear bit 5 to 0 for right adjust result
  //*my_ADMUX  &=  // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &=  0b11100000;// clear bit 4-0 to 0 to reset the channel and gain bits
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

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
void U0putstr(const unsigned char* U0pdata){
  for(int i =0; i < strlen(U0pdata); i++ ) {
    unsigned char c = U0pdata[i];
    U0putchar(c);
  }
}
//Turns fan on
void turnFanOn(){
  WRITE_HIGH_PE(4); //0010 0000
}
//Turns fan off
void turnFanOff(){
  WRITE_LOW_PE(4);
}

/*a function that takes 
a, a number 0-6 (Even only) and switches to letting that led be on
Blue 0
Red 2
Green 4
Yellow 6
*/
void turnLightOn(int a){
  ALL_LIGHTS_DOWN(); //Shut down all Lights
  WRITE_HIGH_PA(a); //TUrn on only ONE light
}

void processIR(){
    Serial.println(irrecv.decodedIRData.command, HEX);
    switch(irrecv.decodedIRData.command){
      case 0x46: //VOL+
        if(curState){
          printLogTime();
          U0putstr("Opening Vent With Stepper Motor\n");
          small_stepper.setSpeed(500); //Max seems to be 500
          stepSize = 2048;  // Rotate CW
          small_stepper.step(stepSize);
          delay(2000);
        }
        break;
      case 0x15: //VOL-
        if(curState){
          small_stepper.setSpeed(500);
          U0putstr("Closing Vent With Stepper Motor\n");
          stepSize = -2048;  // Rotate CCW
          small_stepper.step(stepSize);
          delay(2000); 
        }
        break;
      case 0x45: //power - on/off
        if(curState = 0){
          set_state(1);
        } else {
          set_state(0);
        }
        break;
      case 0x47: //func/stop - reset button
        if(curWat > water_level_threshold){
          set_state(1);
          lcd.setCursor(0, 1);
          lcd.print("          ");
        }
        break;
    }
    irrecv.resume();
    WRITE_LOW_PH(5); //8
    WRITE_LOW_PH(6); //9
    WRITE_LOW_PB(4); //10
    WRITE_LOW_PB(5); //11
}
