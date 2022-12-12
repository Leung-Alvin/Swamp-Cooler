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

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
#include <LiquidCrystal.h>
#define RDA 0x80
#define TBE 0x20  

#define WRITE_HIGH_PB(pin_num)  *port_b |= (0x01 << pin_num);
#define WRITE_LOW_PB(pin_num)  *port_b &= ~(0x01 << pin_num);
#define WRITE_HIGH_PH(pin_num)  *port_h |= (0x01 << pin_num);
#define WRITE_LOW_PH(pin_num)  *port_h &= ~(0x01 << pin_num);

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

LiquidCrystal lcd(12,11,5,4,3,2);
#define DHTTYPE DHT11
#define WLDPIN 0 //Analog pin connected to water detection sensor
#define DHTPIN 14 //Digital pin connected to the DHT sensor
#define FANPIN 9
DHT dht(DHTPIN, DHTTYPE);

//PB7 = 13 0x80
//PH6 = 9 0010 0000 -> 0x20

int water_level_threshold = 100; //If below then water level is too low
int temp_threshold_high = 100; // High Fahrenheit
int temp_threshold_low = 0; // Low Fahrenheit
int curTemp = 105; //Maintain current temperature
void setup() 
{
  U0init(9600); //Serial Begin
  lcd.begin(16,2); //Turn on LCD
  *ddr_b |= 0x80; //Turn on BUILT_IN_LED
  *ddr_h |= 0x20; //Turn on Fan Motor
  dht.begin(); //Start DHT
  adc_init(); //Start ADC and Water Level Sampling
}
void loop() 
{

  if(curTemp >= temp_threshold_high){ //Turn fan on if greater than high temp threshold
    turnFanOn();
  }
  else if(curTemp < temp_threshold_low){ //Turn fan off if less than low temp threshold
    turnFanOff();
  }
  
  lcd.setCursor(0, 0); //First line in LCD Display
  unsigned int adc_reading = adc_read(WLDPIN); //Read the analog pin for water level
  //CODE IS FOR DHT in order to READ HUMIDITY AND TEMP
  /* float h = dht.readHumidity(); //reads humidity
  float t = dht.readTemperature(); //reads temperature as celsius
  float f = dht.readTemperature(true); //reads temperature as fahrenheit
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  lcd.print(f);
  lcd.print("°F ");
  lcd.print(h);
  lcd.print("% "); */
  lcd.setCursor(0, 1); //Moves to second line of LCD Display
  if(adc_reading <= water_level_threshold){ //ADC reads water is lower than low level and prints
    lcd.println("LOW");
  }
  else if (adc_reading > water_level_threshold){  //ADC reads water is higher than low level and prints
    lcd.println("GOOD ");
  }
  delay(1000);
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
void print_int(unsigned int out_num)
{
  // clear a flag (for printing 0's in the middle of numbers)
  unsigned char print_flag = 0;
  // if its greater than 1000
  if(out_num >= 1000)
  {
    // get the 1000's digit, add to '0' 
    U0putchar(out_num / 1000 + '0');
    // set the print flag
    print_flag = 1;
    // mod the out num by 1000
    out_num = out_num % 1000;
  }
  // if its greater than 100 or we've already printed the 1000's
  if(out_num >= 100 || print_flag)
  {
    // get the 100's digit, add to '0'
    U0putchar(out_num / 100 + '0');
    // set the print flag
    print_flag = 1;
    // mod the output num by 100
    out_num = out_num % 100;
  } 
  // if its greater than 10, or we've already printed the 10's
  if(out_num >= 10 || print_flag)
  {
    U0putchar(out_num / 10 + '0');
    print_flag = 1;
    out_num = out_num % 10;
  } 
  // always print the last digit (in case it's 0)
  U0putchar(out_num + '0');
  // print a newline
  U0putchar('\n');
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

//Turns fan on
void turnFanOn(){
  WRITE_HIGH_PH(6); //0010 0000
}
//Turns fan off
void turnFanOff(){
  WRITE_LOW_PH(6);
}


