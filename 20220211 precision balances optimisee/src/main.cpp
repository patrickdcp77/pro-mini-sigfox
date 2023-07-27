#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>

//#include <Adafruit_Sensor.h>
//#include <DHT.h>
#include <DHT_U.h>
#include "DHT.h"

#define DHT_POWER_SUPPLY_PIN 7
#define DHTPIN 6     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 3 (on the right) of the sensor to GROUND (if your sensor has 3 pins)
// Connect pin 4 (on the right) of the sensor to GROUND and leave the pin 3 EMPTY (if your sensor has 4 pins)
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);


#include "HX711.h"
HX711 Hx711_N1;   
HX711 Hx711_N2;
HX711 Hx711_N3;
HX711 Hx711_N4;

#define POWER_PIN_HX711_N1_DATA_OUT 3
#define PIN_HX711_N1_DATA_OUT 4
#define PIN_HX711_N1_SCK_AND_POWER_DOWN 5

float offset_HX711_N1_ChannelA;
float offset_HX711_N1_ChannelB;


boolean AUTODETECT_HX711_N2 = 0 ; // 0 = No 2nd HX711, autodetection of a 2nd HX711 is in Setup sequence will toggle to 1 if detected 
#define POWER_PIN_HX711_N2_DATA_OUT 8
#define PIN_HX711_N2_DATA_OUT 9
#define PIN_HX711_N2_SCK_AND_POWER_DOWN 5

float offset_HX711_N2_ChannelA;
float offset_HX711_N2_ChannelB;


boolean AUTODETECT_HX711_N34 = 0 ; // 0 = No 3rd&4th  HX711, autodetection of a 3rd & 4th  HX711 is in Setup sequence will toggle to 1 if detected 
#define POWER_PIN_HX711_N3_DATA_OUT 8
#define PIN_HX711_N3_DATA_OUT 9
#define PIN_HX711_N3_SCK_AND_POWER_DOWN 5


#define POWER_PIN_HX711_N4_DATA_OUT 8
#define PIN_HX711_N4_DATA_OUT 9
#define PIN_HX711_N4_SCK_AND_POWER_DOWN 5

float offset_HX711_N3_ChannelA;
float offset_HX711_N4_ChannelA;



// #include <avr/wdt.h>
#include <avr/interrupt.h> 

// LED_BUILTIN alredy define @ 13 in \.platformio\packages\framework-arduino-avr\variants\standardstandard\pins_arduino.h
// avr/iom328p.h

#define MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT A0
#define MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN A1

#define SOLAR_PANEL_VOLTAGE_CAN_MEASUREMENT A2

#define RESET_SIGFOX_MODULE 2


//// #include <SoftwareSerial.h>
//// SoftwareSerial Sigfox_BRKWS01_Serial(2,3); // RX, TX



const unsigned int Weight_sensitivity = 4 ;

const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP = 2 ; // MUST BE >=2 , 116 for 15mnsNumber of WATCH DOG before starting the main software  
unsigned int counter_power_down_wake_up; // 

//  Header byte = 1st Byte transmitted int he Sigox Tram ( SIGFOX #01 )
// Bit 7 = Sigfog Debug Mode, =1 for SIGFOX DEBUG MODE
// Bit 6 = Solar Panel Luminosity, measured during software excution ( =0 Voltage solar panel < Voltage Battery, =1 > 
// Bit 5 = FUll Charging Battery status , measured during software excution
// Bit 4 = Charging on going, measured during software excution
// Bit 0 to 3 = Software version : 0000 = Debug Software, then 0001 = V1 ...
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
byte header_byte = B00000000;


boolean DEBUG_MODE =  1; // =1 for debbug , then ALLOW BLINKING LED and statement on Serial Monitor throught SERIAL RX/TX UART0  


// put your setup code here, to run once:
void setup() {

    pinMode(RESET_SIGFOX_MODULE,OUTPUT);

    digitalWrite(RESET_SIGFOX_MODULE,LOW);
    delay (100);
    digitalWrite(RESET_SIGFOX_MODULE,HIGH);
    // delay (100);


Serial.begin(9600);

if (DEBUG_MODE) {   
                        Serial.print("SETUP DONE");
                        Serial.println();
                        pinMode (LED_BUILTIN,OUTPUT); 
                        digitalWrite(LED_BUILTIN,HIGH);
                        delay(2000);
                        digitalWrite(LED_BUILTIN,LOW);
                        // delay(5000);
                }


pinMode(POWER_PIN_HX711_N1_DATA_OUT,OUTPUT);
digitalWrite(POWER_PIN_HX711_N1_DATA_OUT,HIGH);

pinMode(POWER_PIN_HX711_N2_DATA_OUT,OUTPUT);
digitalWrite(POWER_PIN_HX711_N2_DATA_OUT,HIGH);

pinMode(POWER_PIN_HX711_N3_DATA_OUT,OUTPUT);
digitalWrite(POWER_PIN_HX711_N3_DATA_OUT,HIGH);

pinMode(POWER_PIN_HX711_N4_DATA_OUT,OUTPUT);
digitalWrite(POWER_PIN_HX711_N4_DATA_OUT,HIGH);

delay(1000);

float Sample_weight;

Hx711_N1.begin(PIN_HX711_N1_DATA_OUT,PIN_HX711_N1_SCK_AND_POWER_DOWN,64 );
Sample_weight = Hx711_N1.get_units();
offset_HX711_N1_ChannelA = Hx711_N1.get_units();

Hx711_N1.begin(PIN_HX711_N1_DATA_OUT,PIN_HX711_N1_SCK_AND_POWER_DOWN,32 );
Sample_weight = Hx711_N1.get_units();
offset_HX711_N1_ChannelB = Hx711_N1.get_units();

Hx711_N2.begin(PIN_HX711_N2_DATA_OUT,PIN_HX711_N2_SCK_AND_POWER_DOWN,64 );
Sample_weight = Hx711_N2.get_units();
offset_HX711_N2_ChannelA = Hx711_N2.get_units();

Hx711_N2.begin(PIN_HX711_N2_DATA_OUT,PIN_HX711_N2_SCK_AND_POWER_DOWN,32 );
Sample_weight = Hx711_N2.get_units();
offset_HX711_N2_ChannelB = Hx711_N2.get_units();

if ((offset_HX711_N2_ChannelA!=0) & (offset_HX711_N2_ChannelA!=0)) { AUTODETECT_HX711_N2 = 1 ;}



Hx711_N3.begin(PIN_HX711_N3_DATA_OUT,PIN_HX711_N3_SCK_AND_POWER_DOWN,64 );
Sample_weight = Hx711_N3.get_units();
offset_HX711_N3_ChannelA = Hx711_N3.get_units();

Hx711_N4.begin(PIN_HX711_N4_DATA_OUT,PIN_HX711_N4_SCK_AND_POWER_DOWN,64 );
Sample_weight = Hx711_N4.get_units();
offset_HX711_N4_ChannelA = Hx711_N4.get_units();

if ((offset_HX711_N3_ChannelA!=0) & (offset_HX711_N4_ChannelA!=0)) { AUTODETECT_HX711_N34 = 1 ;}




if (DEBUG_MODE) {  
                        Serial.print("Offset N1 Channel A : ");
                        Serial.print(offset_HX711_N1_ChannelA);

                        Serial.print(" Offset N1 Channel B : ");
                        Serial.print(offset_HX711_N1_ChannelB);

                        Serial.println();
                        

                        
                        Serial.print("Offset N2 Channel A : ");
                        Serial.print(offset_HX711_N2_ChannelA);

                        Serial.print(" Offset N2 Channel B : ");
                        Serial.print(offset_HX711_N2_ChannelB);

                        Serial.print(" AUTO DETECT N2 : ");
                        Serial.print(AUTODETECT_HX711_N2);

                        Serial.println();              

                        
                        
                        Serial.print("Offset N3 Channel A : ");
                        Serial.print(offset_HX711_N3_ChannelA);

                        Serial.print(" Offset N4 Channel A : ");
                        Serial.print(offset_HX711_N4_ChannelA);

                        Serial.print(" AUTO DETECT N3&N4 : ");
                        Serial.print(AUTODETECT_HX711_N34);

                        Serial.println();
                        
                        Serial.flush() ;



                }





counter_power_down_wake_up = MAX_COUNTER_POWER_DOWN_WAKE_UP;




// Message AT popur Sigfox - 
Serial.println("AT");  // Test after wake up from sleep mode
Serial.flush() ;
delay(100);

// char Sigfox_message[34] = {0};  // 26 or 34 bytes max ( 1 or 2 HX711)

//sprintf(Sigfox_message,"AT$SF=%02x%02x%02x%02x%04x%04x",header_byte,power_supply_voltage_Arduino,t_byte,h_byte,Weight_HX711_N1_Channel_A,Weight_HX711_N1_Channel_B);
//Serial.println(Sigfox_message);
// Serial.flush() ;
Serial.println("AT$SF=SETUP DONE");

Serial.println("AT$P=2"); // SLeep mode
Serial.flush() ;







// Save Power by writing all Digital IO to LOW EXCLUDING SPECIAL PORT

//pinMode(0,INPUT);
//pinMode(1,INPUT);

for(int i=2; i<19; i++) {
    if (i != RESET_SIGFOX_MODULE) {
        pinMode(i,OUTPUT);
        digitalWrite(i,LOW);
        }
    //else {
    //    digitalWrite(i,HIGH);
    //    }
    }

  digitalWrite(RESET_SIGFOX_MODULE,HIGH);






//SETUP WATCHDOG TIMER
WDTCSR = B00011000; // (24 = B00011000);//change enable and WDE − also resets
WDTCSR = B00100001; // (33 =  B00100001);//prescalers only − get rid of the WDE and WDCE bit
WDTCSR = (1<<6);

// SETUP WATCHDOG TIMER 
//WDTCSR = (24); // Change enable and WDE - Also resets
//WDTCSR = (33); /// prescalers only -get ride of the WDE and WDCE bit
//WDTCSR |=(1<<6);// enable interrupt modue

// SET UP CAN ANALOGU REFERENCE @ 1,1V INTERNAL
analogReference(INTERNAL);


// DISABLE ADC ( ADEN bit set to 0 in ADCSRA REGISTER ) 
// ADCSRA = 0;
ADCSRA &= B01111111;


//noInterrupts (); // timed sequence follows
cli();  // Interrupts impossible

// ENABLE SLEEP 
//set_sleep_mode (SLEEP_MODE_PWR_DOWN);
SMCR |= (1<<2); //SET UP  SLEEP MODE = Power Down Mode
//sleep_enable();
SMCR |= 1; //Enable Sleep

                                      
//interrupts (); // guarantees next instruction executed
sei();

}




// ------------------------------------------------------------------------------------------------------------------------------------------------------




// put your main code here, to run repeatedly:
void loop() {



if (DEBUG_MODE) { 
                        Serial.begin(9600);
                        digitalWrite(LED_BUILTIN,HIGH);
                        delay(100);
                        digitalWrite(LED_BUILTIN,LOW);
                        // delay(500);


                        Serial.print("LOOP CNT = ");
                        Serial.println (counter_power_down_wake_up);
                        Serial.flush() ;
                }



if (counter_power_down_wake_up == 0) {
    
    counter_power_down_wake_up  = MAX_COUNTER_POWER_DOWN_WAKE_UP;
    
    // PUT HERE THE CODE TO BE EXECUTED 

    Serial.begin(9600);

   
    pinMode(RESET_SIGFOX_MODULE,OUTPUT);

    digitalWrite(RESET_SIGFOX_MODULE,LOW);
    delay (100);
    digitalWrite(RESET_SIGFOX_MODULE,HIGH);
    // delay (100);


    if (DEBUG_MODE) { 
                        Serial.println("CODE TO BE EXCUTED");
                    }
    
    
    pinMode(POWER_PIN_HX711_N1_DATA_OUT,OUTPUT);
    digitalWrite(POWER_PIN_HX711_N1_DATA_OUT,HIGH);
    
    if (AUTODETECT_HX711_N2) {
        pinMode(POWER_PIN_HX711_N2_DATA_OUT,OUTPUT);
        digitalWrite(POWER_PIN_HX711_N2_DATA_OUT,HIGH);                   
    }

    if (AUTODETECT_HX711_N34) {
        pinMode(POWER_PIN_HX711_N3_DATA_OUT,OUTPUT);
        digitalWrite(POWER_PIN_HX711_N3_DATA_OUT,HIGH); 

        pinMode(POWER_PIN_HX711_N4_DATA_OUT,OUTPUT);
        digitalWrite(POWER_PIN_HX711_N4_DATA_OUT,HIGH);     

    }


    
    pinMode(DHT_POWER_SUPPLY_PIN,OUTPUT);
    digitalWrite(DHT_POWER_SUPPLY_PIN,HIGH);

    pinMode(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,OUTPUT);
    digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,HIGH);
    pinMode(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT,INPUT); 
    pinMode(SOLAR_PANEL_VOLTAGE_CAN_MEASUREMENT,INPUT);


    //delay(2000);




    dht.begin();
    delay(1000);    // MADATORY 1000 or put code in between



  
    
    
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();

    // Read temperature as Celsius
    float t = dht.readTemperature();


    digitalWrite(DHT_POWER_SUPPLY_PIN,LOW);

    

    int8_t  t_byte = (t + 35 )*2;
    uint8_t h_byte = h*2;

    if (DEBUG_MODE) { 
                        Serial.print("Temperature: "); 

                        Serial.print(t);
                        Serial.print(" *C SIGFOX #03: ");
                        Serial.print(t_byte,HEX);
                        Serial.print(" Byte   ");

                        Serial.print("Humidity : "); 
                        Serial.print(h);
                        Serial.print(" %\t SIGFOX #04: ");
                        Serial.print(h_byte,HEX);
                        Serial.println(" Byte   ");
                    }


    // analogReference(INTERNAL);  dans set up 

    ADCSRA |= B10000000;
    
    int  test = analogRead(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT); // Une mesure pour rien pour eviter les erreurs ( 3F delta contaté), a voir si mettre une fois dans le setup
    
    uint8_t power_supply_voltage_Arduino= (analogRead(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT)>>2);

    test = analogRead(SOLAR_PANEL_VOLTAGE_CAN_MEASUREMENT); // Une mesure pour rien pour eviter les erreurs ( 3F delta contaté), a voir si mettre une fois dans le setup
    
    uint8_t power_supply_voltage_Solar_panel= (analogRead(SOLAR_PANEL_VOLTAGE_CAN_MEASUREMENT)>>2);
    
    digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,LOW); 
    
    ADCSRA &= B01111111;




    if (DEBUG_MODE) { 
                        Serial.print("ADMUX: ");
                        Serial.print(ADMUX,BIN);

                        Serial.print("  ARDUINO VOLTAGE : ");
                        Serial.print(power_supply_voltage_Arduino,BIN);


                        Serial.print("  SIGFOX #02: ");
                        Serial.print(power_supply_voltage_Arduino,HEX);

                        Serial.print("  SOLAR PANEL VOLTAGE: ");
                        Serial.print(power_supply_voltage_Solar_panel,BIN);

                        Serial.println();

                    }



    float Sample_weight;
    
    Hx711_N1.begin(PIN_HX711_N1_DATA_OUT,PIN_HX711_N1_SCK_AND_POWER_DOWN,64 );
    Sample_weight = Hx711_N1.get_units();                                          // For nothing library problem...
    Sample_weight = (offset_HX711_N1_ChannelA - Hx711_N1.get_units())/256*Weight_sensitivity;

    if (Sample_weight < 0) { 
        Sample_weight =0; 
        }
    else { 
       if (Sample_weight > 65535) { 
           Sample_weight = 65535; 
           }
       }

    unsigned int Weight_HX711_N1_Channel_A = Sample_weight;
 
    Hx711_N1.begin(PIN_HX711_N1_DATA_OUT,PIN_HX711_N1_SCK_AND_POWER_DOWN,32 );
    Sample_weight = Hx711_N1.get_units();                                          // For nothing library problem...
    Sample_weight = (offset_HX711_N1_ChannelB - Hx711_N1.get_units())/128*Weight_sensitivity;

    if (Sample_weight < 0) {
        Sample_weight =0; 
        }
        else { 
        if (Sample_weight > 65535) { 
            Sample_weight = 65535; 
            }
        }
    
    unsigned int Weight_HX711_N1_Channel_B = Sample_weight;


    

    unsigned int Weight_HX711_N2_Channel_A;
    unsigned int Weight_HX711_N2_Channel_B;

    if (AUTODETECT_HX711_N2) {
                                Hx711_N2.begin(PIN_HX711_N2_DATA_OUT,PIN_HX711_N2_SCK_AND_POWER_DOWN,64 );
                                Sample_weight = Hx711_N2.get_units();                                          // For nothing library problem...
                                Sample_weight = (offset_HX711_N2_ChannelA - Hx711_N2.get_units())/256*Weight_sensitivity;

                                if (Sample_weight < 0) { 
                                    Sample_weight =0; 
                                    }
                                else { 
                                    if (Sample_weight > 65535) { 
                                                            Sample_weight = 65535; 
                                                            }
                                    }

                                Weight_HX711_N2_Channel_A = Sample_weight;

                                Hx711_N2.begin(PIN_HX711_N2_DATA_OUT,PIN_HX711_N2_SCK_AND_POWER_DOWN,32 );
                                Sample_weight = Hx711_N2.get_units();                                          // For nothing library problem...
                                Sample_weight = (offset_HX711_N2_ChannelB - Hx711_N2.get_units())/128*Weight_sensitivity;

                                if (Sample_weight < 0) {
                                    Sample_weight =0; 
                                    }
                                    else { 
                                    if (Sample_weight > 65535) { 
                                                                Sample_weight = 65535; 
                                                                }
                                    }
                                
                                Weight_HX711_N2_Channel_B = Sample_weight;
                                
                                
                            }

    




    unsigned int Weight_HX711_N3_Channel_A;
    unsigned int Weight_HX711_N4_Channel_A;


    if (AUTODETECT_HX711_N34) {
                                Hx711_N3.begin(PIN_HX711_N3_DATA_OUT,PIN_HX711_N3_SCK_AND_POWER_DOWN,64 );
                                Sample_weight = Hx711_N3.get_units();                                          // For nothing library problem...
                                Sample_weight = (offset_HX711_N3_ChannelA - Hx711_N3.get_units())/256*Weight_sensitivity;

                                if (Sample_weight < 0) { 
                                    Sample_weight =0; 
                                    }
                                else { 
                                    if (Sample_weight > 65535) { 
                                                            Sample_weight = 65535; 
                                                            }
                                    }

                                Weight_HX711_N3_Channel_A = Sample_weight;

                                

                                Hx711_N4.begin(PIN_HX711_N4_DATA_OUT,PIN_HX711_N4_SCK_AND_POWER_DOWN,64 );
                                Sample_weight = Hx711_N4.get_units();                                          // For nothing library problem...
                                Sample_weight = (offset_HX711_N4_ChannelA - Hx711_N4.get_units())/256*Weight_sensitivity;

                                if (Sample_weight < 0) {
                                    Sample_weight =0; 
                                    }
                                    else { 
                                    if (Sample_weight > 65535) { 
                                                                Sample_weight = 65535; 
                                                                }
                                    }
                                
                                Weight_HX711_N4_Channel_A = Sample_weight;

                                digitalWrite(POWER_PIN_HX711_N3_DATA_OUT,LOW);
                                digitalWrite(POWER_PIN_HX711_N4_DATA_OUT,LOW);

                            }


    digitalWrite(POWER_PIN_HX711_N1_DATA_OUT,LOW);
       if (AUTODETECT_HX711_N2) { digitalWrite(POWER_PIN_HX711_N2_DATA_OUT,LOW); }


    if (DEBUG_MODE) {  
                        Serial.print("Weight N1 Channel A : ");
   
                        Serial.print(Weight_HX711_N1_Channel_A,DEC);
                        Serial.print(" "); 
                        Serial.print(Weight_HX711_N1_Channel_A,HEX);
                        Serial.print(" "); 
                        Serial.print(Weight_HX711_N1_Channel_A,BIN);
                        Serial.print(" "); 

                        Serial.print("Weight N1 Channel B : ");

                        Serial.print(Weight_HX711_N1_Channel_B,DEC);
                        Serial.print(" "); 
                        Serial.print(Weight_HX711_N1_Channel_B,HEX);
                        Serial.print(" "); 
                        Serial.print(Weight_HX711_N1_Channel_B,BIN);
                        Serial.print(" ");
   
                        Serial.println();

                        if (AUTODETECT_HX711_N2) {
                                                    Serial.print("Weight N2 Channel A : ");
   
                                                    Serial.print(Weight_HX711_N2_Channel_A,DEC);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N2_Channel_A,HEX);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N2_Channel_A,BIN);
                                                    Serial.print(" "); 

                                                    Serial.print("Weight N2 Channel B : ");

                                                    Serial.print(Weight_HX711_N2_Channel_B,DEC);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N2_Channel_B,HEX);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N2_Channel_B,BIN);
                                                    Serial.print(" ");
                            
                                                    Serial.println();

                                                    }

                        if (AUTODETECT_HX711_N34) {
                                                    Serial.print("Weight N3 Channel A : ");
   
                                                    Serial.print(Weight_HX711_N3_Channel_A,DEC);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N3_Channel_A,HEX);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N3_Channel_A,BIN);
                                                    Serial.print(" "); 

                                                    Serial.print("Weight N4 Channel A : ");

                                                    Serial.print(Weight_HX711_N4_Channel_A,DEC);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N4_Channel_A,HEX);
                                                    Serial.print(" "); 
                                                    Serial.print(Weight_HX711_N4_Channel_A,BIN);
                                                    Serial.print(" ");
                            
                                                    Serial.println();

                                                    }


                    }



    


// Message AT popur Sigfox - 
Serial.println("AT");  // Test after wake up from sleep mode 
Serial.flush() ;
delay(100);
char Sigfox_message[34] = {0};  // 26 or 34 bytes max ( 1 or 2 HX711)


if (AUTODETECT_HX711_N34) {
                        sprintf(Sigfox_message,"AT$SF=%02x%02x%02x%02x%04x%04x%04x%04x",header_byte,power_supply_voltage_Arduino,t_byte,h_byte,Weight_HX711_N1_Channel_A,Weight_HX711_N2_Channel_A,Weight_HX711_N3_Channel_A,Weight_HX711_N4_Channel_A);
                        }
                        else {


                        if (AUTODETECT_HX711_N2) {
                            sprintf(Sigfox_message,"AT$SF=%02x%02x%02x%02x%04x%04x%04x%04x",header_byte,power_supply_voltage_Arduino,t_byte,h_byte,Weight_HX711_N1_Channel_A,Weight_HX711_N1_Channel_B,Weight_HX711_N2_Channel_A,Weight_HX711_N2_Channel_B);
                            }

                            else{
                                sprintf(Sigfox_message,"AT$SF=%02x%02x%02x%02x%04x%04x",header_byte,power_supply_voltage_Arduino,t_byte,h_byte,Weight_HX711_N1_Channel_A,Weight_HX711_N1_Channel_B);
                                }
                        }


Serial.println(Sigfox_message);
Serial.flush() ;

Serial.println("AT$P=2");   // SLeep mode
Serial.flush() ;


}   

else {   
// Decrementaiton counter WDT
counter_power_down_wake_up  =  counter_power_down_wake_up-1 ;

}





// Save Power by writing all Digital IO to LOW EXCLUDING SPECIAL PORT

//pinMode(0,INPUT);

//pinMode(1,INPUT);


for(int i=2; i<19; i++) {
    if (i != RESET_SIGFOX_MODULE) {
        pinMode(i,OUTPUT);
        digitalWrite(i,LOW);
        }
    //else {
    //    digitalWrite(i,HIGH);
    //    }
    }

// EXEPTION POWER DOWN PIN

/////Hx711_N1.power_down();                                          // put the HX711 ADC in sleep mode
/////if (CONNECTION_HX711_N2 !=0) { Hx711_N2.power_down();           // put the HX711 ADC in sleep mode  
/////}


// Brown-out Detector DISABLE , − this must be called right before the __asm__ sleep instruction
MCUCR = bit (BODS) | bit (BODSE)  ; // | bit(PUD) ;  // Do not forget to diseable ALL IO Pull up by >Turn PUD bit in the MCUCR // TBCHECK ???
MCUCR = bit (BODS);
// MCUCR |=(3<<5); // Set both BODS and BODSE  at the same time
// MCUCR = (MCUCR & ~(1 <<5)) | (1<<6); // then set the BODS bit and clear the BODSE at the same time
        
//sleep_cpu (); // sleep within 3 clock cycles of above
__asm__ __volatile__("sleep");


}









//needed for the digital input interrupt
//void digitalInterrupt() {
// }


// watchdog interrupt
//DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
ISR(WDT_vect) {

if (DEBUG_MODE) {      
                    Serial.println("ISR_WDT VECTOR");
                    Serial.flush() ;
                }
}


