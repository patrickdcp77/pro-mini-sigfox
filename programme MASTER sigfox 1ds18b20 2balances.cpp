
/*

fonctionne correctement

+++++++++++++++++++++++++++++++++++++++++++++++
code rédigé pour 
4 balances et une sonde temp hum DHT22 (-35 à 50°) ou DHT11 (0 à 50°)
mesure et hibernation tous les 1/4H
envoi sur réseau sigfox

pour les balances, cablage des connecteurs 
utiliser les canaux B des HX711 si l'on veut économiser les convertisseurs


cordon crème et plat
jaune   A,B+
noir   E-
rouge   E+
vert    A,B-

cordon blanc et plat avec 4 fils couleurs pales
rose    E+
bleu    E-
blanc   AB-
jaunes  AB+



correspondance avec les cables noirs étanches
vert    A,B+
noir    E-
rouge   E+
bleu    A,B-

cablage des balances vues en dessous (donc platine porte jauges retournée)
vue eu U inversé
    -------------------------
    |  E-              A,B+ |
    |                       |    
    |                       |
    |   A,B-            E+  |

fils noirs des jauges relient les jauges E- avec AB-  et AB+ avec E+
fils blancs des jauges telient E- avec AB+    et AB- avec E+


consignes pour programmation du minipro arduino qui n'a pas de schip USB via TX RX pour baisser la consommation
utiliser un FTDI externe qui adaptera le TX RX à l'USB
utiliser la position 3V3
déconnecter le TX RX de sigfox pour ne pas perturber le TX RX du promini
mettre la valeur **** boolean AUTODETECT_HX711_N34 = 0**** ; à UN pour observer sur le terminal les valeurs données par les capteurs
mettre la valeur ****const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP = 116 ;**** à 2 ou 3 pour ne pas attendre un cycle de 15mn entre chaque mesure
ne rien mettre sur les balances avant chaque RESET et voir l'évolution des valeurs selon les poids posés
si tout semble bon, remettre les valeurs à ZERO pour ne pas débugger et à 116 pour avoir un cycle de 15mn entre chque envoi de trame sigfox

mise en mémoire de la tare (offset) tant qu'il n'y a pas de reset


pas de DHT22 sur les derniers prototypes
sinon  alime+ de la DHT sur pin D12  et DATA sur pin A3

on remplace par une sonde DS18B20 alime en D8 et data en D9***********************

pas de panneau solaire mais utilisation d'une pile plate 4V5 alcaline d'environ 3000 mA
l'alimentation se fait via le connecteur latéral du PRO MINI  VCC et GND

modifications sur PRO MINI:
régulateur enlevé
résistance enlevée de la led de visualisation de l'alimentation
mesure de la tension batterie appliquée sur VCC
on met une résistance CMS de 47K entre A0 et A1 coté supérieur du processeur
on met une résistance CMS de 10K entre A0 et le plan de masse en parallele à un condensateur de 1,1 nano soudés l'un sur l'autre

cablage du module radio sigfox*****************
le reset est à coté du + et connecté sur GPIO 2
le + est sur VCC car on ne peut pas alimenter le module avec un GPIO qui serait trop faible 


cablage du HX711 pour 2 balances sur un seul HX711************
GPIO 3 chaque HX711 a une alimentation commune sur un même GPIO mis à zéro pour couper l'alimentation
GPIO 4 connexion au DATA
GPIO 8 connexion à l'horloge SCK
balance N1_Channel A : connecter sur E+ E- A+ A-
balance N1_Channel B : connecter sur E+ E- B+ B-

Operating voltage:
●2.7V to 5.5V for ATmega328P donc pile plate alcaline de 4V5 possible sans régulateur
Low power consumption (caractéristiques constructeur du microproc)
● Active mode: 1.5mA at 3V - 4MHz
● Power-down mode: 1μA at 3V

● Write/erase cycles: 10,000 flash/100,000 EEPROM  (nb de fois que l'on peut écrire dans la EEPROM, 
ici on le fait seulement pour mettre en mémoire la tare )

Mise en sommeil du système:
à priori (voir Sylvain), utilisation d'un timer via un registre à décalage qui est décrémenté ou incrémenté 116 fois pour couvrir 1/4 heure
à la mise en sommeil, les capteurs poids et météo qui sont alimentés par les GPIO (cosommation modeste et supportable par les entrées) 
sont mis à zéro.
le module radio consomme plus en émission et est donc alimenté directement par la batterie car il fonctionne entre 3 et 5V
il est mis en veille avec son propre système

*******************
code de mise en veille (à faire décrire avec précision par Sylvain):
*******************


****************déclaration des variables************
////////////////////////////////////////////   pour modifier l'hibernation
const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP = 116; //2 ; // MUST BE >=2 , 116 for 15mnsNumber of WATCH DOG before starting the main software  
unsigned int counter_power_down_wake_up; // 
/////////////////////////////////////////////

*************dans le setup***************
counter_power_down_wake_up = MAX_COUNTER_POWER_DOWN_WAKE_UP;

//noInterrupts (); // timed sequence follows
cli();  // Interrupts impossible

// ENABLE SLEEP 
//set_sleep_mode (SLEEP_MODE_PWR_DOWN);
SMCR |= (1<<2); //SET UP  SLEEP MODE = Power Down Mode
//sleep_enable();
SMCR |= 1; //Enable Sleep

                                      
//interrupts (); // guarantees next instruction executed
sei();

***********dans le loop**************
if (counter_power_down_wake_up == 0) {
    
    counter_power_down_wake_up  = MAX_COUNTER_POWER_DOWN_WAKE_UP;
    

    PUIS LE CODE ETC..... QUI FONCTIONNE DANS LA CONDITION IF DU COMPTEUR

    // PUT HERE THE CODE TO BE EXECUTED 

    Serial.begin(9600);

   
    pinMode(RESET_SIGFOX_MODULE,OUTPUT);

    digitalWrite(RESET_SIGFOX_MODULE,LOW);
    delay (100);
    digitalWrite(RESET_SIGFOX_MODULE,HIGH);
    // delay (100);


    if (DEBUG_MODE) { 

    PUIS APRÈS LA CONDITION IF ON A LA CONDITION ELSE SUIVANTE

    else {   
// Decrementaiton counter WDT
counter_power_down_wake_up  =  counter_power_down_wake_up-1 ;

}



*/

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>

//concerne la sonde DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS A3
#define DS18B20_POWER_SUPPLY_PIN 12 // l'alimentation de la DHT22 se fait via un GPIO qui sera mis à zéro pour couper l'alimentation
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);


#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 11     // Pin où le pin DATA du DHT22 est connecté
#define DHTTYPE DHT22   // Définir le type de DHT
DHT dht(DHTPIN, DHTTYPE);
#define DHT_POWER_SUPPLY_PIN 10


#include "HX711.h"
HX711 Hx711_N1;   
/*
HX711 Hx711_N2;
HX711 Hx711_N3;
HX711 Hx711_N4;
*/
#define POWER_PIN_HX711_N1_DATA_OUT 3//chaque HX711 a une alimentation commune sur un même GPIO mis à zéro pour couper l'alimentation
#define PIN_HX711_N1_DATA_OUT 4 // connexion au DATA
#define PIN_HX711_N1_SCK_AND_POWER_DOWN 8// connexion à l'horloge SCK

float offset_HX711_N1_ChannelA;
float offset_HX711_N1_ChannelB;//Sylvain utilise les 2 canaux pour mettre une barre de pesée par canal

/*
boolean AUTODETECT_HX711_N2 = 1 ; // 0 = No 2nd HX711, autodetection of a 2nd HX711 is in Setup sequence will toggle to 1 if detected 

#define POWER_PIN_HX711_N2_DATA_OUT 3
#define PIN_HX711_N2_DATA_OUT 5
#define PIN_HX711_N2_SCK_AND_POWER_DOWN 9

float offset_HX711_N2_ChannelA;
float offset_HX711_N2_ChannelB;
*/
/*
boolean AUTODETECT_HX711_N34 = 0 ; // 0 = No 3rd&4th  HX711, autodetection of a 3rd & 4th  HX711 is in Setup sequence will toggle to 1 if detected 

#define POWER_PIN_HX711_N3_DATA_OUT 3
#define PIN_HX711_N3_DATA_OUT 6
#define PIN_HX711_N3_SCK_AND_POWER_DOWN 10


#define POWER_PIN_HX711_N4_DATA_OUT 3
#define PIN_HX711_N4_DATA_OUT 7
#define PIN_HX711_N4_SCK_AND_POWER_DOWN 11

float offset_HX711_N3_ChannelA;
float offset_HX711_N4_ChannelA;
*/
#include <avr/interrupt.h> 

#define MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT A0 
#define MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN A1

#define SOLAR_PANEL_VOLTAGE_CAN_MEASUREMENT A2//dispositif en prévision de la mesure du panneau solaire pour déterminer les cycles jour nuit mais non cablé

#define RESET_SIGFOX_MODULE 2//on utilise le RESET du module sigfox

const unsigned int Weight_sensitivity = 4 ;

////////////////////////////////////////////   pour modifier l'hibernation
const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP =116; //2 ; // MUST BE >=2 , 116 for 15mnsNumber of WATCH DOG before starting the main software  
unsigned int counter_power_down_wake_up; // 
/////////////////////////////////////////////


//  Header byte = 1st Byte transmitted int he Sigox Tram ( SIGFOX #01 )
// Bit 7 = Sigfog Debug Mode, =1 for SIGFOX DEBUG MODE
// Bit 6 = Solar Panel Luminosity, measured during software excution ( =0 Voltage solar panel < Voltage Battery, =1 > 
// Bit 5 = FUll Charging Battery status , measured during software excution
// Bit 4 = Charging on going, measured during software excution
// Bit 0 to 3 = Software version : 0000 = Debug Software, then 0001 = V1 ...
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
byte header_byte = B00000000;


/////////////////////////////////////pour modifier le débuggage
boolean DEBUG_MODE = 0 ;// =1 or 0 debbug , then ALLOW BLINKING LED and statement on Serial Monitor throught SERIAL RX/TX UART0  
//////////////////////////////////////

// put your setup code here, to run once:
void setup() {

    pinMode(RESET_SIGFOX_MODULE,OUTPUT);

    digitalWrite(RESET_SIGFOX_MODULE,LOW);
    delay (100);
    digitalWrite(RESET_SIGFOX_MODULE,HIGH);
    // delay (100);


    /**********************
    pinMode(A3, INPUT);

    pinMode(12, OUTPUT);
    */
Serial.begin(9600);

Serial.println("programme master module sigfox code reference 20220211 precision balances optimisee");

//concerne la sonde DS18B20
sensors.begin();

if (DEBUG_MODE) {   
                        Serial.println("SETUP DONE");
                        
                        
                        pinMode (LED_BUILTIN,OUTPUT); 
                        digitalWrite(LED_BUILTIN,HIGH);
                        delay(2000);
                        digitalWrite(LED_BUILTIN,LOW);
                        // delay(5000);
                }


pinMode(POWER_PIN_HX711_N1_DATA_OUT,OUTPUT);
digitalWrite(POWER_PIN_HX711_N1_DATA_OUT,HIGH);

delay(1000);

float Sample_weight;

Hx711_N1.begin(PIN_HX711_N1_DATA_OUT,PIN_HX711_N1_SCK_AND_POWER_DOWN,64 );
Sample_weight = Hx711_N1.get_units();
offset_HX711_N1_ChannelA = Hx711_N1.get_units();

Hx711_N1.begin(PIN_HX711_N1_DATA_OUT,PIN_HX711_N1_SCK_AND_POWER_DOWN,32 );
Sample_weight = Hx711_N1.get_units();
offset_HX711_N1_ChannelB = Hx711_N1.get_units();


if (DEBUG_MODE) {  
                        Serial.print("Offset N1 Channel A : ");
                        Serial.print(offset_HX711_N1_ChannelA);

                        Serial.print(" Offset N1 Channel B : ");
                        Serial.print(offset_HX711_N1_ChannelB);

                        Serial.println();
                        

                        Serial.flush() ;



                }

//1er message pour voir bon fonctionnement sigfox à l'allumage système
Serial.println("AT$SF=12345678");

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
    
    
    pinMode(DS18B20_POWER_SUPPLY_PIN,OUTPUT);
    digitalWrite(DS18B20_POWER_SUPPLY_PIN,HIGH);

    pinMode(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,OUTPUT);
    digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,HIGH);
    pinMode(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT,INPUT); 
    pinMode(SOLAR_PANEL_VOLTAGE_CAN_MEASUREMENT,INPUT);

    sensors.begin();
    delay(1000);    // MADATORY 1000 or put code in between

        
    
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    //float h = dht.readHumidity();
    float h = 0;
    // Read temperature as Celsius
    //float t = dht.readTemperature();
    
    
    // Send command to all the sensors for temperature conversion
    sensors.requestTemperatures(); 
    // Read temperature in Celsius
    float temperatureC = sensors.getTempCByIndex(0);


    //digitalWrite(DHT_POWER_SUPPLY_PIN,LOW);
    digitalWrite(DS18B20_POWER_SUPPLY_PIN,LOW);
    
    uint8_t h_byte = 0;

    int8_t  t_byte = (temperatureC + 35 )*2;

    if (DEBUG_MODE) { 
                        Serial.print("Temperature ds18b20: "); 

                        //Serial.print(t);
                        Serial.print(temperatureC);
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


   

    digitalWrite(POWER_PIN_HX711_N1_DATA_OUT,LOW);

      
// Message AT popur Sigfox - 
Serial.println("AT");  // Test after wake up from sleep mode 
Serial.flush() ;
delay(100);
char Sigfox_message[34] = {0};  // 26 or 34 bytes max ( 1 or 2 HX711)

sprintf(Sigfox_message,"AT$SF=%02x%02x%02x%02x%04x%04x",header_byte,power_supply_voltage_Arduino,t_byte,h_byte,Weight_HX711_N1_Channel_A,Weight_HX711_N1_Channel_B);

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

