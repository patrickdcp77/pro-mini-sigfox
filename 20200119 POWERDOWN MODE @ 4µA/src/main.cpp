#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
// #include <avr/wdt.h>
#include <avr/interrupt.h> 

// LED_BUILTIN alredy define @ 13 in \.platformio\packages\framework-arduino-avr\variants\standardstandard\pins_arduino.h
// avr/iom328p.h


// put your setup code here, to run once:
void setup() {

// Serial.begin(9600);

pinMode (LED_BUILTIN,OUTPUT); 
digitalWrite(LED_BUILTIN,HIGH);
delay(5000);
digitalWrite(LED_BUILTIN,LOW);
delay(5000);


//SETUP WATCHDOG TIMER
WDTCSR = B00011000; // (24 = B00011000);//change enable and WDE − also resets
WDTCSR = B00100001; // (33 =  B00100001);//prescalers only − get rid of the WDE and WDCE bit
WDTCSR = (1<<6);

// SETUP WATCHDOG TIMER 
//WDTCSR = (24); // Change enable and WDE - Also resets
//WDTCSR = (33); /// prescalers only -get ride of the WDE and WDCE bit
//WDTCSR |=(1<<6);// enable interrupt modue

// Save Power by writing all Digital IO to LOW
for(int i=0; i<20; i++) {
pinMode(i,OUTPUT);
digitalWrite(i,LOW);
}


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




// put your main code here, to run repeatedly:
void loop() {

digitalWrite(LED_BUILTIN,HIGH);
delay(500);
digitalWrite(LED_BUILTIN,LOW);
delay(500);


// Brown-out Detector DISABLE , − this must be called right before the __asm__ sleep instruction
MCUCR = bit (BODS) | bit (BODSE) | bit(PUD) ;  // Do not forget to diseable ALL IO Pull up by >Turn PUD bit in the MCUCR // TBCHECK ???
MCUCR = bit (BODS);
// MCUCR |=(3<<5); // Set both BODS and BODSE  at the same time
// MCUCR = (MCUCR & ~(1 <<5)) | (1<<6); // then set the BODS bit and clear the BODSE at the same time
  
//sleep_cpu (); // sleep within 3 clock cycles of above
__asm__ __volatile__("sleep");



// Serial.println("OK");
}


//needed for the digital input interrupt
//void digitalInterrupt() {
// }


// watchdog interrupt
//DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
ISR(WDT_vect) {
  
}