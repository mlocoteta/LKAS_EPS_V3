#include "io.h"
#include "common.h"

// Sets up LEDs
void setupGPIO(void){
    pinMode(errorLED,OUTPUT);
	pinMode(statusLED, OUTPUT);
	digitalWrite(errorLED, LOW);
	digitalWrite(statusLED, LOW);
}