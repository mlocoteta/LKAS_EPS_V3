#include "io.h"
#include "common.h"

void setupGPIO(void){
    pinMode(errorLED,OUTPUT);
	pinMode(statusLED, OUTPUT);
	digitalWrite(errorLED, LOW);
	digitalWrite(statusLED, LOW);
}