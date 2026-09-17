#include <Servo.h> 
#include <avr/interrupt.h> 

#define LDR1 A0 
#define LDR2 A1
#define eps 10
#define ledPin 12


int Spoint =  0;
Servo servo;

int lastPositionChange = 0;
String lastMessage = "";