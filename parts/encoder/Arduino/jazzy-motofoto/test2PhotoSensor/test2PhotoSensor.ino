// photo interrupter module

int Led = 13 ;// define LED Interface

int buttonpinA = 3; // define the photo interrupter sensor interface
int buttonpinB = 4; // define the photo interrupter sensor interface

int valA ;// define numeric variables val for encoder A
int valB ;// define numeric variables val for encoder A

void setup ()

{
 Serial.begin(9600);
 pinMode (buttonpinA, INPUT) ;// define the photo interrupter sensor output interface  
 pinMode (buttonpinB, INPUT) ;// define the photo interrupter sensor output interface 

}

void loop ()

{

 valA = digitalRead (buttonpinA) ;// digital interface will be assigned a value of 3 to read val
 valB = digitalRead (buttonpinB) ;// digital interface will be assigned a value of B to read val

 if (valA == HIGH) // When the light sensor detects a signal is interrupted, LED flashes

 {
   Serial.print("A-Encoder Tick \n");

 }

 if (valB == HIGH) // When the light sensor detects a signal is interrupted, LED flashes

 {
   Serial.print("B-Encoder Tick \n");

 }



}
