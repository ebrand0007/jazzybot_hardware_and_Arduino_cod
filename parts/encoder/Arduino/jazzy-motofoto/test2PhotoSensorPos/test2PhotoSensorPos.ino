//Turn a DC motor into a stepper motor with an encoder disc
//
//
//The circuit:

//EncoderA pin to digital pin 2
//EncoderB pin to digital pin 3
//Source & info at: www.HomoFaciens.de/technics-computer-arduino-uno_en_navion.htm



// include the library code:
#include <util/delay.h>

int stepStatus = 0;
int sensorStatusA = 0;
int sensorStatusB = 0;
int count = 0;

unsigned long millisStart = 0;
signed long actualPoint = 0;
byte readByte = 0;

int ticsPerMotorRev=8;
double gearRatio=17.7375;
int ticsPerWheelRotation=142; //ticsPerMotorRev*gearRatio;

#define SENSOR_A 2 //PhotoEncoderA
#define SENSOR_B 3 //PhotoEncoderB

#define MOTOR_DIRECTION 7
#define MOTOR_PWM 6

#define STEP_MARGIN 1L        //10 - 1000 (1)
#define MIN_DUTYCYCLE 50      //0 - 255 (125)
#define MAX_DUTYCYCLE 255     //0 - 255 (255)
#define P_FRACTION 10.0        //Proportional factor of control loop 0.001 - 10.0 (1.0)


/*
void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('X');   // send a capital X
    delay(300);
  }
}
*/

void setup() {
  
  pinMode(SENSOR_A, INPUT); //PhotoEncoderA
  pinMode(SENSOR_B, INPUT); //PhotoEncoderB
//  pinMode(MOTOR_DIRECTION, OUTPUT);  
//  pinMode(MOTOR_PWM, OUTPUT);    
  
  count = 0;
  // start serial port 
  Serial.begin(9600);
  millisStart = millis();
}

void loop() {    

  
      sensorStatusA = digitalRead(SENSOR_A);
      sensorStatusB = digitalRead(SENSOR_B);
    
      if(sensorStatusB == 0 && sensorStatusA == 0){
        if(stepStatus == 1){
          actualPoint--;
        }
        if(stepStatus == 3){
          actualPoint++;
        }
        stepStatus = 0;
      }
    
      if(sensorStatusB == 1 && sensorStatusA == 0){
        if(stepStatus == 0){
          actualPoint++;
        }
        if(stepStatus == 2){
          actualPoint--;
        }
        stepStatus = 1;
      }
    
      if(sensorStatusB == 1 && sensorStatusA == 1){
        if(stepStatus == 3){
          actualPoint--;
        }
        if(stepStatus == 1){
          actualPoint++;
        }
        stepStatus = 2;
      }
    
      if(sensorStatusB == 0 && sensorStatusA == 1){
        if(stepStatus == 2){
          actualPoint++;
        }
        if(stepStatus == 0){
          actualPoint--;
        }
        stepStatus = 3;
      }


  
    //update the serial port once a sec
    if(millis() - millisStart < 1000){
     
      Serial.print("actualPoint:");
      Serial.print(actualPoint);
      Serial.print("\n");
      millisStart+=1000;
    }
   
}
