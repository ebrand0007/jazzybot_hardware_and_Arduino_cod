
//Pins
int left_motor_pwm=7; //Sabertooth S1
int left_motor_direction=8; //Sabertooth A1
int right_motor_pwm=9; //Sabertooth S2
int right_motor_direction=10; //Sabertooth A2





void setup()
{
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_direction, OUTPUT);
  analogWrite(left_motor_direction,LOW); //left Motor Direction
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_direction, OUTPUT);
  analogWrite(right_motor_direction,HIGH); //Right motor Dirrection
}





void loop() {

     //Set left moto to foward
     digitalWrite(left_motor_direction, HIGH);
     //Set the speed to 30
     analogWrite(left_motor_pwm, abs(40)); //Range 0 to 254

     delay(1000); //1000milisec is 1 sec 
     digitalWrite(left_motor_direction, LOW);
     //Set the speed to 30
     analogWrite(left_motor_pwm, abs(40)); //Range 0 to 254
     delay(1000); //1000milisec is 1 sec 

}
