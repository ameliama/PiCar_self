 /*
 * Author: AnZou(anzou@wustl.edu)
 * PID speed control for BLCD
 * BLDC(+ESC): TRACKSTAR
 * Encoder: YUMOE6A2
 * Controller: Arduino UNO R3
*/

/*
 * Pin Definition
 * Pin2 for encoder: interrupt signal
 * Pin13 for BLDC ESC: PPM signal
 * 
 *Edited by Chufan Chen and Amelia Ma
 *Fall 2018
*/

#define reference_signal 35 // set the speed in RPS here

//set the pid control parameters
#define K_p 0.1
#define K_i 0
#define K_d 0.001
#include <Servo.h>//servo motor

Servo myservo;

int h = 1;//integer for case discussion
double encoder_counter;
double motor_speed; //rotations per second

double control_signal;//the output signal from the whole system
double e_k; // variable for P
double e_k1; // variable for I (not in use)
double e_k2; // variable for D (not in use)


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(9);
  myservo.writeMicroseconds(1500);//set servo to mid-point
  e_k = 0;
  e_k1 = 0;
  e_k2 = 0;
  control_signal = 150; //PWM

  Serial.println("Start!");//start calibration
  pinMode(13, OUTPUT);
  delay(3000);

  Serial.println("Calibrating ESC for positive RPM");
  calibration(500); // Calibrating ESC for +ve RPM
  Serial.println("Calibrating ESC for negative RPM");
  calibration(-500); // Calibrating ESC for -ve RPM
  Serial.println("Calibrating ESC for zero RPM");
  calibration(0); // Calibrating ESC for 0 RPM
}

void loop() {
  // put your main code here, to run repeatedly:
    if(Serial.available()){//if serial communication reads something
    h = Serial.read()-'0';//get the byte from reading and update integer h
    switch(h){
      case 1 ://when byte=1
      myservo.write(115);//turn left
      //delay(100);
      control_signal = control_signal + increment_pid();//calculate new signal
      if(control_signal > 180)//set the control signal to 180 to avoid rapid speeding
        {
          control_signal = 180;
        }
      PPM_output(control_signal);
      break;
      
      case 2 :
      
      myservo.write(75);//turn right
      //delay(100);
      
      control_signal = control_signal + increment_pid();
      if(control_signal > 180)
        {
          control_signal = 180;
        }
      PPM_output(control_signal);
      break;
      
      case 3 :
      myservo.write(90);
      //delay(50);
      control_signal = control_signal + increment_pid();
      if(control_signal > 180)
        {
          control_signal = 180;
        }
      PPM_output(control_signal);
      break; 
    }
    }
    
  //Troubleshooting
  //control_signal = control_signal + increment_pid(); //calculate the control signal with input signal+error signal
  //PPM_output(control_signal); //output the signal calculated
  //High and low boundary
  //Serial.println("motor speed:");
  //Serial.println(motor_speed);
  //Serial.println("output:");
  //Serial.println(control_signal);
  

/*if the main loop doesn't work, command the main loop then run  
the following loop to move straight*/

    control_signal = control_signal + increment_pid();
    if(control_signal > 180)
      {
        control_signal = 180;
      }
    PPM_output(control_signal); //should be <= 180
    //Serial.println(control_signal);


}
void speed_measure() {
  // measures the motor rpm using quadrature encoder
  encoder_counter = 0;
  attachInterrupt(0, encoder, RISING);
  delay(50); // Wait for 50 ms  
  detachInterrupt(0);
  motor_speed =  encoder_counter * 20 / 200;
}


void encoder()  
{  
  // increments the encoder count when a rising edge
  // is detected by the interrupts 
  encoder_counter = encoder_counter + 1;
}

double increment_pid()
{
  double increment;
  speed_measure();
  e_k2 = e_k1;
  e_k1 = e_k;
  e_k = reference_signal - motor_speed;
  
  if((e_k>5)||(e_k<-5))
  {
    increment = K_p*e_k;
  }
  else
  {
    increment = 0;  
  }
  
  //increment boundary/saturation
  if(increment > 1)
  {
   increment = 1; 
  }
  
  if(increment < -1)
  {
   increment = -1; 
  }
  
  return increment;
}

void calibration(int command)
{
  int i = 400;  
  while(i>1)
  {
    int rate = 1500 + command;
    digitalWrite(13, HIGH);
    delayMicroseconds(rate);
    digitalWrite(13, LOW);
    delayMicroseconds(10000);
    delayMicroseconds(10000 - rate);
    i = i-1;
  }  
}

void PPM_output(int command) {
  int rate = 1500 + command;
  digitalWrite(13, HIGH);
  delayMicroseconds(rate);
  digitalWrite(13, LOW);
  delayMicroseconds(10000);
  delayMicroseconds(10000 - rate);
}
