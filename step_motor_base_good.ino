

//---------------------------------------------------------------------------------------------------------------
//                                  DEFINITIONS FOR ROS STANDARD MESSAGE PUBLISHERS
//---------------------------------------------------------------------------------------------------------------
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h> /* Variables are declared for storing the ROS data types; this is for integers  */
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;
std_msgs::String my_string_data;

/* PUBLISHER BLOCK: One topic for each measurement that I will transfer to ROS */
std_msgs::Float32 AngleMotor1_H; /*Position of motor 1 horizontal, translated into degrees */
ros::Publisher Motor1_Head("/Motor_1", &AngleMotor1_H  ); /*publisher for the same */

std_msgs::Float32 AngleMotor2_V; /*Position of motor 2 vertical, translated into degrees */
ros::Publisher Motor2_Head("/Motor_2", &AngleMotor2_V  ); /*publisher for the same */

std_msgs::Float32 LIDAR_Distance; /*distance measured by the LIDAR, in cms */
ros::Publisher LIDAR_Head("/Motor_2", &LIDAR_Distance  ); /*publisher for the same */





//LIDAR initialisation, for Arduino Mega.
int Measured_Distance;                    // LiDAR actually measured distance value
int Signal_Strength;                      // LiDAR signal strength
int check;                                // check numerical value storage
int i;                                    // counter to input data into the UART array
int uart[9];                              // store nine sets of data measured by LiDAR
const int HEADER=0x59;                    // data package frame header

/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.
 */

int PhotoDRead; // create the variable to receive values from the photodiode

// Wiring for Stepper Motor 1:
// Pin 8 to IN1 on the ULN2003 driver
// Pin 10 to IN2 on the ULN2003 driver
// Pin 9 to IN3 on the ULN2003 driver
// Pin 11 to IN4 on the ULN2003 driver

#include "Stepper.h"
#define STEPS_PER_REV 2048 // change this to fit the number of steps per revolution for your motor 2048 steps for a full revolution
#define STEPS_PER_REV_2 2048 // change this to fit the number of steps per revolution for the Stepper 2 motor on the system's head 2048 steps for a full revolution
// 2048 steps is equivalent to 360 degrees, therefore 5.68 steps=1 degree.1 step is 0.176 degrees.


int counter;
int stepcount;
int SwitchProcessOn=0;

//Stepper 1 Pinset
int in1Pin = 8;
int in2Pin = 9;
int in3Pin = 10;
int in4Pin = 11;

//Stepper 2 Pinset
int in5Pin = 4;
int in6Pin = 5;
int in7Pin = 6;
int in8Pin = 7;

int inPHPin = A0; //This is an Analog Port.

//Block for the reset button on the LIDAR's head
const int buttonPin = 2;  // the number of the pushbutton pin
const int ledPin = 13;    // the number of the LED pin
int buttonState = 0;  // variable for reading the pushbutton status
float accumHeadTilt;
float IncremHeadStep= -5.68; //start one step head up
int directionUpDown=1;
float StopMax=15*IncremHeadStep;

Stepper myStepper(STEPS_PER_REV, in1Pin, in2Pin, in3Pin, in4Pin);
Stepper myStepper2(STEPS_PER_REV_2, in5Pin, in6Pin, in7Pin, in8Pin);
//myStepper2.setSpeed(5);

void setup() {
  // Initialise A0, which carries the signal from the IR.
  pinMode(inPHPin, INPUT);
  
  // Initialise the zero point button on the LIDAR's head
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  // set the speed at 10 rpm for Stepper 1:
  myStepper.setSpeed(15);
  // initialize the serial port:
  Serial.begin(57600); // default 9600 baud
  Serial1.begin(115200);
  delay(100);
  Serial.println(PhotoDRead);
  ZeroButtonLidar_head ();
}

void loop() {
    if (SwitchProcessOn==0)
      {
        Serial.print("The Switch status is: ");
        Serial.println(SwitchProcessOn);
        findzeropoint();
        SwitchProcessOn=findzeropoint_value();
        //Serial.print("The Switch status is: ");
        //Serial.println(SwitchProcessOn);
        adjustback(); 
      }
  else if (SwitchProcessOn==1) //When Processon=1, then the head has found its zero point and it is starting the process.
  {
      myStepper.setSpeed(15);
      int processCounter=20;
      for (processCounter; processCounter <= STEPS_PER_REV/4; processCounter++) 
        {
          myStepper.step(-1); 
          LidarSignal ();
          // Step clockwise one whole revolution
          //Serial.println(processCounter);
          //Serial.println(-STEPS_PER_REV/4);
          delay(20);
        }
      //processCounter=0;
      delay(100);
      tiltup(); // activate tilt up
      for (processCounter; processCounter >= 30; processCounter--) 
        {
          myStepper.step(+1);       // Step clockwise one whole revolution
          LidarSignal ();
          //Serial.println(processCounter);
          delay(20);
        }
        delay(100);
        tiltup(); // activate tilt up
        //Serial.println("FZP=1");
        //myStepper.step(-STEPS_PER_REV/4);
        //delay(100);
        //myStepper.step(STEPS_PER_REV/4);
        //delay(100);
  } 
 }


void findzeropoint()
{
  myStepper.setSpeed(3);
  for (counter = 1; counter <= STEPS_PER_REV; counter+32) {
  myStepper.step(16);       // Step clockwise one whole revolution
  PhotoDRead=analogRead(inPHPin);
  Serial.println(PhotoDRead);
  delay(300);
  if (PhotoDRead>120) {
    Serial.println("FOUND");
    counter=STEPS_PER_REV+1;
    }
  }
  //Serial.println("Exit The Loop");
  findzeropoint_value();
  Serial.println("Return Function");
}

int findzeropoint_value()
{
    Serial.print("Returning Value: ");
    Serial.println("SwitchProcessOn");
  return 1;
  
}

void adjustback()
{
  myStepper.step(192);       // Step clockwise one whole revolution 32*6
  delay(300);
}

//This function zeroes the LIDAR head
void ZeroButtonLidar_head ()
{
  // read the state of the pushbutton value:
  myStepper2.setSpeed(5);
  buttonState = digitalRead(buttonPin);
    do {
      myStepper2.setSpeed(5);
      myStepper2.step(5);
      delay(100);
      //Serial.println("Head Adjusting");
      //Serial.println(buttonState);
      buttonState = digitalRead(buttonPin);
      if (buttonState != 0)
          {
            buttonState =2;
          }
      } while (buttonState == 0);
      
      {
      //Serial.println("Head Raising");
      //Serial.println(buttonState);
      myStepper2.step(-200);
      }
} 

//This fuction tilts the head up, as the horizontal servo sweeps left right

void tiltup()
{
  myStepper2.setSpeed(5);
  IncremHeadStep=IncremHeadStep*directionUpDown;
  myStepper2.step(IncremHeadStep); // minus in this case means up, the head raises one degree.
  accumHeadTilt=accumHeadTilt + IncremHeadStep;
  Serial.print("StopMax: ");
  Serial.println(StopMax);
  Serial.print("accumHeadTilt: ");
  Serial.println(accumHeadTilt);
  Serial.print("IncremHeadStep: ");
  Serial.println(IncremHeadStep);
  Serial.print("directionUpDown: ");
  Serial.println(directionUpDown);
  if(accumHeadTilt<StopMax)
  {
    directionUpDown=-1;
  }
  else if(accumHeadTilt>=StopMax)
  {
    directionUpDown=1;
  }
 
    
}


void LidarSignal ()
{
  {
  if (Serial1.available())                    //check whether the serial port has data input
  {
    //Serial.print("SerialPortHasInput");
    if(Serial1.read()==HEADER)        // determine data package frame header 0x59
    {
      uart[0]=HEADER;
      if(Serial1.read()==HEADER)     //determine data package frame header 0x59
      {
        uart[1]=HEADER;
        for(i=2;i<9;i++)                        // store data to array
        {
          uart[i]=Serial1.read();
        }
      check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
      if(uart[8]==(check&0xff))                            // check the received data as per protocols
      {
        Measured_Distance=uart[2]+uart[3]*256;                    // calculate distance value
        Signal_Strength=uart[4]+uart[5]*256;                      // calculate signal strength value
        //Serial.print("Measured Distance = ");
        //Serial.print(Measured_Distance);                          // output LiDAR tests distance value
        //Serial.print('\t');
        //Serial.print("Signal Strength = ");
        //Serial.print(Signal_Strength);                           // output signal strength value
        //Serial.print('\n');
       }
     }
   }
 }
        //Serial.print("Measured Distance = ");
        //Serial.print(Measured_Distance);                        // output LiDAR tests distance value
        //Serial.print('\t');
        //Serial.print("Signal_Strength = ");
        //Serial.print(Signal_Strength);                          // output signal strength value
        //Serial.print('\n');
}
}
