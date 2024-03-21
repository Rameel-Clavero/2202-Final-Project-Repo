
//  To program and use ESP32-S3
//   
//  Tools->:
//  Board: "Adafruit Feather ESP32-S3 No PSRAM"
//  Upload Speed: "921600"
//  USB CDC On Boot: "Enabled"
//  USB Firmware MSC on Boot: "Disabled"
//  USB DFU On Bot: "Disabled"
//  Upload Mode:"UART0/Hardware CDC"
//  SPU Frequency: "240MHz (WiFi)"
//  Flash Mode: "QIO 80MHz"
//  Flash SIze: "4MB (32Mb)"
//  Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//  Core Debug Level: "Verbose"
//  PSRAM: 'Disabled"
//  Arduino Runs On: "Core 1"
//  Events Run On: "Core 1"
//
//  To program, press and hold the reset button then press and hold program button, release the reset button then 
//  release the program button 
//

// Uncomment keywords to enable debugging output
//#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>



// Function declarations
void calculatePID(long targetSpeed,float deltaT,int motorNumber);
void calculateSpeed();
void botForward();
void botReverse();
void botStop();
void botLeft();
void botRight();


// Port pin constants
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
// Motor Controler Two
#define LEFT_MOTOR_C          4
#define LEFT_MOTOR_D          5
#define RIGHT_MOTOR_C         6
#define RIGHT_MOTOR_D         7


#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)

#define ENCODER_LEFT_C       13                                                // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_D       14                                               // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_C      17                                               // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_D      18                                              // right encoder B signal is connected to pin 20 GPIO12 (J12)


#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = 255;                                       // PWM value for maximum speed



// Variables
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag
boolean timeUp200msec = false;                                                 // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned char driveIndex = 0;                                                      // state index for run mode
unsigned int modePBDebounce;                                                   // pushbutton debounce timer count
unsigned int potClawSetpoint;                                                  // desired position of claw servo read from pot
unsigned int potShoulderSetpoint;                                              // desired position of shoulder servo read from pot
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;                                              // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;                                           // 200 millisecond timer count in milliseconds
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;


// PID code
double pwm[]={0,0,0,0};
float ePrev[] = {0,0,0,0};
const int cPWMFreq = 20000;                        // frequency of PWM signal
const int cCountsRev = 1096;                       // encoder pulses per motor revolution
const int cMaxSpeedInCounts = 1600;                // maximum encoder counts/sec

const float kp = 1.5;                              // proportional gain for PID
const float ki = 2;                                // integral gain for PID
const float kd = 0.2;                              // derivative gain for PID

unsigned long lastTime = 0;
long targetSpeed = 1200;
long position[] = {0,0,0,0};
float eIntegral[4] = {0, 0, 0, 0};

// For wheel speed 
unsigned long lastSpeedCalculation = 0;
const unsigned long speedCalculationInterval = 100; // Calculate speed every 100 ms
long previousEncoderCounts[4] = {0, 0, 0, 0}; // Store previous counts for all four motors
float wheelSpeeds[4] = {0.0, 0.0, 0.0, 0.0}; // Speeds of each wheel

float deltaT = 0;


// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(NEO_RGB + NEO_KHZ800);                            

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)

 Motion Bot = Motion();                                                         // Instance of Motion for motor control

   Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
   Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data
   Encoders LeftEncoderRear = Encoders();
   Encoders RightEncoderRear = Encoders();


void setup() {
   #if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
   Serial.begin(115200);
   #endif
  

   
  // Set up motors
   //Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B);
   //Bot.driveBegin("D2",LEFT_MOTOR_C,LEFT_MOTOR_D,RIGHT_MOTOR_C,RIGHT_MOTOR_D);

   // Set up motors - switch lefts and rights so my wheels are at the front 
   //       Left  Right
   // Front M1    M2
   // Rear  M3    M4

   
   Bot.motorBegin("M3",LEFT_MOTOR_C,LEFT_MOTOR_D);
   Bot.motorBegin("M1",LEFT_MOTOR_A,LEFT_MOTOR_B);
   Bot.motorBegin("M4",RIGHT_MOTOR_C,RIGHT_MOTOR_D);
   Bot.motorBegin("M2",RIGHT_MOTOR_A,RIGHT_MOTOR_B);
   

  //Bot.driveBegin("D1",LEFT_MOTOR_A,LEFT_MOTOR_B,RIGHT_MOTOR_A,RIGHT_MOTOR_B);
  //Bot.driveBegin("D2",LEFT_MOTOR_C,LEFT_MOTOR_D,RIGHT_MOTOR_C,RIGHT_MOTOR_D);

  
   // Set up encoders
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning );
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning);
   LeftEncoderRear.Begin(ENCODER_LEFT_C,ENCODER_LEFT_D,&Bot.iLeftMotorRunning);
   RightEncoderRear.Begin(ENCODER_RIGHT_C,ENCODER_RIGHT_D,&Bot.iRightMotorRunning);

}

void loop() {
   long pos[] = {0, 0};                                                        // current motor positions
   deltaT = 0;
   long e = 0;
   float dedt = 0;                                  // rate of change of position error (de/dt)
   float eIntegral = 0;                             // integral of error 
   float u = 0;                                     // PID control signal
   int dir = 1;                                     // direction that motor should turn

   // Update the values for all encoders
   LeftEncoder.getEncoderRawCount();
   RightEncoder.getEncoderRawCount();
   LeftEncoderRear.getEncoderRawCount();
   RightEncoderRear.getEncoderRawCount();

   // Set position array to updated values
   // Signs account for directions motors turn as they are on the car
   position[0] = LeftEncoder.lRawEncoderCount;
   position[1] = -RightEncoder.lRawEncoderCount;
   position[2] = LeftEncoderRear.lRawEncoderCount;
   position[3] = -RightEncoderRear.lRawEncoderCount;

   currentMicros = micros();                                                   // get current time in microseconds
   if ((currentMicros - previousMicros) >= 1000) {                             // enter when 1 ms has elapsed
      previousMicros = currentMicros;                                          // record current time in microseconds
   }
   // Drive forward using PID
   if (millis() - lastTime > 10) {                // wait ~10 ms
      deltaT = ((float) (millis() - lastTime)) / 1000; // compute actual time interval in seconds
      lastTime = millis();                            // update start time for next control cycle
      //       Left  Right
      // Front M1    M2
      // Rear  M3    M4


      if(timerCount3sec + 3000 < millis()){
         timeUp3sec = true;
      }
      if(timerCount200msec + 200<millis()){
         timeUp200msec = true;
      }

      switch(driveIndex){

      case 0:
      timeUp3sec = false;
      timerCount3sec = millis();
      driveIndex++;
      break;

      case 1:
      Serial.println("forward");
      botForward();
      if(timeUp3sec == true){
         timeUp200msec = false;
         timerCount200msec = millis();
         driveIndex++;
      }
      break;

      case 2:
      Serial.println("stop");
      botStop();
      if(timeUp200msec == true){
         timeUp3sec = false;
         timerCount3sec = millis();
         driveIndex++;
      }
      break;

      case 3:
      Serial.println("reverse");
      botReverse();
      if(timeUp3sec == true){
         timeUp3sec = false;
         timerCount3sec = millis();
         driveIndex++;
      }
      break;

      case 4:
      Serial.println("stop");
      botStop();
      if(timeUp3sec == true){
         timeUp3sec = false;
         timerCount3sec = millis();
         driveIndex++;
      }
      break;

      case 5:
      Serial.println("Left");
      botLeft();
      if(timeUp3sec == true){
         timeUp3sec = false;
         timerCount3sec = millis();
         driveIndex++;
      }
      break;

      case 6:
      Serial.println("Stop");
      botStop();
      if(timeUp3sec == true){
         timeUp3sec = false;
         timerCount3sec = millis();
         driveIndex++;
      }
      break;

      case 7:
      Serial.println("right");
      botRight();
      if(timeUp3sec == true){
         timeUp3sec = false;
         timerCount3sec = millis();
         driveIndex++;
      }
      break;

      case 8:
      Serial.println("stop");
      botStop();
      if(timeUp3sec == true){
         timeUp3sec = false;
         timerCount3sec = millis();
         driveIndex = 0;
      }
      break;
      }
     
      
      // ALL OF MY FUNCTIONS AT THE BOTTOM WORK ON THEIR OWN
      // ALL MOTORS GO FORWARD AND BACKWARD
      /*
      Bot.SetMotorPWMAndDirection("M1",255,false);
      delay(5000);
      Bot.SetMotorPWMAndDirection("M1",255,true);
      delay(5000);
      Bot.SetMotorPWMAndDirection("M2",255,true);
      delay(5000);
      Bot.SetMotorPWMAndDirection("M2",255,false);
      delay(5000);
      Bot.SetMotorPWMAndDirection("M3",255,false);
      delay(5000);
      Bot.SetMotorPWMAndDirection("M3",255,true);
      delay(5000);
      Bot.SetMotorPWMAndDirection("M4",255,true);
      delay(5000);
      Bot.SetMotorPWMAndDirection("M4",255,false);
      */

      //Serial.printf("speed 0 %f, PWM 0 %f\n",wheelSpeeds[0],pwm[0]);
      //Serial.printf("speed 1 %f, PWM 1 %f\n\n",wheelSpeeds[1],pwm[1]);
   }
}


   //       Left  Right
   // Front 0    1
   // Rear  2    3

   // Final value gets put into pwm array
void calculatePID(long targetSpeed,float deltaT,int motorNumber){
// 1200 is a reasonable target speed for now
   float e = 0;
   float dedt = 0;
   float u = 0;                                     // PID control signal
   


   e = targetSpeed - wheelSpeeds[motorNumber];                              // position error
   // Reverse the error while driving backwards so motors dont stop
      if(targetSpeed < 0){
      targetSpeed = targetSpeed*-1;
      e = e*-1;
   }
   dedt = ((float) e - ePrev[motorNumber]) / deltaT;           // derivative of error
   eIntegral[motorNumber] += e * deltaT;            // integral of error (finite difference)
   // Set cap for integral
   if(eIntegral[motorNumber] > 800){
      eIntegral[motorNumber] = 800;
   }
   u = kp * e + kd * dedt + ki * eIntegral[motorNumber];       // compute PID-based control signal
   ePrev[motorNumber] = e;                                     // store error for next control cycle

//Serial.printf("integral = %d, u = %f, e = %d, dedt = %f\n",eIntegral[motorNumber], u, e, dedt);

   int pwmSignal = map(u, -cMaxSpeedInCounts, cMaxSpeedInCounts, 0, 255); // Assuming symmetric control range for simplification
   pwmSignal = constrain(pwmSignal, 0, 255); // Ensure PWM signal stays within valid range
   //Serial.printf("Motor number %d, error %f, pwm %d\n",motorNumber,e,pwmSignal); 
   pwm[motorNumber] = pwmSignal; // Apply the computed PWM value
   //Serial.printf("PWM has been set to %f\n",pwm[motorNumber]);
}
void calculateSpeed() {
    unsigned long currentTime = millis(); // Current time in milliseconds
    static unsigned long lastSpeedCalculation = 0; // Last time we calculated speed
    
    // Calculate elapsed time since last speed calculation in seconds
    float elapsedTime = (currentTime - lastSpeedCalculation) / 1000.0; // Convert milliseconds to seconds

    if (elapsedTime >= 0.1) { // Only calculate if at least 100 ms have passed to avoid division by a very small number
        for (int motorNumber = 0; motorNumber < 4; motorNumber++) {
            // Calculate speed as change in encoder counts divided by elapsed time
            // Speed could be in counts per second or any other relevant unit
            wheelSpeeds[motorNumber] = (position[motorNumber] - previousEncoderCounts[motorNumber]) / elapsedTime;
            
            // Update previousEncoderCounts for next calculation
            previousEncoderCounts[motorNumber] = position[motorNumber];
        }
        
        // Update the last speed calculation time
        lastSpeedCalculation = currentTime;
        
        /*
         Serial.printf("Speeds and pwm: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(wheelSpeeds[i]);
            Serial.print(" ");
            Serial.print(pwm[i]);
            Serial.print(" ");
        }
        Serial.println();
        */
        
    }
}
void botForward(){
      // Update the current speed for all wheels
      calculateSpeed();

      // Calculate all pwm values
      calculatePID(targetSpeed,deltaT,0);
      calculatePID(targetSpeed,deltaT,1);
      calculatePID(targetSpeed,deltaT,2);
      calculatePID(targetSpeed,deltaT,3);

      Bot.SetMotorPWMAndDirection("M1",pwm[0],false);
      Bot.SetMotorPWMAndDirection("M2",pwm[1],true);
      Bot.SetMotorPWMAndDirection("M3",pwm[2],false);
      Bot.SetMotorPWMAndDirection("M4",pwm[3],true);
}
void botStop(){
      Bot.SetMotorPWMAndDirection("M1",0,false);
      Bot.SetMotorPWMAndDirection("M2",0,true);
      Bot.SetMotorPWMAndDirection("M3",0,false);
      Bot.SetMotorPWMAndDirection("M4",0,true);
}
void botReverse(){
   // Update the current speed for all wheels
      calculateSpeed();

      // Calculate all pwm values
      calculatePID(-targetSpeed,deltaT,0);
      calculatePID(-targetSpeed,deltaT,1);
      calculatePID(-targetSpeed,deltaT,2);
      calculatePID(-targetSpeed,deltaT,3);

      Bot.SetMotorPWMAndDirection("M1",pwm[0],true);
      Bot.SetMotorPWMAndDirection("M2",pwm[1],false);
      Bot.SetMotorPWMAndDirection("M3",pwm[2],true);
      Bot.SetMotorPWMAndDirection("M4",pwm[3],false);
}
void botLeft(){
      // Update the current speed for all wheels
      calculateSpeed();

      // Calculate all pwm values
      calculatePID(-targetSpeed,deltaT,0);
      calculatePID(targetSpeed,deltaT,1);
      calculatePID(-targetSpeed,deltaT,2);
      calculatePID(targetSpeed,deltaT,3);

      Bot.SetMotorPWMAndDirection("M1",pwm[0],true);
      Bot.SetMotorPWMAndDirection("M2",pwm[1],true);
      Bot.SetMotorPWMAndDirection("M3",pwm[2],true);
      Bot.SetMotorPWMAndDirection("M4",pwm[3],true);
}
void botRight(){
      // Update the current speed for all wheels
      calculateSpeed();

      // Calculate all pwm values
      calculatePID(targetSpeed,deltaT,0);
      calculatePID(-targetSpeed,deltaT,1);
      calculatePID(targetSpeed,deltaT,2);
      calculatePID(-targetSpeed,deltaT,3);

      Bot.SetMotorPWMAndDirection("M1",pwm[0],false);
      Bot.SetMotorPWMAndDirection("M2",pwm[1],false);
      Bot.SetMotorPWMAndDirection("M3",pwm[2],false);
      Bot.SetMotorPWMAndDirection("M4",pwm[3],false);
}