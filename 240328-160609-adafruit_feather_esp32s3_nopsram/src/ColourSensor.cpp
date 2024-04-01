// 
// MSE 2202 TCS34725 colour sensor example
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Michael Naish
//  Date:     2024 03 05 
//

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data
#define leftmotor 35
#define rightmotor 36

// Plug encoders into these pins
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include "MSE2202_Lib.h"
// Function declarations
void doHeartbeat();
long degreesToDutyCycle(int deg);
void calculateSpeed();
void calculatePID(long targetSpeed,float deltaT);

// Constants
char* rarity;                                         //to store if rock is good or bad
int red, green, blue, colour;                         //save the measured colour values
int rocks=1;                                          //for switch case
int timer=0;
unsigned long rocktime;                               //the time when the rock is collected
const int checktime=900;                              //time that colour sensor has to check colour
const int movetime=750;                               //time servo will move to collect rock
unsigned long wheelCorrect=0;
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725  
int servoPos;                                      // desired servo angle
const int cServoPin          = 5;                 // GPIO pin for servo motor
const int cServoChannel      = 5;                  // PWM channel used for the RC servo motor


// Variables
boolean heartbeatState       = true;                  // state of heartbeat LED
boolean stopped = false;
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds
unsigned long timerTime;

// For PID
unsigned long lastSpeedCalculation = 0;
unsigned long lastTime = 0;
long position = 0;
float wheelSpeeds = 0; // Speeds of each wheel
long previousEncoderCounts = 0; // Store previous counts for all four motors
float deltaT = 0;
double pwm=0;
float ePrev = 0;
float eIntegral = 0;

const int cPWMFreq = 20000;                        // frequency of PWM signal
const int cCountsRev = 1096;                       // encoder pulses per motor revolution
const int cMaxSpeedInCounts = 1600;                // maximum encoder counts/sec


const float kp = 1.5;                              // proportional gain for PID
const float ki = 2;                                // integral gain for PID
const float kd = 0.2;                              // derivative gain for PID

// Change to adjust target speed
long targetSpeed = 432;    // TARGET SPEED



// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Motion Bot=Motion();
Encoders Encoder = Encoders();
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor
  Bot.driveBegin("D1", leftmotor, rightmotor, 37, 38);
  Encoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning);



 //pinMode(cServoPin, OUTPUT);                      // configure servo GPIO for output
  //ledcSetup(7, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  //ledcAttachPin(cServoPin, 7);         // assign servo pin to servo channel

  Bot.servoBegin("S1",7);

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}
/*
if (millis()>3000&&stopped==true&&wheelCorrect+500>millis())
  {

    Bot.Forward("D1",255,255);
  }
  
  else if (millis() - lastTime > 10) {                // wait ~10 ms
      
      if(wheelSpeeds=!0)
      {
          wheelCorrect=millis();
          stopped=false;
      }
      else if(wheelSpeeds==0&&wheelCorrect+500<millis())
      {
          stopped=true;
          wheelCorrect=millis();
      }
*/
void loop() {
  
  //PID Code
  // To change target speed, change the target value at the top of the code
  if (millis()>3000&&stopped==true&&wheelCorrect+500>millis())
  {
    Bot.Forward("D1",255,255);
     Serial.println("backwards");
  }
  else if (millis() - lastTime > 10) {                // wait ~10 ms
    
      wheelCorrect=millis();
      deltaT = ((float) (millis() - lastTime)) / 1000; // compute actual time interval in seconds
      lastTime = millis();                            // update start time for next control cycle
      
      // Update the values for all encoders
      Encoder.getEncoderRawCount();
      position = Encoder.lRawEncoderCount;
      calculateSpeed();
      calculatePID(targetSpeed,deltaT);
      Bot.Reverse("D1",pwm,pwm);
      Serial.printf("pwm %d\n",pwm);
      Serial.printf("speed is %d\n",wheelSpeeds);
      if(wheelSpeeds>10)
      {
        Serial.println("nothing");
          wheelCorrect=millis();
          stopped=false;
      }
      else if(wheelCorrect+500<millis())
      {
         Serial.println("Stopped");
          stopped=true;
          wheelCorrect=millis();
      }
  }
  
  uint16_t r, g, b, c;                                // RGBC values from TCS34725
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
#ifdef PRINT_COLOUR            
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif
  }
  doHeartbeat();                                      // update heartbeat LED
  /*
  switch(timer)
  {
    case 0:
    timerTime=millis();
    timer++;
    break;

    case 1:
    Bot.Reverse("D1",170,170);
    if (timerTime+500<millis())
    {
      timer++;
      timerTime=millis();
    }
    break;

    case 2:
    Bot.Reverse("D1",140 ,140);
    if (timerTime+1500<millis())
    {
      timer=1;
    }
    break;
  }
  */


                                                                //switch statement base on variable declared earlier
switch (rocks)
{
case 1:
servoPos = map(2047.5, 0, 4095, 0, 180);                       //servo positioned to the middle to hold rocks for colour sensor
if(c>210||c<160||b-r>10||g-r>10)                              //checking if there is a rock by colour sensor
{
  rocks=2;                                                    //exiting case 1
}
break;

case 2:
rocktime=millis();                                            //collecting the time that the rock was collected
rocks=3;                                                      //exiting case 2
break;

case 3:
if(millis()-rocktime>checktime)                               //seeing if enough time has passed
{
  rocks=4;                                                    //exiting case 3
  rocktime =millis();
}
if (b>g||r>g||c>200||c<120)                                   //cheking if rock is undesired
{
  rarity="bad";                                               //setting rock variable equal to bad                                          
}
else if(g>b&&g>r)                                             //checking if rock is good
{
  rarity="good";                                              //setting rock variable to good
}
else
{
  rarity="mid";                                               //setting rock value to mid because of false reading
}
break;

case 4:
//these if statements check the rarity of the rock and move servo towards the bad or good pile depending on the result
if (rarity=="good")                                           
{
  servoPos = map(3500, 0, 4095, 0, 180);                      //setting servo to move rock to good pile
  //Serial.println("rock is good");
}
if(rarity=="bad")
{
  servoPos = map(500, 0, 4095, 0, 180);                       //setting servo to move rock to bad pile
}
if(rarity=="mid")
{
  servoPos = map(2047.5, 0, 4095, 0, 180);                    //servo doesn't move because of false reading
}
if (millis()>rocktime+movetime)                               //time servo will be moved into the position
{
  rocks=1;                                                    //moving to start of switch to redo the loop and reset
}
break;
}

  //Serial.printf("servo pos %d\n",degreesToDutyCycle(servoPos));

  Bot.ToPosition("S1",(int)degreesToDutyCycle(servoPos));
  //ledcWrite(7, degreesToDutyCycle(servoPos)); // set the desired servo position
} 
// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}
long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                     // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                    // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;              // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}

// PID functions
void calculateSpeed() {
    unsigned long currentTime = millis(); // Current time in milliseconds
    static unsigned long lastSpeedCalculation = 0; // Last time we calculated speed
    
    // Calculate elapsed time since last speed calculation in seconds
    float elapsedTime = (currentTime - lastSpeedCalculation) / 1000.0; // Convert milliseconds to seconds

    if (elapsedTime >= 0.1) { // Only calculate if at least 100 ms have passed to avoid division by a very small number
        
            // Calculate speed as change in encoder counts divided by elapsed time
            // Speed could be in counts per second or any other relevant unit
            wheelSpeeds = (position - previousEncoderCounts) / elapsedTime;
            
            // Update previousEncoderCounts for next calculation
            previousEncoderCounts = position;
        
        
        // Update the last speed calculation time
        lastSpeedCalculation = currentTime;
    }
}
void calculatePID(long targetSpeed,float deltaT){
// 1200 is a reasonable target speed for now
   float e = 0;
   float dedt = 0;
   float u = 0;                                     // PID control signal

   Serial.println(e);
   e = targetSpeed - wheelSpeeds;                              // position error
   Serial.println(e);

   dedt = ((float) e - ePrev) / deltaT;           // derivative of error
   eIntegral += e * deltaT;            // integral of error (finite difference)

   Serial.println(e);
   // Set cap for integral
   if(eIntegral > 326){
      eIntegral = 326;
   }
   Serial.println(e);
   Serial.printf("int %f dedt %f e %f eprev %d\n",eIntegral,dedt,e,ePrev);
   u = kp * e + kd * dedt + ki * eIntegral;       // compute PID-based control signal
   ePrev = e;                                     // store error for next control cycle

//Serial.printf("integral = %d, u = %f, e = %d, dedt = %f\n",eIntegral[motorNumber], u, e, dedt);

   int pwmSignal = map(u, -cMaxSpeedInCounts, cMaxSpeedInCounts, 0, 255); // Assuming symmetric control range for simplification
   pwmSignal = constrain(pwmSignal, 0, 255); // Ensure PWM signal stays within valid range
   //Serial.printf("Motor number %d, error %f, pwm %d\n",motorNumber,e,pwmSignal); 
   pwm = pwmSignal; // Apply the computed PWM value
   //Serial.printf("PWM has been set to %f\n",pwm[motorNumber]);
}