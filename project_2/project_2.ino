/*
*************************************************************************
 Project 1, ME 4543 Mechatronics
 Code Written By: Jason Crandall
 April 2014
 *************************************************************************
 */

#include <AFMotor.h> //Motor Library for motor functions
#include <Servo.h> //Servo Library
#include <CMUcam4.h>
#include <CMUcom4.h>

//CAMERA & COLOR PROPERTIES********************************************************
#define RED_MIN 75
#define RED_MAX 255
#define GREEN_MIN 20
#define GREEN_MAX 60
#define BLUE_MIN 20
#define BLUE_MAX 60
#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds
#define PIXELS_THRESHOLD 1

// The percent of tracked pixels needs to be greater than this 0=0% - 255=100%.
#define CONFIDENCE_THRESHOLD 1 // The percent of tracked pixels in the bounding box needs to be greater than this 0=0% - 255=100%.
#define NOISE_FILTER_LEVEL 2 // Filter out runs of tracked pixels smaller than this in length 0 - 255.

int a; 
int b;
int c = 0;
int p;

int clDist = 70; //Close distance
int farDist = 105; //Far distance
int prevping = 0; //Previous Ping
int prevprevping; //2nd last Ping
int fSpeed = 255; //Fast Speed
int sSpeed = 150; //Slow Speed
int pancount = 0;
CMUcam4_tracking_data_t data;

CMUcam4 cam(CMUCOM4_SERIAL2);
//Declaring each motor for a 4WD vehicle
AF_DCMotor motor1(1, MOTOR12_8KHZ); //Back Right
AF_DCMotor motor2(2, MOTOR12_8KHZ); //Front Right
AF_DCMotor motor3(3, MOTOR34_8KHZ); //Front Left
AF_DCMotor motor4(4, MOTOR34_8KHZ); //Back Left
Servo myservo;
const int pingPin = 48; //Ping sensor signal is in Pin # 48
const int ledPin = 38; //LED pin is in Pin # 38
long duration, inches, cm, RightDistance, LeftDistance;
long time, pantime,  turn_time;

void setup() {
  void Fast(); //set speed of all motors to fast
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.write(90); //center the servo to face straight ahead

  pinMode(ledPin, OUTPUT);
  //CMUcam4 SETUP****************************************************************************
  cam.begin();
  cam.LEDOn(LED_BLINK);
  delay(WAIT_TIME);
  //***************************************************************************************** 
  // Turn auto gain and auto white balance off.
  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);
  cam.colorTracking(false); // Go to YUV mode! False foir RGB mode!  

  cam.LEDOn(CMUCAM4_LED_ON);
  cam.noiseFilter(NOISE_FILTER_LEVEL);
  cam.trackColor(RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX);
  time = millis();
}

//Loop Structure that repeatedly executes tasks
void loop() {
  cmucam();
  if(c == 0){
    //The Obstacle function moves the car around, avoiding any obstacles in its way.
    Obstacle();
    pantime = millis() - time;//elapsed time

      if(pantime > 12000){//proximity check every 12-1.8 = 10.2 seconds 
      time = millis();//save last time it panned around
      Fast();
      
      //While red is not detected, turn right for 1800 milliseconds
      while(c==0 && (millis()-time < 1800)){
        Right();
        cmucam();
      }
      Stop();
    }
  }
  else{
    chase(); 
  }
}

//If red is detected, the car will chase it.
void chase(){
  if (a < 55){
    Left();
  }
  else if (a > 105){
    Right();
  }
  else {
    Straight();
  }
}

void cmucam(){
  cam.getTypeTDataPacket(&data); // Get a tracking packet.
  a=data.mx;
  b=data.my;
  c=data.confidence;
  p=data.pixels;
  
  //Turns on an LED if red if detected by the cmucam.
  if(c > 0){
    digitalWrite(ledPin, HIGH);}
  else {
    digitalWrite(ledPin, LOW); 
  }
}

void Obstacle(){
  prevprevping = prevping; //Second last ping
  prevping = cm; //Previous ping
  Ping(); //current Ping

  /*If the distance in cm is further than farDist, then set motor to medium speed
   and call the function to drive Straight*/
  if ((cm == 0) || cm > farDist){
    Fast();
    Straight();
  }

  /*If the distance in cm is within the interval of clDist and farDist, then set the motor
   to slow speed and call the function to drive Straight. This will */
  else if (cm >= clDist && cm <= farDist){
    Slow();
    Straight();
  }

  /*If the last two ping distance readings and the current one are less then clDist, and if
   those three ping readings are not equal to 0, then render the statement true and execute 
   the functions inside the conditional.*/
  else if (prevprevping < clDist && prevping < clDist && cm < clDist && prevping != 0 && cm != 0 && prevprevping != 0){
    Fast(); //Set the Speed to Fast for the turn
    Stop(); //Step the motors so that the servos can be activated
    LookRight(); //Call the look right function to turn the servo 90 degrees to the right
    LookLeft(); //Call the look left function to trun the servo 90 degrees to the left
    myservo.write(90); //center the servo to face straight ahead
    //If right distance is greater than left distance, the 
    turn_time = millis();
    if (RightDistance > LeftDistance){
      Right();
      delay(500);
    }
    else{
      Left();
      delay(500);
    }
  }
}

//Sets the Motor Speed to Fast
void Fast(){
  motor1.setSpeed(fSpeed);
  motor2.setSpeed(fSpeed);
  motor3.setSpeed(fSpeed);
  motor4.setSpeed(fSpeed);
}


//Sets the Motor Speed to Slow
void Slow(){
  motor1.setSpeed(sSpeed);
  motor2.setSpeed(sSpeed);
  motor3.setSpeed(sSpeed);
  motor4.setSpeed(sSpeed);
}


//Drives the car forward by setting all motors to FORWARD
void Straight(){
  motor1.run(FORWARD); 
  motor2.run(FORWARD); 
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

//Stops all the motors by commanding RELEASE
void Stop(){
  motor1.run(RELEASE);  
  motor2.run(RELEASE);  
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

/*Turns the car right by spinning the motors on the right side of the car backward, and
 the ones of the opposite side of the turn forward.*/
void Right(){
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

/*Turns the car right by spinning the motors on the right side of the car backward, and
 the ones of the opposite side of the turn forward.*/
void Left(){
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

//Servo turns to right (to 0 degrees) and Ping's
void LookRight(){
  myservo.write(0);
  delay(800); //800ms is a safe time period for the servo to turn and Ping
  Ping();
  RightDistance = cm;
}

//Servo turns to left (to 180 degrees) and Ping's
void LookLeft(){
  myservo.write(180);
  delay(800);
  Ping();
  LeftDistance = cm;
}

/* Ping Sensor code from: 
 http://www.arduino.cc/en/Tutorial/Ping
 created 3 Nov 2008
 by David A. Mellis
 modified 30 Aug 2011
 by Tom Igoe*/
void Ping(){ 
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  //  Serial.print(inches);
  //  Serial.print("in, ");
  //  Serial.print(cm);
  //  Serial.print("cm");
  //  Serial.println();
  delay(100);
}

long microsecondsToInches(long microseconds){
  /*According to Parallax's datasheet for the PING))), there are
   73.746 microseconds per inch (i.e. sound travels at 1130 feet per
   second).  This gives the distance travelled by the ping, outbound
   and return, so we divide by 2 to get the distance of the obstacle.
   See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf*/
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds){
  /*The speed of sound is 340 m/s or 29 microseconds per centimeter.
   The ping travels out and back, so to find the distance of the
   object we take half of the distance travelled.*/
  return microseconds / 29 / 2;
}

