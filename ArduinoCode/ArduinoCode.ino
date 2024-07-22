/* 
Author: Jhanvi Shah
*/

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <Servo.h>
#include <Wire.h>
#include "Timer.h"
#include "misc.h"

Servo servo;  // create servo object to control a servo
Timer timer(MILLIS);

const int motionPin = MOTION_PIN;
const int testLedPin = TEST_PIN;
const int servoPin = SERVO_PIN;
const int trigPin = TRIG_PIN;
const int echoPin = ECHO_PIN;
const int espPin = ESP_PIN;
const int fsrPin = FSR_PIN;

int state;

bool one_time_run = false;
int calibrationTime = 10;
/****************************************************************/
// Timing Code
/****************************************************************/
void setTimerToZero()
{
  timer.start();
  timer.stop();
}

/****************************************************************/
// Motion Sensor
/****************************************************************/
bool isBird()
{
  Serial.println("Checking for Bird...");
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(IS_BIRD);        // sends one byte
  int error = Wire.endTransmission();     

  if (error == 0)
  {
    delay(4*1000);
    Wire.requestFrom(SLAVE_ADDRESS, 1);
    uint8_t result = Wire.read();

    switch (result)
    {
      case BIRD:
        Serial.println("Bird");
        return true;
      case NO_BIRD:
        Serial.println("No Bird");
        return true;
      case INVALID_REQUEST:
        Serial.println("Invalid Request Sent");
        return false;
      default:
        Serial.print("Invalid Result Recevied ");
        Serial.println(result);
        return false;
    }
    Wire.endTransmission();    // stop transmitting TODO: Is this needed? in both sections
  }
  else
  {
    Serial.print("I2C Transmission Error: ");
    Serial.println(error);
    return false;
  }
}

int debug = 0;
bool isBirdGone()
{
  timer.pause(); // pause the timer
  
  if (timer.read() > (TIME_TO_SLEEP))
  {
    debug = 0;
    return true;
  }
  
  if (timer.read() > debug)
  {
    Serial.println(timer.read());
    debug +=1000;
  }
  timer.resume(); // resume the timer

  return false;
}

// active low
void lookForBirds()
{
  int motion = digitalRead(motionPin);

  if (motion == LOW)
  {
    digitalWrite(testLedPin, HIGH);  // detected motion
    if (state == STATE_0)
    {
      setTimerToZero();
      state = STATE_1;
      Serial.println("Motion detected!");
    }
  }
  else 
  {
    digitalWrite(testLedPin, LOW); // no motion

    if (state == STATE_1)
    {
      Serial.println("Motion stopped!");
      timer.start(); 
      state = STATE_0;
    }
  }
}

/****************************************************************/
// Servo Motor
/****************************************************************/
void dispenseSeed()
{
  int pos = 0;    // variable to store the servo position
  servo.write(OPEN_POSITION);
  delay(3*1000);                       // waits 5 s for the servo to reach the position
  servo.write(CLOSE_POSITION);
  delay(2*1000);
}

/****************************************************************/
// Ultrasonic Sensor
/****************************************************************/
bool hasSeedInStorage()
{
  float duration = 0;
  float distance = 0;

 	digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*SPEED_OF_SOUND)/2;

  if (distance > 250 || distance < 7)
  {
    Serial.println("has seed");
    return false;
  }
  
  Serial.println("no seed");
  return false;
}

void sendSeedNotification()
{
  Serial.println("Seed Notification...");
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(SEED);        // sends one byte
  int error = Wire.endTransmission();     

  if (error == 0)
  {
    delay(500);
    uint8_t result = Wire.requestFrom(SLAVE_ADDRESS, 1);
    result = Wire.read(); // receive a byte as character

    switch (result)
    {
      case ACK:
        Serial.println("ACK Received");
        break;
      case NACK:
        Serial.println("Notification Error");
        break;
      case INVALID_REQUEST:
        Serial.println("Invalid Request Sent");
        break;
      default:
        Serial.print("Invalid Result ");
        Serial.println(result);
        break;
    }
    Wire.endTransmission();    // stop transmitting
  }
  else
  {
    Serial.print("I2C Transmission Error: ");
    Serial.println(error);
  }
}
/****************************************************************/
// FSR Sensor
/****************************************************************/
bool hasSeedsOnTray()
{
  int fsrReading = 0;
  fsrReading = analogRead(fsrPin);

  Serial.println("Seed on tray");
  if (fsrReading > 10)
  {
    Serial.println(fsrReading);
    return true;
  }
  Serial.println(fsrReading);
  return false;

  // if (fsrReading < 10) {
  //   Serial.println(" - No pressure");
  // } else if (fsrReading < 200) {
  //   Serial.println(" - Light touch");
  // } else if (fsrReading < 500) {
  //   Serial.println(" - Light squeeze");
  // } else if (fsrReading < 800) {
  //   Serial.println(" - Medium squeeze");
  // } else {
  //   Serial.println(" - Big squeeze");
  // }
}

/****************************************************************/
// Interrupt/Sleep Code
/****************************************************************/

// Anything used in interrupt should be declared volatile
void wakeUpNow()
{
  // sleep_disable();
  detachInterrupt(digitalPinToInterrupt(MOTION_PIN));
  Serial.println("Awake");
  // digitalWrite(espPin, HIGH);
  // delay(15);
  setTimerToZero();
  state = STATE_1;
  one_time_run = true;
  // if(!isBird())
  // {
  //   sleepNow();
  // }
}

void sleepNow()
{
  // Serial.println("Sleeping");
  // delay(200);
  // delay(15);
  // digitalWrite(espPin, LOW);
  // state = STATE_0;
  // // There are five different sleep modes in order of power saving:
	// // SLEEP_MODE_IDLE - the lowest power saving mode
	// // SLEEP_MODE_ADC
	// // SLEEP_MODE_PWR_SAVE
	// // SLEEP_MODE_STANDBY (not recommended unless we have another crystal for timing)
	// // SLEEP_MODE_PWR_DOWN - the highest power saving mode
  // set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(digitalPinToInterrupt(MOTION_PIN), wakeUpNow, LOW);
  // sleep_enable();
  // sleep_mode();
}

/****************************************************************/
// Main Code
/****************************************************************/
void setup()
{
  Serial.begin(115200);

  servo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  pinMode(motionPin, INPUT);
  pinMode(testLedPin, OUTPUT);
  pinMode(espPin, OUTPUT);

  pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);  

  // Initial Values
  digitalWrite(espPin, HIGH);

  Serial.print("calibrating sensor ");
  for(int i = 0; i < calibrationTime; i++)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");

  servo.write(START_POSITION);
  digitalWrite(testLedPin, LOW);
  
  Wire.begin();

  state = STATE_0;

  // delay(40*1000);
  // if(isBird())
  // {
  //   Serial.println("Bird Code Works!");
  // //   sleepNow();
  // }
  // timer.start(); 
}

bool setupFin = false;
uint8_t res = RESET;
uint8_t result = 0;
void loop()
{
  lookForBirds();

  if (!setupFin)
  {
    result = Wire.requestFrom(SLAVE_ADDRESS, 1);
    Serial.println(result);
    delay(10);
    result = Wire.read(); // receive a byte as character
    Serial.println(result);         // print the character    
    if (result == SETUP_FINISHED)
    {
      one_time_run = true;
      setupFin = true;
      Serial.println("Setup Finish");
      if(isBird())
      {
        Serial.println("Bird Code Works!");
      //   sleepNow();
      }

    } 
    else
    {
      Serial.println("Setup UnFin");
      delay(2*1000);
    }
  }

  if (one_time_run) // things that should only happen once
  {
    timer.pause(); // incase this sequence takes a long time

    if (!hasSeedsOnTray())
    {
      dispenseSeed();
    }

    if (!hasSeedInStorage())
    {
      delay(5000);
      sendSeedNotification();
    }

    one_time_run = false;

    timer.resume();
  }

  if (isBirdGone() && !one_time_run && (result == SETUP_FINISHED))
  {
    delay(15);
    timer.stop();
    sleepNow();
  }
}