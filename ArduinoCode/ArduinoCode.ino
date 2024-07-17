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

int motionSensorPin = MOTION_PIN;
int testLedPin = TEST_PIN;
int servoPin = SERVO_PIN;
int trigPin = TRIG_PIN;
int echoPin = ECHO_PIN;
int espPin = ESP_PIN;
int fsrPin = FSR_PIN;

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
bool isBirdGone()
{
  timer.pause(); // pause the timer
  
  if (timer.read() > (TIME_TO_SLEEP))
  {
    return true;
  }
  
  // Serial.println(timer.read());
  timer.resume(); // resume the timer

  return false;
}

// active low
void lookForBirds()
{
  int motion = digitalRead(motionSensorPin);
  Serial.println("motion");
  Serial.println(motion);

  if (motion == LOW)
  {
    digitalWrite(testLedPin, HIGH);  // detected motion

    if (state == STATE_0)
    {
      setTimerToZero();
      state = STATE_1;
      Serial.println("Motion detected!");
      delay(25);
    }
  }
  else 
  {
    digitalWrite(testLedPin, LOW); // no motion

    if (state == STATE_1)
    {
      Serial.println("Motion stopped!");
      timer.start(); 
      delay(25);
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
  delay(5*1000);                       // waits 5 s for the servo to reach the position
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
/****************************************************************/
// FSR Sensor
/****************************************************************/
bool hasSeedsOnTray()
{
  int fsrReading = 0;
  fsrReading = analogRead(fsrPin);

  if (fsrReading > 10)
  {
    return true;
  }
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
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(MOTION_PIN));
  Serial.println("Awake");
  digitalWrite(espPin, HIGH);
  delay(15);
  setTimerToZero();
  state = STATE_1;
  one_time_run = true;
}

void sleepNow()
{
  delay(15);
  digitalWrite(espPin, LOW);
  Serial.println("Sleeping");
  delay(200);
  state = STATE_0;
  // There are five different sleep modes in order of power saving:
	// SLEEP_MODE_IDLE - the lowest power saving mode
	// SLEEP_MODE_ADC
	// SLEEP_MODE_PWR_SAVE
	// SLEEP_MODE_STANDBY (not recommended unless we have another crystal for timing)
	// SLEEP_MODE_PWR_DOWN - the highest power saving mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(digitalPinToInterrupt(MOTION_PIN), wakeUpNow, LOW);
  sleep_enable();
  sleep_mode();
}

/****************************************************************/
// Main Code
/****************************************************************/
void setup()
{
  Serial.begin(115200);
  
  servo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  pinMode(motionSensorPin, INPUT);
  pinMode(testLedPin, OUTPUT);
  pinMode(espPin, OUTPUT);

  pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);  

  // Initial Values
  Serial.print("calibrating sensor ");
  for(int i = 0; i < calibrationTime; i++)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");

  servo.write(START_POSITION);
  digitalWrite(testLedPin, LOW);
  digitalWrite(espPin, HIGH);
  
  state = STATE_0;
  one_time_run = true;
  Wire.begin(MASTER_ADDRESS);
  timer.start(); 
}

void loop()
{
  lookForBirds();

  if (one_time_run) // things that should only happen once
  {
    timer.pause(); // incase this sequence takes a long time
    
    int error = -1;
    uint8_t result = 0;

    if (!hasSeedsOnTray())
    {
      dispenseSeed();
    }

    if (!hasSeedInStorage())
    {
      Wire.beginTransmission(SLAVE_ADDRESS);
      Wire.setWireTimeout(300);
      Wire.write(SEED);        // sends one byte
      error = Wire.endTransmission();     

      if (error == 0)
      {
        delay(10);
        result = Wire.requestFrom(SLAVE_ADDRESS, 1);

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
            Serial.println("Invalid Result");
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

    one_time_run = false;

    timer.resume();
  }

  if (isBirdGone() && !one_time_run)
  {
    delay(15);
    timer.stop();
    sleepNow();
  }
}