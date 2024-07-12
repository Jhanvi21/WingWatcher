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
  // timer.pause(); // pause the timer
  
  if (timer.read() > (20*1000))
  {
    return true;
  }
  
  // Serial.println(timer.read());
  // timer.resume(); // resume the timer

  return false;
}

// active low
void lookForBirds()
{
  int motion = digitalRead(motionSensorPin);
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
    return true;
  }
  
  Serial.println("no seed");
  return false;
}
/****************************************************************/
// FSR Sensor
/****************************************************************/
int fsrReading = 0;

bool hasSeedsOnTray()
{
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
int calibrationTime = 10;

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
  Wire.begin(MASTER_ADDRESS);
  timer.start(); 
}

void loop()
{
  lookForBirds();

  if (state == STATE_0 || state == STATE_1) // things that should only happen once
  {
    timer.pause(); // incase this sequence takes a long time

    if (!hasSeedsOnTray())
    {
      dispenseSeed();
    }

    if (!hasSeedInStorage())
    {
      Wire.beginTransmission(SLAVE_ADDRESS);
      Serial.println("Beginning transmission");
      Wire.write(SEED);        // sends one byte
      Wire.requestFrom(SLAVE_ADDRESS, 1); //request one byte
      int result = Wire.read();
      Serial.println(result);
      if (result == ACK)
      {
        Serial.println("Notification Sent");
      }
      else
      {
        Serial.println("Error");
      }
      Wire.endTransmission();    // stop transmitting
      //send notification to person
    }

    state = STATE_2;

    timer.resume();
  }

  if (isBirdGone() && (state == STATE_2))
  {
    delay(15);
    timer.stop();
    sleepNow();
  }
}