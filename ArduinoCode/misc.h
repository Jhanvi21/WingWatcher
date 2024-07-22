#include "Timer.h"

// Arduino Digital Pins
#define RX_PIN           0
#define TX_PIN           1
#define MOTION_PIN       2
#define DIGITAL_PIN_3    3
#define DIGITAL_PIN_4    4
#define SERVO_PIN        5
#define DIGITAL_PIN_6    6
#define ESP_PIN          7
#define DIGITAL_PIN_8    8
#define DIGITAL_PIN_9    9
#define DIGITAL_PIN_10   10
#define ECHO_PIN         11
#define TRIG_PIN         12
#define TEST_PIN         13

#define SCL_PIN          18
#define SDA_PIN          19


// Arduino Analog Pins
#define FSR_PIN          0
#define ANALOG_PIN_1     1
#define ANALOG_PIN_2     2
#define ANALOG_PIN_3     3
#define ANALOG_PIN_4     4
#define ANALOG_PIN_5     5

// Servo Positions
enum
{
  START_POSITION = 0,
  CLOSE_POSITION = 0,
  OPEN_POSITION = 50
};

// FSR Readings

#define NO_PRESSURE     < 10
#define LIGHT_TOUCH     < 200
#define LIGHT_SQUEEZE   < 500
#define MEDIUM_SQUEEZE  < 800

// State Machine
enum
{
  STATE_0 = 0,
  STATE_1 = 1,
  STATE_2 = 2
};

#define TIME_TO_SLEEP 20*1000
#define SPEED_OF_SOUND 0.0343

// Shared Definitions
#define MASTER_ADDRESS 8
#define SLAVE_ADDRESS  9

#define RESET           0x00
#define ACK             0x01
#define NACK            0x02
#define SEED            0x03
#define IS_BIRD         0x04
#define BIRD            0x05
#define NO_BIRD         0x06
#define SETUP_FINISHED  0x07
#define INVALID_REQUEST 0x0F