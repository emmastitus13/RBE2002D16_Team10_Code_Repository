/* RBE2002D16 Final Project Code Definitions
 *
 *
 *
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 23, 2016 by Ben Titus
 */

#include <Arduino.h>


//movement struct
//holds the number of encoder ticks and the angle
typedef struct {
    uint8_t index;
    int encTicks;
    float angle;
} Movement;


//Values from the robot for calculating various things
const int wheelRad = 16; //mm
const int wheelDiam = 32; //mm
const float wheelCirc = 100.53; //mm
const float track = 119.25;
const int ticksPerShaftRev = 6;
const int fullEncTicksPerWheelRev = 3575;
const int encTicksPerWheelRev = 1788;
const int tickPer90 = 1577;


//length of movement array
#define ARRAY_LENGTH 64


 //defining states
#define STOP 0
#define FIND_CANDLE 1
#define EXTINGUISH_FIRE 2
#define RETURN_HOME 3


//defining wall avoidance states
#define WALL_TEST 0                   //reads the sensors and determines where the walls are
#define TURN_RIGHT 1             //turns right 90 degrees
#define TURN_LEFT 2              //turns left 90 degrees
#define FORWARD 3                //drives forward 1 wheel rotation
#define WALL_LEFT 4              //slows the right side of the robot
#define WALL_RIGHT 5             //slows the left side of the robot
#define NO_WALLS 6               //no walls near the fron, back, or left of the robot
#define NO_WALLS_LEFT 7           //
#define NO_WALLS_RIGHT 8         //


//candle finding states
#define CANDLE_FIND 0
#define CANDLE_FOUND 1


//Ultrasonic sensor Maximum distance
#define MAX_DISTANCE 200


//Flame sensor constant
#define FLAME_SENSOR_CONSTANT 50


//Maximum motor speed
#define MAX_MOTOR_SPEED 255


//Analog Pins
//0
//1
//2
//3
//4
//5
//6
//7
//8
//9
//10
#define FLAME_SENSE_PINA A11
//12
//13
//14
//15
//16


//Pin declarations
//0 RX0
//1 TX0
#define R_ENCODER_PIN 2      // PWM INT0
//3 PWM INT1
//4 PWM
#define LEFT_MOTOR_PIN2 5    // PWM
#define FAN_PIN 6            // PWM
#define TILT_SERVO_PIN 7     // PWM
#define RIGHT_MOTOR_PIN2 8   // PWM
#define RIGHT_MOTOR_PIN1 9   // PWM
#define LEFT_MOTOR_PIN1 10   // PWM
//11 PWM ***BROKEN***
#define ORANGE_LED_PIN 12    // PWM
#define BLUE_LED_PIN 13      // PWM LED
//14 TX3
//15 RX3
//16 TX2
//17 RX2
#define L_ENCODER_PIN 18     // INT5 TX1
#define FRONT_BUMPER 19      // INT4 RX1
//20 INT3 SDA
//21 INT2 SCL
//22
//23
//24
//25
#define LEFT_US_EP 27        //
#define LEFT_US_TP 26        //
#define FORWARD_US_TP 28     //
#define FORWARD_US_EP 29     //
#define FLAME_SENSE_PIND 30  // Flame sensor digital pin
//31
//32
//33
//34
//35
//36
//37
//38
//39
#define RS_PIN 40            // LCD Register Select
#define EN_PIN 41            // LCD Enable
#define DB1_PIN 42           // LCD Data Bit 1
#define DB2_PIN 43           // LCD Data Bit 2
#define DB3_PIN 44           // LCD Data Bit 3
#define DB4_PIN 45           // LCD Data Bit 4
//46 PWM
//47
//48
//49
//50 MISO
//51 MOSI
#define RIGHT_US_TP 52       //SCK
#define RIGHT_US_EP 53       //SS
