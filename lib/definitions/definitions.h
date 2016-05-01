/* RBE2002D16 Final Project Code Definitions
 *
 *
 *
 *
 *
 *
 * Created on Apr 12. 2016 by Ben Titus
 * Last edit made Apr 27, 2016 by Ben Titus
 */

#include <Arduino.h>


//Values from the robot for calculating various things
#define WHEEL_RAD 16 //mm
#define WHEEL_DIAM 32 //mm
#define WHEEL_CIRC 100.53 //mm
#define TRACK 119.25
#define TICK_PER_SHAFT_REV 6
#define TICK_PER_WHEEL_REV 1788
#define TICK_PER_SIXTH_WHEEL_REV 298
#define TICK_PER_5_DEG 87
#define TICK_PER_90_DEG 1577


//defining demo states
#define FULL_DEMO 0
#define FIND_CANDLE_DEMO 1


//IMU value for turning 90 degrees
#define IMU_TURN_90_LEFT 81.70
#define IMU_TURN_90_RIGHT -81.40
#define IMU_TURN_5_LEFT 4.60
#define IMU_TURN_5_RIGHT -4.60


//length of movement array
#define ARRAY_LENGTH 64


 //defining main states
#define STOP 0
#define NAVIGATE_MAZE 1
#define FIND_CANDLE 2
#define CALCULATE_VALUES 3
#define RETURN_HOME 4


//defining turning states
#define ENCODER_TURN 0
#define IMU_TURN 1


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
#define WALL_SCAN_RIGHT 9
#define WALL_SCAN_LEFT 10


//defining wall sweep states
#define SWEEP_FORWARDS 0
#define SWEEP_BACKWARDS 1


//defining maze exploring states
#define MAZE_TEST 0
#define WALL_AVOID 1
#define SCAN_FOR_FIRE 2
#define FIRE_DETECTED 3
#define STEP_FORWARD 4


//defining candle finding states
#define CANDLE_FIND 0
#define CANDLE_FOUND 1
#define CANDLE_NOT_FOUND 3
#define EXTINGUISH_CANDLE 5
#define FIRE_EXTINGUISHED 6


//defining around wall states
#define LEAVE_WALL 0
#define TURN_AROUND_WALL 1
#define CATCH_WALL 2


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
//IMU INT3 SDA
//IMU INT2 SCL
#define LEFT_US_TP 22        //
#define LEFT_US_EP 23        //
//24
//25
//26
#define FLAME_SENSE_PIND 27  // Flame sensor digital pin
#define FORWARD_US_TP 28     //
#define FORWARD_US_EP 29     //
//30
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
#define RIGHT_US_EP 48       //
//49
#define RIGHT_US_TP 50       // MISO
//51 MOSI
//52 SCK
//53 SS
