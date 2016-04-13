//defining states
#define STOP 0
#define FIND_CANDLE 1
#define EXTINGUISH_FIRE 2
#define RETURN_HOME 3

//Ultrasonic sensor Maximum distance
#define MAX_DISTANCE 200

//Flame sensor constant
#define FLAME_SENSOR_CONSTANT 50

//Maximum motor speed
#define MOTOR_MAX_SPEED 255

//Analog Pins
#define FLAME_SENSE_PINA A0
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
//11
//12
//13
//14
//15
//16


//Pin declarations
//0 RX0
//1 TX0
#define L_ENCODER_PIN 2        // PWM INT0
#define R_ENCODER_PIN 3        // PWM INT1
//4 PWM
//5 PWM
#define FAN_PIN 6              // PWM
#define TILT_SERVO_PIN 7       // PWM
#define RIGHT_MOTOR_PIN2 8     // PWM
#define RIGHT_MOTOR_PIN1 9     // PWM
#define LEFT_MOTOR_PIN2 10     // PWM
#define LEFT_MOTOR_PIN1 11     // PWM
//12 PWM
//13 PWM LED
//14 TX3
//15 RX3
//16 TX2
//17 RX2
//18 INT5 TX1
//19 INT4 RX1
//20 INT3 SDA
//21 INT2 SCL
//22
//23
//24
//25
#define LEFT_US_EP 26
#define LEFT_US_TP 27
#define FORWARD_US_TP 28
#define FORWARD_US_EP 29
#define FLAME_SENSE_PIND 30
//31
//32
//33
//34
//35
//36
//37
//38
//39
//40
//41
//42
//43
//44 PWM
//45 PWM
//46 PWM
//47
//48
//49
//50 MISO
//51 MOSI
#define RIGHT_US_TP 52//SCK
#define RIGHT_US_EP 53//SS
