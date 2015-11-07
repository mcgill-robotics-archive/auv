//define header for AUV Teensy code

#define MOTOR_PIN_PORT_SURGE                0 //T100_1 : bow_surge
#define MOTOR_PIN_STARBOARD_SURGE           1 //T100_2 : stern_surge
#define MOTOR_PIN_PORT_BOW_HEAVE            2 //T100_3 : port_bow_heave
#define MOTOR_PIN_STARBOARD_BOW_HEAVE       3 //T100_4 : starboard_bow_heave
#define MOTOR_PIN_PORT_STERN_HEAVE          4 //T100_5 : port_stern_heave
#define MOTOR_PIN_STARBOARD_STERN_HEAVE     5 //T100_6 : starboard_stern_heave

#define MOTOR_T100_RST_VALUE 1500

//Seabotix MOTOR
#define MOTOR_ENABLE_PIN_STARBOARD_SWAY     6
#define MOTOR_ENABLE_PIN_PORT_SWAY          7
//
#define MOTOR_PIN_STARBOARD_SWAY            9    //SeabotimotorCommandValue_1 : port_surge
#define MOTOR_PIN_PORT_SWAY                 10   //SeabotimotorCommandValue_2 : starboard_surge
//
#define MOTOR_SEABOTIX_RST_VALUE            512
#define MOTOR_SEABOTICX_DEADBAND            25
//
#define STATUS_PIN_FAULT                    8
#define STATUS_PIN_OTW                      11

//THRESHOLD for motor control
#define THRESHOLD_MOTOR                     50

#define PWM_FREQUENCY                       23437 //One of the Ideal Frequency of teensy3

//SOLENOID, one used pin for (Pin_33)
#define SOLENOID_PIN_STARBOARD_DROPPER      12
#define SOLENOID_PIN_PORT_DROPPER           24
#define SOLENOID_PIN_STARBOARD_GRABBER      25
#define SOLENOID_PIN_PORT_GRABBER           26
#define SOLENOID_PIN_STARBOARD_TORPEDO      27
#define SOLENOID_PIN_PORT_TORPEDO           28
#define SOLENOID_PIN_EXTRA                  33

//ON BOARD LED PIN
#define LED_PIN                             13

//Mission Control
#define MISSION_PIN                         A3

//Depth Sensor
#define MS5803_I2C_ADDR                     0x76


//TIME INTERVAL(unit microsecond)
#define SOLENOID_TIMEOUT                    200
#define MOTOR_TIMEOUT                       500   //amount of no signal required to start to reset motors
#define TEMPERATURE_INTERVAL                1000  //amount of delay between each temperatures read
#define POWER_MONITOR_INTERVAL              100   //amount of delay between each power monitor read
#define DEPTH_INTERVAL                      25    //amount of delay between each depth read
#define DEPTH_DISCONNECT_INTERVAL           5000
#define EXTERNAL_TEMP_INTERVAL              1000
#define MOTOR_STATUS_INTERVAL               500
#define MISSION_INTERVAL                    100


//ANALOG
#define COMPUTER_VOLTAGE_PIN                A9
#define COMPUTER_CURRENT_PIN                A8
#define MOTOR_VOLTAGE_PIN                   A7
#define MOTOR_CURRENT_PIN                   A6

//POWER MONITORING
// y = 0.1129x - 49.864
//const double kCOM_VOLT_SLOPE = 0.1129;
//const double kCOM_VOLT_OFFSET = -49.864;
const double kCOM_VOLT_SLOPE = 3.3/1024.0/0.1282;
const double kCOM_VOLT_OFFSET = 0.6;


const double kCOM_CURR_SLOPE = 1.0;
const double kCOM_CURR_OFFSET = 0.0;

// y = 0.0142x + 0.0364
//const double kMOT_VOLT_SLOPE = 0.0142;
//const double kMOT_VOLT_OFFSET = 0.0364;
const double kMOT_VOLT_SLOPE = 3.3/1024.0/0.2326;
const double kMOT_VOLT_OFFSET = 0.4;

const double kMOT_CURR_SLOPE = 1.0;
const double kMOT_CURR_OFFSET = 0.0;
