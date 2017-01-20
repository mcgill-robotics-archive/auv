//define header for AUV atmega32u4 code

//Software Reset
#define CPU_RESET_ADDR                     (uint32_t *) 0xE000ED0C
#define CPU_RESET_VAL                      0x5FA0004
#define CPU_RESET                          ( *CPU_RESET_ADDR = CPU_RESET_VAL)


//Depth Sensor
#define MS5803_I2C_ADDR                    0x76


//TIME INTERVAL(unit microsecond)
#define TEMPERATURE_INTERVAL               1000  //amount of delay between each temperatures read
#define DEPTH_INTERVAL                     25    //amount of delay between each depth read
#define DEPTH_DISCONNECT_INTERVAL          5000
#define EXTERNAL_TEMP_INTERVAL             1000
