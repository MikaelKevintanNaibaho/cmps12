#ifndef CMPS12_I2C_HPP
#define CMPS12_I2C_HPP

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define I2C_DEVICE "/dev/12c-2"
#define CMPS12_ADDRESS 0x60

//Registers
#define COMMAND_REG 0x00
#define COMPASS_BEARING_8_BIT_REG 0x01
#define COMPASS_BEARING_16_BIT_REG_HIGH_QUATERNION 0x02
#define COMPASSS_BEARING_16_BIT_REG_LOW_QUATERNION 0x03
#define PITCH_ANGLE_REG_90_DEGRESS 0x04
#define ROLL_ANGLE_REG_90_DEGRESS 0x05
#define MAG_X_RAW_16_BIT_REG_HIGH 0x06
#define MAG_X_RAW_16_BIT_REG_LOW 0x07
#define MAG_Y_RAW_16_BIT_REG_HIGH 0x08
#define MAG_Y_RAW_16_BIT_REG_LOW 0x09
#define MAG_Z_RAW_16_BIT_REG_HIGH 0x0A
#define MAG_Z_RAW_16_BIT_REG_LOW 0x0B
#define ACCELEROMETER_X_RAW_16_BIT_HIGH 0x0C
#define ACCELEROMETER_X_RAW_16_BIT_LOW 0x0D
#define ACCELEROMETER_Y_RAW_16_BIT_HIGH 0x0E
#define ACCELEROMETER_Y_RAW_16_BIT_LOW 0x0F
#define ACCELEROMETER_Z_RAW_16_BIT_HIGH 0x10
#define ACCELEROMETER_Z_RAW_16_BIT_LOW 0x11
#define GYRO_X_RAW_16_BIT_HIGH 0x12
#define GYRO_X_RAW_16_BIT_LOW 0x13
#define GYRO_Y_RAW_16_BIT_HIGH 0x14
#define GYRO_Y_RAW_16_BIT_LOW 0x15
#define GYRO_Z_RAW_16_BIT_HIGH 0x16
#define GYRO_Z_RAW_16_BIT_LOW 0x17
#define TEMP_REG_16_BIT_HIGH 0x18
#define TEMP_REG_16_BIT_LOW  0x19
#define COMPASS_BEARING_16_BIT_REG_HIGH_BNO055 0x1A
#define COMPASSS_BEARING_16_BIT_REG_LOW_BNO055 0x1B
#define PITCH_ANGLE_16_BIT_REG_HIGH_180_DEGRESS 0x1C
#define PITCH_ANGLE_16_BIT_REG_LOW_180_DEGRESS 0x1D
#define CALIBRATION_REG 0x1E

#define I2C_SLAVE_FORCE 0x0706


typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} SensorData;

typedef struct
{
    uint16_t bearing;
    int8_t pitch;
    int8_t roll;
} Orientation;

typedef struct
{
    SensorData magnetometer;
    SensorData accelerometer;
    SensorData gyro;
    Orientation orientation;
} AllSensorData;


//i2c related function
int cmps12_write_register(int file, uint8_t register_addr, uint8_t data, ssize_t data_size);
int cmps12_read_register(int file, uint8_t register_addr, uint8_t *data, ssize_t data_size);
int cmps12_open_i2c_device(const char *device);
int cmps12_release_i2c_device(int file);
int cmps12_init(const char *device);

//compass feature function
int8_t map(uint8_t value);
int8_t cmps12_read_pitch_90_degress(int cmps12_file);
int8_t cmps12_read_roll_90_degress(int cmps12_file);
uint8_t cmps12_read_calibration_state(int cmps12_file);
uint8_t cmps12_read_bearing_8_bit(int cmps12_file);
uint16_t cmps12_read_bearing_16_bit_quaternion(int cmps12_file);
uint16_t cmps12_read_mag_x_16_bit(int cmps12_file);
uint16_t cmps12_read_mag_y_16_bit(int cmps12_file);
uint16_t cmps12_read_mag_z_16_bit(int cmps12_file);
uint16_t cmps12_read_accelerometer_x_16_bit(int cmps12_file);
uint16_t cmps12_read_accelerometer_y_16_bit(int cmps12_file);
uint16_t cmps12_read_accelerometer_z_16_bit(int cmps12_file);
uint16_t cmps12_read_gyro_x_16_bit(int cmps12_file);
uint16_t cmps12_read_gyro_y_16_bit(int cmps12_file);
uint16_t cmps12_read_gyro_z_16_bit(int cmps12_file);
uint16_t cmps12_temperature_16_bit(int cmps12_file);
uint16_t cmps12_read_bearing_16_bit_BNO055(int cmps12_file);
uint16_t cmps12_read_pitch_180_degress(int cmps12_file);


//functions for combine xyz
SensorData cmps12_read_magnetometer_data(int cmps12_file);
SensorData cmps12_read_accelerometer_data(int cmps12_file);
SensorData cmps12_read_gyro_data(int cmps12_file);

//orientation
Orientation cmps12_read_orientation_quaternion(int cmps12_file);
Orientation cmps12_read_orientation_BNO055(int cmps12_file);


//All data
AllSensorData cmps12_read_all_data(int cmps12_file);

#endif /* CMPS12_I2C_HPP*/