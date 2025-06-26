#ifndef _DEFINE_H_
#define _DEFINE_H_
#endif

#define READ_DELAY 0
#define DELAY_BTWN_READ 0

// MAG/BMP SPI
#define CS_MAG 10
#define CS_BMP 36
#define MOSI_P 11
#define MISO_P 12
#define CLK 13

// MPU I2C
#define MPU_SDA 18
#define MPU_SCL 19
#define MPU6500_ADDR 0x68

// IMU Raw -> Roll/Pitch
#define sqr(x) x * x
#define hypotenuse(x, y) sqrt(sqr(x) + sqr(y))

// #define PRINT(x) Serial.print(x)
// #define PRINTLN(x) Serial.println(x)