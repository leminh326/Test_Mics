/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_EXAMPLES_CONF_H
#define APP_EXAMPLES_CONF_H

#define IMU_EXAMPLE 	1
#define EAR_EXAMPLE 	0
#define HEAD_EXAMPLE 	1
#define GEST_EXAMPLE 	0
#define TOF_EXAMPLE		0
#define	MATRIX_EXAMPLE	0
#define BATTERY_EXAMPLE 0
#define SOUND_EXAMPLE   1

#if (MATRIX_EXAMPLE != 0)
//LOVE//
static const uint8_t heart_01_left_bmp[] =
{ 0b00000000,
  0b00000000,
  0b00010000,
  0b00001000,
  0b00010000,
  0b00000000,
  0b00000000,
  0b00000000,
};
static const uint8_t heart_02_left_bmp[] =
{ 0b00000000,
  0b00010000,
  0b00111000,
  0b00011100,
  0b00111000,
  0b00010000,
  0b00000000,
  0b00000000,
};
static const uint8_t heart_03_left_bmp[] =
{ 0b00010000,
  0b00111000,
  0b00111100,
  0b00011110,
  0b00111100,
  0b00111000,
  0b00010000,
  0b00000000,
};
static const uint8_t heart_04_left_bmp[] =
{ 0b00111000,
  0b01111100,
  0b01111110,
  0b00111111,
  0b01111110,
  0b01111100,
  0b00111000,
  0b00000000,
};
static const uint8_t heart_01_right_bmp[] =
{ 0b00000000,
  0b00000000,
  0b00000000,
  0b00010000,
  0b00001000,
  0b00010000,
  0b00000000,
  0b00000000,
};
static const uint8_t heart_02_right_bmp[] =
{ 0b00000000,
  0b00000000,
  0b00010000,
  0b00111000,
  0b00011100,
  0b00111000,
  0b00010000,
  0b00000000,
};
static const uint8_t heart_03_right_bmp[] =
{ 0b00000000,
  0b00010000,
  0b00111000,
  0b00111100,
  0b00011110,
  0b00111100,
  0b00111000,
  0b00010000,
};
static const uint8_t heart_04_right_bmp[] =
{ 0b00000000,
  0b00111000,
  0b01111100,
  0b01111110,
  0b00111111,
  0b01111110,
  0b01111100,
  0b00111000,
};
#endif

#endif /*APP_EXAMPLES_CONF_H */
