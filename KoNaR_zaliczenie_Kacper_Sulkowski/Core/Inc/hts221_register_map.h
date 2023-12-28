/* to akurat pisałem sam*/


#ifndef HTS221_REGISTER_MAP_H
#define HTS221_REGISTER_MAP_H


//unique communication values
#define HTS221_I2C_ADDRESS  (0b01011111<<1)
#define HTS221_I2C_ID       (0b10111100)


//CTRL_REG1 options
#define HTS221_PD_ON      	(0b10000000) //device power ON
#define HTS221_BDU_ON       (0b00000100) /* output register update between the reading of the upper
and lower register parts*/
#define HTS221_ODR_12HZ     (0b00000011)//12.5Hz output rate




#define HTS221_WHO_AM_I         0x0F
#define HTS221_AV_CONF          0x10
#define HTS221_CTRL_REG1        0x20
#define HTS221_CTRL_REG2        0x21
#define HTS221_CTRL_REG3        0x22
#define HTS221_STATUS_REG       0x27
#define HTS221_HUMIDITY_OUT_L   0x28
#define HTS221_HUMIDITY_OUT_H   0x29
#define HTS221_TEMP_OUT_L       0x2A
#define HTS221_TEMP_OUT_H       0x2B
#define HTS221_H0_OUT_L         0x36
#define HTS221_H0_OUT_H         0x37
#define HTS221_H1_OUT_L         0x3A
#define HTS221_H1_OUT_H         0x3B
#define HTS221_T0_OUT_L       	0x3C
#define HTS221_T0_OUT_H       	0x3D
#define HTS221_T1_OUT_L       	0x3E
#define HTS221_T1_OUT_H       	0x3F
#define HTS221_H0_rH_x2         0x30
#define HTS221_H1_rH_x2         0x31
#define HTS221_T0_degC_x8       0x32
#define HTS221_T1_degC_x8       0x33
#define HTS221_T1_T0_MSB_x8     0x35



#endif
