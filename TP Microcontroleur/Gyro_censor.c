/**
  ******************************************************************************
  * @file    BSP/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

  /* Includes ------------------------------------------------------------------*/

#include "stm32l4xx_hal.h"
#include <stdio.h>

#ifdef __GNUC__
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

I2C_HandleTypeDef hI2cHandler;
#define BUTTONn                             ((uint8_t)1)
#define USER_BUTTON_PIN                   GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT             GPIOC
#define USER_BUTTON_EXTI_IRQn             EXTI15_10_IRQn
#define DISCOVERY_I2Cx                             I2C2
#define DISCOVERY_I2Cx_TIMING                     ((uint32_t)0x00702681)
#define LPS22HB_RES_CONF_REG     (uint8_t)0x1A
#define LPS22HB_LCEN_MASK        (uint8_t)0x01
#define DISCOVERY_I2Cx_SCL_PIN                     GPIO_PIN_10
#define DISCOVERY_I2Cx_SDA_PIN                     GPIO_PIN_11                                               
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT           GPIOB
#define DISCOVERY_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C2
#define DISCOVERY_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define DISCOVERY_I2Cx_FORCE_RESET()               __HAL_RCC_I2C2_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C2_RELEASE_RESET()
#define DISCOVERY_I2Cx_EV_IRQn                     I2C2_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                     I2C2_ER_IRQn
#define I2C_ANALOGFILTER_ENABLE         0x00000000U
#define I2C_ANALOGFILTER_ENABLE         0x00000000U
#define LPS22HB_CTRL_REG1      (uint8_t)0x10
#define LPS22HB_WHO_AM_I_REG         (uint8_t)0x0F
#define LPS22HB_WHO_AM_I_VAL         (uint8_t)0xB1
#define LPS22HB_PRESS_OUT_XL_REG        (uint8_t)0x28
#define LPS22HB_ODR_MASK                (uint8_t)0x70
#define LPS22HB_BDU_MASK                (uint8_t)0x02
#define TIMING_CLEAR_MASK   (0xF0FFFFFFU)  /*!< I2C TIMING clear register Mask */
#define I2C_STATE_NONE            ((uint32_t)(HAL_I2C_MODE_NONE)) 
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define LPS22HB_I2C_ADDRESS  (uint8_t)0xBA

#define LSM6DSL_ACC_GYRO_WHO_AM_I           0x6A
/* Gyro Full Scale Selection */
#define LSM6DSL_GYRO_FS_245            ((uint8_t)0x00)  
#define LSM6DSL_GYRO_FS_500            ((uint8_t)0x04)  
#define LSM6DSL_GYRO_FS_1000           ((uint8_t)0x08)  
#define LSM6DSL_GYRO_FS_2000           ((uint8_t)0x0C)

/* Gyro Full Scale Sensitivity */
#define LSM6DSL_GYRO_SENSITIVITY_245DPS            ((float)8.750f) /**< Sensitivity value for 245 dps full scale  [mdps/LSB] */ 
#define LSM6DSL_GYRO_SENSITIVITY_500DPS            ((float)17.50f) /**< Sensitivity value for 500 dps full scale  [mdps/LSB] */ 
#define LSM6DSL_GYRO_SENSITIVITY_1000DPS           ((float)35.00f) /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */ 
#define LSM6DSL_GYRO_SENSITIVITY_2000DPS           ((float)70.00f) /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */ 

/* Gyro Power Mode selection */
#define LSM6DSL_ACC_GYRO_LP_G_DISABLED     ((uint8_t)0x00) /* LP disabled*/
#define LSM6DSL_ACC_GYRO_LP_G_ENABLED    


/* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;


#define LSM6DSL_ACC_GYRO_WHO_AM_I           0x6A



#define LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW    0xD4  // SAD[0] = 0
#define LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH   0xD6  // SAD[0] = 1

/************** Who am I  *******************/

#define LSM6DSL_ACC_GYRO_WHO_AM_I           0x6A

/************** Device Register  *******************/

#define LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS    0x01

#define LSM6DSL_ACC_GYRO_SENSOR_SYNC_TIME   0x04
#define LSM6DSL_ACC_GYRO_SENSOR_RES_RATIO   0x05

#define LSM6DSL_ACC_GYRO_FIFO_CTRL1         0x06
#define LSM6DSL_ACC_GYRO_FIFO_CTRL2         0x07
#define LSM6DSL_ACC_GYRO_FIFO_CTRL3         0x08
#define LSM6DSL_ACC_GYRO_FIFO_CTRL4         0x09
#define LSM6DSL_ACC_GYRO_FIFO_CTRL5         0x0A

#define LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G   0x0B
#define LSM6DSL_ACC_GYRO_INT1_CTRL          0x0D
#define LSM6DSL_ACC_GYRO_INT2_CTRL          0x0E
#define LSM6DSL_ACC_GYRO_WHO_AM_I_REG       0x0F
#define LSM6DSL_ACC_GYRO_CTRL1_XL           0x10
#define LSM6DSL_ACC_GYRO_CTRL2_G            0x11
#define LSM6DSL_ACC_GYRO_CTRL3_C            0x12
#define LSM6DSL_ACC_GYRO_CTRL4_C            0x13
#define LSM6DSL_ACC_GYRO_CTRL5_C            0x14
#define LSM6DSL_ACC_GYRO_CTRL6_C            0x15
#define LSM6DSL_ACC_GYRO_CTRL7_G            0x16
#define LSM6DSL_ACC_GYRO_CTRL8_XL           0x17
#define LSM6DSL_ACC_GYRO_CTRL9_XL           0x18
#define LSM6DSL_ACC_GYRO_CTRL10_C           0x19

#define LSM6DSL_ACC_GYRO_MASTER_CONFIG      0x1A
#define LSM6DSL_ACC_GYRO_WAKE_UP_SRC        0x1B
#define LSM6DSL_ACC_GYRO_TAP_SRC            0x1C
#define LSM6DSL_ACC_GYRO_D6D_SRC            0x1D
#define LSM6DSL_ACC_GYRO_STATUS_REG         0x1E

#define LSM6DSL_ACC_GYRO_OUT_TEMP_L         0x20
#define LSM6DSL_ACC_GYRO_OUT_TEMP_H         0x21
#define LSM6DSL_ACC_GYRO_OUTX_L_G           0x22
#define LSM6DSL_ACC_GYRO_OUTX_H_G           0x23
#define LSM6DSL_ACC_GYRO_OUTY_L_G           0x24
#define LSM6DSL_ACC_GYRO_OUTY_H_G           0x25
#define LSM6DSL_ACC_GYRO_OUTZ_L_G           0x26
#define LSM6DSL_ACC_GYRO_OUTZ_H_G           0x27
#define LSM6DSL_ACC_GYRO_OUTX_L_XL          0x28
#define LSM6DSL_ACC_GYRO_OUTX_H_XL          0x29
#define LSM6DSL_ACC_GYRO_OUTY_L_XL          0x2A
#define LSM6DSL_ACC_GYRO_OUTY_H_XL          0x2B
#define LSM6DSL_ACC_GYRO_OUTZ_L_XL          0x2C
#define LSM6DSL_ACC_GYRO_OUTZ_H_XL          0x2D
#define LSM6DSL_ACC_GYRO_SENSORHUB1_REG     0x2E
#define LSM6DSL_ACC_GYRO_SENSORHUB2_REG     0x2F
#define LSM6DSL_ACC_GYRO_SENSORHUB3_REG     0x30
#define LSM6DSL_ACC_GYRO_SENSORHUB4_REG     0x31
#define LSM6DSL_ACC_GYRO_SENSORHUB5_REG     0x32
#define LSM6DSL_ACC_GYRO_SENSORHUB6_REG     0x33
#define LSM6DSL_ACC_GYRO_SENSORHUB7_REG     0x34
#define LSM6DSL_ACC_GYRO_SENSORHUB8_REG     0x35
#define LSM6DSL_ACC_GYRO_SENSORHUB9_REG     0x36
#define LSM6DSL_ACC_GYRO_SENSORHUB10_REG    0x37
#define LSM6DSL_ACC_GYRO_SENSORHUB11_REG    0x38
#define LSM6DSL_ACC_GYRO_SENSORHUB12_REG    0x39
#define LSM6DSL_ACC_GYRO_FIFO_STATUS1       0x3A
#define LSM6DSL_ACC_GYRO_FIFO_STATUS2       0x3B
#define LSM6DSL_ACC_GYRO_FIFO_STATUS3       0x3C
#define LSM6DSL_ACC_GYRO_FIFO_STATUS4       0x3D
#define LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_L    0x3E
#define LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_H    0x3F
#define LSM6DSL_ACC_GYRO_TIMESTAMP0_REG     0x40
#define LSM6DSL_ACC_GYRO_TIMESTAMP1_REG     0x41
#define LSM6DSL_ACC_GYRO_TIMESTAMP2_REG     0x42

#define LSM6DSL_ACC_GYRO_TIMESTAMP_L        0x49
#define LSM6DSL_ACC_GYRO_TIMESTAMP_H        0x4A

#define LSM6DSL_ACC_GYRO_STEP_COUNTER_L     0x4B
#define LSM6DSL_ACC_GYRO_STEP_COUNTER_H     0x4C

#define LSM6DSL_ACC_GYRO_SENSORHUB13_REG    0x4D
#define LSM6DSL_ACC_GYRO_SENSORHUB14_REG    0x4E
#define LSM6DSL_ACC_GYRO_SENSORHUB15_REG    0x4F
#define LSM6DSL_ACC_GYRO_SENSORHUB16_REG    0x50
#define LSM6DSL_ACC_GYRO_SENSORHUB17_REG    0x51
#define LSM6DSL_ACC_GYRO_SENSORHUB18_REG    0x52

#define LSM6DSL_ACC_GYRO_FUNC_SRC           0x53
#define LSM6DSL_ACC_GYRO_TAP_CFG1           0x58
#define LSM6DSL_ACC_GYRO_TAP_THS_6D         0x59
#define LSM6DSL_ACC_GYRO_INT_DUR2           0x5A
#define LSM6DSL_ACC_GYRO_WAKE_UP_THS        0x5B
#define LSM6DSL_ACC_GYRO_WAKE_UP_DUR        0x5C
#define LSM6DSL_ACC_GYRO_FREE_FALL          0x5D
#define LSM6DSL_ACC_GYRO_MD1_CFG            0x5E
#define LSM6DSL_ACC_GYRO_MD2_CFG            0x5F

#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_X_L    0x66
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_X_H    0x67
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Y_L    0x68
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Y_H    0x69
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Z_L    0x6A
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Z_H    0x6B

#define LSM6DSL_ACC_GYRO_X_OFS_USR          0x73
#define LSM6DSL_ACC_GYRO_Y_OFS_USR          0x74
#define LSM6DSL_ACC_GYRO_Z_OFS_USR          0x75

/************** Embedded functions register mapping  *******************/
#define LSM6DSL_ACC_GYRO_SLV0_ADD                     0x02
#define LSM6DSL_ACC_GYRO_SLV0_SUBADD                  0x03
#define LSM6DSL_ACC_GYRO_SLAVE0_CONFIG                0x04
#define LSM6DSL_ACC_GYRO_SLV1_ADD                     0x05
#define LSM6DSL_ACC_GYRO_SLV1_SUBADD                  0x06
#define LSM6DSL_ACC_GYRO_SLAVE1_CONFIG                0x07
#define LSM6DSL_ACC_GYRO_SLV2_ADD                     0x08
#define LSM6DSL_ACC_GYRO_SLV2_SUBADD                  0x09
#define LSM6DSL_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define LSM6DSL_ACC_GYRO_SLV3_ADD                     0x0B
#define LSM6DSL_ACC_GYRO_SLV3_SUBADD                  0x0C
#define LSM6DSL_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define LSM6DSL_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DSL_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F

#define LSM6DSL_ACC_GYRO_SM_STEP_THS                  0x13
#define LSM6DSL_ACC_GYRO_PEDO_DEB_REG                 0x14
#define LSM6DSL_ACC_GYRO_STEP_COUNT_DELTA             0x15

#define LSM6DSL_ACC_GYRO_MAG_SI_XX                    0x24
#define LSM6DSL_ACC_GYRO_MAG_SI_XY                    0x25
#define LSM6DSL_ACC_GYRO_MAG_SI_XZ                    0x26
#define LSM6DSL_ACC_GYRO_MAG_SI_YX                    0x27
#define LSM6DSL_ACC_GYRO_MAG_SI_YY                    0x28
#define LSM6DSL_ACC_GYRO_MAG_SI_YZ                    0x29
#define LSM6DSL_ACC_GYRO_MAG_SI_ZX                    0x2A
#define LSM6DSL_ACC_GYRO_MAG_SI_ZY                    0x2B
#define LSM6DSL_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define LSM6DSL_ACC_GYRO_MAG_OFFX_L                   0x2D
#define LSM6DSL_ACC_GYRO_MAG_OFFX_H                   0x2E
#define LSM6DSL_ACC_GYRO_MAG_OFFY_L                   0x2F
#define LSM6DSL_ACC_GYRO_MAG_OFFY_H                   0x30
#define LSM6DSL_ACC_GYRO_MAG_OFFZ_L                   0x31
#define LSM6DSL_ACC_GYRO_MAG_OFFZ_H                   0x32
//#define DISCOVERY_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define DISCOVERY_I2Cx_CLK_DISABLE()               __HAL_RCC_I2C2_CLK_DISABLE()  
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define LSM6DSL_ODR_BITPOSITION      ((uint8_t)0xF0)  /*!< Output Data Rate bit position */
#define LSM6DSL_ODR_POWER_DOWN       ((uint8_t)0x00) /* Power Down mode       */


uint8_t BSP_GYRO_Init(void);
void BSP_GYRO_DeInit(void);
void BSP_GYRO_LowPower(uint16_t status);   /* 0 Means Disable Low Power Mode, otherwise Low Power Mode is enabled */
void BSP_GYRO_GetXYZ(float* pfData);
void Gyro_Test();
static void I2Cx_Init(I2C_HandleTypeDef* i2c_handler);
void LSM6DSL_GyroReadXYZAngRate(float* pfData);
uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg);
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef* hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef* hi2c);
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
static void I2Cx_Error(I2C_HandleTypeDef* i2c_handler, uint8_t Addr);

typedef struct
{
  void       (*Init)(uint16_t);
  void       (*DeInit)(void);
  uint8_t(*ReadID)(void);
  void       (*Reset)(void);
  void       (*LowPower)(uint16_t);
  void       (*ConfigIT)(uint16_t);
  void       (*EnableIT)(uint8_t);
  void       (*DisableIT)(uint8_t);
  uint8_t(*ITStatus)(uint16_t, uint16_t);
  void       (*ClearIT)(uint16_t, uint16_t);
  void       (*FilterConfig)(uint8_t);
  void       (*FilterCmd)(uint8_t);
  void       (*GetXYZ)(float*);
}GYRO_DrvTypeDef;

typedef enum
{
  BUTTON_USER = 0
}Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

/* GYRO High Pass Filter struct */
typedef struct
{
  uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
  uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
}GYRO_FilterConfigTypeDef;

/*GYRO Interrupt struct */
typedef struct
{
  uint8_t Latch_Request;                      /* Latch interrupt request into CLICK_SRC register */
  uint8_t Interrupt_Axes;                     /* X, Y, Z Axes Interrupts */
  uint8_t Interrupt_ActiveEdge;               /* Interrupt Active edge */
}GYRO_InterruptConfigTypeDef;


typedef enum
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
}
GYRO_StatusTypeDef;


uint32_t Serial_Scanf(uint32_t value);
static GYRO_DrvTypeDef* GyroscopeDrv;
const uint16_t BUTTON_PIN[BUTTONn] = { USER_BUTTON_PIN };
GPIO_TypeDef* BUTTON_PORT[BUTTONn] = { USER_BUTTON_GPIO_PORT };
const uint16_t BUTTON_IRQn[BUTTONn] = { USER_BUTTON_EXTI_IRQn };


int16_t pDataXYZ[3] = { 0 };
float pGyroDataXYZ[3] = { 0 };


static void I2Cx_MspInit(I2C_HandleTypeDef* i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  /* Configure I2C Tx, Rx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_I2Cx_SCL_PIN | DISCOVERY_I2Cx_SDA_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_OD;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Alternate = DISCOVERY_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

  HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

  /*** Configure the I2C peripheral ***/
  /* Enable I2C clock */
  DISCOVERY_I2Cx_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  DISCOVERY_I2Cx_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  DISCOVERY_I2Cx_RELEASE_RESET();

  /* Enable and set I2Cx Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

  /* Enable and set I2Cx Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn);
}

static void I2Cx_Init(I2C_HandleTypeDef* i2c_handler)
{
  /* I2C configuration */
  i2c_handler->Instance = DISCOVERY_I2Cx;
  i2c_handler->Init.Timing = DISCOVERY_I2Cx_TIMING;
  i2c_handler->Init.OwnAddress1 = 0;
  i2c_handler->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  i2c_handler->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  i2c_handler->Init.OwnAddress2 = 0;
  i2c_handler->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  i2c_handler->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  /* Init the I2C */
  I2Cx_MspInit(i2c_handler);
  HAL_I2C_Init(i2c_handler);

  /**Configure Analogue filter */
  HAL_I2CEx_ConfigAnalogFilter(i2c_handler, I2C_ANALOGFILTER_ENABLE);
}


static void I2Cx_Error(I2C_HandleTypeDef* i2c_handler, uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(i2c_handler);

  /* Re-Initialize the I2C communication bus */
  I2Cx_Init(i2c_handler);
}


static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef* i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t* Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}


static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef* i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t* Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* I2C error occured */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}


void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Value, 1);
}

uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(&hI2cHandler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);

  return read_value;
}

uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t* Buffer, uint16_t Length)
{
  return I2Cx_ReadMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

void LSM6DSL_GyroReadXYZAngRate(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlg = 0;
  uint8_t buffer[6];
  uint8_t i = 0;
  float sensitivity = 0;
  /* Read the gyro control register content */
  ctrlg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

  /* Read output register X, Y & Z acceleration */
  SENSOR_IO_ReadMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_OUTX_L_G, buffer, 6);

  for (i = 0; i < 3; i++)
  {
    pnRawData[i] = ((((uint16_t)buffer[2 * i + 1]) << 8) + (uint16_t)buffer[2 * i]);
  }

  /* Normal mode */
  /* Switch the sensitivity value set in the CRTL2_G */
  switch (ctrlg & 0x0C)
  {
  case LSM6DSL_GYRO_FS_245:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_245DPS;
    break;
  case LSM6DSL_GYRO_FS_500:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_500DPS;
    break;
  case LSM6DSL_GYRO_FS_1000:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_1000DPS;
    break;
  case LSM6DSL_GYRO_FS_2000:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_2000DPS;
    break;
  }

  /* Obtain the mg value for the three axis */
  for (i = 0; i < 3; i++)
  {
    pfData[i] = (float)(pnRawData[i] * sensitivity);
  }
}

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);



extern UART_HandleTypeDef hDiscoUart;


//--------------------------------------------------------------------------------------------------------------------------


void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Enable the BUTTON clock */
  //USER_BUTTON_GPIO_CLK_ENABLE();

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpio_init_structure.Pin = BUTTON_PIN[Button];
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);
  }

  if (ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    gpio_init_structure.Pin = BUTTON_PIN[Button];
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio_init_structure.Mode = GPIO_MODE_IT_RISING;

    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}


void BSP_GYRO_DeInit(void)
{
  /* DeInitialize the Gyroscope IO interfaces */
  if (GyroscopeDrv != NULL)
  {
    if (GyroscopeDrv->DeInit != NULL)
    {
      GyroscopeDrv->DeInit();
    }
  }
}


void BSP_GYRO_LowPower(uint16_t status)
{
  /* Set/Unset component in low-power mode */
  if (GyroscopeDrv != NULL)
  {
    if (GyroscopeDrv->LowPower != NULL)
    {
      GyroscopeDrv->LowPower(status);
    }
  }
}

/**
  * @brief  Get XYZ angular acceleration from the Gyroscope.
  * @param  pfData: pointer on floating array
  */
void BSP_GYRO_GetXYZ(float* pfData)
{
  if (GyroscopeDrv != NULL)
  {
    if (GyroscopeDrv->GetXYZ != NULL)
    {
      GyroscopeDrv->GetXYZ(pfData);
    }
  }
}


uint32_t Serial_Scanf(uint32_t value) //récupère le caractère taper au clavier et
                                      //excécute le programme si le caractère est "n"/"N" et quitte si il est "q"/"Q"
                                      //sinon demande à l'utilisateur de rentrer n/N ou Q/q
{
  uint16_t tmp = 0;

  tmp = getchar(); // 

  if (tmp > value)
  {
    printf("\n\r  !!! Please enter valid number between 0 and %lu \n", value);
    return 0xFF;
  }
  return tmp;
}

void LSM6DSL_GyroInit(uint16_t InitStruct)
{
  uint8_t ctrl = 0x00;
  uint8_t tmp;

  /* Read CTRL2_G */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

  /* Write value to GYRO MEMS CTRL2_G register: FS and Data Rate */
  ctrl = (uint8_t)InitStruct;
  tmp &= ~(0xFC);
  tmp |= ctrl;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G, tmp);

  /* Read CTRL3_C */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);

  /* Write value to GYRO MEMS CTRL3_C register: BDU and Auto-increment */
  ctrl = ((uint8_t)(InitStruct >> 8));
  tmp &= ~(0x44);
  tmp |= ctrl;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);
}

void LSM6DSL_GyroDeInit(void)
{
  uint8_t ctrl = 0x00;

  /* Read control register 1 value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

  /* Clear ODR bits */
  ctrl &= ~(LSM6DSL_ODR_BITPOSITION);

  /* Set Power down */
  ctrl |= LSM6DSL_ODR_POWER_DOWN;

  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G, ctrl);
}


static void I2Cx_MspDeInit(I2C_HandleTypeDef* i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Configure I2C Tx, Rx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_I2Cx_SCL_PIN | DISCOVERY_I2Cx_SDA_PIN;
  HAL_GPIO_DeInit(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, gpio_init_structure.Pin);
  /* Disable GPIO clock */
  DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_DISABLE();

  /* Disable I2C clock */
  DISCOVERY_I2Cx_CLK_DISABLE();
}
static void I2Cx_DeInit(I2C_HandleTypeDef* i2c_handler)
{  /* DeInit the I2C */
  I2Cx_MspDeInit(i2c_handler);
  HAL_I2C_DeInit(i2c_handler);
}

void SENSOR_IO_Init(void)
{
  I2Cx_Init(&hI2cHandler);
}

/**
  * @brief  DeInitializes Sensors low level.
  * @retval None
  */
void SENSOR_IO_DeInit(void)
{
  I2Cx_DeInit(&hI2cHandler);
}

uint8_t LSM6DSL_GyroReadID(void)
{
  /* IO interface initialization */
  SENSOR_IO_Init();
  /* Read value at Who am I register address */
  return SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WHO_AM_I_REG);
}

void LSM6DSL_GyroLowPower(uint16_t status)
{
  uint8_t ctrl = 0x00;

  /* Read CTRL7_G value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL7_G);

  /* Clear Low Power Mode bit */
  ctrl &= ~(0x80);

  /* Set Low Power Mode */
  if (status)
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_G_ENABLED;
  }
  else
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_G_DISABLED;
  }

  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL7_G, ctrl);
}

GYRO_DrvTypeDef Lsm6dslGyroDrv =
{
  LSM6DSL_GyroInit,
  LSM6DSL_GyroDeInit,
  LSM6DSL_GyroReadID,
  0,
  LSM6DSL_GyroLowPower,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  LSM6DSL_GyroReadXYZAngRate
};



void Gyro_Test(void)
{
  uint32_t ret = 0;
  printf("\n***************************************************************\n");
  printf("\n************************* Gyro Test ***************************\n");
  printf("\n***************************************************************\n\n");

  BSP_GYRO_Init();
  printf("\n*** Type n or N to get a first gyro data ***\n\n");
  printf("\n*** Type q or Q to quit Gyro Test ***\n\n");
  while (1)
  {
    ret = Serial_Scanf(255);
    if ((ret == 'n') || (ret == 'N'))
    {
      printf("\n*** This is a new data ***\n\n");
      BSP_GYRO_GetXYZ(pGyroDataXYZ);
      printf("GYRO_X = %.2f \n", pGyroDataXYZ[0]);
      printf("GYRO_Y = %.2f \n", pGyroDataXYZ[1]);
      printf("GYRO_Z = %.2f \n", pGyroDataXYZ[2]);
      printf("\n*** This is a new data ***\n\n");
      printf("\n*** Type n or N to get a new data ***\n\n");
      printf("\n*** Type q or Q to quit Gyro Test ***\n\n");
    }
    else if ((ret == 'q') || (ret == 'Q'))
    {
      BSP_GYRO_DeInit();
      printf("\n*** End of Gyro Test ***\n\n");
      return;
    }
    else
    {
      printf("\n*** Type n or N to get a new data ***\n\n");
      printf("\n*** Type q or Q to quit Gyro Test ***\n\n");
    }
  }
}



//--------------------------------------------------------------------------------------------------------------------------


int main(void)
{
  HAL_Init();

  /* Configure the System clock to have a frequency of 80 MHz */
  SystemClock_Config();

  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  while (1)
  {
    Gyro_Test();
  }
}


static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while (1);
  }

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while (1);
  }
}



PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the serial port and Loop until the end of transmission */
  while (HAL_OK != HAL_UART_Transmit(&hDiscoUart, (uint8_t*)&ch, 1, 30000))
  {
    ;
  }
  return ch;
}


GETCHAR_PROTOTYPE
{
  /* Place your implementation of fgetc here */
  /* e.g. readwrite a character to the USART2 and Loop until the end of transmission */
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t*)&ch, 1, 30000))
  {
    ;
  }
  return ch;
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  printf("!!! ERROR !!!\n");
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
