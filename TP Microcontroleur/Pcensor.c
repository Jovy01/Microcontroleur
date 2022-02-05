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




void Pressure_Test(void);
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
void Error_Handler(void);
void BSP_PB_Init();
float BSP_PSENSOR_ReadPressure(void);

HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef* hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef* hi2c);


typedef enum
{
  PSENSOR_OK = 0,
  PSENSOR_ERROR
}PSENSOR_Status_TypDef;

typedef enum
{
  BUTTON_USER = 0
}Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;


typedef struct
{
  void       (*Init)(uint16_t);
  uint8_t(*ReadID)(uint16_t);
  float      (*ReadPressure)(uint16_t);
}PSENSOR_DrvTypeDef;


uint32_t BSP_PSENSOR_Init(void);
uint8_t  BSP_PSENSOR_ReadID(void);
uint32_t Serial_Scanf(uint32_t value);

float    BSP_PSENSOR_ReadPressure(void);
const uint16_t BUTTON_PIN[BUTTONn] = { USER_BUTTON_PIN };
GPIO_TypeDef* BUTTON_PORT[BUTTONn] = { USER_BUTTON_GPIO_PORT };
const uint16_t BUTTON_IRQn[BUTTONn] = { USER_BUTTON_EXTI_IRQn };



/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static PSENSOR_DrvTypeDef* Psensor_drv;

/* Private functions ---------------------------------------------------------*/

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
}


HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef* hi2c)
{
  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_OWN_ADDRESS1(hi2c->Init.OwnAddress1));
  assert_param(IS_I2C_ADDRESSING_MODE(hi2c->Init.AddressingMode));
  assert_param(IS_I2C_DUAL_ADDRESS(hi2c->Init.DualAddressMode));
  assert_param(IS_I2C_OWN_ADDRESS2(hi2c->Init.OwnAddress2));
  assert_param(IS_I2C_OWN_ADDRESS2_MASK(hi2c->Init.OwnAddress2Masks));
  assert_param(IS_I2C_GENERAL_CALL(hi2c->Init.GeneralCallMode));
  assert_param(IS_I2C_NO_STRETCH(hi2c->Init.NoStretchMode));

  if (hi2c->State == HAL_I2C_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hi2c->Lock = HAL_UNLOCKED;

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    /* Init the I2C Callback settings */
    hi2c->MasterTxCpltCallback = HAL_I2C_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
    hi2c->MasterRxCpltCallback = HAL_I2C_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
    hi2c->SlaveTxCpltCallback = HAL_I2C_SlaveTxCpltCallback;  /* Legacy weak SlaveTxCpltCallback  */
    hi2c->SlaveRxCpltCallback = HAL_I2C_SlaveRxCpltCallback;  /* Legacy weak SlaveRxCpltCallback  */
    hi2c->ListenCpltCallback = HAL_I2C_ListenCpltCallback;   /* Legacy weak ListenCpltCallback   */
    hi2c->MemTxCpltCallback = HAL_I2C_MemTxCpltCallback;    /* Legacy weak MemTxCpltCallback    */
    hi2c->MemRxCpltCallback = HAL_I2C_MemRxCpltCallback;    /* Legacy weak MemRxCpltCallback    */
    hi2c->ErrorCallback = HAL_I2C_ErrorCallback;        /* Legacy weak ErrorCallback        */
    hi2c->AbortCpltCallback = HAL_I2C_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */
    hi2c->AddrCallback = HAL_I2C_AddrCallback;         /* Legacy weak AddrCallback         */

    if (hi2c->MspInitCallback == NULL)
    {
      hi2c->MspInitCallback = HAL_I2C_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    hi2c->MspInitCallback(hi2c);
#else
    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    HAL_I2C_MspInit(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }

  hi2c->State = HAL_I2C_STATE_BUSY;

  /* Disable the selected I2C peripheral */
  __HAL_I2C_DISABLE(hi2c);

  /*---------------------------- I2Cx TIMINGR Configuration ------------------*/
  /* Configure I2Cx: Frequency range */
  hi2c->Instance->TIMINGR = hi2c->Init.Timing & TIMING_CLEAR_MASK;

  /*---------------------------- I2Cx OAR1 Configuration ---------------------*/
  /* Disable Own Address1 before set the Own Address1 configuration */
  hi2c->Instance->OAR1 &= ~I2C_OAR1_OA1EN;

  /* Configure I2Cx: Own Address1 and ack own address1 mode */
  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    hi2c->Instance->OAR1 = (I2C_OAR1_OA1EN | hi2c->Init.OwnAddress1);
  }
  else /* I2C_ADDRESSINGMODE_10BIT */
  {
    hi2c->Instance->OAR1 = (I2C_OAR1_OA1EN | I2C_OAR1_OA1MODE | hi2c->Init.OwnAddress1);
  }

  /*---------------------------- I2Cx CR2 Configuration ----------------------*/
  /* Configure I2Cx: Addressing Master mode */
  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
  {
    hi2c->Instance->CR2 = (I2C_CR2_ADD10);
  }
  /* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process */
  hi2c->Instance->CR2 |= (I2C_CR2_AUTOEND | I2C_CR2_NACK);

  /*---------------------------- I2Cx OAR2 Configuration ---------------------*/
  /* Disable Own Address2 before set the Own Address2 configuration */
  hi2c->Instance->OAR2 &= ~I2C_DUALADDRESS_ENABLE;

  /* Configure I2Cx: Dual mode and Own Address2 */
  hi2c->Instance->OAR2 = (hi2c->Init.DualAddressMode | hi2c->Init.OwnAddress2 | (hi2c->Init.OwnAddress2Masks << 8));

  /*---------------------------- I2Cx CR1 Configuration ----------------------*/
  /* Configure I2Cx: Generalcall and NoStretch mode */
  hi2c->Instance->CR1 = (hi2c->Init.GeneralCallMode | hi2c->Init.NoStretchMode);

  /* Enable the selected I2C peripheral */
  __HAL_I2C_ENABLE(hi2c);

  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_READY;
  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode = HAL_I2C_MODE_NONE;

  return HAL_OK;
}

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



/* Init the I2C */
// I2Cx_MspInit(*i2c_handler);
//HAL_I2C_Init(i2c_handler);


HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef* hi2c, uint32_t AnalogFilter)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_ANALOG_FILTER(AnalogFilter));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State = HAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __HAL_I2C_DISABLE(hi2c);

    /* Reset I2Cx ANOFF bit */
    hi2c->Instance->CR1 &= ~(I2C_CR1_ANFOFF);

    /* Set analog filter bit*/
    hi2c->Instance->CR1 |= AnalogFilter;

    __HAL_I2C_ENABLE(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


/**Configure Analogue filter */
// HAL_I2CEx_ConfigAnalogFilter(i2c_handler, I2C_ANALOGFILTER_ENABLE);  


static void I2Cx_Error(I2C_HandleTypeDef* i2c_handler, uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(i2c_handler);

  /* Re-Initialize the I2C communication bus */
  I2Cx_Init(i2c_handler);
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

extern UART_HandleTypeDef hDiscoUart;

uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(&hI2cHandler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);

  return read_value;
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

void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Value, 1);
}

static void LPS22HB_Init(uint16_t DeviceAddr)
{
  uint8_t tmp;

  /* Set Power mode */
  tmp = SENSOR_IO_Read(DeviceAddr, LPS22HB_RES_CONF_REG);

  tmp &= ~LPS22HB_LCEN_MASK;
  tmp |= (uint8_t)0x01; /* Set low current mode */

  SENSOR_IO_Write(DeviceAddr, LPS22HB_RES_CONF_REG, tmp);

  /* Read CTRL_REG1 */
  tmp = SENSOR_IO_Read(DeviceAddr, LPS22HB_CTRL_REG1);

  /* Set default ODR */
  tmp &= ~LPS22HB_ODR_MASK;
  tmp |= (uint8_t)0x30; /* Set ODR to 25Hz */

  /* Enable BDU */
  tmp &= ~LPS22HB_BDU_MASK;
  tmp |= ((uint8_t)0x02);

  /* Apply settings to CTRL_REG1 */
  SENSOR_IO_Write(DeviceAddr, LPS22HB_CTRL_REG1, tmp);
}



void LPS22HB_P_Init(uint16_t DeviceAddr)
{
  LPS22HB_Init(DeviceAddr);
}


void SENSOR_IO_Init(void)
{
  I2Cx_Init(&hI2cHandler);
}

uint8_t LPS22HB_P_ReadID(uint16_t DeviceAddr)
{
  uint8_t ctrl = 0x00;

  /* IO interface initialization */
  SENSOR_IO_Init();

  /* Read value at Who am I register address */
  ctrl = SENSOR_IO_Read(DeviceAddr, LPS22HB_WHO_AM_I_REG);

  return ctrl;
}


float LPS22HB_P_ReadPressure(uint16_t DeviceAddr)
{
  int32_t raw_press;
  uint8_t buffer[3];
  uint32_t tmp = 0;
  uint8_t i;

  for (i = 0; i < 3; i++)
  {
    buffer[i] = SENSOR_IO_Read(DeviceAddr, (LPS22HB_PRESS_OUT_XL_REG + i));
  }

  /* Build the raw data */
  for (i = 0; i < 3; i++)
    tmp |= (((uint32_t)buffer[i]) << (8 * i));

  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if (tmp & 0x00800000)
    tmp |= 0xFF000000;

  raw_press = ((int32_t)tmp);

  raw_press = (raw_press * 100) / 4096;

  return (float)((float)raw_press / 100.0f);
}


/* Pressure Private Variables */
PSENSOR_DrvTypeDef LPS22HB_P_Drv =
{
  LPS22HB_P_Init,
  LPS22HB_P_ReadID,
  LPS22HB_P_ReadPressure
};


//--------------------------------------------------------------------------------------------------------------------------

uint32_t BSP_PSENSOR_Init(void)
{
  uint32_t ret;

  if (LPS22HB_P_Drv.ReadID(LPS22HB_I2C_ADDRESS) != LPS22HB_WHO_AM_I_VAL)
  {
    ret = PSENSOR_ERROR;
  }
  else
  {
    Psensor_drv = &LPS22HB_P_Drv;

    /* PSENSOR Init */
    Psensor_drv->Init(LPS22HB_I2C_ADDRESS);
    ret = PSENSOR_OK;
  }

  return ret;
}

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


uint8_t BSP_PSENSOR_ReadID(void)
{
  return Psensor_drv->ReadID(LPS22HB_I2C_ADDRESS);
}


float BSP_PSENSOR_ReadPressure(void)
{
  return Psensor_drv->ReadPressure(LPS22HB_I2C_ADDRESS);
}

void Pressure_Test(void)
{
  uint32_t ret = 0;
  float press_value = 0;

  printf("\n*************************************************************\n");
  printf("\n********************** Pressure Test ************************\n");
  printf("\n*************************************************************\n\n");
  BSP_PSENSOR_Init();
  printf("\n*** Type n or N to get a first Pressure data ***\n\n");
  printf("\n*** Type q or Q to quit Pressure Test ***\n\n");
  while (1)
  {
    ret = Serial_Scanf(255);
    if ((ret == 'n') || (ret == 'N'))
    {
      printf("\n*** This is a new data ***\n\n");
      press_value = BSP_PSENSOR_ReadPressure();
      printf("PRESSURE is = %.2f mBar \n", press_value);
      printf("\n*** This is a new data ***\n\n");
      printf("\n*** Type n or N to get a new data ***\n\n");
      printf("\n*** Type q or Q to quit Pressure Test ***\n\n");
    }
    else if ((ret == 'q') || (ret == 'Q'))
    {
      printf("\n*** End of Pressure Test ***\n\n");
      return;
    }
    else
    {
      printf("\n*** Type n or N to get a new data ***\n\n");
      printf("\n*** Type q or Q to quit Pressure Test ***\n\n");
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
    Pressure_Test();
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

    */
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

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval None
  */
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

  */
void assert_failed(uint8_t* file, uint32_t line)
{
 
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/