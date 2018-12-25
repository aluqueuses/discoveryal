/**
  ******************************************************************************
  * File Name          :  stmicroelectronics_x-cube-mems1_5_2_1.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.5.2.1 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2018 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "app_mems-library.h"
#include "main.h"
#include "stm32f413h_discovery.h"
#include "bsp_motion_sensors.h"

#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_exti.h"
#include <stdlib.h> /* abs */

 /* Private typedef -----------------------------------------------------------*/
 typedef enum
 {
   STATUS_SELFTEST,
   STATUS_SLEEP
 } DEMO_STATUS;

 /* Private define ------------------------------------------------------------*/
 #define MAX_BUF_SIZE 256
 #define INDICATION_DELAY  1000 /* LED is ON for this period [ms]. */

 #define X_POWER_UP_DELAY    100 /*!< Delay after accelero power-up [ms] */
 #define X_ST_ENABLED_DELAY  100 /*!< Delay after accelero self-test enabled [ms] */
 #define G_POWER_UP_DELAY    150 /*!< Delay after gyro power-up [ms] */
 #define G_ST_ENABLED_DELAY   50 /*!< Delay after gyro self-test enabled [ms] */

 #define N_SAMPLES  5 /*!< Number of samples */

 #define X_LO_LIM      90 /*!< Accelero low test limit [mg] */
 #define X_HI_LIM    1700 /*!< Accelero high test limit [mg] */
 #define G_LO_LIM  150000 /*!< Gyro low test limit [mdps] */
 #define G_HI_LIM  700000 /*!< Gyro high test limit [mdps] */

 #define ST_REG_COUNT  (sizeof(reg_addr) / sizeof(uint8_t))

 /* Private macro -------------------------------------------------------------*/
 /* Private variables ---------------------------------------------------------*/
 static volatile uint8_t PushButtonDetected = 0;
 static char dataOut[MAX_BUF_SIZE];
 static DEMO_STATUS DemoStatus = STATUS_SLEEP;
 /* Refer to Datasheet / Application Note documents for details about following register settings */
 static uint8_t reg_addr[]        = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19};
 static uint8_t x_st_reg_values[] = {0x38, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
 static uint8_t g_st_reg_values[] = {0x00, 0x5C, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

 /* Private function prototypes -----------------------------------------------*/
 static void MX_IKS01A2_LSM6DSL_SelfTest_Init(void);
 static void MX_IKS01A2_LSM6DSL_SelfTest_Process(void);
 static void Sleep_Mode(void);
 static int32_t LSM6DSL_X_SelfTest(void);
 static int32_t LSM6DSL_G_SelfTest(void);


void MX_X_CUBE_MEMS1_Init(void)
{
   BSP_MOTION_SENSOR_Axes_t Axes;
  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Library_Init_PreTreatment */
  
  /* USER CODE END MEMS_Library_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  /* USER CODE BEGIN SV */ 
	BSP_MOTION_SENSOR_Init(LSM6DSL_0,MOTION_ACCELERO);
	BSP_MOTION_SENSOR_GetAxes(LSM6DSL_0, MOTION_ACCELERO, &Axes);

	/* USER CODE END SV */
  
  /* USER CODE BEGIN MEMS_Library_Init_PostTreatment */
  
  /* USER CODE END MEMS_Library_Init_PostTreatment */
}
/*
 * LM background task
 */
void MX_X_CUBE_MEMS1_Process(void)
{
  /* USER CODE BEGIN MEMS_Library_Process */

  /* USER CODE END MEMS_Library_Process */
}

/**
  * @brief  Initialize the LSM6DSL Self Test application
  * @retval None
  */
void MX_IKS01A2_LSM6DSL_SelfTest_Init(void)
{
  /* Initialize LED */
  BSP_LED_Init(LED3);

  /* Initialize button */
  BSP_PB_Init(BUTTON_WAKEUP, BUTTON_MODE_GPIO);

  /* Initialize Virtual COM Port */
  //BSP_COM_Init(COM1);

  //(void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);

  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n------ LSM6DSL self-test DEMO ------\r\n");
  printf("%s", dataOut);
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
