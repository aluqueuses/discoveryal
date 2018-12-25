/**
 ******************************************************************************
 * @file    bsp_env_sensors_ex.c
 * @author  MEMS Application Team
 * @version V0.0.1
 * @date    29-January-2018
 * @brief   This file provides BSP Environmental Sensors Extended interface
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_env_sensors_ex.h"

extern void *EnvCompObj[ENV_INSTANCES_NBR];

/**
 * @brief  Get the data stored in FIFO
 * @param  Instance the device instance
 * @param  Press the pressure data
 * @param  Temp the temperature data
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Get_Data(uint32_t Instance, float *Press, float *Temp)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Get_Data(EnvCompObj[Instance], Press, Temp) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get FIFO THR status
 * @param  Instance the device instance
 * @param  Status the pointer to the status
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Get_Fth_Status(uint32_t Instance, uint8_t *Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Get_FTh_Status(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get FIFO Full status
 * @param  Instance the device instance
 * @param  Status the pointer to the status
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Get_Full_Status(uint32_t Instance, uint8_t *Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Get_Full_Status(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get the number of unread samples in FIFO
 * @param  Instance the device instance
 * @param  NumSamples the number of unread FIFO samples
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Get_Num_Samples(uint32_t Instance, uint8_t *NumSamples)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Get_Level(EnvCompObj[Instance], NumSamples) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get FIFO OVR status
 * @param  Instance the device instance
 * @param  Status the pointer to the status
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Get_Ovr_Status(uint32_t Instance, uint8_t *Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Get_Ovr_Status(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Reset FIFO Interrupt
 * @param  Instance the device instance
 * @param  Interrupt FIFO interrupt. Could be: FTH, FULL or OVR
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Reset_Interrupt(uint32_t Instance, uint8_t Interrupt)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Reset_Interrupt(EnvCompObj[Instance], Interrupt) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO Interrupt
 * @param  Instance the device instance
 * @param  Interrupt FIFO interrupt. Could be: FTH, FULL or OVR
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Set_Interrupt(uint32_t Instance, uint8_t Interrupt)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Set_Interrupt(EnvCompObj[Instance], Interrupt) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO mode
 * @param  Instance the device instance
 * @param  Mode FIFO mode
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Set_Mode(uint32_t Instance, uint8_t Mode)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Set_Mode(EnvCompObj[Instance], Mode) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO watermark
 * @param  Instance the device instance
 * @param  Watermark FIFO data level threshold
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Set_Watermark_Level(uint32_t Instance, uint8_t Watermark)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Set_Watermark_Level(EnvCompObj[Instance], Watermark) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable FIFO usage
 * @param  Instance the device instance
 * @param  Status the pointer to the status
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_FIFO_Usage(uint32_t Instance, uint8_t Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      ret = BSP_ERROR_COMPONENT_FAILURE;
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_FIFO_Usage(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get the status of data ready bit
 * @param  Instance the device instance
 * @param  Function Environmental sensor function
 * @param  Status the pointer to the status
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_Get_DRDY_Status(uint32_t Instance, uint32_t Function, uint8_t *Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      if ((Function & ENV_HUMIDITY) == ENV_HUMIDITY)
      {
        if (HTS221_HUM_Get_DRDY_Status(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & ENV_TEMPERATURE) == ENV_TEMPERATURE)
      {
        if (HTS221_TEMP_Get_DRDY_Status(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if ((Function & ENV_PRESSURE) == ENV_PRESSURE)
      {
        if (LPS22HB_PRESS_Get_DRDY_Status(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & ENV_TEMPERATURE) == ENV_TEMPERATURE)
      {
        if (LPS22HB_TEMP_Get_DRDY_Status(EnvCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get the register value
 * @param  Instance the device instance
 * @param  Reg address to be read
 * @param  Data pointer where the value is written to
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_Read_Register(uint32_t Instance, uint8_t Reg, uint8_t *Data)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      if (HTS221_Read_Reg(EnvCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_Read_Reg(EnvCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the register value
 * @param  Instance the device instance
 * @param  Reg address to be read
 * @param  Data value to be written
 * @retval BSP status
 */
int32_t BSP_ENV_SENSOR_Write_Register(uint32_t Instance, uint8_t Reg, uint8_t Data)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_ENV_SENSOR_HTS221_0 == 1)
    case HTS221_0:
      if (HTS221_Write_Reg(EnvCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
    case LPS22HB_0:
      if (LPS22HB_Write_Reg(EnvCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
