/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  MEMS Software Solution Team
 * @date    30-November-2017
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "lsm6dsl_reg.h"
#include <string.h>
#include <stdio.h>
typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

#define TX_BUF_DIM          1000
#define MAX_BUF_SIZE 256
/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

char data_out[MAX_BUF_SIZE];
uint8_t int1_flag = 0;
stmdev_ctx_t dev_ctx;
/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   Replace the functions "platform_write" and "platform_read" with your
 *   platform specific read and write function.
 *   This example use an STM32 evaluation board and CubeMX tool.
 *   In this case the "*handle" variable is useful in order to select the
 *   correct interface but the usage of "*handle" is not mandatory.
 */

static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_L, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
      HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_L, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }

  return 0;
}

/*
 *  Function to print messages
 */
void tx_com( uint8_t *tx_buffer, uint16_t len )
{ 
  HAL_UART_Transmit( &huart1, tx_buffer, len, 1000 );
}

/* Main Example --------------------------------------------------------------*/
void example_main(void)
{
  /*
   *  Initialize mems driver interface
   */
//  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1; 
  /*
   *  Check device ID
   */
  whoamI = 0;
  lsm6dsl_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != LSM6DSL_ID )
		while(1); /*manage here device not found */
	printf("Find the right device ID\r\n");
  /*
   *  Restore default configuration
   */
  lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsl_reset_get(&dev_ctx, &rst);
  } while (rst);
  /*
   *  Enable Block Data Update
   */
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_xl_power_mode_set(&dev_ctx,LSM6DSL_XL_NORMAL);			//设置功耗模式：XL_HM_MODE = 1 :high-performance operating mode disabled.
  /*
   * Set Output Data Rate
   */
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5);	//12.5Hz
  lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_12Hz5);
  /*
   * Set full scale
   */ 
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
 
  /*
   * Configure filtering chain(No aux interface)
   */ 
  /* Accelerometer - analog filter */
  lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
 
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
 
  /* Accelerometer - LPF1 + LPF2 path */  
  lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
 
  /* Accelerometer - High Pass / Slope path */
  //lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  //lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);
 
  /* Gyroscope - filtering chain */
  lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);
 
  /*
   * Read samples in polling mode (no int)
   */
//  while(1)
	for(uint8_t i = 0;i < 2; i++) //循环5次
  {
		HAL_Delay(1000 * 3);
    /*
     * Read output only if new value is available
     */
    lsm6dsl_reg_t reg;
    lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda)
    {
      /* Read magnetic field data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[2]);
     
      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
    if (reg.status_reg.gda)
    {
      /* Read magnetic field data */
      memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
      lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      angular_rate_mdps[0] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
     
      sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }   
    if (reg.status_reg.tda)
    {  
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lsm6dsl_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC = lsm6dsl_from_lsb_to_celsius( data_raw_temperature.i16bit );
      
      sprintf((char*)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC );
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}

void lsm6dsl_free_fall_detection(void)
{  
  /*
   *  Initialize mems driver interface
   */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;  
  
  /*
   *  Check device ID
   */
  whoamI = 0;
  lsm6dsl_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != LSM6DSL_ID )
    while(1); /*manage here device not found */
	printf("Find the right device ID\r\n");
  /*
   *  Restore default configuration
   */
  lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsl_reset_get(&dev_ctx, &rst);
  } while (rst);
  /*
   *  Enable Block Data Update
   */
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsl_xl_power_mode_set(&dev_ctx,LSM6DSL_XL_NORMAL);			//设置功耗模式：XL_HM_MODE = 1 :high-performance operating mode disabled.
//	 lsm6dsl_xl_power_mode_set(&dev_ctx,LSM6DSL_XL_HIGH_PERFORMANCE);	
	/*
   * Set Output Data Rate
   */
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_52Hz);   //   LSM6DSL_XL_ODR_52Hz :low power mode
  /*
   * Set full scale
   */  
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
	lsm6dsl_int_notification_set(&dev_ctx, LSM6DSL_INT_LATCHED); // latched interrupt
	
  lsm6dsl_wkup_dur_set(&dev_ctx,0);
//  lsm6dsl_timestamp_res_set(&dev_ctx,LSM6DSL_LSB_6ms4);
//  lsm6dsl_act_sleep_dur_set(&dev_ctx,0);
  lsm6dsl_ff_dur_set(&dev_ctx,1);  // 1/52 
  lsm6dsl_ff_threshold_set(&dev_ctx,LSM6DSL_FF_TSH_312mg);  //LSM6DSL_FF_TSH_312mg
  /*
   * 中断
   */  
  lsm6dsl_int1_route_t lsm6dsl_int1_route;
  lsm6dsl_int1_route.int1_ff = 1;
  lsm6dsl_pin_int1_route_set(&dev_ctx,lsm6dsl_int1_route);
//  lsm6dsl_all_sources_t lsm6dsl_all_sources;
//  while(1)
//  {
//    if(int1_flag == 1)
//    {
//      int1_flag = 0;
//      lsm6dsl_all_sources_get(&dev_ctx,&lsm6dsl_all_sources);
//      if(lsm6dsl_all_sources.wake_up_src.ff_ia == 1)
//      {
//        
//        snprintf(data_out, MAX_BUF_SIZE, "Free fall indication.\r\n");
//        tx_com((uint8_t*)data_out, strlen(data_out));
//				
////				memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
////				lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
////				acceleration_mg[0] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[0]);
////				acceleration_mg[1] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[1]);
////				acceleration_mg[2] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[2]);
////			 
////				sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
////								acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
////				tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
//				
//				
//      }
//			
//    }
//  }
}

void lsm6dsl_exti_config(void)
{ 
  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOD_CLK_ENABLE();//open clock    PD11
  GPIO_InitStruct.Pin = LSM6DSL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//上升沿
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LSM6DSL_INT1_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}