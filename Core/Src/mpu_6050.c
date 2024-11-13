/*
 * Author : Roche Christopher
 * email  : rochextopher@gmail.com
 *
 */

/*
 *
	MIT License

	Copyright (c) 2024 Roche Christopher

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 *
 */


#include "mpu_6050.h"

const uint16_t mpu6050_dev_addr = 0x68 << 1; //0x68 << 1;
int gyroscope_output_rate = 1000; // 8khz

int write_to_register(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t *reg_value){
	HAL_I2C_Mem_Write_IT(hi2c, mpu6050_dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_value, 1);
	while(hi2c->State!= HAL_I2C_STATE_READY);
	return 0;
}

int read_from_register(I2C_HandleTypeDef *hi2c, uint16_t reg_addr, uint8_t *reg_value){
	HAL_I2C_Mem_Read_IT(hi2c, mpu6050_dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_value, 1);
	while(hi2c->State!= HAL_I2C_STATE_READY);
	return 0;
}

int set_interrupt_enable(I2C_HandleTypeDef *hi2c, uint8_t *interrupt_enable_value){
	write_to_register(hi2c, MPU6050_INT_EN_REG, interrupt_enable_value);
	return 0;
}


int set_interrupt_pin_configuration(I2C_HandleTypeDef *hi2c, uint8_t *interrupt_pin_config){
	write_to_register(hi2c, MPU6050_INT_PIN_CFG_REG, interrupt_pin_config);
	return 0;
}

int set_sampling_rate(I2C_HandleTypeDef *hi2c, int  sampling_rate){ // sampling rate in hz

	uint8_t smplrt_div_value = (gyroscope_output_rate/sampling_rate) - 1;
	// set the smprlt to the register
	write_to_register(hi2c, MPU6050_SMPLRT_DIV_REG, &smplrt_div_value);

	return 0;
}

int set_smplrt_div(I2C_HandleTypeDef *hi2c, uint8_t *smplrt_value){
	//smplrt_value = 199;
	HAL_I2C_Mem_Write_IT(hi2c, 0x68 << 1, MPU6050_SMPLRT_DIV_REG, 1, smplrt_value, 1);
	//write_to_register(hi2c, MPU6050_SMPLRT_DIV_REG, &smplrt_value);
	return 0;
}

int set_configuration(I2C_HandleTypeDef *hi2c, uint8_t *configuration){
	// TODO: change the gyroscope rate based on the configuration.
	write_to_register(hi2c, MPU6050_CONFIG_REG, configuration);
	return 0;
}

int set_gyroscope_configuration(I2C_HandleTypeDef *hi2c, uint8_t *gyro_config){
	write_to_register(hi2c, MPU6050_GYRO_CONFIG_REG, gyro_config);
	return 0;
}

int clear_interrupt_status(I2C_HandleTypeDef *hi2c){
	write_to_register(hi2c, MPU6050_INT_STATUS_REG, 0b00000000);
	return 0;
}

int write_to_power_mgmt(I2C_HandleTypeDef *hi2c, uint8_t *pwr_mgmt_reg_value){
	write_to_register(hi2c, MPU6050_POWER_MGMT_REG, pwr_mgmt_reg_value);
	return 0;
}

int get_interrupt_status(I2C_HandleTypeDef *hi2c, uint8_t *interrupt_status){
	read_from_register(hi2c, MPU6050_INT_STATUS_REG, interrupt_status);
	return 0;
}

int get_gyro_x(I2C_HandleTypeDef *hi2c, int16_t *gyro_x){
	uint8_t gyro_low, gyro_high;
	read_from_register(hi2c, MPU6050_GYRO_X_HIGH, &gyro_high);
	HAL_Delay(1);
	read_from_register(hi2c, MPU6050_GYRO_X_LOW, &gyro_low);
	HAL_Delay(1);

	*gyro_x = 0;
	*gyro_x |= (uint16_t)gyro_low;
	*gyro_x |= (uint16_t)gyro_high << 8;
	return 0;
}

int set_user_control(I2C_HandleTypeDef *hi2c, uint8_t *user_control_value){
	write_to_register(hi2c, MPU6050_USER_CTRL, user_control_value);
	return 0;
}
int set_fifo_enable(I2C_HandleTypeDef *hi2c, uint8_t *fifo_enable_value){
	write_to_register(hi2c, MPU6050_FIFO_EN, fifo_enable_value);
	return 0;
}
uint16_t get_fifo_count(I2C_HandleTypeDef *hi2c){
	uint16_t fifo_count = 0;
	uint8_t value = 0;
	read_from_register(hi2c, MPU6050_FIFO_COUNTH, &value);
	fifo_count |= value << 8;
	read_from_register(hi2c, MPU6050_FIFO_COUNTL, &value);
	fifo_count |= value;

	return fifo_count;
}
int read_from_fifo(I2C_HandleTypeDef *hi2c, uint8_t *buffer_data){
	read_from_register(hi2c, MPU6050_FIFO_BUFFER, buffer_data);
	return 0;
}
