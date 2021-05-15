/*
 * irs.c
 *
 *  Created on: Sep 29, 2020
 *      Author: Bradley Schulz
 */

#include "main.h"
#include "irs.h"
#include "delay.h"
#include <stdbool.h>


// This is the buffer that will get filled up with all the measurements
uint16_t adc_buf[NUM_SAMPLES];
// "boolean" variable to keep say when the ADC has finished filling up the buffer
uint8_t complete = 0;


/*
 This function should handle everything for reading a specific IR
 First turn on the correct IR emitter
 Wait for a small amount of time (at least 20 us) so the photodiode can react
 Then read the correct receiver
 Lastly turn off the emitter
 */
uint16_t readIR(IR ir)
{
	   switch(ir)
	    {
	        case IR_LEFT:
	        	HAL_GPIO_WritePin(LeftEmitter_GPIO_Port, LeftEmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(60);
	        	return readLeftIR();
	        	HAL_GPIO_WritePin(LeftEmitter_GPIO_Port, LeftEmitter_Pin, GPIO_PIN_RESET);
	            break;
	        case IR_LFRONT:
	        	HAL_GPIO_WritePin(FrontLEmitter_GPIO_Port, FrontLEmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(60);
	        	return readFrontLIR();
	        	HAL_GPIO_WritePin(FrontLEmitter_GPIO_Port, FrontLEmitter_Pin, GPIO_PIN_RESET);
	            break;
	        case IR_RFRONT:
	        	HAL_GPIO_WritePin(FrontREmitter_GPIO_Port, FrontREmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(60);
	        	return readFrontRIR();
	        	HAL_GPIO_WritePin(FrontREmitter_GPIO_Port, FrontREmitter_Pin, GPIO_PIN_RESET);
	            break;
	        case IR_RIGHT:
	        	HAL_GPIO_WritePin(RightEmitter_GPIO_Port, RightEmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(60);
	        	return readRightIR();
	        	HAL_GPIO_WritePin(RightEmitter_GPIO_Port, RightEmitter_Pin, GPIO_PIN_RESET);
	    }
}

/*
 Implement the following 3 functions to read each IR
 Hint: Each function may just be one line
 */
uint16_t readLeftIR(void)
{
	analogRead(IR_LEFT);
}

uint16_t readFrontLIR(void)
{
	analogRead(IR_LFRONT);
}

uint16_t readFrontRIR(void)
{
	analogRead(IR_RFRONT);
}

uint16_t readRightIR(void)
{
	analogRead(IR_RIGHT);
}


int getFrontReading()
{
	int frontL = readFrontLIR();
	int frontR= readFrontRIR();
	int front = (frontL+frontR)/2;
	int thresholdFront = 700;
	if (front < thresholdFront)
	{
		return false;
	}
	return true;
}

int getRightReading()
{
	int right = readRightIR();
	int thresholdRight = 900;
	if (right < thresholdRight)
	{
		return false;
	}
	return true;
}

int getLeftReading()
{
	int left = readLeftIR();
	int thresholdLeft = 900;
	if (left < thresholdLeft)
	{
		return false;
	}
	return true;
}

/*
 This function reads the specific channel of the ADC corresponding to the correct IR
 You should not have to edit this function
 */
uint16_t analogRead(IR ir)
{
    ADC_ChannelConfTypeDef sConfig = {0}; //this initializes the IR ADC [Analog to Digital Converter]
    ADC_HandleTypeDef *hadc1_ptr = Get_HADC1_Ptr(); //this is a pointer to your hal_adc
    //this pointer will also be used to read the analog value, val = HAL_ADC_GetValue(hadc1_ptr);

    //this picks the IR direction to choose the right ADC.
    switch(ir)
    {
        case IR_LEFT:
            sConfig.Channel = ADC_CHANNEL_15;
            break;
        case IR_LFRONT:
            sConfig.Channel = ADC_CHANNEL_14;
            break;
        case IR_RFRONT:
            sConfig.Channel = ADC_CHANNEL_4;
            break;
        case IR_RIGHT:
            sConfig.Channel = ADC_CHANNEL_5;
            break;
        default:
            return 0;
    }

    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    // make sure everything was set up correctly
    if (HAL_ADC_ConfigChannel(hadc1_ptr, &sConfig) != HAL_OK)
    {
        return 0;
    }
    
    complete = 0;

    // start filling up the ADC buffer
    HAL_ADC_Start_DMA(hadc1_ptr, (uint32_t*)adc_buf, NUM_SAMPLES);

    // wait for the buffer to become full
    while (complete == 0)
    {
        continue;
    }

    uint32_t sum = 0;
    // calculate the sum of the measurements in order to calculate the average
    uint16_t measurement = 0;
    while(measurement < NUM_SAMPLES) //this takes multiple measurements
    {
        sum += adc_buf[measurement];
        ++measurement;
    }

    return sum/NUM_SAMPLES;
}

/*
 This function is called when the ADC buffer is filled
 It stops the ADC and changes our "complete" variable to be "true"
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // stop the ADC
    HAL_ADC_Stop_DMA(hadc);
    complete = 1;
}


