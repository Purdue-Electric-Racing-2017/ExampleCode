/*
 * freertos.h
 *
 *  Created on: Dec 7, 2016
 *      Author: ben
 */

#ifndef FREERTOS_H_
#define FREERTOS_H_

SemaphoreHandle_t 		m_GPIO6;
SemaphoreHandle_t		m_CAN1;
SemaphoreHandle_t		m_ADC;
SemaphoreHandle_t 		s_ToggleTask;
SemaphoreHandle_t		s_TXCANTask;
QueueHandle_t			q_ADCConvertProcessTask;
QueueHandle_t			q_TXCANQueue;
QueueHandle_t			q_RXCANProcessQueue;

uint16_t ADC1ConvertedValues[32];



#endif /* FREERTOS_H_ */
