#ifndef UGV_RTC_H
#define UGV_RTC_H

#include "stm32f4xx.h"

void RTC_Config(void);
void get_RTC_timestamp(uint8_t *buffer);
void send_RTC_timestamp_CAN(void);


#endif
