//#include "RTC.h"
//#include "UGV_CAN.h"
//void RTC_Config(void) {
//    RTC_InitTypeDef RTC_InitStruct;
//    RTC_TimeTypeDef RTC_TimeStruct;

//    // 1. B?t ngu?n cho RTC & PWR (Power Interface)
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
//    RCC_BackupResetCmd(ENABLE);
//    RCC_BackupResetCmd(DISABLE);
//    PWR->CR |= PWR_CR_DBP;  // Cho phép truy c?p RTC (Backup Domain)

//    // 2. Ch?n ngu?n clock cho RTC (LSE - 32.768kHz)
//    RCC_LSEConfig(RCC_LSE_ON);
//    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET); // Ð?i LSE ?n d?nh

//    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
//    RCC_RTCCLKCmd(ENABLE);

//    // 3. C?u hình RTC
//    RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
//    RTC_InitStruct.RTC_AsynchPrediv = 127;
//    RTC_InitStruct.RTC_SynchPrediv = 255;
//    RTC_Init(&RTC_InitStruct);

//    // 4. C?u hình th?i gian ban d?u (n?u c?n)
//    RTC_TimeStruct.RTC_Hours = 11;
//    RTC_TimeStruct.RTC_Minutes = 01;
//    RTC_TimeStruct.RTC_Seconds = 0;
//    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
//}

//void get_RTC_timestamp(uint8_t *buffer) {
//	
//    RTC_TimeTypeDef RTC_TimeStruct;
//    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);

//    buffer[0] = RTC_TimeStruct.RTC_Hours;
//    buffer[1] = RTC_TimeStruct.RTC_Minutes;
//    buffer[2] = RTC_TimeStruct.RTC_Seconds;
//}

//void send_RTC_timestamp_CAN(void) {
//	
//    uint8_t timestamp_data[3];
//    get_RTC_timestamp(timestamp_data);
//    CAN1_Send(0x101, timestamp_data, 3);  // G?i ID 0x101
//}
