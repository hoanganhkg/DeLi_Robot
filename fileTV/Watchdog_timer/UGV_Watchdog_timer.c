#include "UGV_Watchdog_timer.h"

void IWDG_Config(void) {
    // Cho phép truy c?p vào các thanh ghi IWDG
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // Ch?n Prescaler cho Watchdog (ví d?: chia t?n s? LSI v?i giá tr? 64)
    IWDG_SetPrescaler(IWDG_Prescaler_8);

    // Thi?t l?p giá tr? reload (timeout = (Reload + 1) * Prescaler / LSI)
	  IWDG_SetReload(300);  // Timeout: 50ms (tùy thu?c vào LSI)

    // Làm m?i l?n d?u d? chu?n b? kích ho?t IWDG
    IWDG_ReloadCounter();

    // B?t IWDG
    IWDG_Enable();
}

