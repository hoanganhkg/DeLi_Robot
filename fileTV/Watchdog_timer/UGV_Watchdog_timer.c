#include "UGV_Watchdog_timer.h"

void IWDG_Config(void) {
    // Cho ph�p truy c?p v�o c�c thanh ghi IWDG
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // Ch?n Prescaler cho Watchdog (v� d?: chia t?n s? LSI v?i gi� tr? 64)
    IWDG_SetPrescaler(IWDG_Prescaler_8);

    // Thi?t l?p gi� tr? reload (timeout = (Reload + 1) * Prescaler / LSI)
	  IWDG_SetReload(300);  // Timeout: 50ms (t�y thu?c v�o LSI)

    // L�m m?i l?n d?u d? chu?n b? k�ch ho?t IWDG
    IWDG_ReloadCounter();

    // B?t IWDG
    IWDG_Enable();
}

