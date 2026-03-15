/* ===========================================================================
 * PEHLIVAN TEAM -- Guvenlik & Telemetri Sistemi
 * Shell Eco-Marathon 2026 Kural Uyumu
 * v3.0 -- FDCAN1 AKTIF
 *
 * Kapsanan Maddeler:
 *   CRC16 checksum (CRC donanimi)
 *   Failsafe state machine + sensor timeout + yazilim watchdog
 *   Kara kutu -- UART4 yapisal log
 *   Telemetri -- USART2 ASCII + UART4 binary + FDCAN1 CAN bus
 *   Non-blocking -- telemetri motor kontrolunu asla bloklamaz
 * ===========================================================================*/

#ifndef SAFETY_TELEMETRY_H
#define SAFETY_TELEMETRY_H

#include "main.h"
#include "motor_driver.h"
#include <stdbool.h>
#include <stdint.h>

/* ===========================================================================
 * FAILSAFE STATE MACHINE
 * Her durum bir oncekinden daha kisitlayicidir.
 * Ust seviyeye otomatik gecis -- geri donus sadece manuel reset ile.
 * ===========================================================================*/
typedef enum {
    FS_NORMAL        = 0,   /* Normal calisma                            */
    FS_WARNING       = 1,   /* Uyari: sicaklik/voltaj sinir yakın        */
    FS_REDUCED_POWER = 2,   /* maxpwm %60'a duser                        */
    FS_LIMP_HOME     = 3,   /* Sadece %30 guc, geri vites kapali         */
    FS_HALT          = 4,   /* Motor tamamen kapali, reset bekle         */
    FS_PANIC         = 5    /* PANIC_PIN veya kritik HW hatasi           */
} FailsafeState_t;

/* Failsafe gecis sebepleri -- log'a yazilir */
typedef enum {
    FSR_NONE             = 0x000,
    FSR_OVERTEMP_WARN    = 0x001,
    FSR_OVERTEMP_CRIT    = 0x002,
    FSR_OVERCURR_START   = 0x004,
    FSR_OVERCURR_SYS     = 0x008,
    FSR_UNDERVOLT        = 0x010,
    FSR_OVERVOLT         = 0x020,
    FSR_HALL_TIMEOUT     = 0x040,
    FSR_ADC_TIMEOUT      = 0x080,
    FSR_SW_WATCHDOG      = 0x100,
    FSR_PANIC_PIN        = 0x200,
    FSR_CHECKSUM_FAIL    = 0x400
} FailsafeReason_t;

/* ===========================================================================
 * KARA KUTU -- LOG KAYDI YAPISI
 * Her kayit 24 byte. UART4 (PC10 TX) uzerinden cikis.
 *
 * NOT: 0xPE47u gecersiz hex sabiti (P harfi yok).
 * Magic deger .c dosyasinda 0x0E47u olarak kullanilir.
 * ===========================================================================*/
#define LOG_QUEUE_SIZE  32u

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;         /* 0xE47 -- kayit baslangic isareti          */
    uint32_t timestamp_ms;  /* HAL_GetTick()                             */
    uint8_t  fs_state;      /* Anlik FailsafeState                       */
    uint16_t fs_reason;     /* FailsafeReason bitmask                    */
    int16_t  gaz_x10;       /* gaz * 10                                  */
    int16_t  accel_x10;     /* Accel * 10                                */
    int16_t  temp_x10;      /* OrtTemp * 10 -- 23.5 = 235               */
    int16_t  volt_x10;      /* OrtVolt * 10 -- 72.1 = 721               */
    int16_t  curr_x10;      /* OrtCurr * 10 -- 4.2 = 42                 */
    int16_t  rpm_x10;       /* devirdak * 10                             */
    uint8_t  hall;          /* Anlik hall degeri (0-7)                   */
    uint8_t  flags;         /* DirF|BrakeF|hareket|gazizin|panicF        */
    uint16_t crc16;         /* CRC16 -- STM32 CRC donanimiyla hesaplanir */
} LogRecord_t;              /* Toplam: 24 byte                            */
#pragma pack(pop)

/* ===========================================================================
 * YAZILIM WATCHDOG
 * 500ms icinde beslenmezse FS_HALT'a gecer.
 * ===========================================================================*/
#define SW_WATCHDOG_TIMEOUT_MS  500u

/* ===========================================================================
 * FONKSIYON PROTOTIPLERI
 * ===========================================================================*/

/* Baslama */
void Safety_Init(void);

/* Ana guvenlik kontrolu -- Motor_Loop() basinda cagir */
FailsafeState_t Safety_Check(void);

/* Failsafe gecis */
void Safety_SetState(FailsafeState_t newState, FailsafeReason_t reason);

/* Failsafe'e gore maxpwm dondur */
uint8_t Safety_GetMaxPWM(void);

/* Kara kutu */
void BlackBox_Log(FailsafeReason_t reason);
void BlackBox_Flush(void);

/* CRC */
uint16_t Calc_CRC16(const uint8_t *data, uint16_t len);

/* Telemetri -- UART2 ASCII debug + UART4 binary log + FDCAN1 CAN */
void Telemetry_SendUART(void);
void Telemetry_SendCAN(void);

/* Yazilim watchdog besleme */
void SW_Watchdog_Feed(void);
void SW_Watchdog_Check(void);

/* ADC guncelleme bildirimi */
void Safety_ADCUpdated(void);

/* Durum sorgulama */
FailsafeState_t Safety_GetState(void);
const char*     Safety_GetStateName(void);

#endif /* SAFETY_TELEMETRY_H */
