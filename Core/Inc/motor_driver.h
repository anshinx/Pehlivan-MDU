/* ===========================================================================
 * PEHLIVAN TEAM - BLDC Motor Sürücü
 * STM32G474VET6 — Shell Eco Marathon 2026
 * Donanım: IRS21867S + INA241A2
 *
 * v5.0 — FDCAN1 AKTİF
 *   FDCAN1: PD0 (RX) + PD1 (TX), TJA1051T, 500 kbps klasik CAN
 *   Telemetri: USART2 (debug) + UART4 (kara kutu) + FDCAN1 (CAN bus)
 *
 * Sistem Clock: 128 MHz (HSE 8MHz → PLL N=16)
 * ADC Vref    : 3.3V
 * ===========================================================================*/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ===========================================================================
 * PIN HARİTASI
 * ===========================================================================*/

/* Hall Sensörler (GPIO_Input → EXTI eklenecek) */
#define HALL_A_Pin         GPIO_PIN_12
#define HALL_A_GPIO_Port   GPIOD
#define HALL_B_Pin         GPIO_PIN_10
#define HALL_B_GPIO_Port   GPIOD
#define HALL_C_Pin         GPIO_PIN_8
#define HALL_C_GPIO_Port   GPIOD

/* Dijital Girişler */
#define BRAKE_Pin          GPIO_PIN_15
#define BRAKE_GPIO_Port    GPIOE
#define PANIC_Pin          GPIO_PIN_13
#define PANIC_GPIO_Port    GPIOC

/* Dijital Çıkışlar */
#define STATUS_LED_Pin     GPIO_PIN_3
#define STATUS_LED_Port    GPIOB
#define BUZZER_Pin         GPIO_PIN_5
#define BUZZER_GPIO_Port   GPIOD

/* ===========================================================================
 * SİSTEM SABİTLERİ
 * ===========================================================================*/
#define SYS_CLOCK_HZ       128000000UL
#define ADC_VREF_MV        3300.0f
#define ADC_RESOLUTION     4096.0f

/* PWM: 128MHz / 25 / 256 = 20kHz */
#define PWM_PRESCALER      24u
#define PWM_ARR            255u
#define PWM_FREQ_HZ        20000u

/* TIM6 - 1kHz: 128MHz / 128 / 1000 */
#define TIM6_PRESCALER     127u

/* ===========================================================================
 * CAN MESAJ ID'LERİ — FDCAN1 Telemetri (Standart 11-bit ID)
 * ===========================================================================*/
#define CAN_ID_MOTOR_STATUS   0x100u  /* gaz, accel, RPM, hall           */
#define CAN_ID_SENSOR_DATA    0x101u  /* sıcaklık, voltaj, akım          */
#define CAN_ID_FAILSAFE       0x102u  /* failsafe state, reason mask     */
#define CAN_ID_ODOMETER       0x103u  /* totalKm, totalM, km/h           */
#define TIM6_ARR           999u

/* INA241A2 */
#define INA241_GAIN        20.0f
#define INA241_VREF        1.65f
#define INA241_RSHUNT      0.001f

/* Tekerlek: 16 inch jant, lastik yok */
#define WHEEL_CIRC_M       1.27681f
#define WHEEL_CIRC_KM      (WHEEL_CIRC_M / 1000.0f)

/* ===========================================================================
 * EXTERN DEĞİŞKENLER
 * ===========================================================================*/
/* Sistem Parametreleri */
extern int      dtime;
extern int      gazbos;
extern uint8_t  gerimax;
extern int      gazramprate;
extern uint8_t  maxpwm;
extern float    pilmax, pilmin;
extern float    warntemp, maxtemp;
extern float    maxstartamp, maxsystemamp;
extern float    currzero;
extern int      arazaman;

/* Motor Kontrol */
extern volatile int     gaz;
extern volatile int     Accel;
extern volatile bool    hareket;
extern volatile bool    ilkkontak;
extern volatile bool    gazizin;
extern volatile bool    DirF;
extern volatile bool    BrakeF;
extern volatile bool    chgdirF;
extern volatile bool    yon;
extern volatile uint8_t hallval;
extern volatile uint8_t ihallval;

/* Ölçüm */
extern float    OrtTemp, OrtVolt, OrtCurr;
extern float    CurrA, CurrB, CurrC;
extern float    regenVal;
extern float    devirdak, kmsaat;
extern float    totalKm;
extern float    totalM;

/* Bayraklar */
extern volatile bool    overstartcurrentF;
extern volatile bool    oversyscurrentF;
extern volatile bool    acil;
extern volatile bool    panicF;
extern volatile bool    pilmaxF, pilminF;
extern volatile bool    warntempF, maxtempF;
extern volatile int     BlinkStat;

/* Sayaçlar */
extern volatile int  msTime, BlinkCount, adcCount;
extern volatile int  accsay, harsay, rampcnt;
extern volatile int  devzaman, devsay, warntime;
extern volatile bool adcReadF, accelokuF;
extern volatile bool recoveryRampActive;

/* Ardisik akim hata sayaclari -- Safety_Check tarafindan okunur */
extern uint8_t deneStart;
extern uint8_t deneSys;

/* ===========================================================================
 * FONKSİYON PROTOTİPLERİ
 * ===========================================================================*/
void Motor_Init(void);
void Motor_Loop(void);
void ExtInt(void);
void adcRead(void);
void menu(void);
void deadtime(void);
void Buzzer_Beep(uint16_t duration_ms);
void DWT_Init(void);
void DWT_Delay_us(uint32_t us);
void IWDG_Feed(void);
void PWM_SetA(uint16_t val);
void PWM_SetB(uint16_t val);
void PWM_SetC(uint16_t val);
void PWM_AllOff(void);
void Telemetry_SendCAN(void);

#endif /* MOTOR_DRIVER_H */
