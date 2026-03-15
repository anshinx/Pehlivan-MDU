/* ===========================================================================
 *  PEHLIVAN TEAM — BLDC Motor Surucu
 *  STM32G474VET6 | Shell Eco-Marathon 2026
 *  motor_driver.c  —  v5.0
 *
 *  v4 -> v5 Degisiklikler:
 *    [1] FDCAN1 telemetri eklendi (CAN_ID 0x100-0x103)
 *
 *  v3 -> v4 Duzeltmeler (Audit bulgulari):
 *    [1] ihallval guncellenmiyordu: ExtInt() sonunda ihallval = hallval eklendi
 *    [2] dene sayaci yanlis sifirliyordu: deneStart + deneSys ayri sayaclar
 *    [3] WHEEL_CIRC_M/KM cift tanimliydi: .c'deki #define'lar kaldirildi (.h yeterli)
 *    [4] Safety_ADCUpdated() hic cagirilmiyordu: adcRead() sonuna eklendi
 *    [5] Safety sistemi Motor_Init/Loop'a baglanmamisti: baglantilar yapildi
 *    [6] else { dene=0; } her hatada sayaci sifirlatiyordu: kaldirildi
 * ===========================================================================*/

#include "motor_driver.h"
#include "safety_telemetry.h"

/* ===========================================================================
 *  HANDLE REFERANSLARI
 * ===========================================================================*/
extern TIM_HandleTypeDef   htim1;
extern TIM_HandleTypeDef   htim6;
extern ADC_HandleTypeDef   hadc1;
extern ADC_HandleTypeDef   hadc2;
extern ADC_HandleTypeDef   hadc3;
extern ADC_HandleTypeDef   hadc4;
extern ADC_HandleTypeDef   hadc5;
extern I2C_HandleTypeDef   hi2c1;
extern UART_HandleTypeDef  huart2;
extern UART_HandleTypeDef  huart4;
extern IWDG_HandleTypeDef  hiwdg;
extern FDCAN_HandleTypeDef hfdcan1;

/* ===========================================================================
 *  SISTEM PARAMETRELERI
 * ===========================================================================*/
int     dtime         = 10;
int     gazbos        = 10;
uint8_t gerimax       = 100;
int     gazramprate   = 20;
uint8_t maxpwm        = 250;
float   pilmax        = 80.0f;
float   pilmin        = 60.0f;
float   warntemp      = 50.0f;
float   maxtemp       = 70.0f;
float   maxstartamp   = 6.0f;
float   maxsystemamp  = 50.0f;
float   currzero      = 1.65f;
int     arazaman      = 1500;

/* ===========================================================================
 *  MOTOR KONTROL DEGISKENLERI
 * ===========================================================================*/
volatile int     gaz       = 0;
volatile int     Accel     = 0;
volatile bool    hareket   = false;
volatile bool    ilkkontak = true;
volatile bool    gazizin   = false;
volatile bool    DirF      = true;
volatile bool    BrakeF    = false;
volatile bool    chgdirF   = false;
volatile bool    yon       = false;
volatile uint8_t hallval   = 0;
volatile uint8_t ihallval  = 0;   /* BUG FIX: ExtInt() sonunda guncellenir */

/* ===========================================================================
 *  OLCUM DEGISKENLERI
 * ===========================================================================*/
float OrtTemp = 0.0f;
float TTemp   = 0.0f;
float Temp    = 0.0f;

float OrtVolt = 0.0f;
float TVolt   = 0.0f;
float Volt    = 0.0f;

float OrtCurr = 0.0f;
float TCurr   = 0.0f;
float CurrA   = 0.0f;
float CurrB   = 0.0f;
float CurrC   = 0.0f;

float regenVal = 0.0f;
float devirdak = 0.0f;
float kmsaat   = 0.0f;

float totalKm  = 0.0f;
float totalM   = 0.0f;

static int j = 0, k = 0;

/* ===========================================================================
 *  HATA VE DURUM BAYRAKLARI
 * ===========================================================================*/
volatile bool  overstartcurrentF = false;
volatile bool  oversyscurrentF   = false;
volatile bool  acil              = false;
volatile bool  panicF            = false;
volatile bool  pilmaxF           = false;
volatile bool  pilminF           = false;
volatile bool  warntempF         = false;
volatile bool  maxtempF          = false;
volatile int   BlinkStat         = 0;
volatile int   warntime          = 0;

/*
 * BUG FIX [2]: Tek 'dene' sayaci yerine iki ayri sayac.
 *
 * Eski kod:
 *   if (!hareket && curr > maxstartamp) { dene++; }
 *   else if (hareket && curr > maxsystemamp) { oversyscurrentF=true; }
 *   else { dene = 0; }   <-- YANLIS: sistem akimi yoksa kalkis sayacini sifirliyordu
 *
 * Yeni kod:
 *   deneStart: sadece kalkis asiri akimini sayar, yalnizca normal kalkista sifirlanir
 *   deneSys  : sistem asiri akimini sayar (simdilik 1 hit yeterli -- genisletilebilir)
 *
 * Her sayac sadece kendi hata durumunu sayar, diger durum onu sifirlamaz.
 */
uint8_t deneStart = 0;
uint8_t deneSys   = 0;

/* Recovery ramp bayragi -- Safety_RecoveryCheck tarafindan set edilir.
 * Motor_Loop bu bayrak aktifken Accel hedefini Safety_GetMaxPWM() ile kisitlar.
 * Surucü gazi birakirsa veya ramp hedefe ulasirsa Motor_Loop bayragi temizler. */
volatile bool recoveryRampActive = false;

/* ===========================================================================
 *  ZAMANLAMA SAYACLARI
 * ===========================================================================*/
volatile int  msTime     = 0;
volatile int  BlinkCount = 0;
volatile int  adcCount   = 0;
volatile int  accsay     = 0;
volatile int  harsay     = 0;
volatile int  rampcnt    = 0;
volatile int  devzaman   = 0;
volatile int  devsay     = 0;

volatile bool adcReadF   = false;
volatile bool accelokuF  = false;

/* BUG FIX [G2]: int32_t kullan -- 12-bit ADC x 10 ornek = max 40950,
 * int icin overflow yok ama int32_t niyeti acikca belli eder. */
static int32_t acceltop = 0;
static int32_t accelsay = 0;

/* ===========================================================================
 *  DWT — MIKROSANIYE GECIKME
 * ===========================================================================*/
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SYS_CLOCK_HZ / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

/* ===========================================================================
 *  IWDG BESLEME
 * ===========================================================================*/
void IWDG_Feed(void) {
    HAL_IWDG_Refresh(&hiwdg);
}

/* ===========================================================================
 *  ADC YARDIMCILARI
 * ===========================================================================*/
static uint32_t ADC_Read(ADC_HandleTypeDef *hadc) {
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint32_t val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return val;
}

static inline float ADC_ToVolt(uint32_t raw) {
    return (raw / ADC_RESOLUTION) * (ADC_VREF_MV / 1000.0f);
}

/* ===========================================================================
 *  INA241A2 AKIM HESABI
 * ===========================================================================*/
static float INA241_Curr(uint32_t raw) {
    float v    = ADC_ToVolt(raw);
    float curr = (v - currzero) / (INA241_GAIN * INA241_RSHUNT);
    return (curr < 0.0f) ? 0.0f : curr;
}

/* ===========================================================================
 *  ADC5 CIFT KANAL — Sicaklik (PA8/IN1) + Regen (PA9/IN2)
 * ===========================================================================*/
static void ADC5_ReadBoth(uint32_t *tempRaw, uint32_t *regenRaw) {
    *tempRaw = ADC_Read(&hadc5);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_2;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLE_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(&hadc5, &sConfig);
    *regenRaw = ADC_Read(&hadc5);

    sConfig.Channel = ADC_CHANNEL_1;
    HAL_ADC_ConfigChannel(&hadc5, &sConfig);
}

/* ===========================================================================
 *  PWM KONTROL — TIM1 COMPLEMENTARY
 * ===========================================================================*/
void PWM_SetA(uint16_t val) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, val); }
void PWM_SetB(uint16_t val) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, val); }
void PWM_SetC(uint16_t val) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, val); }

void PWM_AllOff(void) {
    PWM_SetA(0); PWM_SetB(0); PWM_SetC(0);
}

void deadtime(void) {
    PWM_AllOff();
    DWT_Delay_us(dtime);
}

/* ===========================================================================
 *  BUZZER — PD5
 * ===========================================================================*/
void Buzzer_Beep(uint16_t duration_ms) {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(duration_ms);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

/* ===========================================================================
 *  HALL SENSOR OKUMA
 * ===========================================================================*/
static inline uint8_t readHall(void) {
    return ((HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) == GPIO_PIN_SET) ? 4u : 0u) |
           ((HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) == GPIO_PIN_SET) ? 2u : 0u) |
           ((HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin) == GPIO_PIN_SET) ? 1u : 0u);
}

/* ===========================================================================
 *  6-STEP COMMUTATION
 *
 *  UYARI: Tablo motorun sarim sirasina gore ayarlanmalidir.
 * ===========================================================================*/
static void commutate_forward(uint8_t hall, int pwm) {
    deadtime();
    switch (hall) {
        case 1: PWM_SetA(pwm); PWM_SetB(0);   PWM_SetC(pwm); break;
        case 2: PWM_SetA(0);   PWM_SetB(pwm); PWM_SetC(pwm); break;
        case 3: PWM_SetA(pwm); PWM_SetB(pwm); PWM_SetC(0);   break;
        case 4: PWM_SetA(0);   PWM_SetB(pwm); PWM_SetC(0);   break;
        case 5: PWM_SetA(0);   PWM_SetB(0);   PWM_SetC(pwm); break;
        case 6: PWM_SetA(pwm); PWM_SetB(0);   PWM_SetC(0);   break;
        default: PWM_AllOff(); break;
    }
}

static void commutate_reverse(uint8_t hall, int pwm) {
    deadtime();
    switch (hall) {
        case 1: PWM_SetA(pwm); PWM_SetB(0);   PWM_SetC(0);   break;
        case 2: PWM_SetA(0);   PWM_SetB(pwm); PWM_SetC(0);   break;
        case 3: PWM_SetA(0);   PWM_SetB(0);   PWM_SetC(pwm); break;
        case 4: PWM_SetA(0);   PWM_SetB(pwm); PWM_SetC(0);   break;
        case 5: PWM_SetA(pwm); PWM_SetB(0);   PWM_SetC(0);   break;
        case 6: PWM_SetA(0);   PWM_SetB(0);   PWM_SetC(pwm); break;
        default: PWM_AllOff(); break;
    }
}

/* ===========================================================================
 *  ExtInt — HALL SENSOR EXTI CALLBACK
 *
 *  stm32g4xx_it.c icine ekle:
 *    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *        if (GPIO_Pin == HALL_A_Pin || GPIO_Pin == HALL_B_Pin ||
 *            GPIO_Pin == HALL_C_Pin) { ExtInt(); }
 *    }
 * ===========================================================================*/
void ExtInt(void) {
    uint8_t newHall = readHall();

    /* Gecersiz hall degeri — sensor arizasi */
    if (newHall == 0 || newHall == 7) {
        PWM_AllOff();
        acil = true;
        return;
    }

    /* BUG FIX [1]: ihallval guncelle.
     * Eski kodda ihallval hic guncellenmiyordu.
     * Safety_Check() icindeki hall timeout: hareket && hallval == ihallval
     * kosulu hic degismediginden yanlis timeout tetikleniyordu.
     * Cozum: her gecerli hall degisiminde once ihallval'i kaydet, sonra
     * hallval'i guncelle. */
    ihallval = hallval;
    hallval  = newHall;

    devsay++;

    /* Hareket halinde yon degisimi tespiti */
    if (hareket) {
        if (DirF != yon) { chgdirF = true; return; }
    }

    harsay  = 0;
    hareket = true;
    yon     = DirF;

    if (gaz == 0 || !gazizin) { PWM_AllOff(); return; }
    if (BrakeF)               { deadtime();  return; }

    if (DirF) {
        commutate_forward(hallval, gaz);
    } else {
        commutate_reverse(hallval, (gaz > gerimax) ? gerimax : gaz);
    }
}

/* ===========================================================================
 *  ADC OKUMA — 100ms'de bir cagirilir
 *
 *  BUG FIX [4]: Sonunda Safety_ADCUpdated() cagrilir.
 *  Eski kodda bu cagri yoktu; Safety_Check() her zaman ADC_TIMEOUT
 *  hatasi uretiyordu (boot'tan ~500ms sonra surekli).
 * ===========================================================================*/
void adcRead(void) {

    /* Faz Akimlari */
    CurrA = INA241_Curr(ADC_Read(&hadc1));
    CurrB = INA241_Curr(ADC_Read(&hadc2));
    CurrC = INA241_Curr(ADC_Read(&hadc3));
    float curr = (CurrA + CurrB + CurrC) / 3.0f;
    TCurr += curr;
    k++;
    if (k >= 10) { OrtCurr = TCurr / 10.0f; TCurr = 0.0f; k = 0; }

    /* -----------------------------------------------------------------------
     * Asiri Akim Tespiti
     *
     * AUDIT BULGUSU #1 (cift basli karar mekanizmasi) DUZELTMESI:
     *
     * Eski hata: adcRead() hem acil=true yapip hem PWM_AllOff() cagiriyordu.
     * Bu Safety_SetState()'in BlackBox log + durum yonetimini devre disi
     * birakiyordu. Hata olunca currentState FS_NORMAL kalabiliyordu.
     *
     * Yeni dogru yapi:
     *   adcRead() sadece ham ADC degerlerini olcer ve flag'leri set eder.
     *   TUM karar verme yetkisi Safety_Check()'e devredildi.
     *   Safety_Check() 100ms sonra OrtCurr degerini gorup Safety_SetState()
     *   cagirarak hem BlackBox'a yazar hem dogrukarar uretir.
     *
     * Tek istisna: deneStart/deneSys sayaclari burada korunur cunku
     * bunlar 100ms periyoduyla biriken ardisik hata sayaclaridir --
     * Safety_Check ise tek olcum anlık degerlere bakar.
     * ----------------------------------------------------------------------- */
    if (!hareket && curr > maxstartamp) {
        overstartcurrentF = true;
        if (deneStart < 255u) deneStart++;
        /* 3 ardisik kalkis asiri akimi: Safety_Check bir sonraki
         * dongude FSR_OVERCURR_START ile FS_HALT'a gecer. */
    }

    if (hareket && curr > maxsystemamp) {
        oversyscurrentF = true;
        if (deneSys < 255u) deneSys++;
        /* Safety_Check bir sonraki dongude FSR_OVERCURR_SYS ile
         * FS_HALT'a gecer ve BlackBox'a yazar. */
    }

    /* Sicaklik + Regen */
    uint32_t tempRaw = 0, regenRaw = 0;
    ADC5_ReadBoth(&tempRaw, &regenRaw);

    /* LM35: 10mV/C */
    Temp = ADC_ToVolt(tempRaw) / 0.01f;
    TTemp += Temp;
    j++;
    if (j >= 10) { OrtTemp = TTemp / 10.0f; TTemp = 0.0f; j = 0; }

    /* Regen pedal: 0-100% */
    regenVal = (ADC_ToVolt(regenRaw) / (ADC_VREF_MV / 1000.0f)) * 100.0f;

    /* Pil voltaji
     * UYARI: Simdilik tempRaw uzerinden hesaplaniyor.
     * Bu sadece voltaj bolucusu sicaklik sensoru ile
     * ayni ADC5 kanalindaysa dogru sonuc verir.
     * Farkli bir ADC kanalindan okunuyorsa asagidaki
     * satirda dogru ADC okunmali. */
    static int vj = 0;
    Volt = ADC_ToVolt(tempRaw) * 25.0f;  /* TODO: voltaj icin dogru ADC kanalini oku */
    TVolt += Volt; vj++;
    if (vj >= 10) { OrtVolt = TVolt / 10.0f; TVolt = 0.0f; vj = 0; }

    /* Sicaklik / voltaj flag'leri -- Safety_Check() bu flag'leri
     * gorup Safety_SetState() ile uygun failsafe durumuna gecer.
     * Burada artik acil=true veya PWM_AllOff() cagrilmiyor. */
    warntempF = (OrtTemp > warntemp);
    maxtempF  = (OrtTemp > maxtemp);

    pilmaxF = (OrtVolt > pilmax);
    pilminF = (OrtVolt < pilmin && OrtVolt > 1.0f);

    /* Safety sistemine ADC'nin guncellendigi bildirilir. */
    Safety_ADCUpdated();
}


/* ===========================================================================
 *  DEBUG CIKISI — USART2 + UART4
 * ===========================================================================*/
void menu(void) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "T=%d.%dC V=%d.%dV A=%d.%dA Gaz=%d Km/h=%d Rpm=%d Odo=%d.%03dm\r\n",
        (int)OrtTemp, (int)(OrtTemp * 10) % 10,
        (int)OrtVolt, (int)(OrtVolt * 10) % 10,
        (int)OrtCurr, (int)(OrtCurr * 10) % 10,
        gaz,
        (int)kmsaat,
        (int)devirdak,
        (int)totalKm,
        (int)totalM);

    HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)len, 20);
    HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)len, 20);
}

/* ===========================================================================
 *  TIM6 1kHz CALLBACK
 *
 *  stm32g4xx_it.c icine ekle:
 *    extern void TIM6_1kHz_Callback(void);
 *    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *        if (htim->Instance == TIM6) { TIM6_1kHz_Callback(); }
 *    }
 * ===========================================================================*/
void TIM6_1kHz_Callback(void) {
    msTime++;
    BlinkCount++;
    adcCount++;
    accsay++;
    harsay++;
    rampcnt++;
    devzaman++;

    if (adcCount >= 100) { adcCount = 0; adcReadF = true; }
    if (accsay  >=  10)  { accsay   = 0; accelokuF = true; }
    if (harsay  >= 100)  { harsay   = 0; hareket = false; }

    /* Brake pin okuma (PE15) */
    BrakeF = (HAL_GPIO_ReadPin(BRAKE_GPIO_Port, BRAKE_Pin) == GPIO_PIN_SET);

    /* RPM hesabi ve Hall tabanli odometre — 1 saniye
     *
     * Odometre NEDEN Hall'dan okunmali?
     * ADC tabanlı odometre (kmsaat/3.6 * 0.1s) kmsaat'in dogruluguna baglidir.
     * Hall tabanlı odometre dogrudan fiziksel tekerlek donusunu sayar:
     *   Her devsay artisi = 1 Hall edge = tekerlegin (1/14) tur donmesi
     *   (7 kutup cifti x 2 = 14 edge/tur)
     *   Kat edilen mesafe = (devsay / 14) * WHEEL_CIRC_M
     *
     * Bu yontem GPS veya hassas encoder olmadan en guvvenilir odometre yontemidir.
     * Hall sensoru takili degilse devsay=0 kalir, odometre sayilmaz -- guvenli.
     */
    if (devzaman >= 1000) {
        devzaman = 0;

        /* RPM: 14 edge = 1 tur, 1 saniyede devsay edge geldi */
        devirdak = (float)devsay * (60.0f / 14.0f);

        /* Hiz: rpm -> km/h */
        kmsaat = (devirdak / 60.0f) * WHEEL_CIRC_KM * 3600.0f;

        /* Odometre: Hall edge sayisindan dogrudan mesafe hesapla
         * Her edge = WHEEL_CIRC_M / 14 metre
         * Bu ADC tabanli hesaptan cok daha dogrundur. */
        float dist_m = (float)devsay * (WHEEL_CIRC_M / 14.0f);
        totalM += dist_m;
        while (totalM >= 1000.0f) {
            totalKm += 1.0f;
            totalM  -= 1000.0f;
        }

        devsay = 0;
    }

    /* LED blink */
    if (BlinkCount >= ((BlinkStat == 2) ? 200 : 500)) {
        BlinkCount = 0;
        HAL_GPIO_TogglePin(STATUS_LED_Port, STATUS_LED_Pin);
    }

    /* Hata sonrasi otomatik kurtarma.
     * acil ve panicF kasitli temizlenmez — bunlar IWDG/manuel reset ile sifirlanir.
     * Sadece gecici hatalar (overcurrent) burada kurtarilir. */
    if (oversyscurrentF || overstartcurrentF) {
        warntime++;
        if (warntime >= arazaman) {
            warntime          = 0;
            oversyscurrentF   = false;
            overstartcurrentF = false;
            deneStart         = 0;
            deneSys           = 0;
        }
    }

    /* Yazilim watchdog kontrolu (1kHz'de) */
    SW_Watchdog_Check();
}

/* ===========================================================================
 *  MOTOR BASLAMA — setup() karsiligi
 *
 *  main() icinde MX_xxx_Init()'lerden SONRA:
 *    Motor_Init();
 *    while (1) { Motor_Loop(); }
 *
 *  BUG FIX [5]: Safety_Init() burada cagirilir.
 *  Eski kodda Safety_Init() hic cagirilmiyordu; safety altsistemi
 *  baslatilmamis halde calisiyor, lastADCRead_ms=0 yuzunden
 *  hemen ADC_TIMEOUT atiyordu.
 * ===========================================================================*/
void Motor_Init(void) {

    /* 1) DWT mikrosaniye sayaci */
    DWT_Init();

    /* 2) TIM1 Complementary PWM */
    HAL_TIM_PWM_Start    (&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start (&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start    (&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start (&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start    (&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start (&htim1, TIM_CHANNEL_3);

    /* 3) 1kHz Tick timer
     * [!] .ioc'a TIM6 ekle: Prescaler=127, ARR=999, NVIC aktif */
    HAL_TIM_Base_Start_IT(&htim6);

    /* 4) Tum PWM cikislarini sifirla */
    deadtime();

    /* 5) INA241A2 sifir kalibrasyonu */
    float czSum = 0.0f;
    for (int n = 0; n < 16; n++) {
        czSum += ADC_ToVolt(ADC_Read(&hadc1));
        HAL_Delay(1);
    }
    currzero = czSum / 16.0f;

    /* 6) Ilk gaz olcumu */
    uint32_t accelRaw = ADC_Read(&hadc4);
    Accel = (int)((accelRaw * maxpwm) / (uint32_t)(ADC_RESOLUTION - 1));

    /* 7) BUG FIX [5]: Safety altsistemini baslat */
    Safety_Init();

    /* 8) Baslangic sinyali */
    HAL_GPIO_WritePin(STATUS_LED_Port, STATUS_LED_Pin, GPIO_PIN_SET);
    Buzzer_Beep(100); HAL_Delay(100); Buzzer_Beep(100);
    BlinkStat = 1;
}

/* ===========================================================================
 *  ANA DONGU — loop() karsiligi
 *
 *  BUG FIX [5]: Safety_Check(), SW_Watchdog_Feed(), BlackBox_Flush(),
 *  Telemetry_SendUART() bu dongude cagirilir.
 *  Eski kodda safety altsistemi hic baglanmamisti.
 * ===========================================================================*/
void Motor_Loop(void) {

    /* 1) Donanim watchdog besleme — ZORUNLU, ilk satir */
    IWDG_Feed();

    /* 2) Yazilim watchdog besleme */
    SW_Watchdog_Feed();

    /* 3) Safety kontrolu — failsafe state machine */
    Safety_Check();

    /* 4) Hata / Acil Durum — motor kapatma */
    if (overstartcurrentF || oversyscurrentF || acil || panicF) {
        gaz = 0;
        BlinkStat = 2;
        deadtime();
        if (panicF) {
            static uint32_t lastBeep = 0;
            if (HAL_GetTick() - lastBeep > 1000) {
                lastBeep = HAL_GetTick();
                Buzzer_Beep(200);
            }
        }
        /* Kara kutu flush — hata kayitlari UART4'e gonderilir */
        BlackBox_Flush();
        return;
    }

    /* 5) ADC Okuma — 100ms */
    if (adcReadF) {
        adcReadF = false;
        adcRead();   /* Safety_ADCUpdated() icinde cagrilir */
        menu();
        /* Telemetri — UART2 ASCII debug + FDCAN1 CAN bus */
        Telemetry_SendUART();
        Telemetry_SendCAN();
    }

    /* 6) Gaz Pedali — 10ms, 10 ornek ortalaması */
    if (accelokuF) {
        accelokuF = false;
        uint32_t raw = ADC_Read(&hadc4);
        acceltop += (int32_t)raw;
        accelsay++;
        if (accelsay > 9) {
            Accel    = (int)(((uint32_t)(acceltop / accelsay) * (uint32_t)maxpwm)
                              / (uint32_t)(ADC_RESOLUTION - 1));
            acceltop = 0;
            accelsay = 0;
        }
        if (ilkkontak && Accel > gazbos)  { gazizin = false; }
        if (ilkkontak && Accel <= gazbos) { gazizin = true; ilkkontak = false; }

        /* Recovery ramp (audit bulgusu #4):
         * Failsafe durumu iyilestikten sonra limit aniden kalkarsa
         * surucü tam gaz basiyorsa araç firlar.
         * recoveryRampActive=true iken Accel Safety_GetMaxPWM() ile
         * kisitlanir -- mevcut gaz rampasi ustune bir ust limit gibi calisir.
         * Limit maxpwm'e esitlendiginde (FS_NORMAL) bayrak temizlenir. */
        if (recoveryRampActive) {
            uint8_t limit = Safety_GetMaxPWM();
            if (Accel > (int)limit) { Accel = (int)limit; }
            if (limit >= maxpwm)    { recoveryRampActive = false; }
        }
    }

    /* 7) Hareketsizken Ilk Gaz */
    if (!hareket && gazizin && !ilkkontak) {
        if (Accel >= gazbos) ExtInt();
    }

    /* 8) Ramp Hiz Artirma */
    if (rampcnt > gazramprate && gaz < Accel && gazizin) {
        rampcnt = 0;
        int d = Accel - gaz;
        gaz += (d >= 5) ? 5 : d;
        if (gaz > 250) gaz = 250;
    }

    /* 9) Ramp Hiz Azaltma */
    if (rampcnt > gazramprate && gaz > Accel && gazizin) {
        rampcnt = 0;
        int d = gaz - Accel;
        gaz -= (d >= 5) ? 5 : d;
    }

    /* 10) Gaz Boslugu */
    if (Accel < gazbos) { gaz = 0; deadtime(); }

    /* 11) Fren Kontrolu */
    if (BrakeF) { deadtime(); }

    /* 12) Yon Degisimi Guvenligi */
    if (chgdirF) {
        if (hareket) { deadtime(); gaz = 0; }
        else         { chgdirF = false; }
    }

    /* 13) Kara kutu flush — her dongude 1 kayit gonderilir */
    BlackBox_Flush();
}
