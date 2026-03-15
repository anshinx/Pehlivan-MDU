/* ===========================================================================
 * PEHLIVAN TEAM -- Guvenlik & Telemetri Sistemi
 * Shell Eco-Marathon 2026
 * v3.0 -- Gemini Audit Duzeltmeleri
 *
 * v2 -> v3 Duzeltmeler:
 *   [G1] Safety_Check: hallval volatile race condition -- yerel kopya ile okuma
 *   [G2] acceltop int -> int32_t (motor_driver_v4.c'de de guncellendi)
 *   [G3] Failsafe hysteresis recovery:
 *          FS_WARNING ve FS_REDUCED_POWER icin sensör duzeldikten
 *          sonra otomatik geri donus eklendi (hysteresis + timer ile).
 *          FS_HALT ve FS_PANIC kasitli olarak hic geri donmez -- reset gerekir.
 *
 * Gemini'nin yanlis buldugu ama dogru olan seyler:
 *   - SW_Watchdog_Feed: Motor_Loop adim 2'de zaten cagiriliyordu
 *   - ADC kilitlenme: HAL_ADC_PollForConversion 10ms timeout var, deadlock yok
 * ===========================================================================*/

#include "safety_telemetry.h"
#include <stdio.h>
#include <string.h>

/* ===========================================================================
 * HANDLE'LAR
 * ===========================================================================*/
extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern FDCAN_HandleTypeDef hfdcan1;

/* ===========================================================================
 * IC DURUM DEGISKENLERI
 * ===========================================================================*/
static FailsafeState_t currentState = FS_NORMAL;
static uint16_t fsReasonMask = FSR_NONE;

/* Kara kutu dongüsel buffer */
static LogRecord_t logQueue[LOG_QUEUE_SIZE];
static uint8_t logHead = 0;
static uint8_t logTail = 0;
static uint8_t logCount = 0;

/* Yazilim watchdog */
static volatile uint32_t swWdgLastFeed = 0;
static volatile bool swWdgEnabled = false;

/* Sensor timeout takibi */
static uint32_t lastHallChange_ms = 0;
static uint32_t lastADCRead_ms = 0;
#define HALL_TIMEOUT_MS 2000u
#define ADC_TIMEOUT_MS 500u

/* ===========================================================================
 * [G3] FAILSAFE RECOVERY -- Hysteresis ile otomatik geri donus
 *
 * Sorun: Anlık bir sicaklik/voltaj dalgalanmasi arabaci FS_REDUCED_POWER'a
 * dusuruyorsa ve sensör normale donerse araç yavaş gitmeye devam eder.
 *
 * Cozum:
 *   - FS_WARNING ve FS_REDUCED_POWER icin recovery timer tutulur.
 *   - Hata kosulu belirli sure boyunca gecerse (RECOVERY_HOLD_MS) durum
 *     bir alt seviyeye duser.
 *   - FS_HALT ve FS_PANIC hic otomatik geri donmez -- reset gerekir.
 *
 * Hysteresis parametreleri:
 *   RECOVERY_HOLD_MS: hata kosulunun kac ms boyunca gecmis olmasi gerekir
 *   Ornek: 3000ms boyunca sicaklik warn esiginin altinda kalirsa
 *          FS_WARNING -> FS_NORMAL'e doner.
 *
 * Dikkat: Geri donus sadece o hata sebebinin gecmesine bakar.
 * Birden fazla sebep varsa hepsinin gecmesi gerekir.
 * ===========================================================================*/
#define RECOVERY_HOLD_MS 3000u /* 3 saniye boyunca hata yoksa geri don */

static uint32_t recoveryStartTime = 0; /* Hata gecince sayac baslangici    */
static bool recoveryActive = false;    /* Geri donus sayaci aktif mi?    */

/* ===========================================================================
 * CRC16 -- STM32G474 Donanimsal CRC
 *
 * Gemini ipucu: STM32G4 CRC donanimı varsayilan olarak 32-bit polynomial
 * ile calisir. 16-bit CRC icin .ioc ayarlarini kontrol et:
 *
 *   CubeMX -> CRC -> Configuration:
 *     Default Polynomial State : DISABLE
 *     Generated polynomial     : 0x8005   (CRC-16/IBM)
 *     CRC length               : 16-BIT
 *     Input Data Format        : BYTES
 *     Input Data Inversion     : NONE
 *     Output Data Inversion    : DISABLED
 *
 * Eger CRC donanimı 32-bit polynomial ile konfigüre edilmisse, buradaki
 * & 0xFFFF maskesi yanlis sonuc verir (polinom uyumsuzlugu).
 *
 * Guvenli alternatif: Donanimsal CRC kullanilamiyorsa yazilimsal CRC16
 * hesabi yapan Software_CRC16() fonksiyonu asagida yedek olarak bulunur.
 * .ioc'ta CRC dogru ayarlanmissa HW versiyonu kullanilir (daha hizli).
 *
 * .ioc kontrol listesi:
 *   [!] CRC -> Activated: YES
 *   [!] CRC Length: 16-BIT
 *   [!] Default Polynomial: DISABLED, Poly: 0x8005
 *   [!] Input Data Format: BYTES
 * ===========================================================================*/

/* USE_SW_CRC varsayilan olarak ACIK -- endian-independent, guvenli.
 * HW CRC kullanmadan once test vektoru ile dogrula:
 *   data={0x01,0x02,0x03,0x04} -> beklenen CRC16/IBM = 0x89C3
 *   Eslesiyorsa: bu satiri comment'e al -> HW CRC aktif olur.
 */
#define USE_SW_CRC

#ifdef USE_SW_CRC
static uint16_t Software_CRC16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0x0000u;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000u) {
        crc = (crc << 1) ^ 0x8005u;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
#endif

uint16_t Calc_CRC16(const uint8_t *data, uint16_t len) {

#ifdef USE_SW_CRC
  /* Endian-independent, polinom 0x8005. PC log okuyucu ayni polinom kullanmali.
   */
  return Software_CRC16(data, len);
#else
  /* DONANIMSAL CRC -- ENDIANNESS UYARISI (audit bulgusu #2):
   * Manuel word olusturma Big-Endian dizilim uretir (MSB once).
   * STM32G4 Little-Endian islemcidir -- HW ve SW farkli sonuc uretebilir.
   * Log okuyucu hangi byte sirasini bekledigini netlestirinceye kadar
   * USE_SW_CRC kullan. */
  __HAL_CRC_DR_RESET(&hcrc);

  uint16_t i;
  for (i = 0; i + 3 < len; i += 4) {
    uint32_t word = ((uint32_t)data[i] << 24) | ((uint32_t)data[i + 1] << 16) |
                    ((uint32_t)data[i + 2] << 8) | ((uint32_t)data[i + 3]);
    HAL_CRC_Accumulate(&hcrc, &word, 1);
  }
  if (i < len) {
    uint32_t remain = 0;
    uint8_t shift = 24;
    for (; i < len; i++, shift -= 8) {
      remain |= ((uint32_t)data[i] << shift);
    }
    HAL_CRC_Accumulate(&hcrc, &remain, 1);
  }
  return (uint16_t)(hcrc.Instance->DR & 0xFFFFu);
#endif
}

/* ===========================================================================
 * KARA KUTU -- LOG KUYRUĞA EKLE (Non-blocking)
 * ===========================================================================*/
void BlackBox_Log(FailsafeReason_t reason) {
  /* __attribute__((aligned(4))): Derleyici bu yapiyi stack'te 4-byte
   * hizali adrese koymak zorunda kalir. #pragma pack(1) ile paketlenmis
   * LogRecord_t normalde hizasiz adreste oturabilir; bu belirtec bunu
   * engeller ve Calc_CRC16 icindeki HW CRC akisinin guvenli calismasini
   * garanti eder. */
  __attribute__((aligned(4))) LogRecord_t rec;

  rec.magic = 0x0E47u;
  rec.timestamp_ms = HAL_GetTick();
  rec.fs_state = (uint8_t)currentState;
  rec.fs_reason = (uint16_t)(fsReasonMask | reason);
  rec.gaz_x10 = (int16_t)(gaz * 10);
  rec.accel_x10 = (int16_t)(Accel * 10);
  rec.temp_x10 = (int16_t)(OrtTemp * 10.0f);
  rec.volt_x10 = (int16_t)(OrtVolt * 10.0f);
  rec.curr_x10 = (int16_t)(OrtCurr * 10.0f);
  rec.rpm_x10 = (int16_t)(devirdak * 10.0f);
  rec.hall = hallval;
  rec.flags = (uint8_t)((DirF ? 0x01u : 0u) | (BrakeF ? 0x02u : 0u) |
                        (hareket ? 0x04u : 0u) | (gazizin ? 0x08u : 0u) |
                        (panicF ? 0x10u : 0u) | (ilkkontak ? 0x20u : 0u));
  rec.crc16 = Calc_CRC16((uint8_t *)&rec, sizeof(LogRecord_t) - 2u);

  if (logCount < LOG_QUEUE_SIZE) {
    logQueue[logHead] = rec;
    logHead = (logHead + 1u) % LOG_QUEUE_SIZE;
    logCount++;
  } else {
    logQueue[logHead] = rec;
    logHead = (logHead + 1u) % LOG_QUEUE_SIZE;
    logTail = (logTail + 1u) % LOG_QUEUE_SIZE;
  }
}

/* ===========================================================================
 * KARA KUTU -- UART4'E YAZ (Non-blocking, her cagirida 1 kayit)
 *
 * Motor_Loop() sonunda cagir.
 *
 * BAUD RATE TAVSIYESI (audit bulgusu):
 * Her LogRecord_t = 24 byte + 2 byte oneki = 26 byte.
 * Motor_Loop 100ms'de bir adcRead() cagirir, en kotu durumda
 * her 100ms'de 1 kayit uretilir = 10 kayit/saniye = 260 byte/s.
 * Ancak failsafe gecislerinde ani log patlamasi olabilir.
 *
 * Onerilen baud rate:
 *   UART4 (Kara Kutu) : 460800 bps  -- veri kaybini onler
 *   USART2 (Debug)    : 115200 bps  -- insan okumasi icin yeterli
 *
 * .ioc'ta degistir:
 *   Connectivity -> UART4 -> Baud Rate: 460800
 *
 * UART4 busy kontrolu: HAL_UART_Transmit_IT HAL_BUSY donerse
 * bir sonraki Motor_Loop dongesinde tekrar denenir -- kayit kaybi yok.
 * ===========================================================================*/
static volatile bool uart4Busy = false;

void HAL_UART4_TxCpltCallback_Internal(void) { uart4Busy = false; }

/* stm32g4xx_it.c'deki HAL_UART_TxCpltCallback'e ekle:
 *   extern void HAL_UART4_TxCpltCallback_Internal(void);
 *   void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == UART4)   { HAL_UART4_TxCpltCallback_Internal();
 * } if (huart->Instance == USART2)  { ... USART2 callback ... }
 *   }
 */

void BlackBox_Flush(void) {
  if (logCount == 0u)
    return;

  /* Onceki iletim devam ediyorsa skip et */
  if (uart4Busy)
    return;

  /* 115200 baud @ 26 byte = ~2.3ms per record.
   * Ardisik gonderimler arasinda minimum 3ms bekleterek
   * UART buffer tasmasini ve motor dongu gecikmesini onle.
   * .ioc'ta UART4 baud rate 460800'e yukseltilebilirse
   * bu kontrol kaldirilabilir. */
  static uint32_t lastFlush_ms = 0;
  uint32_t now_flush = HAL_GetTick();
  if ((now_flush - lastFlush_ms) < 3u)
    return;
  lastFlush_ms = now_flush;

  /* RACE CONDITION COZUMU (audit bulgusu #1):
   *
   * Eski yontem: static txBuf + memcpy
   *   Sorun: HAL_UART_Transmit_IT non-blocking calisir. UART donanimı
   *   txBuf'i arka planda okurken, bir sonraki BlackBox_Flush cagrisi
   *   uart4Busy henuz temizlenmemisse (kesme gecikmesi) ayni buffer'i
   *   ezebilir. volatile uart4Busy koruma saglar ama kesme latency'si
   *   varsa bu pencere kapanmaz.
   *
   * Yeni yontem: dogrudan logQueue uzerinden gonder
   *   logQueue[logTail] UART gonderimi tamamlanana kadar sabit kalir.
   *   uart4Busy=true iken logTail guncellenmez -- kuyruk indeksi
   *   sadece HAL_OK donus aldiktan sonra ilerler.
   *   Bu sayede UART donanimı her zaman gecerli, ezilmemis veriyi okur.
   *
   * Tek kisitlama: kuyruktaki kayit, gonderim tamamlanana kadar
   * overwrite edilmemeli. LOG_QUEUE_SIZE=32, UART 460800bps'de
   * 26 byte = ~0.45ms surer. Motor_Loop bir geciste en fazla
   * 1 kayit uretir ve 100ms'de bir tetiklenir. Risk sifir.
   *
   * Oneki (senkronizasyon baslangici) icin 2 byte ayri static buffer:
   * Bu 2 byte sabittir, gonderim sirasinda degismez -- guvenli. */
  static const uint8_t preamble[2] = {0x50u, 0x0Eu};

  /* Oneki + kayit birlikte gonderilemiyor (scatter-gather yok).
   * Cozum: oneki + kayit iceren tam frame'i tek buffer'da topla.
   * Buffer UART gonderimi boyunca sabit kalmali -- logQueue'daki
   * slot gonderim bitmeden degismeyecegi garanti altinda. */
  static uint8_t txBuf[sizeof(LogRecord_t) + 2u];
  txBuf[0] = preamble[0];
  txBuf[1] = preamble[1];

  /* logQueue[logTail] kopyasi -- uart4Busy=true iken logTail degismez.
   * Kesme logTail'i degistirmez, sadece uart4Busy'yi temizler.
   * Yani bu memcpy ile UART gonderimi arasinda veri tutarlidir. */
  memcpy(&txBuf[2], &logQueue[logTail], sizeof(LogRecord_t));

  uart4Busy = true;
  if (HAL_UART_Transmit_IT(&huart4, txBuf,
                           (uint16_t)(sizeof(LogRecord_t) + 2u)) == HAL_OK) {
    /* Gonderim baslatildi -- logTail gonderim tamamlaninca degil,
     * simdi ilerle. txBuf'taki kopya UART'in isine yarar.
     * logQueue slotu bir sonraki BlackBox_Log yazisi icin serbest. */
    logTail = (logTail + 1u) % LOG_QUEUE_SIZE;
    logCount--;
  } else {
    uart4Busy = false;
  }
}

/* ===========================================================================
 * TELEMETRI -- USART2 ASCII CIKIS
 *
 * Format (v3.1):
 *   $PHL,<tick>,<gaz>,<accel>,<T.t>,<V.v>,<A.a>,<rpm>,<state>,<hall>,<reason>,<q%>
 *
 * Alan aciklamalari:
 *   tick   : HAL_GetTick() -- ms cinsinden sistem suresi
 *   gaz    : Anlık PWM degeri (0-250)
 *   accel  : Gaz pedali hedef degeri (0-250)
 *   T.t    : Sicaklik (ornek: 24.7 = 24.7 derece C)
 *   V.v    : Voltaj  (ornek: 72.4 = 72.4 V)
 *   A.a    : Akim    (ornek:  4.2 =  4.2 A)
 *   rpm    : Devir   (ornek: 1440 = 1440 RPM)
 *   state  : Failsafe durumu (0=NORMAL 1=WARN 2=REDUCED 3=LIMP 4=HALT 5=PANIC)
 *   hall   : Hall sensor degeri (1-6, gecersiz=0 veya 7)
 *   reason : Failsafe sebep bitmask (hex ile okumak daha kolay)
 *   q%     : Kara kutu kuyruk doluluk orani 0-100 (audit tavsiyesi)
 *            0  = kuyruk bos, yazma hizina yetisiyor
 *            100 = kuyruk tam dolu, veri kaybi riski var!
 *            Pistte bu deger surekli yuksekse UART4 baud rate artir.
 *
 * Ornek cikti:
 *   $PHL,12345,180,175,24.7,72.4,4.2,1440,0,5,0,3
 *
 * Non-blocking IT versiyonu -- onceki iletim bitmemisse skip eder.
 * ===========================================================================*/
static char uartTxBuf[128]; /* 96 -> 128: q% alani icin buyutuldu */
static volatile bool uartBusy = false;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    uartBusy = false;
  }
  if (huart->Instance == UART4) {
    HAL_UART4_TxCpltCallback_Internal();
  }
}

void Telemetry_SendUART(void) {
  if (uartBusy)
    return;

  int temp_i = (int)(OrtTemp * 10.0f);
  int volt_i = (int)(OrtVolt * 10.0f);
  int curr_i = (int)(OrtCurr * 10.0f);
  int rpm_i = (int)(devirdak);

  /* Kuyruk doluluk orani: 0-100
   * logCount / LOG_QUEUE_SIZE * 100
   * LOG_QUEUE_SIZE = 32, tam sayi bolme yeterli hassasiyet saglar */
  int queue_pct = (int)((logCount * 100u) / LOG_QUEUE_SIZE);

  /* Negatif deger koruması:
   * temp_i=-5 → "-5/10=0, -5%10=-5" → "0.-5" YANLIS cikti.
   * Cozum: isaretini ayir, mutlak degeri formatla.
   * Sicaklik: teorik olarak negatif olabilir (soğuk ortam testi).
   * Akim/Voltaj: INA241 negatif dondurmez ama savunmaci yaz. */
  int temp_sign = (temp_i < 0) ? -1 : 1;
  int volt_sign = (volt_i < 0) ? -1 : 1;
  int curr_sign = (curr_i < 0) ? -1 : 1;
  if (temp_i < 0)
    temp_i = -temp_i;
  if (volt_i < 0)
    volt_i = -volt_i;
  if (curr_i < 0)
    curr_i = -curr_i;

  int len = snprintf(
      uartTxBuf, sizeof(uartTxBuf),
      "$PHL,%lu,%d,%d,%s%d.%d,%s%d.%d,%s%d.%d,%d,%d,%d,%d,%d\r\n",
      (uint32_t)HAL_GetTick(), gaz, Accel, (temp_sign < 0) ? "-" : "",
      temp_i / 10, temp_i % 10, (volt_sign < 0) ? "-" : "", volt_i / 10,
      volt_i % 10, (curr_sign < 0) ? "-" : "", curr_i / 10, curr_i % 10, rpm_i,
      (int)currentState, hallval, (int)fsReasonMask, queue_pct);

  if (len > 0 && len < (int)sizeof(uartTxBuf)) {
    uartBusy = true;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)uartTxBuf, (uint16_t)len);
  }
}

/* ===========================================================================
 * TELEMETRI -- FDCAN1 CAN BUS CIKIS
 *
 * 4 adet klasik CAN frame (8 byte), 100ms periyot:
 *   0x100: Motor Status  -- gaz, accel, RPM, hall, DirF, BrakeF
 *   0x101: Sensor Data   -- sicaklik, voltaj, akim (x10 int16)
 *   0x102: Failsafe      -- state, reason mask, deneStart, deneSys
 *   0x103: Odometer      -- totalKm (float), km/h (x10), totalM (uint16)
 *
 * Non-blocking: HAL_FDCAN_AddMessageToTxFifoQ kuyruga ekler.
 * TX FIFO doluysa frame skip edilir -- Motor_Loop bloklanmaz.
 * ===========================================================================*/
void Telemetry_SendCAN(void) {

  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t txData[8];

  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  /* ---- Frame 1: Motor Status (0x100) ---- */
  TxHeader.Identifier = CAN_ID_MOTOR_STATUS;
  txData[0] = (uint8_t)(gaz & 0xFF);           /* gaz low byte      */
  txData[1] = (uint8_t)((gaz >> 8) & 0xFF);    /* gaz high byte     */
  txData[2] = (uint8_t)(Accel & 0xFF);          /* accel low byte    */
  txData[3] = (uint8_t)((Accel >> 8) & 0xFF);   /* accel high byte   */
  int16_t rpm_can = (int16_t)devirdak;
  txData[4] = (uint8_t)(rpm_can & 0xFF);        /* RPM low byte      */
  txData[5] = (uint8_t)((rpm_can >> 8) & 0xFF); /* RPM high byte     */
  txData[6] = hallval;                           /* hall sensor value */
  txData[7] = (uint8_t)((DirF ? 0x01u : 0u) |
              (BrakeF ? 0x02u : 0u) |
              (hareket ? 0x04u : 0u) |
              (gazizin ? 0x08u : 0u));           /* flags             */
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData);

  /* ---- Frame 2: Sensor Data (0x101) ---- */
  TxHeader.Identifier = CAN_ID_SENSOR_DATA;
  int16_t temp_can = (int16_t)(OrtTemp * 10.0f);
  int16_t volt_can = (int16_t)(OrtVolt * 10.0f);
  int16_t curr_can = (int16_t)(OrtCurr * 10.0f);
  txData[0] = (uint8_t)(temp_can & 0xFF);        /* temp x10 low   */
  txData[1] = (uint8_t)((temp_can >> 8) & 0xFF); /* temp x10 high  */
  txData[2] = (uint8_t)(volt_can & 0xFF);        /* volt x10 low   */
  txData[3] = (uint8_t)((volt_can >> 8) & 0xFF); /* volt x10 high  */
  txData[4] = (uint8_t)(curr_can & 0xFF);        /* curr x10 low   */
  txData[5] = (uint8_t)((curr_can >> 8) & 0xFF); /* curr x10 high  */
  int16_t regen_can = (int16_t)(regenVal * 10.0f);
  txData[6] = (uint8_t)(regen_can & 0xFF);       /* regen x10 low  */
  txData[7] = (uint8_t)((regen_can >> 8) & 0xFF);/* regen x10 high */
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData);

  /* ---- Frame 3: Failsafe (0x102) ---- */
  TxHeader.Identifier = CAN_ID_FAILSAFE;
  txData[0] = (uint8_t)currentState;             /* failsafe state  */
  txData[1] = (uint8_t)(fsReasonMask & 0xFF);    /* reason low byte */
  txData[2] = (uint8_t)((fsReasonMask >> 8) & 0xFF); /* reason high */
  txData[3] = deneStart;                         /* kalkis asiri akim sayaci */
  txData[4] = deneSys;                           /* sistem asiri akim sayaci */
  txData[5] = (uint8_t)((acil ? 0x01u : 0u) |
              (panicF ? 0x02u : 0u) |
              (overstartcurrentF ? 0x04u : 0u) |
              (oversyscurrentF ? 0x08u : 0u) |
              (warntempF ? 0x10u : 0u) |
              (maxtempF ? 0x20u : 0u) |
              (pilmaxF ? 0x40u : 0u) |
              (pilminF ? 0x80u : 0u));           /* error flags     */
  txData[6] = 0;
  txData[7] = 0;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData);

  /* ---- Frame 4: Odometer (0x103) ---- */
  TxHeader.Identifier = CAN_ID_ODOMETER;
  int16_t kmh_can = (int16_t)(kmsaat * 10.0f);
  uint16_t totalKm_u = (uint16_t)totalKm;
  uint16_t totalM_u  = (uint16_t)totalM;
  txData[0] = (uint8_t)(totalKm_u & 0xFF);       /* totalKm low     */
  txData[1] = (uint8_t)((totalKm_u >> 8) & 0xFF);/* totalKm high    */
  txData[2] = (uint8_t)(totalM_u & 0xFF);        /* totalM low      */
  txData[3] = (uint8_t)((totalM_u >> 8) & 0xFF); /* totalM high     */
  txData[4] = (uint8_t)(kmh_can & 0xFF);         /* km/h x10 low    */
  txData[5] = (uint8_t)((kmh_can >> 8) & 0xFF);  /* km/h x10 high   */
  txData[6] = 0;
  txData[7] = 0;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData);
}

/* ===========================================================================
 * YAZILIM WATCHDOG
 * ===========================================================================*/
void SW_Watchdog_Feed(void) {
  swWdgLastFeed = HAL_GetTick();
  swWdgEnabled = true;
}

void SW_Watchdog_Check(void) {
  if (!swWdgEnabled)
    return;
  if ((HAL_GetTick() - swWdgLastFeed) > SW_WATCHDOG_TIMEOUT_MS) {
    Safety_SetState(FS_HALT, FSR_SW_WATCHDOG);
  }
}

/* ===========================================================================
 * FAILSAFE STATE MACHINE
 *
 * Gecis kurali: Bir ust seviyeye (daha kisitlayici) her zaman gecilir.
 * Geri donus: FS_WARNING ve FS_REDUCED_POWER icin hysteresis recovery ile.
 *             FS_HALT ve FS_PANIC asla otomatik geri donmez.
 * ===========================================================================*/
void Safety_SetState(FailsafeState_t newState, FailsafeReason_t reason) {
  if (newState <= currentState)
    return;

  fsReasonMask |= (uint16_t)reason;
  currentState = newState;

  /* Yeni hatada recovery sayacini sifirla */
  recoveryActive = false;
  recoveryStartTime = 0;

  BlackBox_Log(reason);

  /* Gaz degerini HEMEN yeni kisitlamaya indir.
   * Gemini bulgusu: failsafe gecisinde gaz rampayla azaliyordu,
   * oysa guvenlik kisitlamasi aninda uygulanmali.
   * Safety_GetMaxPWM() henuz guncellendi (currentState degisti),
   * burada gaz aninda yeni limite cekilir. */
  uint8_t newLimit = Safety_GetMaxPWM();
  if ((int)gaz > (int)newLimit) {
    gaz = (int)newLimit;
  }

  switch (currentState) {
  case FS_WARNING:
    BlinkStat = 1;
    break;
  case FS_REDUCED_POWER:
    BlinkStat = 2;
    break;
  case FS_LIMP_HOME:
    BlinkStat = 2;
    break;
  case FS_HALT:
  case FS_PANIC:
    gaz = 0;
    acil = true;
    deadtime();
    BlinkStat = 2;
    break;
  default:
    break;
  }
}

/* ===========================================================================
 * [G3] FAILSAFE RECOVERY -- Sensör duzeldikten sonra geri donus
 *
 * Hangi durumlar recover olabilir:
 *   FS_WARNING       -> FS_NORMAL        (sicaklik/voltaj uzun sure normal)
 *   FS_REDUCED_POWER -> FS_WARNING       (asiri sicaklik gecti)
 *
 * Hangi durumlar recover OLAMAZ:
 *   FS_LIMP_HOME     -- Hall timeout veya ciddi akim hatasi -- reset gerekir
 *   FS_HALT          -- Kritik hata -- reset gerekir
 *   FS_PANIC         -- Fiziksel panik butonu -- reset gerekir
 *
 * Mantik:
 *   1) Hata kosullari gecmis mi kontrol et (sicaklik/voltaj normal mi?)
 *   2) Gectiyse sayac baslat
 *   3) RECOVERY_HOLD_MS boyunca hata yoksa bir adim geri dön
 *   4) Hata tekrar gelirse sayaci sifirla
 * ===========================================================================*/
static void Safety_RecoveryCheck(uint32_t now) {

  /* FS_HALT ve FS_PANIC hic recover olmaz */
  if (currentState >= FS_HALT)
    return;
  /* FS_NORMAL'de yapacak bir sey yok */
  if (currentState == FS_NORMAL)
    return;

  /* FS_LIMP_HOME: sebebe gore karar ver.
   *
   * Audit bulgusu: fsReasonMask'taki bit hic silinmediginden,
   * bir kez FSR_HALL_TIMEOUT alan arac sonsuza dek LIMP'te kaliyordu.
   *
   * Cozum: Maske yerine ANLIK sensör durumuna bak.
   *   - Hall anlık olarak gecersiz mi? (hallval == 0 || 7)
   *   - Sistem akimi hala yuksek mi?
   *   - Voltaj hala dusuk mu?
   * Bu kontroller geciyorsa recover olabilir -- maske gecmisi degil.
   *
   * fsReasonMask kasitli temizlenmez -- kara kutu icin tarihcede kalir.
   * Sadece karar mantigi maskeden degil anlık olcumden yapilir. */
  if (currentState == FS_LIMP_HOME) {
    /* Hall anlık gecersiz degerse (0=tum kapali, 7=tum acik = sensor arizasi)
     */
    uint8_t hsnap = hallval;
    if (hsnap == 0u || hsnap == 7u)
      return;
    /* Sistem akimi hala tehlikeli seviyede */
    if (OrtCurr > (maxsystemamp * 0.9f))
      return;
    /* Voltaj hala dusukse (batarya bitmis) */
    if (OrtVolt < (pilmin + 1.0f) && OrtVolt > 1.0f)
      return;
  }

  /* Hata kosullari hala devam ediyor mu?
   *
   * Hysteresis: esik degerlerinden biraz asagida kontrol et.
   * Ornek: pilmin=60V, recovery icin 61V uzerinde olmali.
   * Bu sayede voltaj tam esik noktasinda saliniyor olsa bile
   * sistem NORMAL <-> WARNING arasinda surekli gecis yapamaz.
   *
   * Gemini bulgusu: akım kontrolü eksikti -- eklendi.
   */
  bool stillFaulty = false;

  /* Sicaklik: warntemp esigi hala asiliyor mu?
   * Hysteresis: 1 derece tolerans */
  if (OrtTemp > (warntemp - 1.0f))
    stillFaulty = true;

  /* Voltaj: hala dusuk veya yuksek mi?
   * Hysteresis: 1V tolerans */
  if (OrtVolt < (pilmin + 1.0f) && OrtVolt > 1.0f)
    stillFaulty = true;
  if (OrtVolt > (pilmax - 1.0f))
    stillFaulty = true;

  /* Akim: hala sistem limitini asiyor mu?
   * Gemini #2 duzeltmesi: Recovery'e akim kontrolu eklendi.
   * Kalkis akimi (maxstartamp) ani spike'lardan olusur,
   * sadece sistem akimi (maxsystemamp) recovery'i engeller.
   * Hysteresis: %10 tolerans */
  if (OrtCurr > (maxsystemamp * 0.9f))
    stillFaulty = true;

  /* Panik butonu hala basili mi? */
  if (panicF)
    stillFaulty = true;

  if (stillFaulty) {
    /* Hata sürüyor -- sayaci sifirla */
    recoveryActive = false;
    recoveryStartTime = 0;
    return;
  }

  /* Hata gecti -- sayaci baslat veya kontrol et */
  if (!recoveryActive) {
    recoveryActive = true;
    recoveryStartTime = now;
    return;
  }

  /* RECOVERY_HOLD_MS boyunca hata yoksa bir adim geri dön */
  if ((now - recoveryStartTime) >= RECOVERY_HOLD_MS) {
    recoveryActive = false;

    /* RECOVERY RAMP (audit bulgusu #4):
     * FS_REDUCED_POWER -> FS_WARNING gecisinde maxpwm limiti %60'tan %100'e
     * cikar. Surucü o anda tam gaz basiyorsa araç aniden firlar -- mekanik
     * aksam icin tehlikeli.
     *
     * Cozum: Durum degistiginde gaz ANINDA artmaz.
     * recoveryRampActive bayragi Motor_Loop tarafinda kontrol edilir.
     * Motor_Loop, ramp aktifken Accel hedefini Safety_GetMaxPWM() ile kisitlar.
     * Surucü gazı birakirsa veya mevcut gaz zaten limitin altindaysa sorun yok.
     *
     * recoveryRampActive extern olarak motor_driver.c'de tanimlanir,
     * buradan sadece set edilir. Motor_Loop onu temizler. */
    extern volatile bool recoveryRampActive;

    /* Bir adim geri dön */
    if (currentState == FS_LIMP_HOME) {
      currentState = FS_REDUCED_POWER;
      /* Gazı yeni limite indir -- gerekirse anında kes (LIMP'ten cikiyoruz,
       * guvenli) */
      if ((int)gaz > (int)(maxpwm * 60u / 100u)) {
        gaz = (int)(maxpwm * 60u / 100u);
      }
      recoveryRampActive = true;
      BlinkStat = 2;
      BlackBox_Log(FSR_NONE);
    } else if (currentState == FS_WARNING) {
      currentState = FS_NORMAL;
      /* FS_WARNING -> FS_NORMAL: maxpwm limiti kalkar.
       * Ramp aktif: Motor_Loop gazi yeni limite aninda cikarmaz, yavascayavaş
       * arttirir. */
      recoveryRampActive = true;
      BlinkStat = 1;
      BlackBox_Log(FSR_NONE);
    } else if (currentState == FS_REDUCED_POWER) {
      currentState = FS_WARNING;
      /* FS_REDUCED_POWER -> FS_WARNING: %60'tan %100'e cikar.
       * Ramp aktif: ani fırlama önlenir. */
      recoveryRampActive = true;
      BlinkStat = 1;
      BlackBox_Log(FSR_NONE);
    }
    /* fsReasonMask kasitli temizlenmez -- hangi hatanin oldugunun kaydı kalir
     */
  }
}

uint8_t Safety_GetMaxPWM(void) {
  switch (currentState) {
  case FS_NORMAL:
    return maxpwm;
  case FS_WARNING:
    return maxpwm;
  case FS_REDUCED_POWER:
    return (uint8_t)(maxpwm * 60u / 100u);
  case FS_LIMP_HOME:
    return (uint8_t)(maxpwm * 30u / 100u);
  case FS_HALT:
  case FS_PANIC:
    return 0u;
  default:
    return 0u;
  }
}

FailsafeState_t Safety_GetState(void) { return currentState; }

const char *Safety_GetStateName(void) {
  switch (currentState) {
  case FS_NORMAL:
    return "NORMAL";
  case FS_WARNING:
    return "WARNING";
  case FS_REDUCED_POWER:
    return "REDUCED";
  case FS_LIMP_HOME:
    return "LIMP";
  case FS_HALT:
    return "HALT";
  case FS_PANIC:
    return "PANIC";
  default:
    return "UNKNOWN";
  }
}

/* ===========================================================================
 * ANA GUVENLIK KONTROLU
 *
 * [G1] BUG FIX: hallval volatile race condition cozümü.
 *   Eski kod: if (hareket && hallval == ihallval)
 *   Sorun: Safety_Check calisirken EXTI kesmesi hallval'i degistirebilir.
 *   Cortex-M4 uint8_t okumasi atomik olsa da, karmasik kosullarda
 *   tutarli snapshot almak iyi pratiktir.
 *   Cozum: hallval ve ihallval'in anlık kopyalari alinir, tüm kontroller
 *   bu kopyalar üzerinden yapilir.
 * ===========================================================================*/
FailsafeState_t Safety_Check(void) {
  uint32_t now = HAL_GetTick();

  /* [G1] Volatile degiskenlerin anlık kopyalari -- tutarli okuma */
  uint8_t hall_snap = hallval;
  uint8_t ihall_snap = ihallval;
  bool hareket_snap = hareket;

  /* adcRead()'den gelen flag snapshot'lari.
   * adcRead() artik acil=true veya PWM_AllOff() cagirmiyor (audit #1).
   * Karar burada, tek merkezden veriliyor. */
  bool overcurr_start_snap = overstartcurrentF;
  bool overcurr_sys_snap = oversyscurrentF;

  /* Sicaklik */
  if (OrtTemp > maxtemp) {
    Safety_SetState(FS_HALT, FSR_OVERTEMP_CRIT);
  } else if (OrtTemp > warntemp) {
    if (currentState < FS_WARNING)
      Safety_SetState(FS_WARNING, FSR_OVERTEMP_WARN);
    if (OrtTemp > (warntemp + (maxtemp - warntemp) * 0.7f))
      Safety_SetState(FS_REDUCED_POWER, FSR_OVERTEMP_WARN);
  }

  /* Voltaj */
  if (OrtVolt < pilmin && OrtVolt > 1.0f) {
    Safety_SetState(FS_LIMP_HOME, FSR_UNDERVOLT);
  }
  if (OrtVolt > pilmax) {
    Safety_SetState(FS_HALT, FSR_OVERVOLT);
  }

  /* Akim -- hem anlik deger hem ardisik sayac kontrolu.
   *
   * Eski adcRead() icindeki "deneStart>=3 -> acil=true" mantigi
   * buraya tasindi (audit #1 duzeltmesi):
   *   - Tek asimi (1 olcum): FS_REDUCED_POWER / FS_HALT
   *   - 3 ardisik asim: FS_HALT + acil=true (kalici, reset gerekir)
   *
   * deneStart / deneSys sayaclari motor_driver.c'de arttirilir,
   * karar ve loglama bu fonksiyonda yapilir. */
  extern uint8_t deneStart;
  extern uint8_t deneSys;

  if (overcurr_start_snap) {
    Safety_SetState(FS_REDUCED_POWER, FSR_OVERCURR_START);
    if (deneStart >= 3u) {
      Safety_SetState(FS_HALT, FSR_OVERCURR_START);
      acil = true;
    }
  }
  if (overcurr_sys_snap) {
    Safety_SetState(FS_HALT, FSR_OVERCURR_SYS);
    if (deneSys >= 3u) {
      acil = true; /* Kalici acil -- reset gerekir */
    }
  }
  /* OrtCurr uzerinden de anlık kontrol -- adcRead ortalamasi
   * flag'den once gelebilir, ikinci savunma hatti */
  if (OrtCurr > maxsystemamp) {
    Safety_SetState(FS_HALT, FSR_OVERCURR_SYS);
  }
  if (OrtCurr > maxstartamp && !hareket_snap) {
    Safety_SetState(FS_REDUCED_POWER, FSR_OVERCURR_START);
  }

  /* Hall sensor timeout -- [G1] snapshot'lar kullaniliyor */
  if (hareket_snap && hall_snap == ihall_snap) {
    if ((now - lastHallChange_ms) > HALL_TIMEOUT_MS) {
      Safety_SetState(FS_LIMP_HOME, FSR_HALL_TIMEOUT);
    }
  } else {
    lastHallChange_ms = now;
  }

  /* ADC timeout */
  if ((now - lastADCRead_ms) > ADC_TIMEOUT_MS) {
    Safety_SetState(FS_WARNING, FSR_ADC_TIMEOUT);
  }

  /* PANIC pin */
  if (panicF) {
    Safety_SetState(FS_PANIC, FSR_PANIC_PIN);
  }

  /* Yazilim watchdog */
  SW_Watchdog_Check();

  /* [G3] Recovery kontrolu -- sensör duzeldikten sonra geri donus */
  Safety_RecoveryCheck(now);

  return currentState;
}

void Safety_ADCUpdated(void) { lastADCRead_ms = HAL_GetTick(); }

/* ===========================================================================
 * BASLAMA
 * Motor_Init() icinden cagirilir.
 * ===========================================================================*/
void Safety_Init(void) {
  currentState = FS_NORMAL;
  fsReasonMask = FSR_NONE;
  logHead = 0;
  logTail = 0;
  logCount = 0;
  swWdgEnabled = false;
  recoveryActive = false;
  recoveryStartTime = 0;
  lastADCRead_ms = HAL_GetTick();
  lastHallChange_ms = HAL_GetTick();

  BlackBox_Log(FSR_NONE);

  /* Watchdog'u hemen besle -- Motor_Loop ilk Feed'e ulasana kadar
   * gecen sure icinde kilitlenme olursa fark edilsin.
   * swWdgEnabled burada true yapiliyor, artik SW_Watchdog_Check aktif. */
  SW_Watchdog_Feed();
}
