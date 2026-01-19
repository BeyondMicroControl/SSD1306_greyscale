/*
GND ━━━━━━━━━━━━━━━━━━━━━━━━━━━┯━━━━━━┯━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                               │      │                      ┃
+5V ─┬─────────────────────────│──────│───┬──────────────────┃────────────────────────────────────────┬───
     │                         │      │   │                  ┃  ╔══════════════════════════════════╗  │
     │                         │      │   │                  ┃  ║           ╰──USB──╯              ║  │
     │  ╔═══════════════════╗  │      │   │                  ┃  ║                                  ║  │
     │  ║ ⬤  [ATTinyX24]   ║  │      │   │                  ┃  ║         [Teensy 4.0]             ║  │
     │  ║                   ║  │      │   │   ┌───────────┐  ┃  ║      ARM Cortex-M7 600MHz        ║  │
     └──╢ VCC           GND ╟──┘      │   │   │   ┌─────┐ │  ┃  ║                                  ║  │
        ║                   ║     ╔═══╧═══╧═══╧═══╧═══╗ │ │  ┗━━╢GND                            VIN╟──┘
        ╢ PA4       SCK PA3 ╟     ║  GND VCC SCL SDA  ║ │ │    ─╢RX1  CRX2   -0                 GND╟─
        ║                   ║     ║┌─────────────────┐║ │ │    ─╢TX1  CTX2   -1                 3V3╟─
        ╢ PA5  RX1 MISO PA2 ╟     ║│                 │║ │ │    ─╢     OUT2   -2  23- MCLK1 CRX1  A9╟─
        ║                   ║     ║│    [SSD1306]    │║ │ │    ─╢     LRCLK2 -3  22-       CTX1  A8╟─
        ╢ PA6  TX1 MOSI PA1 ╟     ║│  64 x 32 OLED   │║ │ │    ─╢     BCLK2  -4  21-  BCLK1 RXS  A7╟─
        ║                   ║     ║│                 │║ │ │    ─╢     IN2    -5  20-       TXS   A6╟─
        ╢ PA7      UPDI PA0 ╟     ║└─────────────────┘║ │ │    ─╢     OUT1D  -6  19-  SCL0       A5╟─────┐
        ║                   ║     ╚═══════════════════╝ │ │    ─╢RX2  OUT1A  -7  18-  SDA0       A4├───┐ │
        ╢ PB3 RX0   SCL PB0 ╟───┐                       │ │    ─╢TX2  IN1    -8  17-  SDA1 TX4   A3╟─  │ │
        ║                   ║   │                       │ │    ─╢     OUT1C  -9  16-  SCL1 RX4   A2╟─  │ │
        ╢ PB2 TX0   SDA PB1 ╟───│───────────────────────┘ │    ─╢MQSR CS     -10 15-      RX3    A1╟─  │ │
        ╚═══════════════════╝   └─────────────────────────┘    ─╢MOSI CTX1   -11 14-      TX3    A0╟─  │ │
                                                               ─╢MISO MQSL   -12 13- CRX1 SCK (LED)╟─  │ │
                                                        ↑ ↑     ║                                  ║   │ │
                                                        │ │     ╚══════════════════════════════════╝   │ │
                                                        └─│────────────────────────────────────────────┘ │
                                                          └──────────────────────────────────────────────┘
*/

#include <Arduino.h>
#include <Wire.h>
#include <Tiny4kOLED.h>

#if defined(TEENSYDUINO) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
  #define USE_TEENSY_INTERVALTIMER 1
#endif

// ---------------- I2C speed ----------------
#ifndef I2C_HZ
  #define I2C_HZ 1000000UL
#endif

// ---------------- Bitmap ----------------

const uint8_t bitmap1_64x32[] PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xef, 0xc3, 0xc1, 0x01, 0x83, 0x83, 0x83, 0x87, 0x0f, 0x0f, 0x1f, 0x1f,
  0x3f, 0x3f, 0x3f, 0x1f, 0x1f, 0x1f, 0x1f, 0x0f, 0x0f, 0x0f, 0x07, 0x07, 0x0f, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0x07, 0x03,
  0x07, 0xff, 0xff, 0xff, 0x9f, 0x03, 0x00, 0x1f, 0x3f, 0x7f, 0x7f, 0x7b, 0xf1, 0xbb, 0xdf, 0xfe,
  0xee, 0xe4, 0xee, 0xf2, 0xd2, 0xbf, 0x7f, 0x7f, 0x3f, 0x3f, 0x0e, 0x03, 0xff, 0xff, 0xff, 0x3f,
  0x1f, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xfc,
  0xfe, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xfc, 0xf0, 0xe4, 0xd7, 0xb7, 0x77, 0xe7, 0xef,
  0xef, 0xef, 0xef, 0xef, 0x6f, 0xaf, 0xcf, 0xcf, 0xee, 0xe8, 0xc6, 0xff, 0xff, 0xff, 0xff, 0xfc,
  0xfc, 0xfc, 0xff, 0xff, 0xff, 0xf8, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfd,
  0xfd, 0xfd, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

const uint8_t bitmap2_64x32[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x10, 0x04, 0x00, 0xc0, 0x80, 0x80, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x08, 0x90, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0x80, 0x00, 0x00, 0x00, 0xf8, 0xfc,
  0xf8, 0x00, 0x00, 0x00, 0x60, 0xfc, 0xe1, 0x9f, 0xbf, 0x3f, 0x7f, 0x71, 0x31, 0x7f, 0x3e, 0x1e,
  0x1e, 0x08, 0x1e, 0x18, 0x39, 0x7e, 0xff, 0x7f, 0x7f, 0x9f, 0xd5, 0xfc, 0x00, 0x00, 0x00, 0xc0,
  0xe0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x01, 0x00, 0x80, 0x80, 0xc0, 0xc1, 0xe1, 0xe3, 0xe3, 0xfb, 0xf2, 0xe8, 0x88, 0x08, 0x18, 0x10,
  0x10, 0x10, 0x10, 0x10, 0x90, 0xd0, 0xb0, 0xf0, 0xd0, 0x97, 0xa1, 0x00, 0x00, 0x00, 0x00, 0x03,
  0x03, 0x03, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xf8, 0xfc, 0xfc,
  0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc,
  0xfa, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xfc, 0xf8,
  0xf0, 0xe0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static constexpr uint8_t W = 64;
static constexpr uint8_t H = 32;
static constexpr uint16_t SIZE = W * (H/8);

//uint8_t fb[SIZE];

// ---------------- PWM config (slot-based) ----------------
// A is shown for A_SLOTS slots, B for the remaining slots.
// Example TOTAL_SLOTS=4, A_SLOTS=3 => 75% / 25% duty, 4 perceived levels.
static constexpr uint16_t PWM_HZ      = 69;
static constexpr uint8_t  TOTAL_SLOTS = 3;
static constexpr uint8_t  A_SLOTS     = 2;
static_assert(A_SLOTS > 0 && A_SLOTS < TOTAL_SLOTS, "A_SLOTS must be 1..TOTAL_SLOTS-1");

static constexpr uint16_t SLOT_HZ = PWM_HZ * TOTAL_SLOTS;

#if defined(USE_TEENSY_INTERVALTIMER)
  // IntervalTimer uses microseconds. Rounded.
  static constexpr uint32_t SLOT_US = (1000000UL + (SLOT_HZ / 2)) / SLOT_HZ;
#endif

// RTC runs from 32.768 kHz clock in INT32K mode on ATtiny3224
static constexpr uint16_t RTC_HZ      = 32768;
static constexpr uint16_t TICKS_PER_SLOT = (RTC_HZ + (SLOT_HZ/2)) / SLOT_HZ; // rounded
static_assert(TICKS_PER_SLOT >= 2, "TICKS_PER_SLOT too small; lower SLOT_HZ or increase prescaler");

// ---------------- ISR <-> loop flags ----------------
volatile uint8_t  g_slot       = 0;
volatile uint8_t  g_toggle_req = 0;
volatile uint16_t g_isr_count  = 0;


// ---------------- Timebase backend (slot tick) ----------------
// Slot tick frequency: SLOT_HZ (= PWM_HZ * TOTAL_SLOTS)

#if defined(USE_TEENSY_INTERVALTIMER)

  #include <IntervalTimer.h>
  static IntervalTimer g_slotTimer;

  static void slot_isr()
  {
    g_isr_count++;

    uint8_t s = g_slot + 1;
    if (s >= TOTAL_SLOTS) s = 0;
    g_slot = s;

    if (s == A_SLOTS || s == 0) {
      g_toggle_req = 1;
    }
  }

  static void rtc_init_slots()
  {
    g_slot = 0;
    g_toggle_req = 0;
    g_isr_count = 0;

    // Starts a hardware timer interrupt every SLOT_US microseconds
    g_slotTimer.begin(slot_isr, SLOT_US);
  }

#else
  // AVR / ATtiny path only
  #include <avr/interrupt.h>

  static bool rtc_wait_sync(uint16_t max_loops = 60000)
  {
  #if defined(RTC_STATUS)
    while (RTC.STATUS && max_loops--) { }
    return (RTC.STATUS == 0);
  #else
    (void)max_loops;
    return true;
  #endif
  }

  static void rtc_init_slots()
  {
    RTC.CTRLA = 0;
    rtc_wait_sync();

  #if defined(RTC_CLKSEL_INT32K_gc)
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  #endif
    rtc_wait_sync();

    RTC.PER = (uint16_t)(TICKS_PER_SLOT - 1);
    RTC.CNT = 0;
    rtc_wait_sync();

  #if defined(RTC_OVF_bm)
    RTC.INTFLAGS = RTC_OVF_bm;
    RTC.INTCTRL  = RTC_OVF_bm;
  #endif

  #if defined(RTC_PRESCALER_DIV1_gc)
    RTC.CTRLA = RTC_RTCEN_bm | RTC_PRESCALER_DIV1_gc;
  #else
    RTC.CTRLA = RTC_RTCEN_bm;
  #endif
    rtc_wait_sync();

    sei();
  }

  ISR(RTC_CNT_vect)
  {
  #if defined(RTC_OVF_bm)
    RTC.INTFLAGS = RTC_OVF_bm;
  #endif
    g_isr_count++;

    uint8_t s = g_slot + 1;
    if (s >= TOTAL_SLOTS) s = 0;
    g_slot = s;

    if (s == A_SLOTS || s == 0) {
      g_toggle_req = 1;
    }
  }

#endif



// Return a 4-level pixel at (x,y): 0=black,1=dark,2=light,3=white.
// Replace the body with your actual AA/font scene sampling.
static inline uint8_t levelAt(uint8_t x, uint8_t y)
{
  // Example placeholder: just shows your existing demo bitmaps as 4-level:
  // (You will replace this with your PCF->AA renderer output.)
  // Interpreting bitmap1 as plane A and bitmap2 as plane B:
  uint8_t page = y >> 3;
  uint8_t bit  = 1 << (y & 7);
  uint8_t idx  = x + page * 64;

  uint8_t a = (pgm_read_byte(&bitmap1_64x32[idx]) & bit) ? 1 : 0;
  uint8_t b = (pgm_read_byte(&bitmap2_64x32[idx]) & bit) ? 1 : 0;

  // Reconstruct level from (A,B):
  // A = level&1, B = level>>1
  return (b << 1) | a;
}

static void preloadBothFrames_OnePass()
{
  for (uint8_t page = 0; page < 4; ++page) {

    // Build one SSD1306 page line (64 bytes) for each plane
    uint8_t lineA[64];
    uint8_t lineB[64];

    for (uint8_t x = 0; x < 64; ++x) {
      uint8_t ba = 0;
      uint8_t bb = 0;

      // 8 vertical pixels in this page byte
      uint8_t y0 = page << 3;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        uint8_t y = y0 + bit;

        uint8_t level = levelAt(x, y);     // 0..3

        // A bit = level&1, B bit = (level>>1)&1
        if (level & 1)       ba |= (1u << bit);
        if (level & 2)       bb |= (1u << bit);
      }

      lineA[x] = ba;
      lineB[x] = bb;
    }

    // Upload this page to Frame A
    oled.switchRenderFrame();
    oled.setCursor(0, page);
    oled.startData();
    for (uint8_t x = 0; x < 64; ++x) oled.sendData(lineB[x]);
    oled.endData();

    // Upload this page to Frame B
    oled.switchFrame();
    oled.setCursor(0, page);
    oled.startData();
    for (uint8_t x = 0; x < 64; ++x) oled.sendData(lineA[x]);
    oled.endData();
  }
}



void setup()
{
  //delay(5000);
  Wire.begin();
  Wire.setClock(I2C_HZ);

  oled.begin(64, 32, sizeof(tiny4koled_init_64x32br), tiny4koled_init_64x32br);
  oled.enableChargePump();
  oled.on();

  preloadBothFrames_OnePass();


  // Start with A displayed (same behavior as your minimal sketch)
  rtc_init_slots();
}

void loop()
{
  if (g_toggle_req)
  {
    g_toggle_req = 0;
    oled.switchDisplayFrame();
  }
}
