/*
GND ━━━━━━━━━━━━━━━━━━━━━━━━━━━┯━━━━━━┯━━━━━━━━━━━━━━━━━━━━
                               │      │
+5V ─┬─────────────────────────│──────│───┬────────────────
     │                         │      │   │
     │                         │      │   │
     │  ╔═══════════════════╗  │      │   │
     │  ║ ⬤  [ATTinyX24]   ║  │      │   │
     │  ║                   ║  │      │   │   ┌───────────┐
     └──╢ VCC           GND ╟──┘      │   │   │   ┌─────┐ │
        ║                   ║     ╔═══╧═══╧═══╧═══╧═══╗ │ │
        ╢ PA4       SCK PA3 ╟     ║  GND VCC SCL SDA  ║ │ │
        ║                   ║     ║┌─────────────────┐║ │ │
        ╢ PA5  RX1 MISO PA2 ╟     ║│                 │║ │ │
        ║                   ║     ║│    [SSD1306]    │║ │ │
        ╢ PA6  TX1 MOSI PA1 ╟     ║│  64 x 32 OLED   │║ │ │
        ║                   ║     ║│                 │║ │ │
        ╢ PA7      UPDI PA0 ╟     ║└─────────────────┘║ │ │
        ║                   ║     ╚═══════════════════╝ │ │
        ╢ PB3 RX0   SCL PB0 ╟───┐                       │ │
        ║                   ║   │                       │ │
        ╢ PB2 TX0   SDA PB1 ╟───│───────────────────────┘ │
        ╚═══════════════════╝   └─────────────────────────┘
*/

#include <Arduino.h>
#include <Wire.h>
#include <Tiny4kOLED.h>
#include "BitmapData.h"

// ---------------- I2C speed ----------------
#ifndef I2C_HZ
  #define I2C_HZ 1000000UL
#endif

// ---------------- PWM config (slot-based) ----------------
// A is shown for A_SLOTS slots, B for the remaining slots.
// Example TOTAL_SLOTS=4, A_SLOTS=3 => 75% / 25% duty, 4 perceived levels.
static constexpr uint16_t PWM_HZ      = 50;
static constexpr uint8_t  TOTAL_SLOTS = 4;
static constexpr uint8_t  A_SLOTS     = 3;
static_assert(A_SLOTS > 0 && A_SLOTS < TOTAL_SLOTS, "A_SLOTS must be 1..TOTAL_SLOTS-1");

static constexpr uint16_t SLOT_HZ     = PWM_HZ * TOTAL_SLOTS;

// RTC runs from 32.768 kHz clock in INT32K mode on ATtiny3224
static constexpr uint16_t RTC_HZ      = 32768;
static constexpr uint16_t TICKS_PER_SLOT = (RTC_HZ + (SLOT_HZ/2)) / SLOT_HZ; // rounded
static_assert(TICKS_PER_SLOT >= 2, "TICKS_PER_SLOT too small; lower SLOT_HZ or increase prescaler");

// ---------------- ISR <-> loop flags ----------------
volatile uint8_t  g_slot       = 0;
volatile uint8_t  g_toggle_req = 0;
volatile uint16_t g_isr_count  = 0;

// Small bounded wait for RTC sync; returns true if synced before timeout.
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
  // Disable RTC while configuring
  RTC.CTRLA = 0;
  rtc_wait_sync();

  // Select internal 32k clock for RTC
#if defined(RTC_CLKSEL_INT32K_gc)
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
#endif
  rtc_wait_sync();

  // Set period to slot length (overflow when CNT reaches PER)
  RTC.PER = (uint16_t)(TICKS_PER_SLOT - 1);
  RTC.CNT = 0;
  rtc_wait_sync();

  // Clear pending flags and enable overflow interrupt
#if defined(RTC_OVF_bm)
  RTC.INTFLAGS = RTC_OVF_bm;
  RTC.INTCTRL  = RTC_OVF_bm;
#endif

  // Enable RTC, prescaler DIV1
#if defined(RTC_PRESCALER_DIV1_gc)
  RTC.CTRLA = RTC_RTCEN_bm | RTC_PRESCALER_DIV1_gc;
#else
  RTC.CTRLA = RTC_RTCEN_bm;
#endif
  rtc_wait_sync();

  // Ensure global interrupts enabled
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

  // Toggle at boundaries:
  // - entering slot A_SLOTS: A -> B
  // - entering slot 0: B -> A (new PWM period)
  if (s == A_SLOTS || s == 0) {
    g_toggle_req = 1;
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

  // Preload frames
  oled.switchRenderFrame();
  oled.bitmap(0, 0, 64, 4, bitmap1_64x32);
  oled.switchFrame();
  oled.bitmap(0, 0, 64, 4, bitmap2_64x32);

  // Start with A displayed (same behavior as your minimal sketch)
  rtc_init_slots();
}

void loop()
{
  if (g_toggle_req) {
    g_toggle_req = 0;
    oled.switchDisplayFrame();
  }
}
