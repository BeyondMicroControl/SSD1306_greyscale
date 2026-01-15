#include <Arduino.h>
#include <Wire.h>
#include <Tiny4kOLED.h>
#include "BitmapData.h"

// ---------------- I2C speed ----------------
#ifndef I2C_HZ
  #define I2C_HZ 1000000UL
#endif

// ---------------- Button (active-low) ----------------

// ---------------- PWM config (slot-based) ----------------
// A is shown for A_SLOTS slots, B for remaining slots.
// Example TOTAL_SLOTS=4, A_SLOTS=3 => 75% / 25% duty.
static constexpr uint8_t TOTAL_SLOTS = 4;
static constexpr uint8_t A_SLOTS     = 3;
static_assert(A_SLOTS > 0 && A_SLOTS < TOTAL_SLOTS, "A_SLOTS must be 1..TOTAL_SLOTS-1");

// RTC runs from ~32.768 kHz internal clock in INT32K mode on ATtiny3224
static constexpr uint32_t RTC_HZ = 32768UL;

// Start at 25 Hz (your request)
static uint16_t g_pwm_hz = 149;

volatile uint8_t  g_btn_req = 0;
volatile uint32_t g_btn_last_tick = 0;


// ---------------- ISR <-> loop flags ----------------
volatile uint8_t  g_slot       = 0;
volatile uint8_t  g_toggle_req = 0;
volatile uint32_t g_isr_count  = 0;

// Track which frame we believe is currently displayed (A or B)
static bool g_showingA = true;

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

// Compute ticks per slot for a given PWM frequency (rounded).
static uint16_t ticks_per_slot_for(uint16_t pwm_hz)
{
  uint32_t slot_hz = (uint32_t)pwm_hz * (uint32_t)TOTAL_SLOTS;
  if (slot_hz == 0) return 0;

  // Rounded division: (RTC_HZ / slot_hz)
  uint32_t tps = (RTC_HZ + (slot_hz / 2)) / slot_hz;
  if (tps > 0xFFFF) tps = 0xFFFF;
  return (uint16_t)tps;
}

// Reconfigure RTC to a new PWM frequency.
// IMPORTANT: no I2C/Wire calls in here; only RTC registers.
static void rtc_config_pwm(uint16_t pwm_hz)
{
  uint16_t tps = ticks_per_slot_for(pwm_hz);
  if (tps < 2) {
    // Too fast for this slot scheme; ignore
    return;
  }

  noInterrupts();

  // Stop RTC while configuring
  RTC.CTRLA = 0;
  rtc_wait_sync();

#if defined(RTC_CLKSEL_INT32K_gc)
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
#endif
  rtc_wait_sync();

  // Overflow when CNT reaches PER
  RTC.PER = (uint16_t)(tps - 1);
  RTC.CNT = 0;
  rtc_wait_sync();

  // Reset phase
  g_slot = 0;
  g_toggle_req = 0;

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

  interrupts();
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
  // - entering slot 0:       B -> A (new PWM period)
  if (s == A_SLOTS || s == 0) {
    g_toggle_req = 1;
  }
}

void ISR_button_falling()
{
  // Debounce lockout in RTC slot ticks (~30ms target)
  // slot_hz = PWM_HZ * TOTAL_SLOTS
  // ticks_for_30ms ≈ slot_hz * 0.03
  uint32_t now = g_isr_count;  // increments in RTC ISR
  uint32_t slot_hz = (uint32_t)g_pwm_hz * (uint32_t)TOTAL_SLOTS;

  uint32_t lockout = (slot_hz * 30UL + 999UL) / 1000UL; // ~30ms, rounded up
  if (lockout < 2) lockout = 2;

  if ((now - g_btn_last_tick) >= lockout) 
  {
    g_btn_last_tick = now;


     g_pwm_hz++;

    oled.setFont(FONT6X8P);
    oled.setCursor(0,0);
    oled.print(g_pwm_hz);
    oled.print(" ");
    oled.switchDisplayFrame();
    oled.setCursor(0,0);
    oled.print(g_pwm_hz);
    oled.print(" ");




 
    if (g_pwm_hz > 200) g_pwm_hz = 25;

    rtc_config_pwm(g_pwm_hz);

    // Optional: re-phase so each new PWM starts at A
    if (!g_showingA) {
      oled.switchDisplayFrame();
      g_showingA = true;
    }


  }
}



void setup()
{

  pinMode(PIN_PA2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PA2), ISR_button_falling, FALLING);

  Wire.begin();
  Wire.setClock(I2C_HZ);

  oled.begin(64, 32, sizeof(tiny4koled_init_64x32r), tiny4koled_init_64x32r);
  oled.on();

  // Preload frames
  oled.switchRenderFrame();
  oled.bitmap(0, 0, 64, 4, bitmap_64x32_A);
  oled.switchFrame();
  oled.bitmap(0, 0, 64, 4, bitmap_64x32_B);

  // Assume we start with A displayed
  g_showingA = true;

  // Start PWM at 25 Hz
  rtc_config_pwm(g_pwm_hz);
}

void loop()
{
  // Scheduled OLED frame toggle (from RTC slots)
  if (g_toggle_req) {
    g_toggle_req = 0;
    oled.switchDisplayFrame();
    g_showingA = !g_showingA;
  }


}
