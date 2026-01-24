#define TINY4KOLED_WIDTH   64
#define TINY4KOLED_HEIGHT  32
//#define TINY4KOLED_MOD     r


#define PORTRAIT_90


enum Orientation {
  ORIENT_LANDSCAPE_0,
  ORIENT_LANDSCAPE_180,
  ORIENT_PORTRAIT_90,
  ORIENT_PORTRAIT_270
};

static constexpr Orientation ORIENT =
  #if defined(LANDSCAPE_0)
    ORIENT_LANDSCAPE_0
  #elif defined(LANDSCAPE_180)
    ORIENT_LANDSCAPE_180
  #elif defined(PORTRAIT_90)
    ORIENT_PORTRAIT_90
  #elif defined(PORTRAIT_270)
    ORIENT_PORTRAIT_270
  #else
    ORIENT_PORTRAIT_90  // default
  #endif
;


#define _SWITCH_DISP_INTERRUPT_FLAG g_toggle_req
#include "greyscaleOLED_pwm.h"


typedef struct pgBuf
{
  uint8_t A[TINY4KOLED_WIDTH];
  uint8_t B[TINY4KOLED_WIDTH];
} pgBuf;
static pgBuf _pgbuf;    // size of one page buffer with 2 bitpanes (smaller than full-screen buffer)

bool bOddEven = true;

void UpdateOLED(pgBuf* buf, uint8_t page)
{
    // Upload this page on an Even frame
    oled.switchRenderFrame();
    oled.setCursor(0, page);
    oled.startData();
    for (uint8_t x = 0; x < 64; ++x) oled.sendData(buf->B[x]);
    oled.endData();

    // Upload this page to Frame B
    oled.switchFrame();
    oled.setCursor(0, page);
    oled.startData();
    for (uint8_t x = 0; x < 64; ++x) oled.sendData(buf->A[x]);
    oled.endData();
}


static inline uint8_t getBitMSB(const uint8_t* bytes, uint16_t bitIndex)
{
  const uint16_t byteIndex = bitIndex >> 3;
  const uint8_t  bitInByte = bitIndex & 7;
  return (bytes[byteIndex] >> (7 - bitInByte)) & 1;
}

static inline uint8_t getPix2bpp_12h(const uint8_t* colBytes, uint8_t yInGlyph)
{
  const uint16_t b0 = (uint16_t)yInGlyph * 2;
  const uint8_t hi = getBitMSB(colBytes, b0);
  const uint8_t lo = getBitMSB(colBytes, b0 + 1);
  return (hi << 1) | lo; // 0..3
}

static int8_t findGlyphIndex(char c)
{
  const uint8_t n = (uint8_t)(sizeof(font_2b12_characters) / sizeof(font_2b12_characters[0]));
  for (uint8_t i = 0; i < n; i++)
  {
    if ((char)pgm_read_byte(&font_2b12_characters[i]) == c) return (int8_t)i;
  }
  return -1;
}

static inline void setPixel2bpp_Page(pgBuf* buf, uint8_t page, int16_t xN, int16_t yN, uint8_t v /*0..3*/)
{
  const int16_t pageY0 = (int16_t)page * 8;
  if (yN < pageY0 || yN > pageY0 + 7) return;

  // Your buffer is mirrored horizontally (from your earlier corner tests):
  const uint8_t bx  = (uint8_t)(63 - xN);
  const uint8_t bit = (uint8_t)(yN - pageY0);
  const uint8_t m   = (uint8_t)(1u << bit);

  if (v & 0x01) buf->A[bx] |= m;  // plane A
  if (v & 0x02) buf->B[bx] |= m;  // plane B
}



// Draw one glyph, rotated 90° clockwise AND flipped horizontally, into the current page buffer.
// Call as: drawGlyph2b12_onPage(buf, page, 'F', /*x=*/0, /*y=*/10);
static void drawGlyph2b12(pgBuf* buf, uint8_t page, char ch, int16_t x0, int16_t y0)
{
  constexpr uint8_t GLYPH_H = 12;
  constexpr uint8_t BYTES_PER_COL = 3; // 12px * 2bpp = 24 bits = 3 bytes

  const int8_t gi = findGlyphIndex(ch);
  if (gi < 0) return;

  const uint8_t gw = pgm_read_byte(&font_2b12_widths[gi]);
  const uint16_t base = pgm_read_word(&font_2b12_offsets_bytes[gi]);

  for (uint8_t gx = 0; gx < gw; gx++)
  {
    uint8_t col[3];
    col[0] = pgm_read_byte(&font_2b12_data[base + gx*BYTES_PER_COL + 0]);
    col[1] = pgm_read_byte(&font_2b12_data[base + gx*BYTES_PER_COL + 1]);
    col[2] = pgm_read_byte(&font_2b12_data[base + gx*BYTES_PER_COL + 2]);

    for (uint8_t gy = 0; gy < GLYPH_H; gy++)
    {
      const uint8_t v = getPix2bpp_12h(col, gy); // 0..3
      if (v == 0) continue;

      // logical pixel position (in the orientation the user expects)
      const int16_t xL = x0 + (int16_t)gx;
      const int16_t yL = y0 + (int16_t)gy;

      // map to native 64x32
      int16_t xN, yN;
      if (!mapLogicalToNative(xL, yL, xN, yN)) continue;

      setPixel2bpp_Page(buf, page, xN, yN, v);
    }
  }
}



static inline bool mapLogicalToNative(int16_t xL, int16_t yL, int16_t &xN, int16_t &yN)
{
  constexpr int16_t W = 64;
  constexpr int16_t H = 32;

  switch (ORIENT)
  {
    case ORIENT_LANDSCAPE_0:
      xN = xL;           yN = (H - 1) - yL;
      break;

    case ORIENT_LANDSCAPE_180:
      xN = xL; yN = (H - 1) - yL;
      break;

    case ORIENT_PORTRAIT_90:
      // logical space is 32(w) x 64(h): xL in [0..31], yL in [0..63]
      // map to native 64x32
      xN = (W - 1) - yL;           yN = (H - 1) - xL;
      break;

    case ORIENT_PORTRAIT_270:
      xN = yL; yN = xL;
      break;
  }

  return (uint16_t)xN < 64 && (uint16_t)yN < 32;
}

  

static void updateScreenData(pgBuf* buf)
{
  for (uint8_t page = 0; page < 4; page++) 
  {
      // TODO: put here a function that updates screen data (page-by-page)
        
      memset(buf->A,0, 64); memset(buf->B, 0, 64);  // clear entire page buffer

      // --- call it inside updateScreenData(), after memset(buf->A/B,0,64) ---
      
      //if(page==0) { buf->A[63] = buf->B[63] = 0b00000001; } // white pixel at (x,y) coordinate (0,63) = top left corner
      //if(page==3) { buf->A[63] = buf->B[63] = 0b10000000; } // white pixel at (x,y) coordinate (0,63) = top right corner
      //if(page==0) { buf->A[0] = 0b00000001; } // light-grey pixel at (x,y) coordinate (0,63) = bottom left corner
      //if(page==3) { buf->B[0] = 0b10000000; } // dark-grey pixel at (x,y) coordinate (31,63) = bottom right corner

      drawGlyph2b12(buf, page, 'F', /*x=*/0, /*y=*/0);
      
      UpdateOLED(buf,page);
  }
}


void setup()
{ 
  Wire.begin();
  Wire.setClock(I2C_HZ);
  oled.begin(TINY4KOLED_WIDTH, TINY4KOLED_HEIGHT, sizeof(TINY4KOLED_INI), TINY4KOLED_INI);
  oled.enableChargePump();
  oled.on();

  updateScreenData(&_pgbuf);
  
  rtc_init_slots();
}

void loop()
{
  if (_SWITCH_DISP_INTERRUPT_FLAG)
  {
    _SWITCH_DISP_INTERRUPT_FLAG = 0; 
    oled.switchDisplayFrame();
    if(bOddEven==true) {  /* UPDATE ANIMATED SCREEN DATA */ }
    else { /* OPTIONAL BACKGROUND ACTIVITIES*/ }
    bOddEven = !bOddEven;
  }
}
