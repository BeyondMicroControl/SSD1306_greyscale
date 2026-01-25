

#define TINY4KOLED_WIDTH   64
#define TINY4KOLED_HEIGHT  32
//#define TINY4KOLED_MOD     r

#define _SWITCH_DISP_INTERRUPT_FLAG g_toggle_req
#include "greyscaleOLED_pwm.h"

// Runtime-selectable orientation (0, 90, 180, 270 degrees).
// NOTE: Logical coordinate space depends on orientation:
//   - ORIENT_0 / ORIENT_180  : logical is 64(w) x 32(h)
//   - ORIENT_90 / ORIENT_270 : logical is 32(w) x 64(h)
// This maps into the SSD1306 native buffer space (64x32).
enum FontOrient : uint8_t {
  ORIENT_0   = 0,
  ORIENT_90  = 1,
  ORIENT_180 = 2,
  ORIENT_270 = 3,
};

typedef struct pgBuf
{
  uint8_t orientation; // FontOrient (0..3). Can be changed on the fly.
  uint8_t A[TINY4KOLED_WIDTH];
  uint8_t B[TINY4KOLED_WIDTH];
} pgBuf;
static pgBuf _pgbuf;    // size of one page buffer with 2 bitpanes (smaller than full-screen buffer)

bool bOddEven = true;

// Map logical coordinates (xL,yL) to native SSD1306 buffer coordinates (xN,yN).
// Returns false if outside native bounds.
static inline bool mapLogicalToNative(uint8_t orient, int16_t xL, int16_t yL, int16_t &xN, int16_t &yN)
{
  constexpr int16_t W = 64;
  constexpr int16_t H = 32;

  switch ((FontOrient)orient)
  {
    case ORIENT_0:
      // logical 64x32
      xN = xL;
      yN = (H - 1) - yL;
      break;

    case ORIENT_180:
      // True 180° rotation vs ORIENT_0, while keeping the SSD1306 Y inversion behavior consistent.
      // ORIENT_0 mapping already flips Y (yN = H-1-yL). Applying 180° first (x'=W-1-xL, y'=H-1-yL)
      // then ORIENT_0 mapping yields: xN = W-1-xL, yN = yL.
      xN = (W - 1) - xL;
      yN = yL;
      break;

    case ORIENT_90:
      // logical 32x64
      xN = (W - 1) - yL;
      yN = (H - 1) - xL;
      break;

    case ORIENT_270:
    default:
      // logical 32x64
      xN = yL;
      yN = xL;
      break;
  }

  return (uint16_t)xN < 64 && (uint16_t)yN < 32;
}

static inline void getLogicalDims(uint8_t orient, int16_t &wL, int16_t &hL)
{
  switch ((FontOrient)orient)
  {
    case ORIENT_0:
    case ORIENT_180:
      wL = 64; hL = 32;
      break;
    case ORIENT_90:
    case ORIENT_270:
    default:
      wL = 32; hL = 64;
      break;
  }
}

// Set a 2bpp pixel into one page buffer using LOGICAL coordinates.
// v: 0=black, 1=dark-grey, 2=light-grey, 3=white
static inline void setPixel2bpp(pgBuf* buf, uint8_t page, int16_t xL, int16_t yL, uint8_t v)
{
  if (!buf || v == 0) return;

  int16_t xN, yN;
  if (!mapLogicalToNative(buf->orientation, xL, yL, xN, yN)) return;

  const int16_t pageY0 = (int16_t)page * 8;
  if (yN < pageY0 || yN > pageY0 + 7) return;

  // Your buffer is mirrored horizontally (from your earlier corner tests):
  const uint8_t bx  = (uint8_t)(63 - xN);
  const uint8_t bit = (uint8_t)(yN - pageY0);
  const uint8_t m   = (uint8_t)(1u << bit);

  if (v & 0x01) buf->A[bx] |= m;  // plane A
  if (v & 0x02) buf->B[bx] |= m;  // plane B
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
      if (!mapLogicalToNative(buf->orientation, xL, yL, xN, yN)) continue;

      setPixel2bpp_Page(buf, page, xN, yN, v);
    }
  }
}

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

static void updateScreenData(pgBuf* buf)
{
  for (uint8_t page = 0; page < 4; page++) 
  {
      // TODO: put here a function that updates screen data (page-by-page)
        
      memset(buf->A,0, 64); memset(buf->B, 0, 64);  // clear entire page buffer

      // Example test pattern: 4 corners in the *current* logical coordinate space.
      // (setPixel2bpp will only affect the current page if the pixel's y falls inside it.)
      int16_t wL, hL;
      getLogicalDims(buf->orientation, wL, hL);
      //setPixel2bpp(buf, page, 0,      0,      3); // white: top-left
      //setPixel2bpp(buf, page, wL - 1, 0,      3); // white: top-right
      //setPixel2bpp(buf, page, 0,      hL - 1, 1); // dark-grey: bottom-left
      //setPixel2bpp(buf, page, wL - 1, hL - 1, 2); // light-grey: bottom-right

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

  // Default orientation (change anytime, then re-render the screen).
  _pgbuf.orientation = ORIENT_90; // your current use case

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
