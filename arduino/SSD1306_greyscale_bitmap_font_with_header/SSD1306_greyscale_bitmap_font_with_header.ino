

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
  ORIENT_270 = 3
};

typedef struct pgBuf
{
  uint8_t orientation; // FontOrient (0..3). Can be changed on the fly.
  uint8_t A[TINY4KOLED_WIDTH];
  uint8_t B[TINY4KOLED_WIDTH];
} pgBuf;
static pgBuf _pgbuf;    // size of one page buffer with 2 bitpanes (smaller than full-screen buffer)


typedef struct FontView {
  const uint8_t* bytes;   // PROGMEM pointer
  uint16_t       length;  // sizeof(dataset)
} FontView;


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


// ---------------- Font decoding helpers (header-based dataset) ----------------


static inline uint8_t f8(const FontView& f, uint16_t i) { return (uint8_t)pgm_read_byte(f.bytes + i); }

static inline uint16_t glyphDataOffsetBytes(const FontView& f) { return f8(f, 0); }
static inline uint8_t  fontHeightPx(const FontView& f)         { return f8(f, 1); }
static inline uint8_t  fontBpp(const FontView& f)              { return f8(f, 2); }
static inline uint8_t  fontSpaceWidthPx(const FontView& f)     { return f8(f, 3); }
static inline uint8_t  fontDelimiterByte(const FontView& f)    { return f8(f, 4); }
static inline uint8_t  fontRangeCount(const FontView& f)       { return f8(f, 5); }

static inline uint8_t  fontBytesPerColumn(const FontView& f)
{
  // (height * bpp) is guaranteed to be multiple of 8 for our use (8/12/16 px at 2bpp).
  return (uint8_t)((fontHeightPx(f) * fontBpp(f)) / 8);
}

// Ranges: records are (first,last) bytes after the 6-byte fixed header.
static int16_t findGlyphIndex_ranges(const FontView& f, uint8_t c)
{
  if (c == ' ') return -2; // "space" sentinel: no glyph data stored
  const uint8_t rc = fontRangeCount(f);
  uint16_t idxBase = 0;
  uint16_t off = 6;

  for (uint8_t r = 0; r < rc; r++, off += 2)
  {
    const uint8_t first = f8(f, off + 0);
    const uint8_t last  = f8(f, off + 1);
    if (c >= first && c <= last) return (int16_t)(idxBase + (uint16_t)(c - first));
    idxBase += (uint16_t)(last - first + 1);
  }
  return -1;
}

static inline bool isDelimiterColumnAt(const FontView& f, uint16_t byteOffset, uint8_t bytesPerCol, uint8_t delimByte)
{
  if (byteOffset + bytesPerCol > f.length) return false;
  for (uint8_t k = 0; k < bytesPerCol; k++) {
    if (f8(f, byteOffset + k) != delimByte) return false;
  }
  return true;
}

// Locate glyph i in the glyph stream by scanning delimiter columns.
// Returns false if out of bounds.
static bool getGlyphSpan(const FontView& f, uint16_t glyphIndex, uint16_t &glyphStart, uint8_t &glyphWidthCols)
{
  const uint8_t bytesPerCol = fontBytesPerColumn(f);
  const uint8_t delimByte   = fontDelimiterByte(f);
  const uint16_t startOfStream = glyphDataOffsetBytes(f);

  // Small sequential-print cache (dramatically speeds up drawing text).
  static uint16_t cacheGlyph = 0;
  static uint16_t cachePos   = 0; // byte offset at start of cacheGlyph
  static bool     cacheValid = false;

  uint16_t gi  = 0;
  uint16_t pos = startOfStream;

  if (cacheValid && glyphIndex >= cacheGlyph) {
    gi  = cacheGlyph;
    pos = cachePos;
  }

  // Advance to requested glyph
  while (gi < glyphIndex)
  {
    // scan to delimiter (end of current glyph)
    while (pos + bytesPerCol <= f.length && !isDelimiterColumnAt(f, pos, bytesPerCol, delimByte)) {
      pos += bytesPerCol;
    }
    if (pos + bytesPerCol > f.length) return false; // ran off end
    // skip delimiter
    pos += bytesPerCol;
    gi++;
  }

  // We are at start of glyphIndex
  glyphStart = pos;

  // Measure width until delimiter or end-of-stream
  uint8_t w = 0;
  while (pos + bytesPerCol <= f.length && !isDelimiterColumnAt(f, pos, bytesPerCol, delimByte)) {
    pos += bytesPerCol;
    w++;
  }
  glyphWidthCols = w;

  cacheGlyph = glyphIndex;
  cachePos   = glyphStart;
  cacheValid = true;

  return true;
}

static inline uint8_t getBitMSB_progmem(const uint8_t* base, uint16_t bitIndex)
{
  const uint16_t byteIndex = bitIndex >> 3;
  const uint8_t  bitInByte = bitIndex & 7;
  const uint8_t  b = (uint8_t)pgm_read_byte(base + byteIndex);
  return (b >> (7 - bitInByte)) & 1;
}

static inline uint8_t getPix2bpp_MSB_progmem(const uint8_t* colBytes, uint8_t yInGlyph)
{
  // 2bpp -> 2 bits per pixel, packed MSB-first (matching the JS tool).
  const uint16_t b0 = (uint16_t)yInGlyph * 2;
  const uint8_t lo = getBitMSB_progmem(colBytes, b0);
  const uint8_t hi = getBitMSB_progmem(colBytes, b0 + 1);
  return (uint8_t)((hi << 1) | lo);
}


inline void setPixel2bpp_Page(pgBuf* buf, uint8_t page, int16_t xN, int16_t yN, uint8_t v /*0..3*/);


// Draw one glyph from a header-based 2bpp font dataset into the current page buffer.
static void drawGlyph2bppFont(pgBuf* buf, uint8_t page, const FontView& font, char ch, int16_t x0, int16_t y0)
{
  const int16_t gi = findGlyphIndex_ranges(font, (uint8_t)ch);
  if (gi < 0) return; // out-of-range or space sentinel

  uint16_t glyphStart = 0;
  uint8_t  gwCols = 0;
  if (!getGlyphSpan(font, (uint16_t)gi, glyphStart, gwCols)) return;

  const uint8_t h  = fontHeightPx(font);
  const uint8_t bpc = fontBytesPerColumn(font);

  for (uint8_t gx = 0; gx < gwCols; gx++)
  {
    const uint16_t colOff = glyphStart + (uint16_t)gx * bpc;
    const uint8_t* colPtr = font.bytes + colOff; // PROGMEM pointer

    for (uint8_t gy = 0; gy < h; gy++)
    {
      const uint8_t v = getPix2bpp_MSB_progmem(colPtr, gy);
      if (v == 0) continue;

      const int16_t xL = x0 + (int16_t)gx;
      const int16_t yL = y0 + (int16_t)gy;

      setPixel2bpp(buf, page, xL, yL, v);
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

      const FontView f12 = { font_2b12, (uint16_t)sizeof(font_2b12) };
      drawGlyph2bppFont(buf, page, f12, 'F', /*x=*/0, /*y=*/0);
      drawGlyph2bppFont(buf, page, f12, 'r', /*x=*/5, /*y=*/0);
      drawGlyph2bppFont(buf, page, f12, 'e', /*x=*/9, /*y=*/0);
      drawGlyph2bppFont(buf, page, f12, 'd', /*x=*/14, /*y=*/0);
      
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

  _pgbuf.orientation = ORIENT_90;
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
