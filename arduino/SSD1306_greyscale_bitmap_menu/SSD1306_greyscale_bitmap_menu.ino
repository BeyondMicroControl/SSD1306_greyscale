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

#define TINY4KOLED_WIDTH   64
#define TINY4KOLED_HEIGHT  32
#define TINY4KOLED_MOD   r

#define SWITCH_1_PIN A8
#define SWITCH_2_PIN A9

#define _SWITCH_DISP_INTERRUPT_FLAG g_toggle_req
#include "greyscaleOLED_pwm.h"



typedef struct pgBuf
{
  uint8_t A[64];
  uint8_t B[64];
} pgBuf;
static pgBuf _pgbuf;    // page buffer (smaller than full-screen buffer)

typedef struct AnimD
{
  uint8_t frame_cnt  = 0;
  bool flash = 0;
} AnimD;
static AnimD _anim;     // animation data

typedef struct MenuD
{
  uint8_t Ypos;
  uint8_t Yh;
  uint8_t Yo;
  
  uint8_t id;
  uint8_t item_idx;
  bool bUpdateScreen;
  bool bShow;
  uint8_t* srcA[4];
  uint8_t* srcB[4];
} MenuD;
static MenuD _menu;     // menu data
//static MenuD _bck_menu;
  


bool bOddEven = true;
static uint16_t btnStatus  = 0;
static constexpr uint8_t W = 64;
static constexpr uint8_t H = 32;
static constexpr uint16_t SIZE = W * (H/8);
//uint8_t fb[SIZE];


static void loadLineBuf(uint8_t* srcA, uint8_t* srcB, pgBuf* buf, uint8_t page)
{
  for (uint8_t x = 0; x < 64; ++x)
  {
    uint8_t ba = 0, bb = 0;
    uint8_t y0 = page << 3;      // 8 vertical pixels in this page byte
    for (uint8_t bit = 0; bit < 8; ++bit) 
    {
      uint8_t y = y0 + bit;

      // calulate level
      uint8_t bt  = 1 << (y & 7);
      uint8_t idx  = x +  (y >> 3) * 64;
      uint8_t a = (pgm_read_byte(&srcA[idx]) & bt) ? 1 : 0;
      uint8_t b = (pgm_read_byte(&srcB[idx]) & bt) ? 1 : 0;


      // 4-level pixel at (x,y): 0=black,1=dark,2=light,3=white.
      uint8_t level = (b << 1) | a;   // Reconstruct level from (A,B):  A = level&1, B = level>>1
      if (level & 1)       ba |= (1u << bit);    // A bit = level&1, B bit = (level>>1)&1
      if (level & 2)       bb |= (1u << bit);
    }
    buf->A[x] = ba;
    buf->B[x] = bb;
  }
}


void fillBits(uint8_t* buf, uint8_t bit0, uint8_t bit1)
{
    if (bit1 < bit0) return;

    uint8_t byte0 = bit0 >> 3;
    uint8_t byte1 = bit1 >> 3;

    uint8_t mask0 = 0xFF << (bit0 & 7);
    uint8_t mask1 = 0xFF >> (7 - (bit1 & 7));

    if (byte0 == byte1) buf[byte0] |= (mask0 & mask1);
    else
    {
        buf[byte0] |= mask0;
        for (uint8_t i = byte0 + 1; i < byte1; i++) buf[i] = 0xFF;
        buf[byte1] |= mask1;
    }
}

uint8_t findAdr( uint8_t coord)
{
  return coord>>3;
}

void RoundArea(uint8_t* maskA, uint8_t* maskB, uint8_t page, uint8_t x0, uint8_t x1, uint8_t yd)
{
  uint8_t pageAdr_x0 = findAdr(x0);
  uint8_t pageAdr_x1 = findAdr(x1);
  
  //uint8_t cornerDim[2] = {2,2};   // later extention - craft corners of different size
  uint8_t corner[8] = {0b10000000,0b01000000,0b00000000,0b10000000, 0b01,0b10,0b00,0b01};
  memset(maskA, 0, yd);
  memset(maskB, 0, yd);

  if(page==pageAdr_x0)
  {
    maskA[0]    = corner[4];  // top left corner
    maskB[0]    = corner[5];
    maskA[1]    = corner[6];
    maskB[1]    = corner[7];
  
    maskA[yd-1] = corner[6];  // bottom left corner
    maskB[yd-1] = corner[7];
    maskA[yd]    = corner[4];
    maskB[yd]    = corner[5];
  }
  
  if(page==pageAdr_x1)
  {
    maskA[0]    = corner[0]; // top right corner
    maskB[0]    = corner[1];
    maskA[1]    = corner[2];
    maskB[1]    = corner[3];
    
    maskA[yd-1] = corner[2]; // bottom right corner
    maskB[yd-1] = corner[3];
    maskA[yd]    = corner[0];
    maskB[yd]    = corner[1];    
  }
}

static void InvertArea(pgBuf* buf, uint8_t page, uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1)
{
  uint8_t pageAdr_x0 = findAdr(x0);
  uint8_t pageAdr_x1 = findAdr(x1);
  
  uint8_t mask_bufA[y1-y0+1],mask_bufB[y1-y0+1]; // TODO: declare a 2-bit layer struct ?
  RoundArea(mask_bufA,mask_bufB, page  ,x0,x1,y1-y0);


  uint8_t mask[4]; memset(mask, 0, 4);  fillBits(mask,x0,x1);


  for (uint8_t y = y0; y<=y1 ; y++)
  { 
    uint8_t yi = TINY4KOLED_WIDTH - y;
    uint8_t dy = y-y0;

    buf->A[yi] ^= mask[page];   // invert bitplane A
    buf->B[yi] ^= mask[page];   // invert bitplane B
    
    if(page==pageAdr_x0 || page==pageAdr_x1)  // left corner roundings    
    {
        buf->A[yi] ^= mask_bufA[dy]; // round bitplane A
        buf->B[yi] ^= mask_bufB[dy]; // round bitplane B
    }
  }
}


void UpdateOLED(pgBuf* buf, uint8_t page)
{
    // Upload this page to Frame A
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

static void updateAnim(pgBuf* buf, AnimD* anim)
{
  if(anim->frame_cnt == 0)
  {
    updateScreen(buf,anim);
    anim->frame_cnt = 8;  // TODO parametrise animation frequency
    anim->flash = !anim->flash; // flicker behavior
  }
  anim->frame_cnt--;
}

static void updateScreen(pgBuf* buf, AnimD* anim)
{
  for (uint8_t page = 0; page < 4; page++) 
  {
      loadLineBuf(_menu.srcA[_menu.id],_menu.srcB[_menu.id],buf,page);
      if(_menu.bShow) InvertArea(buf,page,0,31,_menu.Ypos,_menu.Ypos+_menu.Yh);
      UpdateOLED(buf,page);
  }
}

static uint8_t calc_Ypos(MenuD* menu)
{
  return menu->Yo + menu->item_idx * menu->Yh;
}

static void go2_menu(MenuD* menu, uint8_t id )
{
  switch(id)
  {
    case 1:
        menu->id = 1;
        menu->item_idx = 1;
        menu->Yh = 12;
        menu->Yo = 15;
        menu->bShow = true;
        menu->bUpdateScreen = true;
    break;
    case 2:
        menu->id = 2;
        menu->item_idx = 1;
        menu->Yo = 1;
        menu->Yh = 21;
        menu->bShow = true;
        menu->bUpdateScreen = true;
    break;
    case 3:
        menu->id = 3;
        menu->item_idx = 1;
        menu->Yo = 14;
        menu->Yh = 10;
        menu->bShow = true;
        menu->bUpdateScreen = true;
    break;
    default:
        menu->id = 0;
        menu->item_idx = 0;
        menu->Yh = 10;
        menu->Yo = 44;
        menu->bShow = true;
        menu->bUpdateScreen = true;
    break;
  }
  menu->Ypos = calc_Ypos(menu);
}

static void update_menu(MenuD* menu, uint8_t cur, uint8_t chg, uint8_t dur)
{ 
  switch(menu->id)
  {

//////////  HOME ///////////
          
    case 0:    
     if(dur<60 && cur==0)  // less than 60 loops detected in previous state (short push) & keyup event
      {
        menu->bUpdateScreen = true;
        if (chg & 0b01)  menu->item_idx = menu->item_idx==0 ? 0 : menu->item_idx-1;  // button 1 change detected > move caret up
        if (chg & 0b10)  menu->item_idx = menu->item_idx==1 ? 1 : menu->item_idx+1;  // button 2 change detected > move caret down
        menu->Ypos = calc_Ypos(menu);
      }
      else if(dur>60 && cur==0)
      { 
        switch(menu->item_idx)
        {
          case 0: go2_menu(menu,0); break;
          case 1: go2_menu(menu,1); break;
        }
      }
    break;

//////////  INPUT SETUP MENU ///////////
    
    case 1:
      if(dur<60 && cur==0)  // less than 60 loops detected in previous state (short push) & keyup event
      {
        menu->bUpdateScreen = true;
        if ((chg & 0b01) && menu->item_idx==0)  { update_menu(menu, cur, chg, 0xFF); return; }  // going past the first item -> go to parent menu
        if (chg & 0b01)  menu->item_idx = menu->item_idx==0 ? 0 : menu->item_idx-1;  // button 1 change detected > move caret up
        if (chg & 0b10)  menu->item_idx = menu->item_idx==3 ? 3 : menu->item_idx+1;  // button 2 change detected > move caret down

        if(menu->item_idx==0) // special menu item
        {
          //memcpy(&_bck_menu,&_menu,sizeof(_menu));
          menu->Ypos = 1;
          menu->Yh = 20;
        }
        else
        {
          //go2_menu(menu,1);  // can we fix override menu->item_idx ?
          menu->Yh = 12;
          menu->Ypos = calc_Ypos(menu);
        }
      }
      else if(dur>60 && cur==0)
      { 
        switch(menu->item_idx)
        {
          case 1:  go2_menu(menu,2);  break;
          case 2:  go2_menu(menu,3);  break;
          default: go2_menu(menu,0);
        }
      }
    break;

//////////  USB SETUP MENU ///////////
    
    case 2:
      if(dur<60 && cur==0)  // less than 60 loops detected in previous state (short push) & keyup event
      {
        menu->bUpdateScreen = true;
        if ((chg & 0b01) && menu->item_idx==0)  { update_menu(menu, cur, chg, 0xFF); return; }  // going past the first item -> go to parent menu
        if (chg & 0b01)  menu->item_idx = menu->item_idx==0 ? 0 : menu->item_idx-1;  // button 1 change detected > move caret up
        if (chg & 0b10)  menu->item_idx = menu->item_idx==2 ? 2 : menu->item_idx+1;  // button 2 change detected > move caret down
        menu->Ypos = calc_Ypos(menu);
      }
      else if(dur>60 && cur==0)
      { 
        switch(menu->item_idx)
        {
          case 0:  go2_menu(menu,1); break;
          default: go2_menu(menu,0);
        }
      }
    break;

//////////  AUDIO SETUP MENU ///////////

    case 3:
      if(dur<60 && cur==0)  // less than 60 loops detected in previous state (short push) & keyup event
      {
        menu->bUpdateScreen = true;
        if ((chg & 0b01) && menu->item_idx==0)  { update_menu(menu, cur, chg, 0xFF); return; }  // going past the first item -> go to parent menu
        if (chg & 0b01)  menu->item_idx = menu->item_idx==0 ? 0 : menu->item_idx-1;  // button 1 change detected > move caret up
        if (chg & 0b10)  menu->item_idx = menu->item_idx==4 ? 4 : menu->item_idx+1;  // button 2 change detected > move caret down
        
        if(menu->item_idx==0) // special menu item
        {
          //memcpy(&_bck_menu,&_menu,sizeof(_menu));
          menu->Ypos = 1;
          menu->Yh = 20;
        }
        else
        {
          //go2_menu(menu,3);
          menu->Yh = 10;
          menu->Ypos = calc_Ypos(menu);
        }
      }
      else if(dur>60 && cur==0)
      { 
        switch(menu->item_idx)
        {
          case 0:  go2_menu(menu,1); break;
          default: go2_menu(menu,0);
        }
      }
    break;

    
    
  }
}

void setup()
{
  Serial.begin(115200);
  #ifdef BUTTON_INPUT
    pinMode(SWITCH_1_PIN,INPUT_PULLUP);
    pinMode(SWITCH_2_PIN,INPUT_PULLUP);
  #endif
  
  //delay(5000);
  Wire.begin();
  Wire.setClock(I2C_HZ);

  //oled.begin(64, 32, sizeof(tiny4koled_init_64x32br), tiny4koled_init_64x32br);
  oled.begin(TINY4KOLED_WIDTH, TINY4KOLED_HEIGHT, sizeof(TINY4KOLED_INI), TINY4KOLED_INI);
  oled.enableChargePump();
  oled.on();

  _menu.srcA[0] = (uint8_t*)HOME1_64x32;
  _menu.srcB[0] = (uint8_t*)HOME2_64x32;
  _menu.srcA[1] = (uint8_t*)INPUT1_64x32;
  _menu.srcB[1] = (uint8_t*)INPUT2_64x32;
  _menu.srcA[2] = (uint8_t*)USB1_64x32;
  _menu.srcB[2] = (uint8_t*)USB2_64x32;  
  _menu.srcA[3] = (uint8_t*)AUDIO1_64x32;
  _menu.srcB[3] = (uint8_t*)AUDIO2_64x32; 
  go2_menu(&_menu,0);
  //_anim.flash = true;
  //updateAnim(&_pgbuf,&_anim);
  updateScreen(&_pgbuf,&_anim);

  // Start with A displayed (same behavior as your minimal sketch)
  rtc_init_slots();
}

uint8_t read_buttons(uint16_t* status)
{
    uint8_t new_status =  (digitalRead(SWITCH_1_PIN) == LOW ? 1 : 0) |   // bitmap:  [status changed] [PIN_PA2] [PIN_PA3)]
                          (digitalRead(SWITCH_2_PIN) == LOW ? 2 : 0);    
    uint8_t cnt = (*status >> 8)==0xFF ? 0xFF: ((*status >> 8)+1);    // increase press length counter until max reached
    *status =  (cnt<<8) | ((*status & 0xFF) << 4) | new_status;       // shift old status aside && increase counter
    if((*status & 0x0F) != new_status) cnt = 0;  // reset counter after change 
    return (new_status << 4) | new_status;       // prepare status quo
}

bool isLongPush(uint16_t* status)
{
  return (btnStatus>>8) > 100 ? true : false;
}

void loop()
{
  if (_SWITCH_DISP_INTERRUPT_FLAG)
  {
    _SWITCH_DISP_INTERRUPT_FLAG = 0;
    oled.switchDisplayFrame();
    if(bOddEven==true)
    {
      if(_menu.bUpdateScreen == true)
      { 
        updateScreen(&_pgbuf,&_anim);
        //updateAnim(&_pgbuf,&_anim); 
        _menu.bUpdateScreen = false;
      }
    }
    else
    {
      uint8_t btn = read_buttons(&btnStatus);
      
      uint8_t curr    = btn & 0x0F;                        // current state
      uint8_t changed = (curr ^ (btnStatus >> 4)) & 0x0F;  // delta between current and previous state
      
      if (changed != 0)
      { 
        update_menu(&_menu, curr,changed,btnStatus>>8);   // menu handler
        btnStatus = curr;                                 // reset change status
      }
      else if(isLongPush(&btnStatus) && curr != 0)       // long push --> don't wait until change/keyup event!
      {
        update_menu(&_menu, 0,changed,btnStatus>>8);           // menu handler
        btnStatus = curr;                                      // reset change status
      }
    }
    bOddEven = !bOddEven;
  }
}
