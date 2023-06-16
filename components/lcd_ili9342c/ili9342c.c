#include "ili9342c.h"
#include "../../build/config/sdkconfig.h"
//-------------------------------------------------------------------
typedef struct
{
  uint16_t TextColor;
  uint16_t BackColor;
  sFONT *pFont;
}LCD_DrawPropTypeDef;
LCD_DrawPropTypeDef lcdprop;
//-------------------------------------------------------------------
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//-------------------------------------------------------------------
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//-------------------------------------------------------------------
void TFT9341_reset(void)
{
    gpio_set_level(CONFIG_PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(CONFIG_PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
//-------------------------------------------------------------------
static void send_hor_blocks(spi_device_handle_t spi, int ypos, uint16_t *data)
{
  esp_err_t ret;
  int x;
  static spi_transaction_t trans[6];
  for (x=0; x<6; x++) {
    memset(&trans[x], 0, sizeof(spi_transaction_t));
    if ((x&1)==0) {
        //Even transfers are commands
        trans[x].length=8;
        trans[x].user=(void*)0;
    } else {
        //Odd transfers are data
        trans[x].length=8*4;
        trans[x].user=(void*)1;
    }
    trans[x].flags=SPI_TRANS_USE_TXDATA;
  }
  trans[0].tx_data[0]=0x2A;           //Column Address Set
  trans[1].tx_data[0]=0;              //Start Col High
  trans[1].tx_data[1]=0;              //Start Col Low
  trans[1].tx_data[2]=(TFT9341_WIDTH)>>8;       //End Col High
  trans[1].tx_data[3]=(TFT9341_WIDTH)&0xff;     //End Col Low
  trans[2].tx_data[0]=0x2B;           //Page address set
  trans[3].tx_data[0]=ypos>>8;        //Start page high
  trans[3].tx_data[1]=ypos&0xff;      //start page low
  trans[3].tx_data[2]=(ypos+16)>>8;    //end page high
  trans[3].tx_data[3]=(ypos+16)&0xff;  //end page low
  trans[4].tx_data[0]=0x2C;           //memory write
  trans[5].tx_buffer=data;        //finally send the line data
  trans[5].length=TFT9341_WIDTH*2*8*16;          //Data length, in bits
  trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag
  for (x=0; x<6; x++) {
      ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
      assert(ret==ESP_OK);
  }
}
//-------------------------------------------------------------------
static void send_blocks(spi_device_handle_t spi, int x1, int y1,
    int x2, int y2, uint16_t *data)
{
  esp_err_t ret;
  int x;
  static spi_transaction_t trans[6];
  for (x=0; x<6; x++) {
    memset(&trans[x], 0, sizeof(spi_transaction_t));
    if ((x&1)==0) {
        //Even transfers are commands
        trans[x].length=8;
        trans[x].user=(void*)0;
    } else {
        //Odd transfers are data
        trans[x].length=8*4;
        trans[x].user=(void*)1;
    }
    trans[x].flags=SPI_TRANS_USE_TXDATA;
  }
  trans[0].tx_data[0]=0x2A;            //Column Address Set
  trans[1].tx_data[0]=(x1 >> 8) & 0xFF;//Start Col High
  trans[1].tx_data[1]=x1 & 0xFF;       //Start Col Low
  trans[1].tx_data[2]=(x2 >> 8) & 0xFF;//End Col High
  trans[1].tx_data[3]=x2 & 0xFF;       //End Col Low
  trans[2].tx_data[0]=0x2B;            //Page address set
  trans[3].tx_data[0]=(y1 >> 8) & 0xFF;//Start page high
  trans[3].tx_data[1]=y1 & 0xFF;       //start page low
  trans[3].tx_data[2]=(y2 >> 8) & 0xFF;//end page high
  trans[3].tx_data[3]=y2 & 0xFF;       //end page low
  trans[4].tx_data[0]=0x2C;           //memory write
  trans[5].tx_buffer=data;            //finally send the line data
  trans[5].length=(x2-x1+1)*(y2-y1+1)*2*8;//Data length, in bits
  trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag
  for (x=0; x<6; x++) {
      ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
      assert(ret==ESP_OK);
  }
}
//-------------------------------------------------------------------
static void send_block_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}
//-------------------------------------------------------------------
void TFT9341_FillScreen(spi_device_handle_t spi, uint16_t color)
{
  uint16_t *blck;
  blck=heap_caps_malloc(TFT9341_WIDTH*16*sizeof(uint16_t), MALLOC_CAP_DMA);
  //swap bytes;
   color = color<<8 | color>>8;
   for(int y=0; y<16; y++)
   {
     for(int x=0; x<TFT9341_WIDTH; x++)
     {
       blck[y*TFT9341_WIDTH+x] = color;
     }
   }
   for (int i=0; i<TFT9341_HEIGHT/16; i++) {
     send_hor_blocks(spi, i*16, blck);
     send_block_finish(spi);
   }
   heap_caps_free(blck);
}
//-------------------------------------------------------------------
/*void TFT9341_FillRect(spi_device_handle_t spi, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  if((x1 >= TFT9341_WIDTH) || (y1 >= TFT9341_HEIGHT) || (x2 >= TFT9341_WIDTH) || (y2 >= TFT9341_HEIGHT)) return;
  if(x1>x2) swap(x1,x2);
  if(y1>y2) swap(y1,y2);
  uint16_t xsize = (x2-x1+1), ysize = (y2-y1+1);
  uint16_t *blck;
  uint16_t ysize_block;
  uint32_t size_max = TFT9341_WIDTH*16;
  blck=heap_caps_malloc(size_max*sizeof(uint16_t), MALLOC_CAP_DMA);
  //swap bytes;
  color = color<<8 | color>>8;
  uint32_t size = xsize*ysize;
  uint32_t size_block;
  while(1){
    if(size>size_max)
    {
      size_block = size_max - (size_max % xsize); //������� �������, ����� ��������� ����������� �����
      size -= size_block;
      ysize_block = size_max / xsize;
      for(int y=0; y<ysize_block; y++)
      {
        for(int x=0; x<xsize; x++)
        {
          blck[y*xsize+x] = color;
        }
      }
      send_blocks(spi, x1, y1, x2, y1 + ysize_block - 1, blck);
      send_block_finish(spi);
      y1 += ysize_block;
    }
    else{
      ysize_block = size / xsize;
      for(int y=0; y<ysize_block; y++)
      {
        for(int x=0; x<xsize; x++)
        {
          blck[y*xsize+x] = color;
        }
      }
      send_blocks(spi, x1, y1, x2, y1 + ysize_block - 1, blck);
      send_block_finish(spi);
      break;
    }
  }
  heap_caps_free(blck);
}*/
//-------------------------------------------------------------------
static void TFT9341_WriteData(spi_device_handle_t spi, uint8_t* buff, size_t buff_size) {
  esp_err_t ret;
  spi_transaction_t t;
  while(buff_size > 0) {
    uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
    if (chunk_size==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=chunk_size*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=buff;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    buff += chunk_size;
    buff_size -= chunk_size;
  }
}
//-------------------------------------------------------------------
static void TFT9341_SetAddrWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  // column address set
  lcd_cmd(spi, 0x2A); // CASET
  {
    uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
    TFT9341_WriteData(spi, data, sizeof(data));
  }
  // row address set
  lcd_cmd(spi, 0x2B); // RASET
  {
    uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
    TFT9341_WriteData(spi, data, sizeof(data));
  }
  // write to RAM
  lcd_cmd(spi, 0x2C); // RAMWR
}
//-------------------------------------------------------------------
void TFT9341_DrawPixel(spi_device_handle_t spi, int x, int y, uint16_t color)
{
  uint8_t data[2];
  if((x<0)||(y<0)||(x>=TFT9341_WIDTH)||(y>=TFT9341_HEIGHT)) return;
  data[0] = color>>8;
  data[1] = color & 0xFF;
  TFT9341_SetAddrWindow(spi, x,y,x,y);
  lcd_cmd(spi, 0x2C); //memory write 0_o?
  lcd_data(spi, data, 2);
}
//------------------------------------------------------------------
void TFT9341_DrawLine(spi_device_handle_t spi, uint16_t color, uint16_t x1, uint16_t y1,
                      uint16_t x2, uint16_t y2)
{
  int steep = abs(y2-y1)>abs(x2-x1);
  if(steep)
  {
    swap(x1,y1);
    swap(x2,y2);
  }
  if(x1>x2)
  {
    swap(x1,x2);
    swap(y1,y2);
  }
  int dx,dy;
  dx=x2-x1;
  dy=abs(y2-y1);
  int err=dx/2;
  int ystep;
  if(y1<y2) ystep=1;
  else ystep=-1;
  for(;x1<=x2;x1++)
  {
    if(steep) TFT9341_DrawPixel(spi, y1,x1,color);
    else TFT9341_DrawPixel(spi, x1,y1,color);
    err-=dy;
    if(err<0)
    {
      y1 += ystep;
      err=dx;
    }
  }
}
//------------------------------------------------------------------
void TFT9341_DrawRect(spi_device_handle_t spi, uint16_t color, uint16_t x1, uint16_t y1,
                      uint16_t x2, uint16_t y2)
{
  TFT9341_DrawLine(spi, color,x1,y1,x2,y1);
  TFT9341_DrawLine(spi, color,x2,y1,x2,y2);
  TFT9341_DrawLine(spi, color,x1,y1,x1,y2);
  TFT9341_DrawLine(spi, color,x1,y2,x2,y2);
}
//-------------------------------------------------------------------
void TFT9341_DrawCircle(spi_device_handle_t spi, uint16_t x0, uint16_t y0, int r, uint16_t color)
{
  int f = 1-r;
  int ddF_x=1;
  int ddF_y=-2*r;
  int x = 0;
  int y = r;
  TFT9341_DrawPixel(spi, x0,y0+r,color);
  TFT9341_DrawPixel(spi, x0,y0-r,color);
  TFT9341_DrawPixel(spi, x0+r,y0,color);
  TFT9341_DrawPixel(spi, x0-r,y0,color);
  while (x<y)
  {
    if (f>=0)
    {
      y--;
      ddF_y+=2;
      f+=ddF_y;
    }
    x++;
    ddF_x+=2;
    f+=ddF_x;
    TFT9341_DrawPixel(spi, x0+x,y0+y,color);
    TFT9341_DrawPixel(spi, x0-x,y0+y,color);
    TFT9341_DrawPixel(spi, x0+x,y0-y,color);
    TFT9341_DrawPixel(spi, x0-x,y0-y,color);
    TFT9341_DrawPixel(spi, x0+y,y0+x,color);
    TFT9341_DrawPixel(spi, x0-y,y0+x,color);
    TFT9341_DrawPixel(spi, x0+y,y0-x,color);
    TFT9341_DrawPixel(spi, x0-y,y0-x,color);
  }
}
//-------------------------------------------------------------------
void TFT9341_SetTextColor(uint16_t color)
{
  lcdprop.TextColor=color;
}
//-------------------------------------------------------------------
void TFT9341_SetBackColor(uint16_t color)
{
  lcdprop.BackColor=color;
}
//-------------------------------------------------------------------
void TFT9341_SetFont(sFONT *pFonts)
{
  lcdprop.pFont=pFonts;
}
//-------------------------------------------------------------------
void TFT9341_DrawChar(spi_device_handle_t spi, uint16_t x, uint16_t y, uint8_t c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t offset;
  uint8_t *c_t;
  uint8_t *pchar;
  uint32_t line=0;
  height = lcdprop.pFont->Height;
  width  = lcdprop.pFont->Width;
  offset = 8 *((width + 7)/8) -  width ;
  c_t = (uint8_t*) &(lcdprop.pFont->table[(c-' ') * lcdprop.pFont->Height * ((lcdprop.pFont->Width + 7) / 8)]);
  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c_t + (width + 7)/8 * i);
    switch(((width + 7)/8))
    {
      case 1:
          line =  pchar[0];
          break;
      case 2:
          line =  (pchar[0]<< 8) | pchar[1];
          break;
      case 3:
      default:
        line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
        break;
    }
    for (j = 0; j < width; j++)
    {
      if(line & (1 << (width- j + offset- 1)))
      {
        TFT9341_DrawPixel(spi, (x + j), y, lcdprop.TextColor);
      }
      else
      {
        TFT9341_DrawPixel(spi, (x + j), y, lcdprop.BackColor);
      }
    }
    y++;
  }
}
//-------------------------------------------------------------------
void TFT9341_String(spi_device_handle_t spi, uint16_t x,uint16_t y, char *str)
{
  while(*str)
  {
    TFT9341_DrawChar(spi, x,y,str[0]);
    x+=lcdprop.pFont->Width;
    (void)*str++;
  }
}
//-------------------------------------------------------------------
void TFT9341_SetRotation(spi_device_handle_t spi, uint8_t r)
{
  uint8_t data[1];
  lcd_cmd(spi, 0x36);
  switch(r)
  {
    case 0:
      data[0] = 0x48;
      lcd_data(spi, data, 1);
      TFT9341_WIDTH = 240;
      TFT9341_HEIGHT = 320;
      break;
    case 1:
      data[0] = 0x28;
      lcd_data(spi, data, 1);
      TFT9341_WIDTH = 320;
      TFT9341_HEIGHT = 240;
      break;
    case 2:
      data[0] = 0x88;
      lcd_data(spi, data, 1);
      TFT9341_WIDTH = 240;
      TFT9341_HEIGHT = 320;
      break;
    case 3:
      data[0] = 0xE8;
      lcd_data(spi, data, 1);
      TFT9341_WIDTH = 320;
      TFT9341_HEIGHT = 240;
      break;
  }
}
//-------------------------------------------------------------------

void TFT9341_ini(spi_device_handle_t spi, uint16_t w_size, uint16_t h_size)
{
  uint8_t data[15];
  //Initialize non-SPI GPIOs
  gpio_set_direction(CONFIG_PIN_NUM_DC, GPIO_MODE_OUTPUT);
  gpio_set_direction(CONFIG_PIN_NUM_RST, GPIO_MODE_OUTPUT);
  gpio_set_direction(CONFIG_PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
  gpio_set_level(CONFIG_PIN_NUM_BCKL, 1);
  TFT9341_reset();
  //Software Reset
  lcd_cmd(spi, 0x01);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  //Power control A, Vcore=1.6V, DDVDH=5.6V
  data[0] = 0x39;
  data[1] = 0x2C;
  data[2] = 0x00;
  data[3] = 0x34;
  data[4] = 0x02;
  lcd_cmd(spi, 0xCB);
  lcd_data(spi, data, 5);
  //Power contorl B, power control = 0, DC_ENA = 1
  data[0] = 0x00;
  data[1] = 0x83;
  data[2] = 0x30;
  lcd_cmd(spi, 0xCF);
  lcd_data(spi, data, 3);
  //Driver timing control A,
  //non-overlap=default +1
  //EQ=default - 1, CR=default
  //pre-charge=default - 1
  data[0] = 0x85;
  data[1] = 0x01;
  data[2] = 0x79;
  lcd_cmd(spi, 0xE8);
  lcd_data(spi, data, 3);
  //Driver timing control, all=0 unit
  data[0] = 0x00;
  data[1] = 0x00;
  lcd_cmd(spi, 0xEA);
  lcd_data(spi, data, 2);
  //Power on sequence control,
  //cp1 keeps 1 frame, 1st frame enable
  //vcl = 0, ddvdh=3, vgh=1, vgl=2
  //DDVDH_ENH=1
  data[0] = 0x64;
  data[1] = 0x03;
  data[2] = 0x12;
  data[3] = 0x81;
  lcd_cmd(spi, 0xED);
  lcd_data(spi, data, 4);
  //Pump ratio control, DDVDH=2xVCl
  data[0] = 0x20;
  lcd_cmd(spi, 0xF7);
  lcd_data(spi, data, 1);
  //Power control 1, GVDD=4.75V
  data[0] = 0x26;
  lcd_cmd(spi, 0xC0);
  lcd_data(spi, data, 1);
  //Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3
  data[0] = 0x11;
  lcd_cmd(spi, 0xC1);
  lcd_data(spi, data, 1);
  //VCOM control 1, VCOMH=4.025V, VCOML=-0.950V
  data[0] = 0x35;
  data[1] = 0x3E;
  lcd_cmd(spi, 0xC5);
  lcd_data(spi, data, 2);
  //VCOM control 2, VCOMH=VMH-2, VCOML=VML-2
  data[0] = 0xBE;
  lcd_cmd(spi, 0xC7);
  lcd_data(spi, data, 1);
  //Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0
  data[0] = 0x8; // 0x28
  lcd_cmd(spi, 0x36);
  lcd_data(spi, data, 1);
  //Pixel format, 16bits/pixel for RGB/MCU interface
  data[0] = 0x55;
  lcd_cmd(spi, 0x3A);
  lcd_data(spi, data, 1);
  //Frame rate control, f=fosc, 70Hz fps
  data[0] = 0x00;
  data[1] = 0x1B;
  lcd_cmd(spi, 0xB1);
  lcd_data(spi, data, 2);
  //Display function control
  data[0] = 0x08;
  data[1] = 0x82;
  data[2] = 0x27;
  lcd_cmd(spi, 0xB6);
  lcd_data(spi, data, 3);
  //Enable 3G, disabled
  data[0] = 0x08;
  lcd_cmd(spi, 0xF2);
  lcd_data(spi, data, 1);
  //Gamma set, curve 1
  data[0] = 0x01;
  lcd_cmd(spi, 0x26);
  lcd_data(spi, data, 1);
  //Positive gamma correction
  data[0] = 0x0F;
  data[1] = 0x31;
  data[2] = 0x2B;
  data[3] = 0x0C;
  data[4] = 0x0E;
  data[5] = 0x08;
  data[6] = 0x4E;
  data[7] = 0xF1;
  data[8] = 0x37;
  data[9] = 0x07;
  data[10] = 0x10;
  data[11] = 0x03;
  data[12] = 0x0E;
  data[13] = 0x09;
  data[14] = 0x00;
  lcd_cmd(spi, 0xE0);
  lcd_data(spi, data, 15);
  //Negative gamma correction
  data[0] = 0x00;
  data[1] = 0x0E;
  data[2] = 0x14;
  data[3] = 0x03;
  data[4] = 0x11;
  data[5] = 0x07;
  data[6] = 0x31;
  data[7] = 0xC1;
  data[8] = 0x48;
  data[9] = 0x08;
  data[10] = 0x0F;
  data[11] = 0x0C;
  data[12] = 0x31;
  data[13] = 0x36;
  data[14] = 0x0F;
  lcd_cmd(spi, 0xE1);
  lcd_data(spi, data, 15);
  //Column address set, SC=0, EC=0xEF
  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0xEF;
  lcd_cmd(spi, 0x2A);
  lcd_data(spi, data, 4);
  //Page address set, SP=0, EP=0x013F
  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x01;
  data[3] = 0x3F;
  lcd_cmd(spi, 0x2B);
  lcd_data(spi, data, 4);
  //Memory write
  lcd_cmd(spi, 0x2C);
  //Entry mode set, Low vol detect disabled, normal display
  data[0] = 0x07;
  lcd_cmd(spi, 0xB7);
  lcd_data(spi, data, 1);
  //Sleep out
  lcd_cmd(spi, 0x21);
  lcd_cmd(spi, 0x11);
  //Display on
  lcd_cmd(spi, 0x29);
  TFT9341_WIDTH = w_size;
  TFT9341_HEIGHT = h_size;
}
//-------------------------------------------------------------------
/*void TFT9341_spi_init(void)
{
	  uint16_t i,j;
	  esp_err_t ret;

	  // Configure SPI bus
	  spi_bus_config_t cfg = {
	      .mosi_io_num = CONFIG_PIN_NUM_MOSI,
	      .miso_io_num = -1,
	      .sclk_io_num = CONFIG_PIN_NUM_CLK,
	      .quadwp_io_num = -1,
	      .quadhd_io_num = -1,
	      .max_transfer_sz = 16*320*2+8,
	  };
	  ret = spi_bus_initialize(HSPI_HOST, &cfg, SPI_DMA_CH_AUTO);
	  //ESP_LOGI(TAG, "spi bus initialize: %d", ret);
	  spi_device_interface_config_t devcfg={
	        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
	        .mode=0,                                //SPI mode 0
	        .spics_io_num=CONFIG_PIN_NUM_CS,               //CS pin
	        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
	        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
	  };
	  ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	  //ESP_LOGI(TAG, "spi bus add device: %d", ret);
	  TFT9341_ini(spi, 320, 240);
	  //TFT9341_SetRotation(spi, 0);

}
//==========================================================================================
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(CONFIG_PIN_NUM_DC, dc);
}*/
