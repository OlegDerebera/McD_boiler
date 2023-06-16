#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include "ds18b20.h"
#include "ili9342c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <driver/spi_master.h>
#include "esp_log.h"
#include "esp_err.h"

#define TEMP_BUS 22
#define LED 2
#define HIGH 1
#define LOW 0
#define digitalWrite gpio_set_level

void lcd_spi_pre_transfer_callback(spi_transaction_t *t);
void TFT9341_spi_init(void);
//---------------------------------------------------------------------------
static const char* TAG = "McD";
uint8_t dt[9];
float cTemp;
char str[26];
uint8_t rom[9];
typedef struct {
	uint16_t x1;
	uint16_t x2;
	uint16_t y1;
	uint16_t y2;
}line_t;

line_t line_arr[60];
//----------------------------------------------
uint8_t rom_blue[8] = {0x28, 0xc5, 0xc4, 0x7, 0xd6, 0x1, 0x3c, 0xc0};
uint8_t rom_black[8] = {0x28, 0x3b, 0xe5, 0x7, 0xd6, 0x1, 0x3c, 0x94};

//---------------------------------------------


void app_main(void)
{
	//off fucking stereo
	gpio_set_direction(25, GPIO_MODE_OUTPUT);
	gpio_set_level(25, 0);
	//Intro animation
	//Initialization
	 /* uint16_t i,j;
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
	  TFT9341_FillScreen(spi, TFT9341_YELLOW);
	  TFT9341_DrawLine(spi, TFT9341_CYAN, 10, 30, 100, 200);
	  for(int i = 150; i < 250; i++){
		  for(int j = 100; j < 200; j++){
			  TFT9341_DrawPixel(spi, j, i, TFT9341_MAGENTA);
		  }
	  }*/
	  //TFT9341_SetRotation(spi, 0);
	uint16_t a = 15;
	line_arr[0].x1 = 15;
	for(int i = 0; i < 60; i++){
		line_arr[i].x2 = (line_arr[i].x1 + 5);
		line_arr[i + 1].x1 = line_arr[i].x2;
	}
	ESP_LOGI(TAG, "%d %d %d %d %d %d ", line_arr[0].x1, line_arr[0].x2, line_arr[1].x1,
			line_arr[1].x2, line_arr[59].x1, line_arr[59].x2);
	TFT9341_spi_init(); // Init LCD and SPI
	ds18b20_init(TEMP_BUS);
	ds18b20_reset();
	ds18b20_write_byte(SKIP_ROM);
	ds18b20_write_byte(WRITE_SCRATCH);
	ds18b20_write_byte(33); // high alarm temp
	ds18b20_write_byte(22); // low alarm temp
	ds18b20_write_byte(TEMP_12_BIT);
	ds18b20_reset();

	//ds18b20_readScratchPad(SKIP_ROM, 1, dt);
	//Visualization
	//Check if OK
	//Error task
	//Startup
	//Measurements+

	//PID output
	  TFT9341_FillScreen(spi, TFT9341_YELLOW);
	  TFT9341_DrawLine(spi, TFT9341_CYAN, 10, 30, 100, 200);
	  for(int i = 150; i < 250; i++){
		  for(int j = 100; j < 200; j++){
			  TFT9341_DrawPixel(spi, j, i, TFT9341_MAGENTA);
		  }
	  }
	TFT9341_FillScreen(spi, TFT9341_MAGENTA);
	ds18b20_reset();
	ds18b20_write_byte(READ_ROM);
	for(int i = 0; i < 8; i++){
		rom[i] = ds18b20_read_byte();
	}

	ESP_LOGI(TAG, "ROM: %x %x %x %x %x %x %x %x ",
			rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (true) {

    	ds18b20_requestTemperatures();
    	cTemp = ds18b20_get_temp(MATCH_ROM, rom_black);

    	TFT9341_SetTextColor(TFT9341_YELLOW);
    	TFT9341_SetBackColor(TFT9341_BLUE);
    	TFT9341_SetFont(&Font24);

    	sprintf(str, "Esp32");
    	TFT9341_String(spi, 10, 10, str);
    	sprintf(str, "Temp blk:%0.2fC", cTemp);
    	TFT9341_String(spi, 5, 35, str);
    	cTemp = ds18b20_get_temp(MATCH_ROM, rom_blue);
    	sprintf(str, "Temp blu:%0.2fC", cTemp);
    	TFT9341_String(spi, 5, 70, str);
    	vTaskDelay(10 / portTICK_PERIOD_MS);

    }
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(CONFIG_PIN_NUM_DC, dc);
}

void TFT9341_spi_init(void)
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
