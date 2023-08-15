#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include "ds18b20.h"
#include "ili9342c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <driver/spi_master.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/dac.h"

#define TEMP_BUS 22
#define LED 2
#define HIGH 1
#define LOW 0
#define digitalWrite gpio_set_level

void lcd_spi_pre_transfer_callback(spi_transaction_t *t);
void TFT9341_spi_init(void);
void task_temp(void *params);
void task_line(void);
void task_line_move(void);
void task_visualize(void);
void task_menu(void);
void visualize(void);
void led_init(void);
//---------------------------------------------------------------------------
static const char* TAG = "McD";
uint8_t dt[9];
float cTemp;
float temp_water, temp_outside, temp_set;
char str_water[26], str_outside[26];
uint8_t rom[9];
TaskHandle_t task_temp_h, task_line_h, task_line_move_h,
task_visualize_h, task_menu_h, task_h;
SemaphoreHandle_t spiSemaphore;

typedef struct {
	uint16_t x1;
	uint16_t x2;
	uint16_t y1;
	uint16_t y2;
}line_t;

line_t line_arr[56];
//----------------------------------------------
uint8_t rom_blue[8] = {0x28, 0xc5, 0xc4, 0x7, 0xd6, 0x1, 0x3c, 0xc0};
uint8_t rom_black[8] = {0x28, 0x3b, 0xe5, 0x7, 0xd6, 0x1, 0x3c, 0x94};

QueueHandle_t queue = NULL;
uint32_t buf[5];
//---------------------------------------------
static void IRAM_ATTR gpio_isr_menu(void* arg)
{
	//if(xSemaphoreTake(spiSemaphore, portMAX_DELAY)){
	//spi_device_abort_trans();

	vTaskSuspend(task_line_h);
	//vTaskSuspend(task_visualize_h);
	//vTaskResume(task_menu_h);
	xQueueSendFromISR(queue, buf, NULL);
	//}
}

void task(void)//0
{
	for(;;)
	{
		if(xQueueReceive(queue, buf, portMAX_DELAY)){
			xSemaphoreTake(spiSemaphore, portMAX_DELAY);
			TFT9341_FillRect(spi, 0, 0, 160, 120, TFT9341_BLUE);
			//vTaskDelay(500 / portTICK_PERIOD_MS);
			TFT9341_FillRect(spi, 120, 0, 320, 240, TFT9341_WHITE);
			//vTaskDelay(500 / portTICK_PERIOD_MS);
			TFT9341_FillRect(spi, 0, 120, 160, 240, TFT9341_RED);
				//vTaskDelay(500 / portTICK_PERIOD_MS);
				TFT9341_FillRect(spi, 120, 160, 320, 240, TFT9341_GREEN_2);
					//vTaskDelay(500 / portTICK_PERIOD_MS);
				xSemaphoreGive(spiSemaphore);
				visualize();
				vTaskResume(task_line_h);
		}
	}
}

void app_main(void)
{
	//off fucking stereo
	gpio_set_direction(25, GPIO_MODE_OUTPUT);
	gpio_set_level(25, 0);

	dac_output_enable(DAC_CHANNEL_2);
	dac_output_voltage(DAC_CHANNEL_2, 2);
	//535, 220 omh res
	//Intro animation
	//Initialization


	line_arr[0].x1 = 27;
	for(int i = 0; i < 56; i++){
		line_arr[i].x2 = (line_arr[i].x1 + 5);
		line_arr[i + 1].x1 = line_arr[i].x2;
	}
	ESP_LOGI(TAG, "%d %d %d %d %d %d ", line_arr[0].x1, line_arr[0].x2, line_arr[1].x1,
			line_arr[1].x2, line_arr[55].x1, line_arr[55].x2);
	TFT9341_spi_init(); // Init LCD and SPI
	ds18b20_init(TEMP_BUS);
	ds18b20_reset();
	ds18b20_write_byte(SKIP_ROM);
	ds18b20_write_byte(WRITE_SCRATCH);
	ds18b20_write_byte(33); // high alarm temp
	ds18b20_write_byte(22); // low alarm temp
	ds18b20_write_byte(TEMP_12_BIT);
	ds18b20_reset();



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
	TFT9341_FillScreen(spi, TFT9341_ORANGE);
	ds18b20_reset();
	ds18b20_write_byte(READ_ROM);
	for(int i = 0; i < 8; i++){
		rom[i] = ds18b20_read_byte();
	}

	ESP_LOGI(TAG, "ROM: %x %x %x %x %x %x %x %x ",
			rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);

	spiSemaphore = xSemaphoreCreateMutex();
	visualize();
	//buttons interrupt init
	xTaskCreate((void*)task_menu, "task_menu", 2048, NULL,
			0, &task_menu_h);
	xTaskCreate((void*)task, "task", 2048, NULL,
			0, &task_h);
	vTaskSuspend(task_menu_h);

	gpio_config_t gpio_intrpt;
	gpio_intrpt.intr_type = GPIO_INTR_NEGEDGE;
	gpio_intrpt.mode = GPIO_MODE_INPUT;
	gpio_intrpt.pin_bit_mask = 1ULL << GPIO_NUM_38;
	gpio_intrpt.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_intrpt.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&gpio_intrpt);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_38, gpio_isr_menu, NULL);

	queue = xQueueCreate(5, sizeof(32));
	led_init();
	//xTaskCreate((void*)task_temp, "Temp_calc", 2048,
			//NULL, 0, &task_temp_h);
	xTaskCreate((void*)task_line, "Temp_line", 2048,
				NULL, 1, &task_line_h);

	vTaskDelete(NULL);
	while (true) {

    }
}//main

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
	        .clock_speed_hz=30*1000*1000,           //Clock out at 10 MHz
	        .mode=0,                                //SPI mode 0
	        .spics_io_num=CONFIG_PIN_NUM_CS,               //CS pin
	        .queue_size=20,                          //We want to be able to queue 7 transactions at a time
	        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
	  };
	  ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	  //ESP_LOGI(TAG, "spi bus add device: %d", ret);
	  TFT9341_ini(spi, 320, 240);
	  //TFT9341_SetRotation(spi, 0);

}

void led_init(void)
{
	// Configure LEDC peripheral
	    ledc_timer_config_t ledc_timer = {
	        .duty_resolution = LEDC_TIMER_13_BIT, // Set duty resolution to 13 bits
	        .freq_hz = 7000,                      // Set PWM frequency to 5 kHz
	        .speed_mode = LEDC_HIGH_SPEED_MODE,    // Use high-speed LEDC mode
	        .timer_num = LEDC_TIMER_0,             // Use timer 0
	        .clk_cfg = LEDC_APB_CLK,              // Use auto-select clock
	    };
	    ledc_timer_config(&ledc_timer);

	    // Configure LEDC channel
	    ledc_channel_config_t ledc_channel = {
	        .channel = LEDC_CHANNEL_0,             // Use channel 0
	        .duty = 0,                             // Initial duty cycle of 0%
	        .gpio_num = GPIO_NUM_5,                // GPIO pin for PWM output
	        .speed_mode = LEDC_HIGH_SPEED_MODE,    // Use high-speed LEDC mode
	        .timer_sel = LEDC_TIMER_0,             // Use timer 0
	    };
	    ledc_channel_config(&ledc_channel);

	    // Start PWM generation
	    ledc_fade_func_install(0);
	    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 4096); // Set duty cycle to 50%
	    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
	    // 2200 Omh 20uF RC or 9150 omh
	    // Other application code...
	}


void visualize(void)
{
	ds18b20_requestTemperatures();

	TFT9341_SetTextColor(TFT9341_YELLOW);
	TFT9341_SetBackColor(TFT9341_BLUE);
	TFT9341_SetFont(&Font24);

	temp_water = ds18b20_get_temp(MATCH_ROM, rom_black);
	sprintf(str_water, "Water");
	TFT9341_String(spi, 10, 5, str_water);
	sprintf(str_water, "%0.2fC", temp_water);
	TFT9341_String(spi, 10, 29, str_water);

	temp_outside = ds18b20_get_temp(MATCH_ROM, rom_blue);
	sprintf(str_outside, "Outside");
	TFT9341_String(spi, 120, 5, str_outside);
	sprintf(str_outside, "%0.2fC", temp_outside);
	TFT9341_String(spi, 120, 29, str_outside);
	TFT9341_DrawRect(spi, TFT9341_BLUE, 26, 60, 308, 230);

	TFT9341_SetFont(&Font16);
	sprintf(str_water, "40");
	TFT9341_String(spi, 2, 55, str_water);
	sprintf(str_water, "20");
	TFT9341_String(spi, 2, 225, str_water);
	TFT9341_SetFont(&Font24);
	//TFT9341_SetAddrWindow(spi, 100, 100, 300, 200);
	//TFT9341_FillScreen(spi, TFT9341_RED);
	//TFT9341_FillRect(spi, 31, 61, 309, 229, TFT9341_BLACK);
	//vTaskDelay(2000);
}

void task_temp(void *params)// 0
{
	//PID

	for(;;)
	{
		ds18b20_requestTemperatures();
		temp_water = ds18b20_get_temp(MATCH_ROM, rom_black);
		temp_outside = ds18b20_get_temp(MATCH_ROM, rom_blue);
		sprintf(str_water, "%0.2fC", temp_water);
		sprintf(str_outside, "%0.2fC", temp_outside);
		xTaskNotifyGive(task_visualize_h);
		/*xSemaphoreTake(spiSemaphore, portMAX_DELAY);
		TFT9341_String(spi, 10, 29, str);

		sprintf(str, "%0.2fC", temp_outside);
		TFT9341_String(spi, 120, 29, str);
		xSemaphoreGive(spiSemaphore);*/
	}
}

void task_line(void)//1
{
	uint8_t pixels = 0;
	float decrement = temp_water;
	while(decrement > 20){
		decrement -= (float)0.1;
		pixels++;
	}
	line_arr[0].y1 = 230 - pixels;

	pixels = 0;
	for(;;)
	{
		for(uint8_t i = 0; i < 56; i++){
			if(temp_water > 19 && temp_water < 38){
				ds18b20_requestTemperatures();
				temp_water = ds18b20_get_temp(MATCH_ROM, rom_black);
				temp_outside = ds18b20_get_temp(MATCH_ROM, rom_blue);
				sprintf(str_water, "%0.2fC", temp_water);
				sprintf(str_outside, "%0.2fC", temp_outside);
				xSemaphoreTake(spiSemaphore, portMAX_DELAY);
				TFT9341_String(spi, 10, 29, str_water);
				TFT9341_String(spi, 120, 29, str_outside);
				xSemaphoreGive(spiSemaphore);

				decrement = temp_water;
				while(decrement > (float)20){
					decrement -= (float)0.1;
					pixels++;
				}
				line_arr[i].y2 = 230 - pixels;
				line_arr[i + 1].y1 = line_arr[i].y2;
				xSemaphoreTake(spiSemaphore, portMAX_DELAY);
				TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
						line_arr[i].y1, line_arr[i].x2, line_arr[i].y2);
				TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
						line_arr[i].y1 + 1, line_arr[i].x2, line_arr[i].y2 + 1);
				TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
						line_arr[i].y1 - 1, line_arr[i].x2, line_arr[i].y2 - 1);
				xSemaphoreGive(spiSemaphore);
				pixels = 0;
				ESP_LOGI(TAG, "%d %d %d %d ", line_arr[i].x1,
						line_arr[i].y1, line_arr[i].x2,
						line_arr[i].y2);
				//vTaskDelay(500 / portTICK_PERIOD_MS);
			}

		}
		break;
	}
	xTaskCreate((void*)task_temp, "Temp_calc", 2048,
					NULL, 0, &task_temp_h);
	xTaskCreate((void*) task_line_move, "task_line_move", 2048,
			NULL, 0, &task_line_move_h);
	xTaskCreate((void*) task_visualize, "task_visualize", 2048,
			NULL, 1, &task_visualize_h);

	vTaskDelete(NULL);
}

void task_visualize(void)//1
{
	//TickType_t xLastWakeTime;
	//const TickType_t xFrequency = 1000;

	     // Initialise the xLastWakeTime variable with the current time.
	//xLastWakeTime = xTaskGetTickCount();
	for(;;){
		for(int i = 0; i < 5; i++){
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			xSemaphoreTake(spiSemaphore, portMAX_DELAY);
			TFT9341_String(spi, 10, 29, str_water);
			TFT9341_String(spi, 120, 29, str_outside);
			xSemaphoreGive(spiSemaphore);
			//vTaskDelay(750 / portTICK_PERIOD_MS);
			//vTaskDelayUntil( &xLastWakeTime, xFrequency );
		}
		xSemaphoreTake(spiSemaphore, portMAX_DELAY);
		TFT9341_FillRect(spi, 27, 61, 307, 229, TFT9341_ORANGE);
		//draw new graph fast
		for(int i = 0; i < 56; i++)
		{
			TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
					line_arr[i].y1, line_arr[i].x2, line_arr[i].y2);
			TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
					line_arr[i].y1 + 1, line_arr[i].x2, line_arr[i].y2 + 1);
			TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
					line_arr[i].y1 - 1, line_arr[i].x2, line_arr[i].y2 - 1);
		}
		xSemaphoreGive(spiSemaphore);
		xTaskNotifyGive(task_line_move_h);
	}
}

void task_line_move(void)//0
{
	uint8_t pixels = 0;
	float decrement;
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		for(int i = 0; i < 55; i++)
		{
			line_arr[i].y1 = line_arr[i + 1].y1;
			line_arr[i].y2 = line_arr[i + 1].y2;
		}
		//calc new last line
		decrement = temp_water;
		while(decrement > (float)20){
			decrement -= (float)0.1;
			pixels++;
		}
		line_arr[55].y2 = 230 - pixels;
		line_arr[55].y1 = line_arr[54].y2;

		pixels = 0;
		//clear old graph
		/*xSemaphoreTake(spiSemaphore, portMAX_DELAY);
		TFT9341_FillRect(spi, 27, 61, 307, 229, TFT9341_ORANGE);
		//draw new graph fast
		for(int i = 0; i < 56; i++)
		{
			TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
					line_arr[i].y1, line_arr[i].x2, line_arr[i].y2);
			TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
					line_arr[i].y1 + 1, line_arr[i].x2, line_arr[i].y2 + 1);
			TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[i].x1,
					line_arr[i].y1 - 1, line_arr[i].x2, line_arr[i].y2 - 1);
		}
		xSemaphoreGive(spiSemaphore);*/
		/*TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[55].x1,
				line_arr[55].y1, line_arr[55].x2, line_arr[55].y2);
		TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[55].x1,
				line_arr[55].y1 + 1, line_arr[55].x2, line_arr[55].y2 + 1);
		TFT9341_DrawLine(spi, TFT9341_GREEN_2, line_arr[55].x1,
				line_arr[55].y1 - 1, line_arr[55].x2, line_arr[55].y2 - 1);
		*/
		//vTaskDelay(4500 / portTICK_PERIOD_MS);
	}  //for ever
}  //task

void task_menu(void)//0
{
	TFT9341_FillRect(spi, 0, 0, 160, 120, TFT9341_BLUE);
	//vTaskDelay(500 / portTICK_PERIOD_MS);
	TFT9341_FillRect(spi, 120, 0, 320, 240, TFT9341_WHITE);
		//vTaskDelay(500 / portTICK_PERIOD_MS);
		TFT9341_FillRect(spi, 0, 120, 160, 240, TFT9341_RED);
			//vTaskDelay(500 / portTICK_PERIOD_MS);
			TFT9341_FillRect(spi, 120, 160, 320, 240, TFT9341_GREEN_2);
				//vTaskDelay(500 / portTICK_PERIOD_MS);
}
//TO DO: debug line task with LOGI		W
//TO DO: learn PWM output & apply with PID formula




