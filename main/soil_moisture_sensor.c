#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "u8g2.h"
#include "freertos/queue.h"

static const char *TAG1 = "ADC";
static const char *TAG2 = "WiFi";
static const char *TAG3 = "OLED";
static const char *TAG4 = "Wifi connection";
static const char *TAG5 = "IOT SETUP";
static const char *TAG6 = "queue receive success";
#define I2C_PORT    I2C_NUM_0
#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_FREQ    100000
#define OLED_ADDR   0x3C
static u8g2_t u8g2;

static bool calibrated = false;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t cali_handle = NULL;
//uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
//uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

typedef struct {
    QueueHandle_t ts_queue;   // To receive from Sensor
    QueueHandle_t o_queue;    // To send to OLED
} task_params_t;//this is passed as parameter to thingspeak_task



//-------------------linkers------------------------------------------
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static i2c_cmd_handle_t cmd;
    switch (msg) {
        case U8X8_MSG_BYTE_START_TRANSFER:
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
            break;
        case U8X8_MSG_BYTE_SEND:
            i2c_master_write(cmd, (uint8_t *)arg_ptr, arg_int, true);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            break;
        default: return 0;
    }
    return 1;
}

uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    if (msg == U8X8_MSG_DELAY_MILLI) {
        vTaskDelay(pdMS_TO_TICKS(arg_int));
    }
    return 1;
}




void init(){
	//ADC initialization--------------------------------------------------------------------------------------
	adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // Modern name for 11dB (covers up to ~3.1V)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    
    //bool calibrated = false;
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        calibrated = true;
        printf("Calibration Success! eFuse data is being used.\n");
    }
      ESP_LOGI(TAG1, "Init Complete");
    //-------------------------------------------ADC initialization done--------------------------------------

    
    // Wifi connection--------------------------------------------------------------------------------
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta(); 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SSID NAME HERE",
            .password = "ENTER YOUR PASSWORD HERE",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG4, "Attempting to connect to SSID: %s", wifi_config.sta.ssid);
    esp_wifi_connect();
    ESP_LOGI(TAG2, "Init Complete");


    //------------------------------------------------------------------------------------------
    
    //SH1106 OLED initialization ----------------------------------------------------------------------
   
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, i2c_conf.mode, 0, 0, 0));

    u8g2_Setup_sh1106_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // Wake up display

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 0, 20, "System Starting...");
    u8g2_SendBuffer(&u8g2);
    ESP_LOGI(TAG3, "Init Complete");
  //OLED initialization DONE----------------------------------------------------------------------
   
}


//sensor task:-----------------------------------------------------------------------------------------------
void sensor_data_Task(void *param){
  QueueHandle_t qhandle = (QueueHandle_t)param; //typecasting into the required type
  

        int raw;
        int voltage;
        BaseType_t status;


        while(1){

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &raw));
        
        if (calibrated) {
           
         ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage));
          
         status = xQueueSendToBack(qhandle, &voltage, pdMS_TO_TICKS(100));
         if(status == pdPASS){
         	printf("successfully sent data: %d mV\n", voltage);

         }

         else {
            printf("unable to send data to queue!\n");
        }

       }
       vTaskDelay(pdMS_TO_TICKS(1000)); 
        
    }

}


//thingspeak task:---------------------------------------------------------------------------------------------
void thingspeak_Task(void *param){
	 task_params_t *tparameters = (task_params_t *)param;
     QueueHandle_t sensor_handle = tparameters->ts_queue;
     QueueHandle_t oled_handle = tparameters->o_queue;
     int received_voltage = 0; //receiving integer
     int sum = 0;  //sum of first 10 voltage values
     float average_voltage = 0.0; //average of first 10 voltage values
     BaseType_t get_status;
     BaseType_t send_status;
     while(1){
     	sum = 0; //reset the sum
     	  for (int i =0; i<10; i++){
     	  	get_status = xQueueReceive(sensor_handle, &received_voltage, portMAX_DELAY);
            if(get_status == pdPASS){
          	//converting the voltage to percentage and sending it to cloud and to oled queue
            	ESP_LOGI(TAG6, "received voltage data");
            	sum += received_voltage;
            }
     	  }
          
        average_voltage = sum/10.0;
        float percentage_moisture = ((3165.0 - received_voltage)/(3165.0 - 1360.0))*100;
        if (percentage_moisture <0) percentage_moisture = 0;
        if(percentage_moisture>100) percentage_moisture = 100;

        
        //upload data to thingspeak --------------------------------------------------------------
         char url_buffer[256];
         snprintf(url_buffer, sizeof(url_buffer), 
         "http://api.thingspeak.com/update?api_key=%s&field1=%.2f&field2=%.2f", 
         "EnterYourWriteAPIKeyHere", average_voltage, percentage_moisture); 
         esp_http_client_config_t config = {
             .url = url_buffer,
             .method = HTTP_METHOD_GET,
             .timeout_ms = 5000, // 5 second timeout for slow hotspot connections
             };

            esp_http_client_handle_t client = esp_http_client_init(&config);
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
              int status_code = esp_http_client_get_status_code(client);
              if (status_code == 200) {
                 printf("ThingSpeak Update Successful!!!!\n");
              } 
              else { printf("ThingSpeak Server Error. Status: %d", status_code); }
               } 

               else { printf("connection failed............."); }
               esp_http_client_cleanup(client);                              


        //end of transmission--------------------------------------------------------------------------
        
        send_status = xQueueSendToBack(oled_handle, &average_voltage, pdMS_TO_TICKS(100));
        if(send_status == pdPASS){
         	printf("successfully sent data: %f mV\n", average_voltage);

         }

         else {
            printf("unable to send data to oled_queue!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10000));

     }


}

//OLED task--------------------------------------------------------------------------------------------------------------
void oled_Task (void *param){
  QueueHandle_t ohandle = (QueueHandle_t)param;
  float received_voltage = 0.0;
  float percentage_moisture = 0.0;
  char oled_buffer[32];
  BaseType_t status;

  while(1){
  	status = xQueueReceive(ohandle, &received_voltage, portMAX_DELAY);
  	if(status == pdPASS){
       percentage_moisture = ((3165.0 - received_voltage)/(3165.0 - 1360.0))*100;
       if (percentage_moisture <0) percentage_moisture = 0;
       if(percentage_moisture>100) percentage_moisture = 100;

    snprintf(oled_buffer, sizeof(oled_buffer), "Moisture: %.1f%%", percentage_moisture);
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 0, 20, oled_buffer);

    char voltage_buffer[20];
    snprintf(voltage_buffer, sizeof(voltage_buffer), "Voltage level: %.0f%%", received_voltage);
    
    u8g2_DrawStr(&u8g2, 0, 40, voltage_buffer);

    u8g2_SendBuffer(&u8g2);
  	}
  }
  

}



void app_main(void)
{

	init();
	ESP_LOGI(TAG5, "SETUP SUCCESSFUL!!!");

    //building data queues
	QueueHandle_t thingspeak_queue = xQueueCreate(15, sizeof(int)); //contains the voltages in mV (int)
	QueueHandle_t oled_queue = xQueueCreate(5, sizeof(float)); //contains the percentages
	if( thingspeak_queue != NULL && oled_queue != NULL){
	printf("queue successfully created!\n");
    
    static task_params_t task2_parameters;
    task2_parameters.ts_queue = thingspeak_queue;
    task2_parameters.o_queue = oled_queue;

	xTaskCreate(sensor_data_Task, "sensor_data_Task", 1024*4, (void *)thingspeak_queue, 3, NULL);
	xTaskCreate(thingspeak_Task, "thingspeak_Task", 1024*8, (void *)&task2_parameters, 2, NULL);
	xTaskCreate(oled_Task, "oled_Task", 1024*4, (void *)oled_queue, 1, NULL);

}
else{
	printf("queue NOT created\n");
}

    

}
