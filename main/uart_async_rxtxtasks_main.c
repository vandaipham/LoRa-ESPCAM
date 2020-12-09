/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "esp_camera.h"
/**
 * @brief Data structure of camera frame buffer
 */
// typedef struct {
//     uint8_t * buf;              /*!< Pointer to the pixel data */
//     size_t len;                 /*!< Length of the buffer in bytes */
//     size_t width;               /*!< Width of the buffer in pixels */
//     size_t height;              /*!< Height of the buffer in pixels */
//     pixformat_t format;         /*!< Format of the pixel data */
//     struct timeval timestamp;   /*!< Timestamp since boot of the first DMA buffer of the frame */
// } camera_fb_t;

//ESP32Cam (AiThinker) PIN Map
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static const char *TAG = "ESP32Cam";

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,//QQVGA-QXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

esp_err_t init_camera(){
    //power up the camera if PWDN pin is defined
    if(CAM_PIN_PWDN != -1){
    	gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
        // pinMode(CAM_PIN_PWDN, OUTPUT);
        gpio_set_level(GPIO_NUM_32, 0);
    }

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

esp_err_t camera_capture(){
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    //replace this with your own function
    ESP_LOGI(TAG, "Picture taken! Its width was: %zu pixels", fb->width);
    ESP_LOGI(TAG, "Picture taken! Its height was: %zu pixels", fb->height);
    // ESP_LOGI(TAG, "Picture taken! Its format was: %zu bytes", fb->format);
    // ESP_LOGI(TAG, "Picture taken! Its buf was: %zu bytes", fb->buf);
    ESP_LOGI(TAG, "Picture taken! Its len was: %zu bytes", fb->len);
  
    //return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return ESP_OK;
}

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

int sendGeneralFrame(uint8_t frameType, int8_t cmdType, uint8_t frameLoadLen, uint8_t* frameLoad)
{
    uint8_t checksum = 0;
    checksum ^= frameType;
    checksum ^= 0; // frameNum which is unused and always 0
    checksum ^= cmdType;
    checksum ^= frameLoadLen;

    uint8_t *frameByte = (uint8_t *) malloc((uint8_t) frameLoadLen + 5);
    frameByte[0] = frameType;
    frameByte[1] = (uint8_t) 0; // Frame Number = 0 by default.
    frameByte[2] = cmdType;
    frameByte[3] = frameLoadLen;
    memcpy(frameByte + sizeof(uint8_t) * 4, frameLoad, frameLoadLen);

    for (size_t i = 0; i < frameLoadLen; i++)
    {
        checksum ^= frameLoad[i];
    }
    frameByte[(uint8_t) frameLoadLen + 5 - 1] = checksum;

    uart_write_bytes(UART_NUM_1, (const char *) frameByte, (uint8_t) frameLoadLen + 5);
    
    free(frameByte);
    return 1;
}

int sendAppDataRequest( uint16_t target, uint8_t dataLen, uint8_t* data)
{
    // We add 7 bytes to the head of data for this payload
    uint8_t frameLoadLen = 6 + dataLen;
    uint8_t* frameLoad = (uint8_t *) malloc(sizeof(uint8_t) * frameLoadLen);

    // target address as big endian short
    frameLoad[0] = (uint8_t) ((target >> 8) & 0xFF);
    frameLoad[1] = (uint8_t) (target & 0xFF);

    // ACK request == 1 -> require acknowledgement of recv
    frameLoad[2] = (uint8_t) 0;

    // Send radius: which defaults to max of 7 hops, we can use that
    frameLoad[3] = (uint8_t) 7;

    // Discovery routing params == 1 -> automatic routing
    frameLoad[4] = (uint8_t) 1;

    // Data length
    frameLoad[5] = dataLen;

    // Data from index 7 to the end should be the data
    memcpy(frameLoad + (sizeof(uint8_t) * 6), data, dataLen);

    // frameType = 0x05, cmdType = 0x01 for sendData
    sendGeneralFrame(0x05, 0x01, frameLoadLen, frameLoad);
    
    free(frameLoad);

    return 1;
}

int testingSend()
{
    uint8_t* data = malloc(sizeof(uint8_t) * 4);
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x03;
    data[3] = 0x04;
    sendAppDataRequest(0x0000, 4, data);
    free(data);
    return 1;
}

int sendImage()
{
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }
    uint8_t* imageData = fb->buf;
    size_t imageDataLen = fb->len;

    // imageDataLen - Image Size in Byte
    int numOfFragment = 0;                  // Number of Fragments
    int fragmentSize = 100;                 // each fragment has 100 Bytes.
    int imageID = 0;

    if (imageDataLen % fragmentSize == 0)
        numOfFragment = imageDataLen / fragmentSize;
    else
        numOfFragment = imageDataLen / fragmentSize + 1;
    
    for (int8_t i = 0; i < numOfFragment; i++) {
        // Sending each fragment
        if (i == numOfFragment - 1)
            fragmentSize = imageDataLen - (numOfFragment-1)*100;

        uint8_t* data = malloc(sizeof(uint8_t) * (fragmentSize + 2));
        data[0] = (uint8_t) imageID;     // Image ID
        data[1] = (uint8_t) i;     // Sequence Number

        for (int8_t j = 0; j < 100; j++) {
            // Data from index 3 to the end should be the image fragment data.
            memcpy(data + (sizeof(uint8_t) * (2+j)), &imageData[j + i*100], 1);
        }
        // sendAppDataRequest(0x0000, 4, data);
        free(data);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    return 1;
}

void sendPacket()
{
    uint8_t *dataByte = (uint8_t *) malloc(16);
    dataByte[0] = 0x05;
    dataByte[1] = (uint8_t) 0;
    dataByte[2] = 0x01;
    dataByte[3] = 0x0b;
    dataByte[4] = 0x02;
    dataByte[5] = (uint8_t) 0;
    dataByte[6] = (uint8_t) 0;
    dataByte[7] = 0x07;
    dataByte[8] = 0x01;
    dataByte[9] = 0x05;
    dataByte[10] = 0x48;
    dataByte[11] = 0x65;
    dataByte[12] = 0x6c;
    dataByte[13] = 0x6c;
    dataByte[14] = 0x6f;
    dataByte[15] = 0x4c;
    uart_write_bytes(UART_NUM_1, (const char *) dataByte, 16);
    free(dataByte);
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        // sendData(TX_TASK_TAG, "Hello world");
        //char data[] = {0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64};
        //char data[] = {0x05, 0, 0x01, 0x0b, 0x02, 0, 0, 0x07, 0x01, 0x05, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x4c};
        //sendData(TX_TASK_TAG, data);

        //testingSend();
        //sendImage();

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

void app_main(void)
{
    init();
    init_camera();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
