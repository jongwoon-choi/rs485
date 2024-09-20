#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"
#include <CST816S.h>
CST816S touch(6, 7, 13, 5); // sda, scl, rst, irq
#include <SPI.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdint.h>

 
typedef struct {
    uint8_t node_number;
    uint8_t temp1;
    uint8_t temp2;
    uint8_t power;
    uint8_t state1;
    uint8_t state2;
} NodeData;

 
// 글로벌 변수 선언
NodeData slave1_data;
NodeData slave2_data;
NodeData slave1_data_prev;
NodeData slave2_data_prev;

// 자신의 데이터 저장 변수
NodeData my_data;
NodeData my_data_prev;

// 함수 프로토타입 선언
void RS485_HandleReceivedPacket(uint8_t *data, uint16_t length);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length);
void RS485_SendDataPacket(uint8_t node_address, NodeData *data);
void uart_event_task(void *pvParameters);
 

// UART 설정
#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

// 핀 설정
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define RS485_DE_PIN (GPIO_NUM_21)

// UART 이벤트 큐 핸들러
static QueueHandle_t uart_queue;

 // 현재 화면 상태를 추적하는 변수
lv_obj_t *currentScreen;


#define ADC_PIN 16
 const int numReadings = 20; // Number of readings to average
int readings[numReadings]; // Array to store the readings
int currentIndex = 0; // Index of the current reading 
TFT_eSPI tft = TFT_eSPI();  // Create an instance of the library

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

//TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
lv_obj_t *btn;


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

void RS485_UART_Init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);

    // UART 핀 설정
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // UART 드라이버 설치
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);

    // DE 핀 설정
    gpio_set_direction(RS485_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_DE_PIN, 0); // 수신 모드로 초기화
}

void RS485_SetTransmitMode(void)
{
    gpio_set_level(RS485_DE_PIN, 1); // 송신 모드
}

void RS485_SetReceiveMode(void)
{
    gpio_set_level(RS485_DE_PIN, 0); // 수신 모드
}

void RS485_SendData(uint8_t *data, uint16_t length)
{
    RS485_SetTransmitMode();
    uart_write_bytes(UART_NUM, (const char *)data, length);
    uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));
    RS485_SetReceiveMode();
}

void RS485_ProcessReceivedData(void)
{
    uint8_t data[RD_BUF_SIZE];
    int len = uart_read_bytes(UART_NUM, data, RD_BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len > 0)
    {
        // 수신된 패킷 처리 함수 호출
        RS485_HandleReceivedPacket(data, len);
    }
}

void DataChangeDetection(void) {
    if (memcmp(&slave1_data, &slave1_data_prev, sizeof(NodeData)) != 0) {
        RS485_SendDataPacket(1, &slave1_data); // 인자 순서 수정
        memcpy(&slave1_data_prev, &slave1_data, sizeof(NodeData));
    }
    if (memcmp(&slave2_data, &slave2_data_prev, sizeof(NodeData)) != 0) {
        RS485_SendDataPacket(2, &slave2_data); // 인자 순서 수정
        memcpy(&slave2_data_prev, &slave2_data, sizeof(NodeData));
    }
}


// 함수 정의
void RS485_SendDataPacket(uint8_t node_address, NodeData *data) {
    uint8_t packet[20];
    uint8_t idx = 0;
    packet[idx++] = 0xAA; // 시작 바이트
    packet[idx++] = node_address;
    packet[idx++] = 0x01; // 명령 코드
    packet[idx++] = sizeof(NodeData);
    memcpy(&packet[idx], data, sizeof(NodeData));
    idx += sizeof(NodeData);
    packet[idx++] = CalculateChecksum(packet, idx);
    RS485_SendData(packet, idx);
}

uint8_t CalculateChecksum(uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

void RS485_HandleReceivedPacket(uint8_t *data, uint16_t length)
{
    if (length < 5)
    {
        // 최소 패킷 길이보다 짧으면 무시
        return;
    }

    uint8_t idx = 0;

    // 시작 바이트 확인
    if (data[idx++] != 0xAA)
    {
        // 시작 바이트가 아니면 무시
        return;
    }

    uint8_t node_address = data[idx++];
    uint8_t command = data[idx++];
    uint8_t data_length = data[idx++];

    // 데이터 길이 확인
    if (data_length + 5 > length)
    {
        // 데이터 길이가 맞지 않으면 무시
        return;
    }

    // 데이터 추출
    NodeData received_data;
    memcpy(&received_data, &data[idx], sizeof(NodeData));
    idx += sizeof(NodeData);

    // 체크섬 확인
    uint8_t received_checksum = data[idx++];
    uint8_t calculated_checksum = CalculateChecksum(data, idx - 1);

    if (received_checksum != calculated_checksum)
    {
        // 체크섬이 맞지 않으면 무시
        return;
    }

    // 수신된 노드 번호 확인
    if (received_data.node_number == 1)
    {
        // 슬레이브 1의 데이터 업데이트
        slave1_data = received_data;
    }
    else if (received_data.node_number == 2)
    {
        // 슬레이브 2의 데이터 업데이트
        slave2_data = received_data;
    }
}



/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}
 
 
/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
      if (!touch.available())
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = touch.data.x;
        data->point.y = touch.data.y;     
    }
}

 

void setup()
{
     
    touch.begin();

    Serial.begin(115200);
    
    pinMode(ADC_PIN, INPUT);
    pinMode(18, INPUT);
 
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( lcd_orientation ); /* Landscape orientation, flipped */

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    //lv_anim_del(ui_event_Spinner2, NULL);

    for (int i = 0; i < numReadings; i++) 
    {
    readings[i] = 0;
    }

    ui_init();

    Serial.println( "Setup done" );
 
      // 초기 화면 설정 (사용자가 선택한 화면을 설정할 수 있음)
    //lv_scr_load(ui_Screen1);
    currentScreen = ui_Screen1;  // 현재 화면을 ui_Screen1으로 설정
    
}

// 전송할 값 (ui_Label7의 값)
char value_from_screen3[3];  // 전역으로 선언

void update_labels() {
    // ui_Label7의 텍스트 값을 가져와 value_from_screen3에 저장
    strcpy(value_from_screen3, lv_label_get_text(ui_Label7));

    // ui_Label3과 ui_Label5에 ui_Label7의 값을 설정
    lv_label_set_text(ui_Label3, value_from_screen3);
    lv_label_set_text(ui_Label5, value_from_screen3);
}
 
void loop()
{
    lv_timer_handler(); /* let the GUI do its work */    

        memset(&my_data, 0, sizeof(NodeData));
    memset(&my_data_prev, 0, sizeof(NodeData));

    RS485_UART_Init();

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);


   
 // 현재 활성화된 화면 확인
    lv_obj_t *currentScreen = lv_scr_act();

    // 스크린3에서 결정된 값을 스크린4와 스크린5로 전달
    if (currentScreen == ui_Screen3) {
        // ui_Label7의 값을 ui_Label3과 ui_Label5로 전송
        update_labels();
    }


    // 현재 화면이 screen3가 아닌 경우에만 GPIO 상태를 확인
    if (currentScreen == ui_Screen4 || currentScreen == ui_Screen5) {
        int gpio18State = digitalRead(18);  // GPIO 18 상태 읽기

        // GPIO 18이 LOW일 때 ui_Screen5로 전환
        if (gpio18State == LOW && currentScreen != ui_Screen5) {
            lv_scr_load(ui_Screen5);  // ui_Screen5로 전환
            currentScreen = ui_Screen5;  // 현재 화면을 ui_Screen5로 갱신
        }

        // GPIO 18이 HIGH일 때 ui_Screen4로 전환
        else if (gpio18State == HIGH && currentScreen != ui_Screen4) {
            lv_scr_load(ui_Screen4);  // ui_Screen4로 전환
            currentScreen = ui_Screen4;  // 현재 화면을 ui_Screen4로 갱신
        }
    }

    
        DataChangeDetection();
        RS485_ProcessReceivedData();
        vTaskDelay(pdMS_TO_TICKS(100));
  
    delay(10);
}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *) malloc(RD_BUF_SIZE);
    for (;;)
    {
        if (xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            switch (event.type)
            {
                case UART_DATA:
                    uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                    // 수신된 데이터 처리
                    break;
                default:
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}