#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"
#include <CST816S.h>
CST816S touch(6, 7, 13, 5); // sda, scl, rst, irq
#include <SPI.h>

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
    delay(10);
}
