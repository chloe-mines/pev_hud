#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include <LilyGo_Wristband.h>
#include <LV_Helper.h>
#include "ui.h"
#include <WiFi.h>
#include <esp_sntp.h>
#include <MadgwickAHRS.h> //MadgwickAHRS from https://github.com/arduino-libraries/MadgwickAHRS
#include <cbuf.h>
#include <esp_now.h>
#include "EEPROM.h"

// The esp_vad function is currently only reserved for Arduino version 2.0.9
#if ESP_ARDUINO_VERSION_VAL(2, 0, 9) == ESP_ARDUINO_VERSION
#include <esp_vad.h>
vad_handle_t vad_inst;
int16_t *vad_buff = NULL;
uint32_t noise_count;
#else
#define TILEVIEW_CNT 5
#endif // Version check

// define
#define CFG_TIME_ZONE "CST-8" // TZ_Asia_Shanghai
#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.nist.gov"
#define WIFI_MSG_ID 0x1001

// object
LilyGo_Class amoled;
LilyGo_Button bootPin;
lv_ui ui;
Madgwick filter;

struct bhy2_dev bhy2;
struct_message esp_now_data;

// var
int8_t choose_direction = 0;
int8_t current_set_tileview = 0;
bool touch_press = false;
bool page_lock = false;
bool wifi_connect_status = false;
cbuf accel_data_buffer(1);
cbuf gyro_data_buffer(1);
cbuf mag_data_buffer(1);
ActivityBitMask activityArray[16] = {
    {0, "Still activity ended"},
    {1, "Walking activity ended"},
    {2, "Running activity ended"},
    {3, "On bicycle activity ended"},
    {4, "In vehicle activity ended"},
    {5, "Tilting activity ended"},
    {6, "In vehicle still ended"},
    {7, ""},
    {8, "Still activity started"},
    {9, "Walking activity started"},
    {10, "Running activity started"},
    {11, "On bicycle activity started"},
    {12, "IN vehicle activity started"},
    {13, "Tilting activity started"},
    {14, "In vehicle still started"},
    {15, ""}};

struct accel_data_xyz
{
    int16_t x;
    int16_t y;
    int16_t z;
};
struct gyro_data_xyz
{
    int16_t x;
    int16_t y;
    int16_t z;
};
struct accel_data_xyz accel_data;
struct gyro_data_xyz gyro_data;

// funtion
void button_event_callback(ButtonState state);
void touch_event_callback(ButtonState state);
static void timeavailable(struct timeval *t);
static void WiFiEvent(WiFiEvent_t event);
static void accel_process_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len);
static void gyro_process_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len);

void setup()
{
    bool res = false;
    Serial.begin(115200);
    delay(800);
    // while (!Serial)
    //     ;

    if (EEPROM.begin(10))
    {
        Serial.println("EEPROM begin");
    }

    if (EEPROM.read(0) == 1)
    {
        srceen_cont_pos_x = EEPROM.read(1);
        srceen_cont_pos_y = EEPROM.read(2);
        if (std::fabs(srceen_cont_pos_x) > 60)
            srceen_cont_pos_x = 0;
        if (std::fabs(srceen_cont_pos_y) > 330)
            srceen_cont_pos_y = 170;
    }
    else
    {
        Serial.println("eeprom not write");
    }

    res = amoled.begin();
    if (!res)
    {
        while (1)
        {
            Serial.println("The board model cannot be detected, please raise the Core Debug Level to an error");
            delay(1000);
        }
    }

    amoled.setRotation(0);
    amoled.setBrightness(255);
    amoled.setPins(BOARD_BHI_RST, BOARD_BHI_IRQ);

    bootPin.init(BOARD_BOOT_PIN);
    bootPin.setEventCallback(button_event_callback);

    amoled.setTouchThreshold(200);
    amoled.setEventCallback(touch_event_callback);

    if (amoled.initMicrophone())
    {
        Serial.println("Microphone init success!!!");
    }
    else
    {
        Serial.println("Microphone init fail!!!");
    }

    /* NOT USE */
    // uint8_t klio_example_pattern_id = 0;
    // uint8_t klio_example_pattern[] = {
    //     0x52, 0x42, 0x31, 0x06, 0x03, 0x8b, 0xff, 0x3c, 0x40, 0x0a, 0xd7, 0x23, 0x3c, 0x73, 0xfe, 0xa7, 0xc0, 0x38,
    //     0x44, 0xbd, 0x40, 0xbb, 0x1f, 0xe5, 0x3e, 0x38, 0x6f, 0x30, 0x3f, 0x50, 0x89, 0x74, 0x3f, 0x4d, 0x2a, 0xf8,
    //     0x3c, 0x45, 0x61, 0xd9, 0x40, 0x6d, 0x21, 0x7f, 0x40, 0xd0, 0x80, 0x8f, 0x3d, 0x9e, 0x39, 0x33, 0xbd, 0x51,
    //     0xc5, 0x0e, 0x3f, 0x64, 0x94, 0x80, 0x3c, 0xba, 0x90, 0xd2, 0x3e, 0xf8, 0xd8, 0x37, 0xbc, 0xed, 0x50, 0xea,
    //     0x3d, 0xf4, 0x61, 0x16, 0x3f, 0x75, 0xc9, 0x9b, 0xbe, 0x24, 0x20, 0xf7, 0x3c, 0x91, 0x16, 0x5b, 0xbd, 0x0f,
    //     0x61, 0x21, 0xbc, 0x23, 0xce, 0x80, 0x3e, 0x46, 0x8c, 0x93, 0x3d, 0x0c, 0x70, 0x16, 0x3e, 0x02, 0xf9, 0x9b,
    //     0x3a, 0x12, 0x48, 0xbc, 0x3d, 0x2e, 0x1f, 0xba, 0x3d, 0xe9, 0x82, 0xf5, 0xbe, 0xb4, 0xbd, 0xe8, 0x3d, 0xc6,
    //     0x79, 0x02, 0xbd, 0x8a, 0x1a, 0x00, 0x3b, 0x87, 0x22, 0x81, 0x3e, 0x96, 0x57, 0x05, 0x3e, 0xcb, 0x03, 0xcb,
    //     0xbf, 0x34, 0x2d, 0x93, 0x3e, 0x26, 0x6c, 0xff, 0xbd, 0x52, 0xb0, 0x84, 0x3b};

    // amoled.klio_get_parameter(KLIO_PARAM_RECOGNITION_MAX_PATTERNS);
    // amoled.klio_get_parameter(KLIO_PARAM_PATTERN_BLOB_SIZE);
    // bhy2_klio_sensor_state_t sensor_state = {.learning_enabled =1,.learning_reset = 1,.recognition_enabled = 1,.recognition_reset = 1};
    // amoled.klio_set_state(sensor_state);
    // amoled.klio_set_parameter(KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT);

    // Initialize Sensor
    accel_data_buffer.resize(sizeof(struct bhy2_data_xyz) * 2);
    gyro_data_buffer.resize(sizeof(struct bhy2_data_xyz) * 2);

    // initialize variables to pace updates to correct rate
    filter.begin(100);

    float sample_rate = 100.0;      /* Read out hintr_ctrl measured at 100Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */

    // Enable acceleration
    amoled.configure(SENSOR_ID_ACC_PASS, sample_rate, report_latency_ms);
    // Set the acceleration sensor result callback function
    amoled.onResultEvent(SENSOR_ID_ACC_PASS, accel_process_callback);

    // Enable gyroscope
    amoled.configure(SENSOR_ID_GYRO_PASS, sample_rate, report_latency_ms);
    // Set the gyroscope sensor result callback function
    amoled.onResultEvent(SENSOR_ID_GYRO_PASS, gyro_process_callback);

    // Results can only be displayed on the serial port after uncommenting
    amoled.configure(SENSOR_ID_AR, sample_rate, report_latency_ms);
    amoled.onResultEvent(SENSOR_ID_AR, activityArray_callback);

    amoled.configure(SENSOR_ID_STC, sample_rate, report_latency_ms);
    amoled.onResultEvent(SENSOR_ID_STC, stc_callback);

// The esp_vad function is currently only reserved for Arduino version 2.0.9
#if ESP_ARDUINO_VERSION_VAL(2, 0, 9) == ESP_ARDUINO_VERSION
    // Initialize esp-sr vad detected
#if ESP_IDF_VERSION_VAL(4, 4, 1) == ESP_IDF_VERSION
    vad_inst = vad_create(VAD_MODE_0, MIC_I2S_SAMPLE_RATE, VAD_FRAME_LENGTH_MS);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 1)
    vad_inst = vad_create(VAD_MODE_0);
#else
#error "No support this version."
#endif
    if (psramFound())
    {
        vad_buff = (int16_t *)ps_malloc(VAD_BUFFER_LENGTH * sizeof(short));
    }
    else
    {
        vad_buff = (int16_t *)malloc(VAD_BUFFER_LENGTH * sizeof(short));
    }
    if (vad_buff == NULL)
    {
        Serial.println("Memory allocation failed!");
    }
#else
    Serial.println("The currently used version does not support esp_vad and cannot run the noise detection function. If you need to use esp_vad, please change the version to 2.0.9");
#endif

    sntp_set_time_sync_notification_cb(timeavailable);
    configTzTime(CFG_TIME_ZONE, NTP_SERVER1, NTP_SERVER2);

    WiFi.mode(WIFI_MODE_STA);
    Serial.printf("WIFI address:%s\r\n", WiFi.macAddress().c_str());
    delay(500);
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiEvent); // Register WiFi event

    Serial.print("SSID:");
    Serial.println(WIFI_SSID);
    Serial.print("PASSWORD:");
    Serial.println(WIFI_PASSWORD);
    if (String(WIFI_SSID) == "Your WiFi SSID" || String(WIFI_PASSWORD) == "Your WiFi PASSWORD")
    {
        Serial.println("[Error] : WiFi ssid and password are not configured correctly");
        Serial.println("[Error] : WiFi ssid and password are not configured correctly");
        Serial.println("[Error] : WiFi ssid and password are not configured correctly");
    }
    else
    {
        if (sw_wifi_status)
        {
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            WiFi.setAutoReconnect(true);
        }
    }

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    beginLvglHelper(amoled, false);
    lv_gui_init(&ui);

    // --- Minimal HUD test: draw a single big "88" ---
    // Clear whatever the demo UI created
    lv_obj_clean(lv_scr_act());

    // Make sure the background is solid black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

    // Create a centered label
    lv_obj_t *hud_label = lv_label_create(lv_scr_act());
    lv_label_set_text(hud_label, "88");
    lv_obj_set_style_text_color(hud_label, lv_color_white(), 0);

    // Use the largest built-in Montserrat font available in this build
    #if LV_FONT_MONTSERRAT_48
    lv_obj_set_style_text_font(hud_label, &lv_font_montserrat_48, 0);
    #elif LV_FONT_MONTSERRAT_36
    lv_obj_set_style_text_font(hud_label, &lv_font_montserrat_36, 0);
    #elif LV_FONT_MONTSERRAT_28
    lv_obj_set_style_text_font(hud_label, &lv_font_montserrat_28, 0);
    #else
    lv_obj_set_style_text_font(hud_label, LV_FONT_DEFAULT, 0);
    #endif

    lv_obj_center(hud_label);
    // --- End minimal HUD test ---
}

void loop()
{
    bootPin.update();
    // Delay time to avoid conflicts between touch keys and physical keys
    delay(1);
    amoled.update();
    lv_timer_handler();
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&esp_now_data, incomingData, sizeof(esp_now_data));
    // Serial.print("Bytes received: ");
    // Serial.println(len);
    // Serial.print("Char: ");
    // Serial.println(esp_now_data.a);
    // Serial.print("Int: ");
    // Serial.println(esp_now_data.b);
    // Serial.print("Float: ");
    // Serial.println(esp_now_data.c);
    // Serial.print("Bool: ");
    // Serial.println(esp_now_data.d);
    // Serial.println();
}

static void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_READY:
        Serial.println("WiFi interface ready");
        break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
        Serial.println("Completed scan for access points");
        break;
    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("WiFi client started");
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
        Serial.println("WiFi clients stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        wifi_connect_status = true;
        Serial.println("Connected to access point");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        wifi_connect_status = false;
        Serial.println("Disconnected from WiFi access point");
        break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
        Serial.println("Authentication mode of access point has changed");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        Serial.println("Lost IP address and IP address is reset to 0");
        break;
    default:
        break;
    }
}

// Callback function (get's called when time adjusts via NTP)
static void timeavailable(struct timeval *t)
{
    Serial.println("Got time adjustment from NTP!");
    // Synchronize the synchronized time to the hardware RTC
    amoled.hwClockWrite();
}

void button_event_callback(ButtonState state)
{
    switch (state)
    {
    case BTN_PRESSED_EVENT:
        // Serial.println("BTN_PRESSED");
        break;
    case BTN_RELEASED_EVENT:
        // Serial.println("BTN_RELEASED");
        break;
    case BTN_CLICK_EVENT:
        if (page_lock == false)
        {
            srceen_last = srceen_current;
            srceen_current++;
            if (srceen_current > PAGE_NUM - 1)
            {
                srceen_current = 0;
            }
        }

        switch (srceen_current)
        {
        case 0:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            // Serial.println("srceen time");
            break;
        case 1:
            if (page_lock == false)
            {
                // Serial.println("srceen pos");
                lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            }
            else
            {
                choose_direction++;
                if (choose_direction > 3)
                {
                    choose_direction = 0;
                }
                lv_obj_set_tile_id(ui.screen_tileview_direction, 0, choose_direction, LV_ANIM_ON);
            }
            break;
        case 2:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            break;
        case 3:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            break;
        case 4:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            break;
        case 5:
            if (page_lock == false)
            {
                // Serial.println("srceen set");
                lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            }
            else
            {
                current_set_tileview++;
                if (current_set_tileview > 2)
                {
                    current_set_tileview = 0;
                }
                Serial.printf("current_set_tileview:%d\n", current_set_tileview);
                lv_obj_set_tile_id(ui.screen_tileview_set_tile, 0, current_set_tileview, LV_ANIM_ON);
            }
            break;
        case 6:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            break;
        case 7:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
            break;
        default:
            break;
        }

        Serial.println("BTN_CLICK");
        break;
    case BTN_LONG_PRESSED_EVENT:
        Serial.println("BTN_LONG");
        break;
    case BTN_DOUBLE_CLICK_EVENT:
        Serial.println("BTN_DOUBLE_CLICK");
        if (page_lock == false)
        {
            srceen_current = srceen_last;
            srceen_last = srceen_last - 1;
            if (srceen_last < 0)
            {
                srceen_last = PAGE_NUM - 1;
            }
        }

        switch (srceen_current)
        {
        case 0:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            // Serial.println("srceen time");
            break;
        case 1:
            if (page_lock == false)
            {
                // Serial.println("srceen pos");
                lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            }
            else
            {
                choose_direction--;
                if (choose_direction < 0)
                {
                    choose_direction = 3;
                }
                lv_obj_set_tile_id(ui.screen_tileview_direction, 0, choose_direction, LV_ANIM_ON);
            }
            break;
        case 2:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            break;
        case 3:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            break;
        case 4:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            break;
        case 5:
            if (page_lock == false)
            {
                // Serial.println("srceen set");
                lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            }
            else
            {
                current_set_tileview--;
                if (current_set_tileview < 0)
                {
                    current_set_tileview = 2;
                }
                lv_obj_set_tile_id(ui.screen_tileview_set_tile, 0, current_set_tileview, LV_ANIM_ON);
            }
            break;
        case 6:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            break;
        case 7:
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
            break;
        default:
            break;
        }
        break;
    case BTN_TRIPLE_CLICK_EVENT:
        Serial.println("BTN_TRIPLE_CLICK");
        break;

    default:
        break;
    }
}

void touch_event_callback(ButtonState state)
{
    // int static press_count = 0;
    switch (state)
    {
    case BTN_PRESSED_EVENT:
        // Serial.println("TOUCH_PRESSED");
        touch_press = true;
        break;
    case BTN_RELEASED_EVENT:
        // Serial.println("TOUCH_RELEASED");
        if (touch_press)
        {
            touch_press = false;
        }
        break;
    case BTN_CLICK_EVENT:
        switch (srceen_current)
        {
        case 0:
            // Serial.println("srceen time");
            break;
        case 1:
            // enter direction page
            // Serial.println("srceen postion");
            lv_scr_load_anim(ui.screen_direction, LV_SCR_LOAD_ANIM_MOVE_TOP, 200, 0, false);
            if (page_lock == true)
            {
                switch (choose_direction)
                {
                case 0:
                    srceen_cont_pos_y = srceen_cont_pos_y - 5;
                    break;
                case 1:
                    srceen_cont_pos_y = srceen_cont_pos_y + 5;
                    break;
                case 2:
                    srceen_cont_pos_x = srceen_cont_pos_x - 5;
                    break;
                case 3:
                    srceen_cont_pos_x = srceen_cont_pos_x + 5;
                    break;
                default:
                    break;
                }
                Serial.printf("X: %d\n", srceen_cont_pos_x);
                Serial.printf("Y: %d\n", srceen_cont_pos_y);
                EEPROM.write(0, 1);
                EEPROM.write(1, srceen_cont_pos_x);
                EEPROM.write(2, srceen_cont_pos_y);
                EEPROM.commit();
                lv_obj_set_pos(ui.screen_time_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_pos_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_direction_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_sensor_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_voltage_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_esp_now_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_set_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_tileview_set_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_wifi_rssi_cont, srceen_cont_pos_x, srceen_cont_pos_y);
                lv_obj_set_pos(ui.screen_mic_cont, srceen_cont_pos_x, srceen_cont_pos_y);

                lv_obj_update_layout(ui.screen_time_cont);
                lv_obj_update_layout(ui.screen_pos_cont);
                lv_obj_update_layout(ui.screen_direction_cont);
                lv_obj_update_layout(ui.screen_sensor_cont);
                lv_obj_update_layout(ui.screen_voltage_cont);
                lv_obj_update_layout(ui.screen_esp_now_cont);
                lv_obj_update_layout(ui.screen_set_cont);
                lv_obj_update_layout(ui.screen_tileview_set_cont);
                lv_obj_update_layout(ui.screen_wifi_rssi_cont);
                lv_obj_update_layout(ui.screen_mic_cont);
            }
            page_lock = true;

            break;

        case 5:
            lv_scr_load_anim(ui.screen_tileview_set, LV_SCR_LOAD_ANIM_MOVE_TOP, 200, 0, false);
            if (page_lock)
            {
                switch (current_set_tileview)
                {
                case 0:
                    // Serial.println("srceen set_tileview WIFI");
                    lv_event_send(ui.screen_tileview_set_WiFi_sw, LV_EVENT_VALUE_CHANGED, NULL);
                    break;

                case 1:
                    // Serial.println("srceen set_tileview ESP-NOW");
                    lv_event_send(ui.screen_tileview_set_ESP_NOW_sw, LV_EVENT_VALUE_CHANGED, NULL);
                    break;

                case 2:
                    // Serial.println("srceen set_tileview SWITCH PAGE");
                    lv_event_send(ui.screen_tileview_set_Switch_page_sw, LV_EVENT_VALUE_CHANGED, NULL);
                    break;

                default:
                    break;
                }
            }
            page_lock = true;
            break;

        default:
            break;
        }
        break;
    case BTN_LONG_PRESSED_EVENT:
        Serial.println("TOUCH_LONG");
        page_lock = false;
        if (srceen_current == 1)
        {
            // srceen_current = 1;
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 200, 0, false);
        }
        else if (srceen_current == 5)
        {
            // srceen_current = 5;
            lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 200, 0, false);
        }
        else
        {
            lv_scr_load_anim(scr_arry[0], LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 200, 0, false);
        }

        break;
    case BTN_DOUBLE_CLICK_EVENT:
        break;

    case BTN_TRIPLE_CLICK_EVENT:
        // Serial.println("TOUCH_TRIPLE_CLICK");
        break;

    default:
        break;
    }
}

static void accel_process_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len)
{
    struct bhy2_data_xyz data;
    bhy2_parse_xyz(data_ptr, &data);
    // float scaling_factor = get_sensor_default_scaling(sensor_id);
    // Serial.print(amoled.getSensorName(sensor_id));
    // Serial.print(":");
    // Serial.printf("x: %f, y: %f, z: %f;\r\n",
    //               data.x * scaling_factor,
    //               data.y * scaling_factor,
    //               data.z * scaling_factor
    // );
    accel_data_buffer.write((const char *)&data, sizeof(data));
}

static void gyro_process_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len)
{
    struct bhy2_data_xyz data;
    bhy2_parse_xyz(data_ptr, &data);
    // float scaling_factor = get_sensor_default_scaling(sensor_id);
    // Serial.print(amoled.getSensorName(sensor_id));
    // Serial.print(":");
    // Serial.printf("x: %f, y: %f, z: %f;\r\n",
    //               data.x * scaling_factor,
    //               data.y * scaling_factor,
    //               data.z * scaling_factor
    //              );
    gyro_data_buffer.write((const char *)&data, sizeof(data));
    mag_data_buffer.write((const char *)&data, sizeof(data));
}

static void activityArray_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len)
{
    uint16_t value;
    for (uint32_t i = 0; i < len; i += 2)
    {
        value = (data_ptr[i + 1] << 8) | data_ptr[i];
        // Serial.print(value, BIN);
        // Serial.println();
    }

    for (int i = 0; i < 16; i++)
    {
        uint16_t maskedVal = (value & (0x0001 << i)) >> i;
        if (maskedVal)
        {
            Serial.printf("activity:%s\n", activityArray[i].activityMessage.c_str());
            return;
        }
    }
}

static void stc_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len)
{
    uint16_t value;
    for (uint32_t i = 0; i < len; i++)
    {
        value = data_ptr[i];
        if (value != 0)
            Serial.printf("stc_count:%d\n", value);
    }
}