#include <Arduino.h>
#include "lvgl.h"
#include <stdio.h>
#include <LilyGo_Wristband.h>
#include "cbuf.h"
#include <MadgwickAHRS.h> //MadgwickAHRS from https://github.com/arduino-libraries/MadgwickAHRS
#include <WiFi.h>

#define PAGE_NUM (8)

#ifndef WIFI_SSID
#define WIFI_SSID "xinyuandianzi"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "AA15994823428"
#endif

#define VAD_FRAME_LENGTH_MS             30
#define VAD_BUFFER_LENGTH               (VAD_FRAME_LENGTH_MS * MIC_I2S_SAMPLE_RATE / 1000)

#pragma once
typedef struct
{
	lv_obj_t *screen;
	bool screen_del;
	lv_obj_t *screen_time_cont; //0
	lv_obj_t *screen_pos_cont;  //1
	lv_obj_t *screen_direction_cont;  //1.1
	lv_obj_t *screen_sensor_cont; //2
	lv_obj_t *screen_voltage_cont;//3
	lv_obj_t *screen_esp_now_cont;//4
	lv_obj_t *screen_set_cont;//5
	lv_obj_t *screen_tileview_set_cont;//5.1
	lv_obj_t *screen_wifi_rssi_cont;//6
	lv_obj_t *screen_mic_cont;//7

	lv_obj_t *screen_start;
	lv_obj_t *screen_start_cont;
	lv_obj_t *screen_start_lottie;

	lv_obj_t *screen_time;
	lv_obj_t *screen_cont_label;
	lv_obj_t *screen_label_time;
	lv_obj_t *screen_label_week;
	lv_obj_t *screen_label_data;

	lv_obj_t *screen_postion;
	lv_obj_t *screen_btn_movepos;

	lv_obj_t *screen_direction;
    lv_obj_t *screen_tileview_direction;
	lv_obj_t *screen_tileview_up;
	lv_obj_t *screen_tileview_down;
	lv_obj_t *screen_tileview_left;
	lv_obj_t *screen_tileview_right;
    lv_obj_t * set_direction_up_btn;
    lv_obj_t * set_direction_down_btn;
    lv_obj_t * set_direction_left_btn;
    lv_obj_t * set_direction_right_btn;

	lv_obj_t *screen_voltage;
	lv_obj_t *screen_voltage_bar;
	lv_obj_t *screen_voltage_label_vlotage;
	lv_obj_t *screen_voltage_label_num;
	lv_obj_t *screen_voltage_meter;
	lv_meter_scale_t *screen_voltage_meter_scale;
	lv_meter_indicator_t *screen_voltage_meter_scale_arc_gray;
	lv_meter_indicator_t *screen_voltage_meter_scale_arc_green;
	lv_obj_t *screen_voltage_label;

	lv_obj_t *screen_sensor;
    lv_obj_t *screen_sensor_label_info;

	lv_obj_t *screen_esp_now;
	lv_obj_t *screen_esp_now_label_title;
	lv_obj_t *screen_esp_now_label_info;

	lv_obj_t *screen_set;
	lv_obj_t *screen_set_btn;

    lv_obj_t *screen_tileview_set;
    lv_obj_t *screen_tileview_set_tile;
    lv_obj_t *screen_tileview_set_WiFi;
    lv_obj_t *screen_tileview_set_WiFi_sw;
    lv_obj_t *screen_tileview_set_WiFi_label;
    lv_obj_t *screen_tileview_set_ESP_NOW;
    lv_obj_t *screen_tileview_set_ESP_NOW_sw;
    lv_obj_t *screen_tileview_set_ESP_NOW_label;
    lv_obj_t *screen_tileview_set_Switch_page;
    lv_obj_t *screen_tileview_set_Switch_page_sw;
    lv_obj_t *screen_tileview_set_Switch_page_label;

	lv_obj_t *screen_wifi_rssi;
	lv_obj_t *screen_wifi_rssi_label;
	lv_obj_t *screen_wifi_rssi_num;
	lv_obj_t *screen_wifi_cont_Signal;
	lv_obj_t *screen_wifi_line4;
	lv_obj_t *screen_wifi_line3;
	lv_obj_t *screen_wifi_line2;
	lv_obj_t *screen_wifi_line1;
	
	lv_obj_t *screen_mic;
	lv_obj_t *screen_mic_label;
	lv_obj_t *screen_mic_count;


}lv_ui;

typedef struct struct_message
{
    char a[32];
    int b;
    float c;
    bool d;
} struct_message;

struct ActivityBitMask{
  uint8_t bit;
  String activityMessage;
};

enum {
    SCREEN0_ID = 0,
    SCREEN1_ID,
    SCREEN2_ID,
    SCREEN3_ID,
    SCREEN4_ID,
    SCREEN5_ID,
    SCREEN6_ID,
    SCREEN7_ID,
    SCREEN8_ID,
    SCREEN9_ID,
    SCREEN10_ID,
};

extern lv_ui ui;
extern int8_t choose_direction;
extern int16_t srceen_cont_pos_x;
extern int16_t srceen_cont_pos_y;
extern int8_t  srceen_current;
extern int8_t  srceen_last;
extern cbuf  accel_data_buffer;
extern cbuf  gyro_data_buffer;
extern cbuf  mag_data_buffer;
extern cbuf  ar_data_buffer;
extern Madgwick filter;
extern struct_message esp_now_data;
extern bool reset;
extern lv_obj_t* scr_arry[PAGE_NUM];
extern bool sw_wifi_status;
extern bool wifi_connect_status;
extern bool sw_espnow_status;
extern bool sw_page_status;

#if ESP_ARDUINO_VERSION_VAL(2, 0, 9) == ESP_ARDUINO_VERSION
#include <esp_vad.h>
extern int16_t *vad_buff;
extern uint32_t noise_count;
extern vad_handle_t vad_inst;
#endif // Version check

LV_IMG_DECLARE(_weitiao1_60x60);
LV_IMG_DECLARE(_up_60x60);
LV_IMG_DECLARE(_down_100x50);
LV_IMG_DECLARE(_left_100x50);
LV_IMG_DECLARE(_right_60x60);
LV_IMG_DECLARE(_set_60x60);

LV_FONT_DECLARE(lv_font_Acme_Regular_16);
LV_FONT_DECLARE(lv_font_Acme_Regular_20);
LV_FONT_DECLARE(lv_font_Acme_Regular_24);
LV_FONT_DECLARE(lv_font_Acme_Regular_30);
LV_FONT_DECLARE(lv_font_SourceHanSerifSC_Regular_16);
LV_FONT_DECLARE(lv_font_SourceHanSerifSC_Regular_14);

void lv_gui_init(lv_ui* ui);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
