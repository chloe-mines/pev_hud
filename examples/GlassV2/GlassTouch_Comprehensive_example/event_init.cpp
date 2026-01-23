
#include <Arduino.h>
#include "event_init.h"
#include <stdio.h>
#include "lvgl.h"
#include "WiFi.h"

void events_init_set_screen(lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_tileview_set_WiFi_sw, gui_set_wifi_cb, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_tileview_set_Switch_page_sw, gui_set_switch_page_cb, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_tileview_set_ESP_NOW_sw, gui_set_espnow_cb, LV_EVENT_ALL, ui);
}

void gui_set_wifi_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code)
    {
    case LV_EVENT_VALUE_CHANGED:
        sw_wifi_status = !sw_wifi_status;
        Serial.printf("sw_wifi_status: %d\n", sw_wifi_status);
        if (sw_wifi_status)
        {
            lv_obj_add_state(ui.screen_tileview_set_WiFi_sw, LV_STATE_CHECKED);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            WiFi.setAutoReconnect(true);
        }
        else
        {
            lv_obj_clear_state(ui.screen_tileview_set_WiFi_sw, LV_STATE_CHECKED);
            //Avoid taking up too much resources for callbacks when there is no wifi
            WiFi.disconnect();
            WiFi.setAutoReconnect(false);
        }
        break;

    default:
        break;
    }
}

void gui_set_switch_page_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code)
    {
    case LV_EVENT_VALUE_CHANGED:
        sw_page_status = !sw_page_status;
        Serial.printf("sw_page_status: %d\n", sw_page_status);
        if (sw_page_status)
        {
            lv_obj_add_state(ui.screen_tileview_set_Switch_page_sw, LV_STATE_CHECKED);
        }
        else
        {
            lv_obj_clear_state(ui.screen_tileview_set_Switch_page_sw, LV_STATE_CHECKED);
        }
        break;

    default:
        break;
    }
}

void gui_set_espnow_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code)
    {
    case LV_EVENT_VALUE_CHANGED:
        sw_espnow_status = !sw_espnow_status;
        Serial.printf("sw_espnow_status: %d\n", sw_espnow_status);
        if (sw_espnow_status)
        {
            lv_obj_add_state(ui.screen_tileview_set_ESP_NOW_sw, LV_STATE_CHECKED);
            esp_now_register_recv_cb(OnDataRecv);
        }
        else
        {
            lv_obj_clear_state(ui.screen_tileview_set_ESP_NOW_sw, LV_STATE_CHECKED);
            esp_now_unregister_recv_cb();
        }
        break;

    default:
        break;
    }
}