#include "ui.h"
#include <Arduino.h>
#include "event_init.h"

// width:126
// hight:294
#define srceen_width 126
#define screen_hight 294
extern LilyGo_Class amoled;

int16_t srceen_cont_pos_x = 0;
int16_t srceen_cont_pos_y = 170; // 170
int8_t srceen_current = 0;       // main srceen
int8_t srceen_last = 0;          // main srceen
const char *week_char[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char *month_char[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};
lv_timer_t *datetime_timer;
int imu_time_count = 0;
bool change_page = false;
bool sw_wifi_status = true;
bool sw_espnow_status = false;
bool sw_page_status = true;

float last_roll = -181;
float last_pitch = 91;
float last_yaw = -1;
lv_obj_t *scr_arry[PAGE_NUM];

void lv_set_direction_btn(lv_obj_t *btn, const void *image, int size_X, int size_Y, int pos_X, int pos_Y);
void screen_time_create();
void screen_postion_create(lv_obj_t *parent);
static void update_datetime(lv_timer_t *e);
void imu_timer_cb(lv_timer_t *t);
void timer_cb(lv_timer_t *t);
bool process_data(float data, int stabilityRange, std::vector<float> &output_data);
int imu_process_direction(float roll, float pitch, float yaw);

void gui_time_init(lv_ui *ui);
void gui_pos_init(lv_ui *ui);
void gui_direction_tileview_init(lv_ui *ui);
void gui_sensor_init(lv_ui *ui);
void gui_vlotage_init(lv_ui *ui);
void gui_esp_now_init(lv_ui *ui);
void gui_set_init(lv_ui *ui);
void gui_set_tileview_init(lv_ui *ui);
void lv_set_tileview_add(lv_obj_t *sw, lv_obj_t *label, const char *text, uint16_t sw_status);
void gui_wifi_rssi_init(lv_ui *ui);
void gui_mic_init(lv_ui *ui);

struct menu_icon
{
    const void *icon_src;
    const char *icon_str;
    int id;
};

void lv_gui_init(lv_ui *ui)
{
    Serial.println("lv_gui_init");
    Serial.printf("srceen_pos_y %d\n", srceen_cont_pos_y);
    Serial.printf("srceen_pos_x %d\n", srceen_cont_pos_x);

    lv_timer_create(imu_timer_cb, 20, NULL);
    lv_timer_create(timer_cb, 20, NULL);

    gui_time_init(ui);
    gui_pos_init(ui);
    gui_direction_tileview_init(ui);
    gui_sensor_init(ui);
    gui_vlotage_init(ui);
    gui_esp_now_init(ui);
    gui_set_init(ui);
    gui_set_tileview_init(ui);
    gui_wifi_rssi_init(ui);
    gui_mic_init(ui);

    events_init_set_screen(ui);
    srceen_current = 0;
}

void timer_cb(lv_timer_t *t) // 20ms
{
    int static time_count = 0;
    int static voltage_count = 500;
    int static esp_now_count = 0;
    voltage_count++;
    esp_now_count++;
    if (change_page)
    {
        time_count++;
        if (time_count > 100)
        {
            change_page = false;
            time_count = 0;
        }
    }
    if (voltage_count >= 500)
    {
        voltage_count = 0;
        // uint16_t battery_voltage = amoled.getBattVoltage();
        int percentage = amoled.getBatteryPercent();
        // Serial.printf("battery_voltage %d\n", battery_voltage);
        // Serial.printf("percentage %d\n", percentage);
        if (percentage <= 100 && percentage >= 0)
        {
            lv_label_set_text_fmt(ui.screen_voltage_label_num, "%d%%", percentage);
            lv_label_set_text_fmt(ui.screen_voltage_label, "%d%%", percentage);
            lv_slider_set_value(ui.screen_voltage_bar, percentage, LV_ANIM_ON);
            lv_meter_set_indicator_end_value(ui.screen_voltage_meter, ui.screen_voltage_meter_scale_arc_green, percentage);
        }
    }

    if (esp_now_count >= 50)
    {
        esp_now_count = 0;
        lv_label_set_text_fmt(ui.screen_esp_now_label_info, "Int:%d\nFloat:%.2f\nBool:%s", esp_now_data.b,
                              esp_now_data.c,
                              esp_now_data.d ? "true" : "false");
        if (wifi_connect_status)
        {
            int rssi = WiFi.RSSI();
            lv_label_set_text_fmt(ui.screen_wifi_rssi_num, "Rssi:%d", rssi);
            if (rssi >= -20)
            {
                lv_obj_set_style_line_color(ui.screen_wifi_line1, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line2, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line3, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line4, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (rssi >= -40)
            {
                lv_obj_set_style_line_color(ui.screen_wifi_line1, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line2, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line3, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line4, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (rssi >= -80)
            {
                lv_obj_set_style_line_color(ui.screen_wifi_line1, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line2, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line3, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line4, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (rssi >= -100)
            {
                lv_obj_set_style_line_color(ui.screen_wifi_line1, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line2, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line3, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line4, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (rssi < -100)
            {
                lv_obj_set_style_line_color(ui.screen_wifi_line1, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line2, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line3, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_line_color(ui.screen_wifi_line4, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
        }
        else
        {
            lv_obj_set_style_line_color(ui.screen_wifi_line1, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_line_color(ui.screen_wifi_line2, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_line_color(ui.screen_wifi_line3, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_line_color(ui.screen_wifi_line4, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

void imu_timer_cb(lv_timer_t *t) // 20ms
{
    imu_time_count++;
    float roll, pitch, yaw;
    bool static left_first = false, right_first = false;
    // Read data from buffer
    struct bhy2_data_xyz gyr;
    gyro_data_buffer.read((char *)&gyr, sizeof(gyr));
    float gyro_scaling_factor = get_sensor_default_scaling(BHY2_SENSOR_ID_GYRO_PASS);

    struct bhy2_data_xyz acc;
    accel_data_buffer.read((char *)&acc, sizeof(acc));
    float accel_scaling_factor = get_sensor_default_scaling(BHY2_SENSOR_ID_ACC_PASS);

    struct bhy2_data_xyz mag;
    mag_data_buffer.read((char *)&mag, sizeof(mag));
    float mag_scaling_factor = get_sensor_default_scaling(BHY2_SENSOR_ID_MAG_PASS);

    // update the filter, which computes orientation
    filter.update(gyr.x * gyro_scaling_factor,
                  gyr.y * gyro_scaling_factor,
                  gyr.z * gyro_scaling_factor,
                  acc.x * accel_scaling_factor,
                  acc.y * accel_scaling_factor,
                  acc.z * accel_scaling_factor,
                  mag.x * mag_scaling_factor,
                  mag.y * mag_scaling_factor,
                  mag.z * mag_scaling_factor);

    // get the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    // roll  绕roll轴 逆时针增加(左) 顺时针减少（右）[-180：180]
    // pitch 绕pitch轴 逆时针增加 顺时针减少 超过90°继续旋转角度变小 超过-90°继续旋转角度变大（-90：90）
    // yaw   绕yaw轴 逆时针增加 顺时针减少（0：360）
    if (change_page == false && sw_page_status == true) // 20*5 = 100ms
    {
        uint8_t static right_rotate = 0, left_rotate = 0;
        if (((pitch > -85 && pitch < -50) || (pitch < 85 && pitch > 50))) // 防止万向锁、确定在一个范围内
        {
            if ((std::fabs((int)last_roll - (int)roll) > 2) && (std::fabs(last_yaw - yaw) <= 9) && (std::fabs(last_yaw - yaw) >= 5))
            {
                if (last_roll < roll)
                {
                    left_rotate++;
                    if (left_rotate >= 2)
                        right_rotate = 0;
                    // Serial.printf("last_roll: %3f roll: %3f\n", last_roll, roll);
                    // Serial.printf("last_yaw: %3f yaw: %3f\n", last_yaw, yaw);
                    // Serial.printf("right_rotate: %d left_rotate: %d\n", right_rotate, left_rotate);
                }
                if (last_roll > roll)
                {
                    right_rotate++;
                    if (right_rotate >= 2)
                        left_rotate = 0;
                    // Serial.printf("last_roll: %3f roll: %3f\n", last_roll, roll);
                    // Serial.printf("last_yaw: %3f yaw: %3f\n", last_yaw, yaw);
                    // Serial.printf("right_rotate: %d left_rotate: %d\n", right_rotate, left_rotate);
                }
            }

            if (!left_first && !right_first)
            {
                if (right_rotate >= 3)
                {
                    right_first = true;
                    left_rotate = 0;
                    right_rotate = 0;
                }
                else if (left_rotate >= 3)
                {
                    left_first = true;
                    right_rotate = 0;
                    left_rotate = 0;
                }
                // Serial.printf("left_first: %d right_first: %d\n", left_first, right_first);
            }

            if (left_first) // 先左后右  判定左转
            {
                if (right_rotate >= 2)
                {
                    left_first = false;
                    right_first = false;
                    Serial.println("left_rotate");
                    change_page = true;
                    srceen_current = srceen_last;
                    srceen_last = srceen_last - 1;
                    if (srceen_last < 0)
                    {
                        srceen_last = PAGE_NUM - 1;
                    }
                    lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_RIGHT, 350, 0, false);
                }
            }
            else if (right_first) // 先右后左  判定右转
            {
                if (left_rotate >= 2)
                {
                    left_first = false;
                    right_first = false;
                    change_page = true;
                    Serial.println("right_rotate");
                    srceen_last = srceen_current;
                    srceen_current++;
                    if (srceen_current > PAGE_NUM - 1)
                    {
                        srceen_current = 0;
                    }
                    lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_MOVE_LEFT, 350, 0, false);
                }
            }
        }
        // else
        // {
        //     Serial.println("pitch out of measurement range");
        // }

        if (imu_time_count >= 100 && (std::fabs((int)last_roll - (int)roll) < 2)) // 2s  定时清除
        {
            imu_time_count = 0;
            left_rotate = 0;
            right_rotate = 0;
            left_first = false;
            right_first = false;
        }

        if (imu_time_count % 10 == 0) // 200ms保存上一次的数据
        {
            last_roll = roll;
            last_yaw = yaw;
        }

        // Serial.printf("roll: %6f yaw: %6f \n", roll, yaw);
    }
    lv_label_set_text_fmt(ui.screen_sensor_label_info, "Roll:% 3.2f\nPitch:% 3.2f\nYaw:% 3.2f\n", roll, pitch, yaw);
}

//************************************[ screen 0 ]****************************************** time
void gui_time_init(lv_ui *ui)
{
    ui->screen_time = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_time, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_time, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_time, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_time, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_time_srceen_cont
    ui->screen_time_cont = lv_obj_create(ui->screen_time);
    lv_obj_set_pos(ui->screen_time_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_time_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_time_cont, LV_SCROLLBAR_MODE_OFF);

    // Write style for screen_time_cont, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_time_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_time_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_time_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_time_cont, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_time_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_time_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_time_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_time_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_time_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_time_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // 绘制页面
    screen_time_create();
    scr_arry[srceen_current] = ui->screen_time;
    lv_scr_load_anim(scr_arry[srceen_current], LV_SCR_LOAD_ANIM_NONE, 200, 0, false);
    srceen_current++;
}

static void update_datetime(lv_timer_t *e)
{
    struct tm timeinfo;
    /*
    You can use localtime_r to directly read the system time.
    time_t now;
    time(&now);
    localtime_r(&now, &timeinfo);
    */
    // Here need to test the hardware time, so get the time in the RTC PCF85063
    amoled.getDateTime(&timeinfo);

    lv_label_set_text_fmt(ui.screen_label_time, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    lv_label_set_text_fmt(ui.screen_label_week, "%s", week_char[timeinfo.tm_wday]);
    lv_label_set_text_fmt(ui.screen_label_data, "%02d.%02d", (timeinfo.tm_mon + 1), timeinfo.tm_mday);
}

void screen_time_create()
{
    ui.screen_label_week = lv_label_create(ui.screen_time_cont);
    lv_label_set_text(ui.screen_label_week, "---");
    lv_label_set_long_mode(ui.screen_label_week, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui.screen_label_week, 0, 20);
    lv_obj_set_size(ui.screen_label_week, 50, 20);

    // Write style for screen_label_week, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_scrollbar_mode(ui.screen_label_week, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_border_width(ui.screen_label_week, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui.screen_label_week, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui.screen_label_week, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui.screen_label_week, &lv_font_SourceHanSerifSC_Regular_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui.screen_label_week, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui.screen_label_week, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui.screen_label_week, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui.screen_label_week, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui.screen_label_week, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_label_data
    ui.screen_label_data = lv_label_create(ui.screen_time_cont);
    lv_label_set_text(ui.screen_label_data, "--.--");
    lv_label_set_long_mode(ui.screen_label_data, LV_LABEL_LONG_WRAP);
    lv_obj_set_size(ui.screen_label_data, 66, 20);
    lv_obj_set_pos(ui.screen_label_data, srceen_width - 66, 20);

    // Write style for screen_label_data, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui.screen_label_data, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui.screen_label_data, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui.screen_label_data, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui.screen_label_data, &lv_font_SourceHanSerifSC_Regular_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui.screen_label_data, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui.screen_label_data, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui.screen_label_data, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui.screen_label_data, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui.screen_label_data, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_label_time
    ui.screen_label_time = lv_label_create(ui.screen_time_cont);
    lv_label_set_text(ui.screen_label_time, "--:--");
    lv_label_set_long_mode(ui.screen_label_time, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui.screen_label_time, 23, 50);
    lv_obj_set_size(ui.screen_label_time, 80, 30);

    // Write style for screen_label_time, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui.screen_label_time, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui.screen_label_time, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui.screen_label_time, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui.screen_label_time, &lv_font_Acme_Regular_30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui.screen_label_time, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui.screen_label_time, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui.screen_label_time, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui.screen_label_time, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui.screen_label_time, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    datetime_timer = lv_timer_create(update_datetime, 500, NULL);
    lv_timer_ready(datetime_timer);
}
//************************************[ screen 1 ]****************************************** set direction
void gui_pos_init(lv_ui *ui)
{
    ui->screen_postion = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_postion, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_postion, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_postion, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_postion, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui->screen_pos_cont = lv_obj_create(ui->screen_postion);
    // Write codes screen_pos_cont
    ui->screen_pos_cont = lv_obj_create(ui->screen_postion);
    lv_obj_set_pos(ui->screen_pos_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_pos_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_pos_cont, LV_SCROLLBAR_MODE_OFF);

    // Write style for screen_pos_cont, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_pos_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pos_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pos_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pos_cont, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pos_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pos_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pos_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pos_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pos_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pos_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    screen_postion_create(ui->screen_pos_cont);
    scr_arry[srceen_current++] = ui->screen_postion;
}

void screen_postion_create(lv_obj_t *parent)
{
    ui.screen_btn_movepos = lv_btn_create(parent);
    lv_obj_set_pos(ui.screen_btn_movepos, 33, 20);
    lv_obj_set_size(ui.screen_btn_movepos, 60, 60);
    lv_obj_set_style_bg_color(ui.screen_btn_movepos, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui.screen_btn_movepos, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui.screen_btn_movepos, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui.screen_btn_movepos, &_weitiao1_60x60, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *text_null = lv_label_create(ui.screen_btn_movepos);
    lv_label_set_text(text_null, "");
}

//************************************[ screen 1.1 ]****************************************** direction
void gui_direction_tileview_init(lv_ui *ui)
{
    ui->screen_direction = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_direction, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_direction, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_direction, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_direction, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_direction_cont = lv_obj_create(ui->screen_direction);
    lv_obj_set_pos(ui->screen_direction_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_direction_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_direction_cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(ui->screen_direction_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_direction_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_direction_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_direction_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_direction_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_direction_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_direction_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_direction_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_direction_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_direction_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_tileview_direction = lv_tileview_create(ui->screen_direction_cont);
    lv_obj_set_size(ui->screen_tileview_direction, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(ui->screen_tileview_direction, lv_color_black(), LV_PART_MAIN);
    ui->screen_tileview_up = lv_tileview_add_tile(ui->screen_tileview_direction, 0, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    ui->screen_tileview_down = lv_tileview_add_tile(ui->screen_tileview_direction, 0, 1, LV_DIR_HOR | LV_DIR_BOTTOM);
    ui->screen_tileview_left = lv_tileview_add_tile(ui->screen_tileview_direction, 0, 2, LV_DIR_HOR | LV_DIR_BOTTOM);
    ui->screen_tileview_right = lv_tileview_add_tile(ui->screen_tileview_direction, 0, 3, LV_DIR_HOR | LV_DIR_BOTTOM);

    ui->set_direction_up_btn = lv_btn_create(ui->screen_tileview_up);
    ui->set_direction_down_btn = lv_btn_create(ui->screen_tileview_down);
    ui->set_direction_left_btn = lv_btn_create(ui->screen_tileview_left);
    ui->set_direction_right_btn = lv_btn_create(ui->screen_tileview_right);

    lv_set_direction_btn(ui->set_direction_up_btn, &_up_60x60, 60, 60, 0, 0);
    lv_set_direction_btn(ui->set_direction_down_btn, &_down_100x50, 60, 60, 0, 0);
    lv_set_direction_btn(ui->set_direction_left_btn, &_left_100x50, 60, 60, 0, 0);
    lv_set_direction_btn(ui->set_direction_right_btn, &_right_60x60, 60, 60, 0, 0);
}

void lv_set_direction_btn(lv_obj_t *btn, const void *image, int size_X, int size_Y, int pos_X, int pos_Y)
{
    lv_obj_set_style_bg_color(btn, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_size(btn, size_X, size_Y);
    // lv_obj_set_pos(btn, pos_X, pos_Y);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *null_str = lv_label_create(btn);
    lv_label_set_text(null_str, "");
    lv_obj_set_style_bg_img_src(btn, image, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_opa(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(btn, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(btn, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
}

//************************************[ screen 2 ]****************************************** sensor
void gui_sensor_init(lv_ui *ui)
{
    ui->screen_sensor = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_sensor, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_sensor, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_sensor, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_sensor, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_sensor_cont = lv_obj_create(ui->screen_sensor);
    lv_obj_set_pos(ui->screen_sensor_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_sensor_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_sensor_cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(ui->screen_sensor_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_sensor_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_sensor_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_sensor_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_sensor_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_sensor_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_sensor_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_sensor_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_sensor_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_sensor_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_sensor_label_info = lv_label_create(ui->screen_sensor_cont);
    lv_obj_set_pos(ui->screen_sensor_label_info, 13, 20);
    lv_obj_set_size(ui->screen_sensor_label_info, 100, 90);
    lv_label_set_long_mode(ui->screen_sensor_label_info, LV_LABEL_LONG_WRAP);

    // Write style for screen_sensor_label_info, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_sensor_label_info, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_sensor_label_info, &lv_font_SourceHanSerifSC_Regular_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_sensor_label_info, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_sensor_label_info, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_sensor_label_info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_sensor_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    scr_arry[srceen_current++] = ui->screen_sensor;
}

//************************************[ screen 3 ]****************************************** voltage
void gui_vlotage_init(lv_ui *ui)
{
    ui->screen_voltage = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_voltage, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_voltage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_voltage, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_voltage, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_voltage_bar
    ui->screen_voltage_bar = lv_bar_create(ui->screen_voltage);
    lv_obj_set_style_anim_time(ui->screen_voltage_bar, 1000, 0);
    lv_bar_set_mode(ui->screen_voltage_bar, LV_BAR_MODE_NORMAL);
    lv_bar_set_range(ui->screen_voltage_bar, 0, 100);
    lv_bar_set_value(ui->screen_voltage_bar, 0, LV_ANIM_OFF);
    lv_obj_set_pos(ui->screen_voltage_bar, 13, 19);
    lv_obj_set_size(ui->screen_voltage_bar, 100, 35);

    // Write style for screen_voltage_bar, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_voltage_bar, 115, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_voltage_bar, lv_color_hex(0x00f27f), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_voltage_bar, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui->screen_voltage_bar, lv_color_hex(0xc5c5c5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui->screen_voltage_bar, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui->screen_voltage_bar, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_voltage_bar, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_voltage_bar, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write style for screen_voltage_bar, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_voltage_bar, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_voltage_bar, lv_color_hex(0x00ffaf), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_voltage_bar, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_voltage_bar, 30, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    // Write codes screen_voltage_label_vlotage
    ui->screen_voltage_label_vlotage = lv_label_create(ui->screen_voltage);
    lv_label_set_text(ui->screen_voltage_label_vlotage, "vlotage");
    lv_label_set_long_mode(ui->screen_voltage_label_vlotage, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_voltage_label_vlotage, 13, 70);
    lv_obj_set_size(ui->screen_voltage_label_vlotage, 100, 30);

    // Write style for screen_voltage_label_vlotage, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_voltage_label_vlotage, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_voltage_label_vlotage, &lv_font_Acme_Regular_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_voltage_label_vlotage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_voltage_label_vlotage, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_voltage_label_vlotage, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_voltage_label_vlotage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_voltage_label_num
    ui->screen_voltage_label_num = lv_label_create(ui->screen_voltage);
    lv_label_set_text(ui->screen_voltage_label_num, "0%");
    lv_label_set_long_mode(ui->screen_voltage_label_num, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_voltage_label_num, 13, 90);
    lv_obj_set_size(ui->screen_voltage_label_num, 100, 30);

    // Write style for screen_voltage_label_num, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_voltage_label_num, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_voltage_label_num, &lv_font_Acme_Regular_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_voltage_label_num, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_voltage_label_num, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_voltage_label_num, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_voltage_label_num, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // glass part
    ui->screen_voltage_cont = lv_obj_create(ui->screen_voltage);
    lv_obj_set_pos(ui->screen_voltage_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_voltage_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_voltage_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_voltage_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_voltage_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_voltage_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_voltage_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_voltage_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_voltage_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_voltage_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_voltage_meter
    ui->screen_voltage_meter = lv_meter_create(ui->screen_voltage_cont);
    // add scale ui->screen_voltage_meter_scale
    ui->screen_voltage_meter_scale = lv_meter_add_scale(ui->screen_voltage_meter);
    lv_meter_set_scale_ticks(ui->screen_voltage_meter, ui->screen_voltage_meter_scale, 4, 1, 10, lv_color_hex(0x000000));
    lv_meter_set_scale_range(ui->screen_voltage_meter, ui->screen_voltage_meter_scale, 0, 100, 360, 270);

    // add arc for ui->screen_voltage_meter_scale
    ui->screen_voltage_meter_scale_arc_gray = lv_meter_add_arc(ui->screen_voltage_meter, ui->screen_voltage_meter_scale, 12, lv_color_hex(0xc5c5c5), 10);
    lv_meter_set_indicator_start_value(ui->screen_voltage_meter, ui->screen_voltage_meter_scale_arc_gray, 0);
    lv_meter_set_indicator_end_value(ui->screen_voltage_meter, ui->screen_voltage_meter_scale_arc_gray, 100);

    // add arc for ui->screen_voltage_meter_scale
    ui->screen_voltage_meter_scale_arc_green = lv_meter_add_arc(ui->screen_voltage_meter, ui->screen_voltage_meter_scale, 12, lv_color_hex(0x00ffaf), 10);
    lv_meter_set_indicator_start_value(ui->screen_voltage_meter, ui->screen_voltage_meter_scale_arc_green, 0);
    lv_meter_set_indicator_end_value(ui->screen_voltage_meter, ui->screen_voltage_meter_scale_arc_green, 0);
    lv_obj_set_pos(ui->screen_voltage_meter, 13, 8);
    lv_obj_set_size(ui->screen_voltage_meter, 100, 100);

    // Write style for screen_voltage_meter, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_voltage_meter, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_voltage_meter, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_voltage_meter, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_voltage_meter, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_voltage_meter, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_voltage_meter, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_voltage_meter, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_voltage_meter, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write style for screen_voltage_meter, Part: LV_PART_TICKS, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_voltage_meter, lv_color_hex(0xff0000), LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_voltage_meter, 255, LV_PART_TICKS | LV_STATE_DEFAULT);

    // Write style for screen_voltage_meter, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_voltage_meter, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_voltage_meter, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_voltage_meter, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    // Write codes screen_voltage_label_1
    ui->screen_voltage_label = lv_label_create(ui->screen_voltage_cont);
    lv_label_set_text(ui->screen_voltage_label, "0%");
    lv_label_set_long_mode(ui->screen_voltage_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_voltage_label, 43, 50);
    lv_obj_set_size(ui->screen_voltage_label, 40, 20);

    // Write style for screen_voltage_label, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_voltage_label, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_voltage_label, &lv_font_Acme_Regular_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_voltage_label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_voltage_label, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_voltage_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_voltage_label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    scr_arry[srceen_current++] = ui->screen_voltage;

    // lv_scr_load(ui->screen_voltage);
    // srceen_current = 4;
}

//************************************[ screen 4 ]****************************************** esp_now
void gui_esp_now_init(lv_ui *ui)
{
    ui->screen_esp_now = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_esp_now, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_esp_now, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_esp_now, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_esp_now, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_esp_now_cont = lv_obj_create(ui->screen_esp_now);
    lv_obj_set_pos(ui->screen_esp_now_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_esp_now_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_esp_now_cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(ui->screen_esp_now_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_esp_now_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_esp_now_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_esp_now_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_esp_now_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_esp_now_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_esp_now_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_esp_now_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_esp_now_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_esp_now_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_esp_now_label_title = lv_label_create(ui->screen_esp_now_cont);
    lv_obj_set_pos(ui->screen_esp_now_label_title, 13, 10);
    lv_obj_set_size(ui->screen_esp_now_label_title, 100, 25);
    lv_label_set_text(ui->screen_esp_now_label_title, "ESP NOW");
    lv_label_set_long_mode(ui->screen_esp_now_label_title, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_bg_color(ui->screen_esp_now_label_title, lv_color_black(), LV_PART_MAIN);

    lv_obj_set_style_border_width(ui->screen_esp_now_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_esp_now_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_esp_now_label_title, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_esp_now_label_title, &lv_font_Acme_Regular_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_esp_now_label_title, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_esp_now_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_esp_now_label_title, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_esp_now_label_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_esp_now_label_info = lv_label_create(ui->screen_esp_now_cont);
    lv_obj_set_pos(ui->screen_esp_now_label_info, 3, 40);
    lv_obj_set_size(ui->screen_esp_now_label_info, 120, 120);
    lv_label_set_text(ui->screen_esp_now_label_info, "****");
    lv_label_set_long_mode(ui->screen_esp_now_label_info, LV_LABEL_LONG_WRAP);

    lv_obj_set_style_border_width(ui->screen_esp_now_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_esp_now_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_esp_now_label_info, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_esp_now_label_info, &lv_font_SourceHanSerifSC_Regular_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_esp_now_label_info, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_esp_now_label_info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_esp_now_label_info, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_esp_now_label_info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_esp_now_label_info, lv_color_black(), LV_PART_MAIN);

    scr_arry[srceen_current++] = ui->screen_esp_now;
}

//************************************[ screen 5 ]****************************************** set
void gui_set_init(lv_ui *ui)
{
    ui->screen_set = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_set, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_set, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_set, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_set, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_set_cont = lv_obj_create(ui->screen_set);
    lv_obj_set_pos(ui->screen_set_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_set_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_set_cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(ui->screen_set_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_set_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_set_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_set_btn = lv_btn_create(ui->screen_set_cont);
    lv_obj_set_pos(ui->screen_set_btn, 33, 30);
    lv_obj_set_size(ui->screen_set_btn, 60, 60);
    lv_obj_set_style_bg_color(ui->screen_set_btn, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_set_btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_set_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui->screen_set_btn, &_set_60x60, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *text_null = lv_label_create(ui->screen_set_btn);
    lv_label_set_text(text_null, "");
    scr_arry[srceen_current++] = ui->screen_set;
}

//************************************[ screen 5.1 ]****************************************** set_tileview
void gui_set_tileview_init(lv_ui *ui)
{
    ui->screen_tileview_set = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_tileview_set, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_tileview_set, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_tileview_set, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_tileview_set, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_tileview_set_cont = lv_obj_create(ui->screen_tileview_set);
    lv_obj_set_pos(ui->screen_tileview_set_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_tileview_set_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_tileview_set_cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(ui->screen_tileview_set_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_tileview_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_tileview_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_tileview_set_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_tileview_set_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_tileview_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_tileview_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_tileview_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_tileview_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_tileview_set_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_tileview_set_tile = lv_tileview_create(ui->screen_tileview_set_cont);
    lv_obj_set_size(ui->screen_tileview_set_tile, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(ui->screen_tileview_set_tile, lv_color_black(), LV_PART_MAIN);

    // add tile
    ui->screen_tileview_set_WiFi = lv_tileview_add_tile(ui->screen_tileview_set_tile, 0, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    ui->screen_tileview_set_ESP_NOW = lv_tileview_add_tile(ui->screen_tileview_set_tile, 0, 1, LV_DIR_HOR | LV_DIR_BOTTOM);
    ui->screen_tileview_set_Switch_page = lv_tileview_add_tile(ui->screen_tileview_set_tile, 0, 2, LV_DIR_HOR | LV_DIR_BOTTOM);

    ui->screen_tileview_set_WiFi_sw = lv_switch_create(ui->screen_tileview_set_WiFi);
    ui->screen_tileview_set_ESP_NOW_sw = lv_switch_create(ui->screen_tileview_set_ESP_NOW);
    ui->screen_tileview_set_Switch_page_sw = lv_switch_create(ui->screen_tileview_set_Switch_page);

    ui->screen_tileview_set_WiFi_label = lv_label_create(ui->screen_tileview_set_WiFi);
    ui->screen_tileview_set_ESP_NOW_label = lv_label_create(ui->screen_tileview_set_ESP_NOW);
    ui->screen_tileview_set_Switch_page_label = lv_label_create(ui->screen_tileview_set_Switch_page);

    lv_set_tileview_add(ui->screen_tileview_set_WiFi_sw, ui->screen_tileview_set_WiFi_label, "WiFi", LV_STATE_CHECKED);
    lv_set_tileview_add(ui->screen_tileview_set_ESP_NOW_sw, ui->screen_tileview_set_ESP_NOW_label, "ESP-NOW", LV_STATE_DEFAULT);
    lv_set_tileview_add(ui->screen_tileview_set_Switch_page_sw, ui->screen_tileview_set_Switch_page_label, "Switch Page", LV_STATE_CHECKED);
}

void lv_set_tileview_add(lv_obj_t *sw, lv_obj_t *label, const char *text, uint16_t sw_status)
{
    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(label, 13, 30);
    lv_obj_set_size(label, 100, 30);

    // Write style for screen_set_label_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label, &lv_font_SourceHanSerifSC_Regular_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(label, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_set_sw_1
    lv_obj_set_pos(sw, 33, 70);
    lv_obj_set_size(sw, 60, 25);

    lv_obj_add_state(sw, sw_status);

    // Write style for screen_set_sw_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(sw, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xb2b2b2), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(sw, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(sw, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(sw, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(sw, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write style for screen_set_sw_1, Part: LV_PART_INDICATOR, State: LV_STATE_CHECKED.
    lv_obj_set_style_bg_opa(sw, 223, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0x13359a), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_grad_dir(sw, LV_GRAD_DIR_HOR, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_grad_color(sw, lv_color_hex(0x2195f6), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_main_stop(sw, 0, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_grad_stop(sw, 0, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(sw, 0, LV_PART_INDICATOR | LV_STATE_CHECKED);

    // Write style for screen_set_sw_1, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(sw, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xffffff), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(sw, LV_GRAD_DIR_NONE, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(sw, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(sw, 10, LV_PART_KNOB | LV_STATE_DEFAULT);
}

//************************************[ screen 6 ]****************************************** wifi rssi
void gui_wifi_rssi_init(lv_ui *ui)
{
    ui->screen_wifi_rssi = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_wifi_rssi, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_wifi_rssi, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_wifi_rssi, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_wifi_rssi, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_wifi_rssi_cont = lv_obj_create(ui->screen_wifi_rssi);
    lv_obj_set_pos(ui->screen_wifi_rssi_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_wifi_rssi_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_wifi_rssi_cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(ui->screen_wifi_rssi_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_wifi_rssi_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_wifi_rssi_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_wifi_rssi_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_wifi_rssi_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_wifi_rssi_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_wifi_rssi_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_wifi_rssi_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_wifi_rssi_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_wifi_rssi_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_wifi_rssi_label = lv_label_create(ui->screen_wifi_rssi_cont);
    lv_obj_set_style_text_color(ui->screen_wifi_rssi_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_wifi_rssi_label, &lv_font_Acme_Regular_20, LV_PART_MAIN);
    lv_obj_set_pos(ui->screen_wifi_rssi_label, 25, 10);
    lv_obj_set_size(ui->screen_wifi_rssi_label, 50, 30);
    lv_label_set_text(ui->screen_wifi_rssi_label, "WiFi:");
    lv_obj_set_style_text_align(ui->screen_wifi_rssi_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_wifi_rssi_num = lv_label_create(ui->screen_wifi_rssi_cont);
    lv_obj_set_pos(ui->screen_wifi_rssi_num, 13, 50);
    lv_obj_set_size(ui->screen_wifi_rssi_num, 100, 20);
    lv_obj_set_style_text_color(ui->screen_wifi_rssi_num, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui->screen_wifi_rssi_num, &lv_font_SourceHanSerifSC_Regular_16, LV_PART_MAIN);
    lv_label_set_text_fmt(ui->screen_wifi_rssi_num, "Rssi:%d", 0);
    // lv_obj_align_to(ui->screen_wifi_rssi_num, ui->screen_wifi_rssi_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    lv_obj_set_style_text_align(ui->screen_wifi_rssi_num, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_wifi_cont_Signal
    ui->screen_wifi_cont_Signal = lv_obj_create(ui->screen_wifi_rssi_cont);
    lv_obj_set_pos(ui->screen_wifi_cont_Signal, 80, 0);
    lv_obj_set_size(ui->screen_wifi_cont_Signal, 20, 30);
    lv_obj_set_scrollbar_mode(ui->screen_wifi_cont_Signal, LV_SCROLLBAR_MODE_OFF);

    // Write style for screen_wifi_cont_Signal, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_wifi_cont_Signal, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_wifi_cont_Signal, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_wifi_cont_Signal, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_wifi_cont_Signal, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_wifi_cont_Signal, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_wifi_cont_Signal, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_wifi_cont_Signal, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_wifi_cont_Signal, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_wifi_cont_Signal, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_wifi_cont_Signal, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_wifi_line4
    ui->screen_wifi_line4 = lv_line_create(ui->screen_wifi_cont_Signal);
    static lv_point_t screen_wifi_line4[] = {
        {0, 0},
        {0, 20},
    };
    lv_line_set_points(ui->screen_wifi_line4, screen_wifi_line4, 2);
    lv_obj_set_pos(ui->screen_wifi_line4, 18, 5);
    lv_obj_set_size(ui->screen_wifi_line4, 2, 25);

    // Write style for screen_wifi_line4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_line_width(ui->screen_wifi_line4, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(ui->screen_wifi_line4, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui->screen_wifi_line4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(ui->screen_wifi_line4, true, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_wifi_line3
    ui->screen_wifi_line3 = lv_line_create(ui->screen_wifi_cont_Signal);
    static lv_point_t screen_wifi_line3[] = {
        {0, 5},
        {0, 20},
    };
    lv_line_set_points(ui->screen_wifi_line3, screen_wifi_line3, 2);
    lv_obj_set_pos(ui->screen_wifi_line3, 13, 5);
    lv_obj_set_size(ui->screen_wifi_line3, 2, 25);

    // Write style for screen_wifi_line3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_line_width(ui->screen_wifi_line3, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(ui->screen_wifi_line3, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui->screen_wifi_line3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(ui->screen_wifi_line3, true, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_wifi_line2
    ui->screen_wifi_line2 = lv_line_create(ui->screen_wifi_cont_Signal);
    static lv_point_t screen_wifi_line2[] = {
        {0, 10},
        {0, 20},
    };
    lv_line_set_points(ui->screen_wifi_line2, screen_wifi_line2, 2);
    lv_obj_set_pos(ui->screen_wifi_line2, 8, 5);
    lv_obj_set_size(ui->screen_wifi_line2, 2, 25);

    // Write style for screen_wifi_line2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_line_width(ui->screen_wifi_line2, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(ui->screen_wifi_line2, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui->screen_wifi_line2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(ui->screen_wifi_line2, true, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Write codes screen_wifi_line1
    ui->screen_wifi_line1 = lv_line_create(ui->screen_wifi_cont_Signal);
    static lv_point_t screen_wifi_line1[] = {
        {0, 15},
        {0, 20},
    };
    lv_line_set_points(ui->screen_wifi_line1, screen_wifi_line1, 2);
    lv_obj_set_pos(ui->screen_wifi_line1, 3, 5);
    lv_obj_set_size(ui->screen_wifi_line1, 2, 25);

    // Write style for screen_wifi_line1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_line_width(ui->screen_wifi_line1, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(ui->screen_wifi_line1, lv_color_hex(0x757575), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui->screen_wifi_line1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_rounded(ui->screen_wifi_line1, true, LV_PART_MAIN | LV_STATE_DEFAULT);

    scr_arry[srceen_current++] = ui->screen_wifi_rssi;
}

//************************************[ screen 7 ]****************************************** mic
void gui_mic_init(lv_ui *ui)
{
    ui->screen_mic = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_mic, srceen_width, screen_hight);
    lv_obj_set_style_bg_opa(ui->screen_mic, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(ui->screen_mic, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(ui->screen_mic, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_mic_cont = lv_obj_create(ui->screen_mic);
    lv_obj_set_pos(ui->screen_mic_cont, srceen_cont_pos_x, srceen_cont_pos_y);
    lv_obj_set_size(ui->screen_mic_cont, 126, 126);
    lv_obj_set_scrollbar_mode(ui->screen_mic_cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(ui->screen_mic_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(ui->screen_mic_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_mic_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_mic_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_mic_cont, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_mic_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_mic_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_mic_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_mic_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_mic_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_mic_label = lv_label_create(ui->screen_mic_cont);
    lv_obj_set_style_text_color(ui->screen_mic_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui->screen_mic_label, &lv_font_Acme_Regular_24, LV_PART_MAIN);
    lv_obj_set_pos(ui->screen_mic_label, 13, 10);
    lv_obj_set_size(ui->screen_mic_label, 100, 30);
    lv_label_set_text(ui->screen_mic_label, "Mic:");
    lv_obj_set_style_text_align(ui->screen_mic_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->screen_mic_count = lv_label_create(ui->screen_mic_cont);
    lv_obj_set_pos(ui->screen_mic_count, 13, 50);
    lv_obj_set_size(ui->screen_mic_count, 100, 25);
    lv_obj_set_style_text_color(ui->screen_mic_count, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui->screen_mic_count, &lv_font_SourceHanSerifSC_Regular_16, LV_PART_MAIN);
    lv_label_set_text_fmt(ui->screen_mic_count, "Noise:%d", 0);
    // lv_obj_align_to(ui->screen_mic_count, ui->screen_mic_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    lv_obj_set_style_text_align(ui->screen_mic_count, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_timer_create([](lv_timer_t *t)
                    {
                                      lv_obj_t *label = (lv_obj_t *)t->user_data;

                                      size_t read_len;

                                      if (!vad_buff)
                                          return;
                                      if (amoled.readMicrophone((char *)vad_buff, VAD_BUFFER_LENGTH * sizeof(short), &read_len))
                                      {
            // Feed samples to the VAD process and get the result
#if ESP_IDF_VERSION_VAL(4, 4, 1) == ESP_IDF_VERSION
                                          vad_state_t vad_state = vad_process(vad_inst, vad_buff);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 1)
                                          vad_state_t vad_state = vad_process(vad_inst, vad_buff, MIC_I2S_SAMPLE_RATE, VAD_FRAME_LENGTH_MS);
#else
#error "No support this version."
#endif
                                          if (vad_state == VAD_SPEECH)
                                          {
                                              lv_label_set_text_fmt(label, "Noise:%lu", noise_count++);
                                          }
                                      } },
                    100, ui->screen_mic_count);

    scr_arry[srceen_current++] = ui->screen_mic;
}