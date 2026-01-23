#ifndef EVENTS_INIT_H_
#define EVENTS_INIT_H_

#include "ui.h"
#include <esp_now.h>

void events_init_set_screen(lv_ui *ui);

void gui_set_wifi_cb(lv_event_t *e);
void gui_set_switch_page_cb(lv_event_t *e);
void gui_set_espnow_cb(lv_event_t *e);

#endif /* EVENT_CB_H_ */
