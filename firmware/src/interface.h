#ifndef INTERFACE_H
#define INTERFACE_H

#ifdef LCD_SUPPORTED

#include <TFT_eSPI.h>
#include <lvgl.h>
#include "ui/ui.h"
#include "status.h"

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES 130
#define TFT_VER_RES 161
#define TFT_ROTATION LV_DISPLAY_ROTATION_0

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))

enum PROBE { PROBE1, PROBE2, PROBE3, PROBE4 };

void lcd_init(status_state *state);
void lcd_loop(unsigned long &now);
void lcd_wifiConnected();
void lcd_wifiDisconnected();
void lcd_setProbeLabel(PROBE probe, char *label);
void lcd_updateSmokerState();

#endif // LCD_SUPPORTED
#endif // INTERFACE_H