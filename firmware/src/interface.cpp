#include "config.h"

#ifdef LCD_SUPPORTED
#include "interface.h"
#include <Arduino.h>
#include "ui/ui.h"
#include <TFT_eSPI.h>
#include <lvgl.h>

uint32_t lcd_drawBufffer[DRAW_BUF_SIZE / 4];

SmokerState *lcd_localSmokerState;

int lcd_cookTimeSeconds = 0;
int lcd_cookTimeMinutes = 0;
int lcd_cookTimeHours = 0;
bool lcd_toggleclockColon = true;

uint32_t lcd_tickProvider()
{
    return millis();
}

void lcd_updateCookTime(lv_timer_t *timer)
{
    lcd_cookTimeSeconds += 1;

    lcd_cookTimeMinutes += lcd_cookTimeSeconds / 60; // Increment minutes if seconds exceed 60
    lcd_cookTimeSeconds = lcd_cookTimeSeconds % 60;  // Reset seconds to 0 if

    lcd_cookTimeHours += lcd_cookTimeMinutes / 60;  // Increment hours if minutes exceed 60
    lcd_cookTimeMinutes = lcd_cookTimeMinutes % 60; // Reset minutes to 0 if they exceed 60

    lv_label_set_text_fmt(ui_CookTimeHoursLabel, "%02d", lcd_cookTimeHours);     // Update the label with the new temperature
    lv_label_set_text_fmt(ui_CookTimeMinutesLabel, "%02d", lcd_cookTimeMinutes); // Update the label with the new temperature

    if (lcd_toggleclockColon = !lcd_toggleclockColon)
    {
        lv_label_set_text(ui_CookTimeSeperatorLabel, ":");
    }
    else
    {
        lv_label_set_text(ui_CookTimeSeperatorLabel, " ");
    }

    if (lcd_cookTimeHours > 2)
    {
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR,
                                               _ui_theme_color_Primary);
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA,
                                               _ui_theme_alpha_Primary);
    }
}

int lcd_toLocalTemperature(double value)
{
#ifndef FERINHEIT
    return value;
#endif
#ifdef FERINHEIT
    return value * 1.8 + 32;
#endif
}

void lcd_init(SmokerState *smokerState)
{
    lcd_localSmokerState = smokerState;

    lv_init();

    /*Set a tick source so that LVGL will know how much time elapsed. */
    lv_tick_set_cb(lcd_tickProvider);

    /*TFT_eSPI can be enabled lv_conf.h to initialize the display in a simple way*/
    lv_display_t *disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, lcd_drawBufffer, sizeof(lcd_drawBufffer));
    // lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

    ui_init();
    Serial.println("Setup done");

    lv_timer_create(lcd_updateCookTime, 1000, NULL); // Create a timer to update the temperature every second
}

void lcd_taskHandler()
{
    lv_task_handler(); // Call the LVGL task handler to process events and draw
    delay(5);          // Add a small delay to allow for smoother updates
}

void lcd_updateSmokerState()
{

    if (lcd_localSmokerState->probe1 > 0)
    {
        lv_label_set_text_fmt(ui_Temp1Label, "%d°", lcd_toLocalTemperature(lcd_localSmokerState->probe1));
    }
    else
    {
        lv_label_set_text(ui_Temp1Label, "---°");
    }

    if (lcd_localSmokerState->probe2 > 0)
    {
        lv_label_set_text_fmt(ui_Temp2Label, "%d°", lcd_toLocalTemperature(lcd_localSmokerState->probe2));
    }
    else
    {
        lv_label_set_text(ui_Temp2Label, "---°");
    }

    if (lcd_localSmokerState->probe3 > 0)
    {
        lv_label_set_text_fmt(ui_Temp3Label, "%d°", lcd_toLocalTemperature(lcd_localSmokerState->probe3));
    }
    else
    {
        lv_label_set_text(ui_Temp3Label, "---°");
    }

    if (lcd_localSmokerState->probe4 > 0)
    {
        lv_label_set_text_fmt(ui_Temp4Label, "%d°", lcd_toLocalTemperature(lcd_localSmokerState->probe4));
    }
    else
    {
        lv_label_set_text(ui_Temp4Label, "---°");
    }

    if (lcd_localSmokerState->temperature > 0)
    {
        lv_label_set_text_fmt(ui_CookTempLabel, "%d°", lcd_toLocalTemperature(lcd_localSmokerState->temperature));
    }
    else
    {
        lv_label_set_text(ui_CookTempLabel, "---°");
    }

    if (lcd_localSmokerState->targetTemperature > 0)
    {
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR,
                                               _ui_theme_color_Primary);
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA,
                                               _ui_theme_alpha_Primary);
    } else {
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR,
                                               _ui_theme_color_Dark_Text);
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA,
                                               _ui_theme_alpha_Dark_Text);
    }
}

void lcd_handler()
{
    lcd_taskHandler();
    lcd_updateSmokerState();
}

#endif // LCD_SUPPORTED