#include "interface.h"

#ifdef LCD_SUPPORTED
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

int cookTimeSeconds = 0;
int cookTimeMinutes = 0;
int cookTimeHours = 0;
bool toggleColen = true;

uint32_t tick_provider()
{
    return millis();
}

void update_cook_time(lv_timer_t *timer)
{
    cookTimeSeconds += 1;

    cookTimeMinutes += cookTimeSeconds / 60; // Increment minutes if seconds exceed 60
    cookTimeSeconds = cookTimeSeconds % 60;  // Reset seconds to 0 if

    cookTimeHours += cookTimeMinutes / 60;  // Increment hours if minutes exceed 60
    cookTimeMinutes = cookTimeMinutes % 60; // Reset minutes to 0 if they exceed 60

    lv_label_set_text_fmt(ui_CookTimeHoursLabel, "%02d", cookTimeHours);     // Update the label with the new temperature
    lv_label_set_text_fmt(ui_CookTimeMinutesLabel, "%02d", cookTimeMinutes); // Update the label with the new temperature

    if (toggleColen = !toggleColen)
    {
        lv_label_set_text(ui_CookTimeSeperatorLabel, ":");
    }
    else
    {
        lv_label_set_text(ui_CookTimeSeperatorLabel, " ");
    }

    if (cookTimeHours > 2)
    {
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_COLOR,
                                               _ui_theme_color_Primary);
        ui_object_set_themeable_style_property(ui_OnOffLabel, LV_PART_MAIN | LV_STATE_DEFAULT, LV_STYLE_TEXT_OPA,
                                               _ui_theme_alpha_Primary);
    }
}

void lcd_init()
{
    lv_init();

    /*Set a tick source so that LVGL will know how much time elapsed. */
    lv_tick_set_cb(tick_provider);

    /*TFT_eSPI can be enabled lv_conf.h to initialize the display in a simple way*/
    lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));

    ui_init();
    Serial.println("Setup done");

    lv_label_set_text(ui_Temp1Label, "450°");
    lv_timer_create(update_cook_time, 1000, NULL); // Create a timer to update the temperature every second
}

void lcd_task_handler()
{
    lv_task_handler(); // Call the LVGL task handler to process events and draw
    delay(5);          // Add a small delay to allow for smoother updates
}

void lcd_update_temps(int temp, int probe1, int probe2, int probe3, int probe4)
{
    lv_label_set_text_fmt(ui_Temp1Label, "%d°", temp);
    lv_label_set_text_fmt(ui_Probe1Label, "%d°", probe1);
    lv_label_set_text_fmt(ui_Probe2Label, "%d°", probe2);
    lv_label_set_text_fmt(ui_Probe3Label, "%d°", probe3);
    lv_label_set_text_fmt(ui_Probe4Label, "%d°", probe4);
}
#endif // LCD_SUPPORTED