/*******************************************************************************
 * Size: 12 px
 * Bpp: 4
 * Opts: --bpp 4 --size 12 --font /home/john/SquareLine/assets/fa-solid-900.ttf -o /home/john/SquareLine/assets/ui_font_Solid_Wifi.c --format lvgl -r 0xf1eb -r 0xf06d --no-compress --no-prefilter
 ******************************************************************************/

#include "ui.h"

#ifndef UI_FONT_SOLID_WIFI
#define UI_FONT_SOLID_WIFI 1
#endif

#if UI_FONT_SOLID_WIFI

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+F06D "" */
    0x0, 0x1, 0x20, 0x0, 0x0, 0x0, 0x1, 0xdf,
    0x20, 0x0, 0x0, 0x0, 0xcf, 0xfd, 0x3b, 0x20,
    0x0, 0x7f, 0xff, 0xff, 0xfd, 0x0, 0x1e, 0xff,
    0xff, 0xff, 0xf6, 0x7, 0xff, 0xff, 0xff, 0xff,
    0xe0, 0xcf, 0xf8, 0x7f, 0xff, 0xff, 0x3e, 0xfe,
    0x0, 0x9e, 0x9f, 0xf6, 0xef, 0xb0, 0x0, 0x12,
    0xff, 0x6b, 0xfd, 0x0, 0x0, 0x4f, 0xf3, 0x5f,
    0xf8, 0x0, 0x3d, 0xfd, 0x0, 0xbf, 0xff, 0xef,
    0xff, 0x30, 0x0, 0x8f, 0xff, 0xfd, 0x30, 0x0,
    0x0, 0x3, 0x52, 0x0, 0x0,

    /* U+F1EB "" */
    0x0, 0x0, 0x15, 0x89, 0x97, 0x30, 0x0, 0x0,
    0x1, 0x9f, 0xff, 0xee, 0xff, 0xd5, 0x0, 0x4,
    0xef, 0x93, 0x0, 0x1, 0x5d, 0xfa, 0x0, 0xed,
    0x30, 0x0, 0x0, 0x0, 0x8, 0xf6, 0x1, 0x0,
    0x1, 0x58, 0x73, 0x0, 0x1, 0x0, 0x0, 0x5,
    0xff, 0xff, 0xfb, 0x10, 0x0, 0x0, 0x3, 0xfb,
    0x20, 0x6, 0xfc, 0x0, 0x0, 0x0, 0x5, 0x0,
    0x0, 0x1, 0x30, 0x0, 0x0, 0x0, 0x0, 0x4,
    0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0xfc,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1b, 0x60,
    0x0, 0x0, 0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 168, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 77, .adv_w = 216, .box_w = 15, .box_h = 11, .ofs_x = -1, .ofs_y = -1}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0x17e
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 61549, .range_length = 383, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 2, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t ui_font_Solid_Wifi = {
#else
lv_font_t ui_font_Solid_Wifi = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 14,          /*The maximum line height required by the font*/
    .base_line = 2,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = 0,
    .underline_thickness = 0,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if UI_FONT_SOLID_WIFI*/

