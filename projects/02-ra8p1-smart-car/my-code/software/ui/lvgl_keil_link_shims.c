#include "third_party/lvgl/lvgl.h"
#include "third_party/lvgl/src/draw/convert/lv_draw_buf_convert.h"
#include "third_party/lvgl/src/indev/lv_indev.h"
#include "third_party/lvgl/src/indev/lv_indev_scroll.h"
#include "third_party/lvgl/src/layouts/lv_layout_private.h"
#include "third_party/lvgl/src/libs/bin_decoder/lv_bin_decoder.h"
#include "third_party/lvgl/src/misc/cache/instance/lv_image_cache.h"
#include "third_party/lvgl/src/misc/cache/instance/lv_image_header_cache.h"
#include "third_party/lvgl/src/misc/cache/lv_cache.h"
#include "third_party/lvgl/src/misc/cache/lv_cache_entry.h"
#include "third_party/lvgl/src/themes/default/lv_theme_default.h"
#include "third_party/lvgl/src/themes/mono/lv_theme_mono.h"
#include "third_party/lvgl/src/themes/simple/lv_theme_simple.h"
#include "third_party/lvgl/src/themes/lv_theme.h"

/*
 * Keil 工程文件只能由用户维护。当前工程没有挂载部分 LVGL 基础源文件，
 * 这里为本屏幕 UI 不使用的输入、布局、主题、图片缓存等模块提供最小实现。
 */
void lv_layout_init(void)
{
}

void lv_layout_deinit(void)
{
}

bool lv_layout_get_min_size(lv_obj_t *obj, int32_t *size, bool width)
{
    LV_UNUSED(obj);
    LV_UNUSED(size);
    LV_UNUSED(width);
    return false;
}

void lv_layout_apply(lv_obj_t *obj)
{
    LV_UNUSED(obj);
}

void lv_indev_delete(lv_indev_t *indev)
{
    LV_UNUSED(indev);
}

lv_indev_t *lv_indev_get_next(lv_indev_t *indev)
{
    LV_UNUSED(indev);
    return NULL;
}

lv_indev_t *lv_indev_active(void)
{
    return NULL;
}

void lv_indev_set_display(lv_indev_t *indev, lv_display_t *disp)
{
    LV_UNUSED(indev);
    LV_UNUSED(disp);
}

lv_indev_type_t lv_indev_get_type(const lv_indev_t *indev)
{
    LV_UNUSED(indev);
    return LV_INDEV_TYPE_NONE;
}

lv_indev_state_t lv_indev_get_state(const lv_indev_t *indev)
{
    LV_UNUSED(indev);
    return LV_INDEV_STATE_RELEASED;
}

lv_group_t *lv_indev_get_group(const lv_indev_t *indev)
{
    LV_UNUSED(indev);
    return NULL;
}

lv_display_t *lv_indev_get_display(const lv_indev_t *indev)
{
    LV_UNUSED(indev);
    return NULL;
}

void lv_indev_reset(lv_indev_t *indev, lv_obj_t *obj)
{
    LV_UNUSED(indev);
    LV_UNUSED(obj);
}

void lv_indev_set_group(lv_indev_t *indev, lv_group_t *group)
{
    LV_UNUSED(indev);
    LV_UNUSED(group);
}

lv_dir_t lv_indev_get_scroll_dir(const lv_indev_t *indev)
{
    LV_UNUSED(indev);
    return (lv_dir_t)0;
}

lv_obj_t *lv_indev_get_scroll_obj(const lv_indev_t *indev)
{
    LV_UNUSED(indev);
    return NULL;
}

void lv_indev_wait_release(lv_indev_t *indev)
{
    LV_UNUSED(indev);
}

lv_obj_t *lv_indev_get_active_obj(void)
{
    return NULL;
}

void lv_indev_scroll_get_snap_dist(lv_obj_t *obj, lv_point_t *p)
{
    LV_UNUSED(obj);
    if (NULL != p)
    {
        p->x = 0;
        p->y = 0;
    }
}

void lv_theme_apply(lv_obj_t *obj)
{
    LV_UNUSED(obj);
}

lv_theme_t *lv_theme_default_init(lv_display_t *disp,
                                  lv_color_t color_primary,
                                  lv_color_t color_secondary,
                                  bool dark,
                                  const lv_font_t *font)
{
    LV_UNUSED(disp);
    LV_UNUSED(color_primary);
    LV_UNUSED(color_secondary);
    LV_UNUSED(dark);
    LV_UNUSED(font);
    return NULL;
}

bool lv_theme_default_is_inited(void)
{
    return false;
}

lv_theme_t *lv_theme_default_get(void)
{
    return NULL;
}

void lv_theme_default_deinit(void)
{
}

lv_theme_t *lv_theme_simple_init(lv_display_t *disp)
{
    LV_UNUSED(disp);
    return NULL;
}

bool lv_theme_simple_is_inited(void)
{
    return false;
}

lv_theme_t *lv_theme_simple_get(void)
{
    return NULL;
}

void lv_theme_simple_deinit(void)
{
}

lv_theme_t *lv_theme_mono_init(lv_display_t *disp, bool dark_bg, const lv_font_t *font)
{
    LV_UNUSED(disp);
    LV_UNUSED(dark_bg);
    LV_UNUSED(font);
    return NULL;
}

bool lv_theme_mono_is_inited(void)
{
    return false;
}

lv_theme_t *lv_theme_mono_get(void)
{
    return NULL;
}

void lv_theme_mono_deinit(void)
{
}

lv_result_t lv_image_cache_init(uint32_t size)
{
    LV_UNUSED(size);
    return LV_RESULT_OK;
}

void lv_image_cache_drop(const void *src)
{
    LV_UNUSED(src);
}

bool lv_image_cache_is_enabled(void)
{
    return false;
}

lv_result_t lv_image_header_cache_init(uint32_t count)
{
    LV_UNUSED(count);
    return LV_RESULT_OK;
}

bool lv_image_header_cache_is_enabled(void)
{
    return false;
}

void lv_cache_destroy(lv_cache_t *cache, void *user_data)
{
    LV_UNUSED(cache);
    LV_UNUSED(user_data);
}

lv_cache_entry_t *lv_cache_acquire(lv_cache_t *cache, const void *key, void *user_data)
{
    LV_UNUSED(cache);
    LV_UNUSED(key);
    LV_UNUSED(user_data);
    return NULL;
}

lv_cache_entry_t *lv_cache_add(lv_cache_t *cache, const void *key, void *user_data)
{
    LV_UNUSED(cache);
    LV_UNUSED(key);
    LV_UNUSED(user_data);
    return NULL;
}

void lv_cache_release(lv_cache_t *cache, lv_cache_entry_t *entry, void *user_data)
{
    LV_UNUSED(cache);
    LV_UNUSED(entry);
    LV_UNUSED(user_data);
}

void *lv_cache_entry_get_data(lv_cache_entry_t *entry)
{
    LV_UNUSED(entry);
    return NULL;
}

lv_result_t lv_draw_buf_convert_premultiply(lv_draw_buf_t *buf)
{
    LV_UNUSED(buf);
    return LV_RESULT_OK;
}

void lv_bin_decoder_init(void)
{
}

void lv_span_stack_init(void)
{
}

void lv_span_stack_deinit(void)
{
}

/* RGB565 软件渲染必须保留真实实现，否则文字、线条和矩形不会被绘制。 */
#include "third_party/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.c"
#include "third_party/lvgl/src/draw/sw/blend/lv_draw_sw_blend.c"
