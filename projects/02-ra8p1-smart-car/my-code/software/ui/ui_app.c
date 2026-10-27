#include "software/ui/ui_app.h"

#include "software/ui/ui_port.h"
#include "third_party/lvgl/lvgl.h"

#include <stddef.h>

static lv_obj_t *g_ui_screen = NULL;

static lv_color_t ui_app_background_color(void)
{
    return lv_color_hex(0x070A18U);
}

fsp_err_t ui_app_init(void)
{
    fsp_err_t err = ui_port_init();

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    g_ui_screen = lv_display_get_screen_active(NULL);
    if (NULL == g_ui_screen)
    {
        return FSP_ERR_INTERNAL;
    }

    lv_obj_clean(g_ui_screen);
    lv_obj_remove_style_all(g_ui_screen);
    lv_obj_set_style_bg_color(g_ui_screen, ui_app_background_color(), 0);
    lv_obj_set_style_bg_opa(g_ui_screen, LV_OPA_COVER, 0);

    return FSP_SUCCESS;
}

void ui_app_process(uint32_t elapsed_ms)
{
    ui_port_tick_inc(elapsed_ms);
    ui_port_handler();
}
