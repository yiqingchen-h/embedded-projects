#ifndef SOFTWARE_UI_UI_APP_H_
#define SOFTWARE_UI_UI_APP_H_

#include "hal_data.h"

#include <stdint.h>

fsp_err_t ui_app_init(void);
void ui_app_process(uint32_t elapsed_ms);

#endif /* SOFTWARE_UI_UI_APP_H_ */
