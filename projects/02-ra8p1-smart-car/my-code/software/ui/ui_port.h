#ifndef SOFTWARE_UI_UI_PORT_H_
#define SOFTWARE_UI_UI_PORT_H_

#include "hal_data.h"

#include <stdint.h>

#ifndef UI_PORT_USE_MIPI_LCD
#define UI_PORT_USE_MIPI_LCD (1)
#endif

fsp_err_t ui_port_init(void);
void ui_port_tick_inc(uint32_t elapsed_ms);
void ui_port_handler(void);

#endif /* SOFTWARE_UI_UI_PORT_H_ */
