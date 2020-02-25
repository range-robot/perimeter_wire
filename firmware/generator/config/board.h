

#ifndef BOARD_H_
#define BOARD_H_
#include <config/peripheral_clk_config.h>
#include <config/hpl_tc_config.h>

#define FIRMWARE_VERSION (2)

#define TIMER_FREQUENCY (CONF_GCLK_TC3_FREQUENCY / CONF_TC3_PRESCALE / CONF_TC3_CC0)
#define CONF_ADC_0_MUX_TEMP 0x5
#define CONF_ADC_0_MUX_VOLTAGE 0x4


#endif /* BOARD_H_ */