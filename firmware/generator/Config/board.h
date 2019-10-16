

#ifndef BOARD_H_
#define BOARD_H_
#include <Config/peripheral_clk_config.h>
#include <Config/hpl_rtc_config.h>

#define FIRMWARE_VERSION (1)

#define TIMER_FREQUENCY (CONF_GCLK_RTC_FREQUENCY / (1 << CONF_RTC_PRESCALER))


#endif /* BOARD_H_ */