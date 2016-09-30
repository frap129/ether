#ifndef _FIH_POWERKEY_H
#define _FIH_POWERKEY_H

#include <linux/types.h>

struct fih_power_key_platform_data {
	int power_key_detect;
	struct input_dev *input;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
};

#endif
