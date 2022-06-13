/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _PINCTRL_RENESAS_RA_H_
#define _PINCTRL_RENESAS_RA_H_

#include <dt-bindings/dt-util.h>

/* Create a pincfg device tree node:
 *
 * The node name and nodelabel will be of the form:
 *
 * NODE = p<port><pin><config>_<inst>_<signal>
 *
 * NODE: NODE {
 *      renesas,pins = < &port<port> <pin> CFG_<config> >;
 *      flags_1;
 *       ...
 *      flags_N;
 * }
 *
 * So for example:
 *
 * DT_RENESAS_RA_PIN(uart, urxd, 0, 8, SCI_EVEN);
 * 
 * Will become:
 *
 * p08_uart_urxd: p08_uart_urxd {
 *    renesas,pins = <&port0 0x8 CFG_SCI_EVEN>;
 * }
 *
 * Flags are optional and should be pass one by one as arguments:
 *
 * DT_RENESAS_RA_PORT(sci0, uart, 1,  10, CFG_<config>, pinmux-enable);
 *
 * Will become:
 *
 * p110a_sci0_uart: p110a_sci0_uart {
 *    renesas,pins = <&port1 10 CFG_SCI_ODD >;
 *    pinmux-enable;
 * }
 *
 * For the complete list of flags see atmel,sam[0]-pinctrl.yaml
 */

#define DT_RENESAS_RA_PINCTRL_FLAG(flag) flag;
#define DT_RENESAS_RA_PINCTRL_FLAGS(...) \
	MACRO_MAP_CAT(DT_RENESAS_RA_PINCTRL_FLAG __VA_OPT__(,) __VA_ARGS__)

#define DT_RENESAS_RA_PORT_PIN(port, pin) ((port << 8) | pin)

#define DT_RENESAS_RA_PIN(inst, signal, port, pin, pingroup, config, ...) \
	p##port##pin##pingroup##_##inst##_##signal: \
	p##port##pin##pingroup##_##inst##_##signal { \
	renesas,pins = < &pio##port DT_RENESAS_RA_PORT_PIN(port, pin) config >; \
		DT_RENESAS_RA_PINCTRL_FLAGS(__VA_ARGS__) \
	}

#define DT_RENESAS_RA_GPIO(inst, signal, port, pin, pingroup, config, ...) \
	p##port##pin##pingroup##_##inst##_##signal: \
	p##port##pin##pingroup##_##inst##_##signal { \
	renesas,pins = < &gpio##port DT_RENESAS_RA_PORT_PIN(port, pin) config >; \
		DT_RENESAS_RA_PINCTRL_FLAGS(__VA_ARGS__) \
	}

#define DT_RENESAS_RA_PORT(inst, signal, groupport, pin, pingroup, config, ...) \
	p##groupport##pin##pingroup##_##inst##_##signal: \
	p##groupport##pin##pingroup##_##inst##_##signal { \
	renesas,pins = < &port##groupport DT_RENESAS_RA_PORT_PIN(groupport, pin) config >; \
		DT_RENESAS_RA_PINCTRL_FLAGS(__VA_ARGS__) \
	}

#endif /* _PINCTRL_RENESAS_RA_H_ */


