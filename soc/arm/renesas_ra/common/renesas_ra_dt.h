/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Renesas RA MCU family devicetree helper macros
 */

#ifndef _RENESAS_RA_DT_H_
#define _RENESAS_RA_DT_H_


/* Use to check if a sci 'n' is enabled for a given 'compat' */
#define RENESAS_RA_DT_SCI_CHECK(n, compat) \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(sci##n), compat, okay)

/* Use to check if TCC 'n' is enabled for a given 'compat' */
#define RENESAS_RA_DT_TCC_CHECK(n, compat) \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(tcc##n), compat, okay)

/* Common macro for use to set HCLK_FREQ_HZ */
#define RENESAS_RA_DT_CPU_CLK_FREQ_HZ \
	DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

/* Devicetree related macros to construct pin mux config data */

/* Get PIN associated with pinctrl-0 pin at index 'i' */
#define RENESAS_RA_PIN(node_id, i) \
	DT_PHA(DT_PINCTRL_0(node_id, i), renesas_pins, pin)

/* Get PIO register address associated with pinctrl-0 pin at index 'i' */
#define RENESAS_RA_PIN_TO_PORT_REG_ADDR(node_id, i) \
	DT_REG_ADDR(DT_PHANDLE(DT_PINCTRL_0(node_id, i), renesas_pins))

/* Get cfg associated wiith pinctrl-0 pin at index 'i' */
#define RENESAS_RA_PIN_CONFIG(node_id, i) \
	DT_PHA(DT_PINCTRL_0(node_id, i), renesas_pins, config)

/* Helper function for PIN_FLAGS */
#define RENESAS_RA_PIN_FLAG(node_id, i, flag) \
	DT_PROP(DT_PINCTRL_0(node_id, i), flag)

/* Convert DT flags to SoC flags */
#define RENESAS_RA_PIN_FLAGS(node_id, i) \
	(RENESAS_RA_PIN_FLAG(node_id, i, bias_pull_up) | \
	 RENESAS_RA_PIN_FLAG(node_id, i, input_enable) | \
	 RENESAS_RA_PIN_FLAG(node_id, i, output_enable) | \

/* Construct a soc_port_pin element for pin cfg */
#define RENESAS_RA_DT_PORT(node_id, idx)					\
	{									\
		(R_SCI0_Type *)RENESAS_RA_PIN_TO_PORT_REG_ADDR(node_id, idx),	\
		RENESAS_RA_PIN(node_id, idx),					\
		RENESAS_RA_PIN_CONFIG(node_id, idx) /*| RENESAS_RA_PIN_FLAGS(node_id, idx)*/	\
	}

/* Get the number of pins for pinctrl-0 */
#define RENESAS_RA_DT_NUM_PINS(node_id) DT_NUM_PINCTRLS_BY_IDX(node_id, 0)

#define RENESAS_RA_DT_INST_NUM_PINS(inst) RENESAS_RA_DT_NUM_PINS(DT_DRV_INST(inst))

/* internal macro to structure things for use with UTIL_LISTIFY */
#define RENESAS_RA_DT_PIN_ELEM(idx, node_id) RENESAS_RA_DT_PORT(node_id, idx),

/* Construct an array intializer for soc_port_pin for a device instance */
#define RENESAS_RA_DT_PINS(node_id)				\
	{ UTIL_LISTIFY(RENESAS_RA_DT_NUM_PINS(node_id),		\
		       RENESAS_RA_DT_PIN_ELEM, node_id)		\
	}

#define RENESAS_RA_DT_INST_PINS(inst) RENESAS_RA_DT_PINS(DT_DRV_INST(inst))

#endif /* _RENESAS_RA_DT_H_ */
