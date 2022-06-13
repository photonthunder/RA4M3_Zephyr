/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include <bsp_api.h>
#include <common_data.h>
#include <r_sci_uart.h>
#include <r_uart_api.h>
#include <r_dtc.h>
#include <r_transfer_api.h>
#include <r_sci_spi.h>
#include <r_spi_api.h>
#include <r_iic_master.h>
#include <r_i2c_master_api.h>
FSP_HEADER
/** UART on SCI Instance. */
extern const uart_instance_t g_uart_sci9;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_uart_instance_ctrl_t g_uart_sci9_ctrl;
extern const uart_cfg_t g_uart_sci9_cfg;
extern const sci_uart_extended_cfg_t g_uart_sci9_cfg_extend;

#ifndef sci9_uart_callback
void sci9_uart_callback(uart_callback_args_t *p_args);
#endif
/** UART on SCI Instance. */
extern const uart_instance_t g_uart_sci4;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_uart_instance_ctrl_t g_uart_sci4_ctrl;
extern const uart_cfg_t g_uart_sci4_cfg;
extern const sci_uart_extended_cfg_t g_uart_sci4_cfg_extend;

#ifndef sci4_uart_callback
void sci4_uart_callback(uart_callback_args_t *p_args);
#endif
/** UART on SCI Instance. */
extern const uart_instance_t g_uart_sci2;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_uart_instance_ctrl_t g_uart_sci2_ctrl;
extern const uart_cfg_t g_uart_sci2_cfg;
extern const sci_uart_extended_cfg_t g_uart_sci2_cfg_extend;

#ifndef sci2_uart_callback
void sci2_uart_callback(uart_callback_args_t *p_args);
#endif
/** UART on SCI Instance. */
extern const uart_instance_t g_uart_sci1;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_uart_instance_ctrl_t g_uart_sci1_ctrl;
extern const uart_cfg_t g_uart_sci1_cfg;
extern const sci_uart_extended_cfg_t g_uart_sci1_cfg_extend;

#ifndef sci1_uart_callback
void sci1_uart_callback(uart_callback_args_t *p_args);
#endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer3;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer3_ctrl;
extern const transfer_cfg_t g_transfer3_cfg;
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer2;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer2_ctrl;
extern const transfer_cfg_t g_transfer2_cfg;
/** SPI on SCI Instance. */
extern const spi_instance_t g_spi_sci3;

/** Access the SCI_SPI instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_spi_instance_ctrl_t g_spi_sci3_ctrl;
extern const spi_cfg_t g_spi_sci3_cfg;

/** Called by the driver when a transfer has completed or an error has occurred (Must be implemented by the user). */
#ifndef sci3_spi_callback
void sci3_spi_callback(spi_callback_args_t *p_args);
#endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer1;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer1_ctrl;
extern const transfer_cfg_t g_transfer1_cfg;
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer0;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer0_ctrl;
extern const transfer_cfg_t g_transfer0_cfg;
/** SPI on SCI Instance. */
extern const spi_instance_t g_spi_sci0;

/** Access the SCI_SPI instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_spi_instance_ctrl_t g_spi_sci0_ctrl;
extern const spi_cfg_t g_spi_sci0_cfg;

/** Called by the driver when a transfer has completed or an error has occurred (Must be implemented by the user). */
#ifndef sci0_spi_callback
void sci0_spi_callback(spi_callback_args_t *p_args);
#endif
/* I2C Master on IIC Instance. */
extern const i2c_master_instance_t g_i2c_master0;

/** Access the I2C Master instance using these structures when calling API functions directly (::p_api is not used). */
extern iic_master_instance_ctrl_t g_i2c_master0_ctrl;
extern const i2c_master_cfg_t g_i2c_master0_cfg;

#ifndef NULL
void NULL(i2c_master_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */