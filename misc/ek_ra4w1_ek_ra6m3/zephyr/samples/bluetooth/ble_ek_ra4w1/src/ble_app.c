/*
 * ble_app.c
 *
 *  Created on: Nov 29, 2020
 *      Author: zoltan.ianosi.bp
 */

#include <zephyr.h>
#include <rm_ble_abs.h>
#include "ble_app.h"
#include <hal_data.h>

#include "../qe_gen/ble/gatt_db.h"
#include "../qe_gen/ble/profile_cmn/r_ble_servc_if.h"
#include "../qe_gen/ble/r_ble_lss.h"

/* 100 msec */
#define SLEEP_TIME_MS   100
#define BLE_GATTS_QUEUE_ELEMENTS_SIZE       (14)
#define BLE_GATTS_QUEUE_BUFFER_LEN          (245)
#define BLE_GATTS_QUEUE_NUM                 (1)

k_tid_t ble_thread_id;

K_FIFO_DEFINE(ble_led_fifo);

/* Connection handle */
uint16_t g_conn_hdl;

void gap_cb(uint16_t type, ble_status_t result, st_ble_evt_data_t *p_data);

st_ble_gap_conn_param_t g_conn_updt_param =
{
 .conn_intv_min = 0x0050,
 .conn_intv_max = 0x0050,
 .conn_latency = 0x0000,
 .sup_to = 0x0C80,
 .min_ce_length = 0x0000,
 .max_ce_length = 0x0000,
};

/* Advertising Data */
static uint8_t gs_advertising_data[] =
{
    /* Flags */
    0x02, /**< Data Size */
    0x01, /**< Data Type */
    ( 0x06 ), /**< Data Value */

    /* Shortened Local Name */
    0x05, /**< Data Size */
    0x08, /**< Data Type */
    0x52, 0x42, 0x4c, 0x45, /**< Data Value */
};

/* Scan Response Data */
static uint8_t gs_scan_response_data[] =
{
    /* Complete Local Name */
    0x0A, /**< Data Size */
    0x09, /**< Data Type */
    0x5a,0x45,0x50,0x48,0x5f,0x52,0x42,0x4c,0x45, /**< Data Value */
};


ble_abs_legacy_advertising_parameter_t g_ble_advertising_parameter =
{
 .p_peer_address             = NULL,       ///< Peer address.
 .slow_advertising_interval  = 0x00000640, ///< Slow advertising interval. 1,000.0(ms)
 .slow_advertising_period    = 0x0000,     ///< Slow advertising period.
 .p_advertising_data         = gs_advertising_data,             ///< Advertising data. If p_advertising_data is specified as NULL, advertising data is not set.
 .advertising_data_length    = ARRAY_SIZE(gs_advertising_data), ///< Advertising data length (in bytes).
 .p_scan_response_data       = gs_scan_response_data,             ///< Scan response data. If p_scan_response_data is specified as NULL, scan response data is not set.
 .scan_response_data_length  = ARRAY_SIZE(gs_scan_response_data), ///< Scan response data length (in bytes).
 .advertising_filter_policy  = BLE_ABS_ADVERTISING_FILTER_ALLOW_ANY, ///< Advertising Filter Policy.
 .advertising_channel_map    = ( BLE_GAP_ADV_CH_37 | BLE_GAP_ADV_CH_38 | BLE_GAP_ADV_CH_39 ), ///< Channel Map.
 .own_bluetooth_address_type = BLE_GAP_ADDR_RAND, ///< Own Bluetooth address type.
 .own_bluetooth_address      = { 0 },
};

/* GATT server Prepare Write Queue parameters */
static st_ble_gatt_queue_elm_t  gs_queue_elms[BLE_GATTS_QUEUE_ELEMENTS_SIZE];
static uint8_t gs_buffer[BLE_GATTS_QUEUE_BUFFER_LEN];
static st_ble_gatt_pre_queue_t gs_queue[BLE_GATTS_QUEUE_NUM] = {
    {
        .p_buf_start = gs_buffer,
        .buffer_len  = BLE_GATTS_QUEUE_BUFFER_LEN,
        .p_queue     = gs_queue_elms,
        .queue_size  = BLE_GATTS_QUEUE_ELEMENTS_SIZE,
    }
};

/******************************************************************************
Generated function definitions
*******************************************************************************/

/******************************************************************************
 * Function Name: gatts_cb
 * Description  : Callback function for GATT Server API.
 * Arguments    : uint16_t type -
 *                  Event type of GATT Server API.
 *              : ble_status_t result -
 *                  Event result of GATT Server API.
 *              : st_ble_gatts_evt_data_t *p_data -
 *                  Event parameters of GATT Server API.
 * Return Value : none
 ******************************************************************************/
void gatts_cb(uint16_t type, ble_status_t result, st_ble_gatts_evt_data_t *p_data)
{
/* Hint: Input common process of callback function such as variable definitions */
/* Start user code for GATT Server callback function common process. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

	R_BLE_SERVS_GattsCb(type, result, p_data);
    switch(type)
    {
/* Hint: Add cases of GATT Server event macros defined as BLE_GATTS_XXX */
/* Start user code for GATT Server callback function event process. Do not edit comment generated here */
        default:
		{
			printk("BLE GATTS cb unhandled event: %x\n", type);
			printk("BLE GATSS cb result: %d\n", result);
		}
/* End user code. Do not edit comment generated here */
    }
}

/******************************************************************************
 * Function Name: gattc_cb
 * Description  : Callback function for GATT Client API.
 * Arguments    : uint16_t type -
 *                  Event type of GATT Client API.
 *              : ble_status_t result -
 *                  Event result of GATT Client API.
 *              : st_ble_gattc_evt_data_t *p_data -
 *                  Event parameters of GATT Client API.
 * Return Value : none
 ******************************************************************************/
void gattc_cb(uint16_t type, ble_status_t result, st_ble_gattc_evt_data_t *p_data)
{
/* Hint: Input common process of callback function such as variable definitions */
/* Start user code for GATT Client callback function common process. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

    R_BLE_SERVC_GattcCb(type, result, p_data);
    switch(type)
    {

    default:
	{
		printk("BLE GATTC cb unhandled event: %x\n", type);
		printk("BLE GATTC cb result: %d\n", result);
	}

/* Hint: Add cases of GATT Client event macros defined as BLE_GATTC_XXX */
/* Start user code for GATT Client callback function event process. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
    }
}

/* GATT server callback parameters */
ble_abs_gatt_server_callback_set_t gs_abs_gatts_cb_param[] =
{
    {
        .gatt_server_callback_function = gatts_cb,
        .gatt_server_callback_priority = 1,
    },
    {
        .gatt_server_callback_function = NULL,
    }
};

/* GATT client callback parameters */
ble_abs_gatt_client_callback_set_t gs_abs_gattc_cb_param[] =
{
    {
        .gatt_client_callback_function = gattc_cb,
        .gatt_client_callback_priority = 1,
    },
    {
        .gatt_client_callback_function = NULL,
    }
};

/******************************************************************************
 * Function Name: vs_cb
 * Description  : Callback function for Vendor Specific API.
 * Arguments    : uint16_t type -
 *                  Event type of Vendor Specific API.
 *              : ble_status_t result -
 *                  Event result of Vendor Specific API.
 *              : st_ble_vs_evt_data_t *p_data -
 *                  Event parameters of Vendor Specific API.
 * Return Value : none
 ******************************************************************************/
void vs_cb(uint16_t type, ble_status_t result, st_ble_vs_evt_data_t *p_data)
{
/* Hint: Input common process of callback function such as variable definitions */
/* Start user code for vender specific callback function common process. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
	fsp_err_t status = 0;
   // R_BLE_SERVS_VsCb(type, result, p_data);
    switch(type)
    {
        case BLE_VS_EVENT_GET_ADDR_COMP:
        {
            /* Start advertising when BD address is ready */
            st_ble_vs_get_bd_addr_comp_evt_t * get_address = (st_ble_vs_get_bd_addr_comp_evt_t *)p_data->p_param;
            memcpy(g_ble_advertising_parameter.own_bluetooth_address, get_address->addr.addr, BLE_BD_ADDR_LEN);
            status = RM_BLE_ABS_StartLegacyAdvertising(&g_ble_abs0_ctrl, &g_ble_advertising_parameter);
            printk("BLE address: %x:%x:%x:%x:%x:%x\n", get_address->addr.addr[0], get_address->addr.addr[1], get_address->addr.addr[2], get_address->addr.addr[3], get_address->addr.addr[4], get_address->addr.addr[5]);
            printk("BLE adverising status: %d\n", status);
        } break;

        case BLE_VS_EVENT_GET_TX_POWER:
        {
        	st_ble_vs_get_tx_pwr_comp_evt_t * tx_pwr = (st_ble_vs_get_tx_pwr_comp_evt_t *)p_data->p_param;
        	printk("BLE get tx power ev result: %d\n", result);
        	printk("BLE current pwr: %d max pwr: %d\n", tx_pwr->curr_tx_pwr, tx_pwr->max_tx_pwr);

        }break;

        default:
		{
			printk("BLE VS cb unhandled event: %x\n", type);
			printk("BLE VS cb result: %d\n", result);
		}

/* Hint: Add cases of vender specific event macros defined as BLE_VS_XXX */
/* Start user code for vender specific callback function event process. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
    }
}

/******************************************************************************
 * Function Name: gap_cb
 * Description  : Callback function for GAP API.
 * Arguments    : uint16_t type -
 *                  Event type of GAP API.
 *              : ble_status_t result -
 *                  Event result of GAP API.
 *              : st_ble_vs_evt_data_t *p_data -
 *                  Event parameters of GAP API.
 * Return Value : none
 ******************************************************************************/
void gap_cb(uint16_t type, ble_status_t result, st_ble_evt_data_t *p_data)
{
/* Hint: Input common process of callback function such as variable definitions */
/* Start user code for GAP callback function common process. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

    switch(type)
    {
        case BLE_GAP_EVENT_STACK_ON:
        {
            /* Get BD address for Advertising */
            R_BLE_VS_GetBdAddr(BLE_VS_ADDR_AREA_REG, BLE_GAP_ADDR_RAND);
            printk("BLE GAP cb: %s\n", "BLE_GAP_EVENT_STACK_ON");
        } break;

        case BLE_GAP_EVENT_CONN_IND:
        {
            if (BLE_SUCCESS == result)
            {
                /* Store connection handle */
                st_ble_gap_conn_evt_t *p_gap_conn_evt_param = (st_ble_gap_conn_evt_t *)p_data->p_param;
                g_conn_hdl = p_gap_conn_evt_param->conn_hdl;

                /* Send Connection update to ensure RF Low Power */
                R_BLE_GAP_UpdConn(p_gap_conn_evt_param->conn_hdl,
                                    BLE_GAP_CONN_UPD_MODE_REQ,
                                    BLE_GAP_CONN_UPD_ACCEPT,
                                    &g_conn_updt_param);
            }
            else
            {
                /* Restart advertising when connection failed */
                RM_BLE_ABS_StartLegacyAdvertising(&g_ble_abs0_ctrl, &g_ble_advertising_parameter);
            }
            printk("BLE GAP cb: %s\n", "BLE_GAP_EVENT_CONN_IND");
        } break;

        case BLE_GAP_EVENT_DISCONN_IND:
        {
            /* LED OFF */
            //g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_04_PIN_04, BSP_IO_LEVEL_HIGH);
            //g_led_blink_active = false;

            /* Restart advertising when disconnected */
            g_conn_hdl = BLE_GAP_INVALID_CONN_HDL;
            RM_BLE_ABS_StartLegacyAdvertising(&g_ble_abs0_ctrl, &g_ble_advertising_parameter);
            printk("BLE GAP cb: %s\n", "BLE_GAP_EVENT_DISCONN_IND");
        } break;

        case BLE_GAP_EVENT_CONN_PARAM_UPD_REQ:
        {
            /* Send connection update response with value received on connection update request */
            st_ble_gap_conn_upd_req_evt_t *p_conn_upd_req_evt_param = (st_ble_gap_conn_upd_req_evt_t *)p_data->p_param;

            st_ble_gap_conn_param_t conn_updt_param = {
                .conn_intv_min = p_conn_upd_req_evt_param->conn_intv_min,
                .conn_intv_max = p_conn_upd_req_evt_param->conn_intv_max,
                .conn_latency  = p_conn_upd_req_evt_param->conn_latency,
                .sup_to        = p_conn_upd_req_evt_param->sup_to,
            };

            R_BLE_GAP_UpdConn(p_conn_upd_req_evt_param->conn_hdl,
                              BLE_GAP_CONN_UPD_MODE_RSP,
                              BLE_GAP_CONN_UPD_ACCEPT,
                              &conn_updt_param);
            printk("BLE GAP cb: %s\n", "BLE_GAP_EVENT_CONN_PARAM_UPD_REQ");
        } break;

        case BLE_GAP_EVENT_ADV_ON:
        {
        	st_ble_gap_adv_set_evt_t * adv_ev = (st_ble_gap_adv_set_evt_t *)p_data->p_param;
        	g_conn_hdl = adv_ev->adv_hdl;
        	printk("BLE GAP cb: %s\n", "BLE_GAP_EVENT_ADV_ON");
        	printk("BLE adv on result: %d\n", result);
        	printk("BLE adv on hdl: %d\n", adv_ev->adv_hdl);
        }break;

        default:
        {
        	printk("BLE GAP cb unhandled event: %x\n", type);
        	printk("BLE GAP cb result: %d\n", result);
        }

/* Hint: Add cases of GAP event macros defined as BLE_GAP_XXX */
/* Start user code for GAP callback function event process. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
    }
}

/******************************************************************************
 * Function Name: lss_cb
 * Description  : Callback function for LED Switch Service server feature.
 * Arguments    : uint16_t type -
 *                  Event type of LED Switch Service server feature.
 *              : ble_status_t result -
 *                  Event result of LED Switch Service server feature.
 *              : st_ble_servs_evt_data_t *p_data -
 *                  Event parameters of LED Switch Service server feature.
 * Return Value : none
 ******************************************************************************/
static void lss_cb(uint16_t type, ble_status_t result, st_ble_servs_evt_data_t *p_data)
{
/* Hint: Input common process of callback function such as variable definitions */
/* Start user code for LED Switch Service Server callback function common process. Do not edit comment generated here */
    uint16_t    data;
    FSP_PARAMETER_NOT_USED(result);
    struct ble_led_data_t tx_data;
/* End user code. Do not edit comment generated here */

    switch(type)
    {
/* Hint: Add cases of LED Switch Service server events defined in e_ble_lss_event_t */
/* Start user code for LED Switch Service Server callback function event process. Do not edit comment generated here */
        case BLE_LSS_EVENT_SWITCH_STATE_CLI_CNFG_WRITE_COMP :
        {
            R_BLE_LSS_GetSwitchStateCliCnfg(p_data->conn_hdl, &data);

          /*  if (data)
                set_lss_event(LSS_WAIT_EN_CCCD);
            else
                set_lss_event(LSS_WAIT_DIS_CCCD);
           */
            printk("BLE LSS cb BLE_LSS_EVENT_SWITCH_STATE_CLI_CNFG_WRITE_COMP event: %x\n", type);
        } break;

        case BLE_LSS_EVENT_BLINK_RATE_WRITE_COMP:
        {
           // set_lss_event(LSS_WAIT_WR_BLINK);
        	printk("BLE LSS cb BLE_LSS_EVENT_BLINK_RATE_WRITE_COMP event: %x\n", type);
        } break;

        case BLE_LSS_EVENT_BLINK_RATE_WRITE_REQ:
		{
			tx_data.cnt = *(uint8_t *)p_data->p_param;

			if(tx_data.cnt > 0 && tx_data.cnt < 0xFF)
			{
				k_fifo_put(&ble_led_fifo, &tx_data);
			}
			else
			{
				printk("BLE LLS write event: value out of range\n");
			}

			printk("BLE LSS cb BLE_LSS_EVENT_BLINK_RATE_WRITE_REQ event: %x\n", type);
		} break;

        default:
		{
			printk("BLE LSS cb unhandled event: %x\n", type);
			printk("BLE LSS cb result: %d\n", result);
		}

/* End user code. Do not edit comment generated here */
    }
}

uint16_t ble_app_init()
{
	uint16_t status = 0;
    fsp_err_t err = 0;

    /* Open IO port */
	g_ioport.p_api->open(g_ioport.p_ctrl, g_ioport.p_cfg);

	/* Enable Interrupt (push switch) */
	g_ble_sw_irq.p_api->open(g_ble_sw_irq.p_ctrl, g_ble_sw_irq.p_cfg);
	g_ble_sw_irq.p_api->enable(g_ble_sw_irq.p_ctrl);

    /* Initialize BLE */
    err = RM_BLE_ABS_Open(&g_ble_abs0_ctrl, &g_ble_abs0_cfg);
    if (FSP_SUCCESS != err)
    {
    	return err;
    }

    /* Initialize GATT Database */
    status = R_BLE_GATTS_SetDbInst(&g_gatt_db_table);
    if (BLE_SUCCESS != status)
    {
    	printk("*** ble app init GATT Database failed with status: %d ***\n", status);
        return BLE_ERR_INVALID_OPERATION;
    }

    /* Initialize GATT server */
    status = R_BLE_SERVS_Init();
    if (BLE_SUCCESS != status)
    {
    	printk("*** ble app init GATT server failed with status: %d ***\n", status);
        return BLE_ERR_INVALID_OPERATION;
    }

    /*Initialize GATT client */
    status = R_BLE_SERVC_Init();
    if (BLE_SUCCESS != status)
    {
    	printk("*** ble app init GATT client failed with status: %d ***\n", status);
        return BLE_ERR_INVALID_OPERATION;
    }

    /* Set Prepare Write Queue */
    R_BLE_GATTS_SetPrepareQueue(gs_queue, BLE_GATTS_QUEUE_NUM);

    /* Initialize LED Switch Service server API */
    status = R_BLE_LSS_Init(lss_cb);
    if (BLE_SUCCESS != status)
    {
    	printk("*** ble app init LED Switch failed with status: %d ***\n", status);
        return BLE_ERR_INVALID_OPERATION;
    }

    return status;
}

void ble_app_main()
{
	R_BLE_Execute();

	if (0 != R_BLE_IsTaskFree())
	{
	}
}

/******************************************************************************
 * Function Name: sw_cb
 * Description  : Send notification when pushing switch.
 * Arguments    : none
 * Return Value : none
 ******************************************************************************/
static void sw_cb(void)
{
    uint8_t state = 1;
    R_BLE_LSS_NotifySwitchState(g_conn_hdl, &state);
}

/******************************************************************************
 * Function Name: Callback_ble_sw_irq
 * Description  : Callback for push switch interrupt.
 * Arguments    : external_irq_callback_args_t *p_args
 * Return Value : none
 ******************************************************************************/
void Callback_ble_sw_irq(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    R_BLE_SetEvent(sw_cb);
}

