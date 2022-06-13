/*
 * ra-pinctrl.h
 *
 *  Created on: Nov 19, 2020
 *      Author: zoltan.ianosi.bp
 */

#ifndef INCLUDE_DT_BINDINGS_PINCTRL_RA_PINCTRL_H_
#define INCLUDE_DT_BINDINGS_PINCTRL_RA_PINCTRL_H_


#define IOPORT_PRV_PERIPHERAL_FUNCTION    (1U << 16)

/* Private definition to set enumeration values. */
#define IOPORT_PRV_PFS_PSEL_OFFSET    (24)

/** Superset of all peripheral functions.  */

    /** Pin will functions as an IO pin */
#define     IOPORT_PERIPHERAL_IO   0x00

    /** Pin will function as a DEBUG pin */
#define     IOPORT_PERIPHERAL_DEBUG   (0x00UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as an AGT peripheral pin */
#define     IOPORT_PERIPHERAL_AGT   (0x01UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a GPT peripheral pin */
#define     IOPORT_PERIPHERAL_GPT0   (0x02UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a GPT peripheral pin */
#define     IOPORT_PERIPHERAL_GPT1   (0x03UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as an SCI peripheral pin */
#define     IOPORT_PERIPHERAL_SCI0_2_4_6_8   (0x04UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as an SCI peripheral pin */
#define     IOPORT_PERIPHERAL_SCI1_3_5_7_9   (0x05UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a SPI peripheral pin */
#define     IOPORT_PERIPHERAL_SPI   (0x06UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a IIC peripheral pin */
#define     IOPORT_PERIPHERAL_IIC   (0x07UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a KEY peripheral pin */
#define     IOPORT_PERIPHERAL_KEY   (0x08UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a clock/comparator/RTC peripheral pin */
#define     IOPORT_PERIPHERAL_CLKOUT_COMP_RTC   (0x09UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a CAC/ADC peripheral pin */
#define     IOPORT_PERIPHERAL_CAC_AD   (0x0AUL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a BUS peripheral pin */
#define     IOPORT_PERIPHERAL_BUS   (0x0BUL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a CTSU peripheral pin */
#define     IOPORT_PERIPHERAL_CTSU   (0x0CUL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a segment LCD peripheral pin */
#define     IOPORT_PERIPHERAL_LCDC   (0x0DUL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a DALI peripheral pin */
#define     IOPORT_PERIPHERAL_DALI   (0x0EUL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a CAN peripheral pin */
#define     IOPORT_PERIPHERAL_CAN   (0x10UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a QSPI peripheral pin */
#define     IOPORT_PERIPHERAL_QSPI   (0x11UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as an SSI peripheral pin */
#define     IOPORT_PERIPHERAL_SSI   (0x12UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a USB full speed peripheral pin */
#define     IOPORT_PERIPHERAL_USB_FS   (0x13UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a USB high speed peripheral pin */
#define     IOPORT_PERIPHERAL_USB_HS   (0x14UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as an SD/MMC peripheral pin */
#define     IOPORT_PERIPHERAL_SDHI_MMC   (0x15UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as an Ethernet MMI peripheral pin */
#define     IOPORT_PERIPHERAL_ETHER_MII   (0x16UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as an Ethernet RMMI peripheral pin */
#define     IOPORT_PERIPHERAL_ETHER_RMII   (0x17UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a PDC peripheral pin */
#define     IOPORT_PERIPHERAL_PDC   (0x18UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a graphics LCD peripheral pin */
#define     IOPORT_PERIPHERAL_LCD_GRAPHICS   (0x19UL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a debug trace peripheral pin */
#define     IOPORT_PERIPHERAL_TRACE   (0x1AUL << IOPORT_PRV_PFS_PSEL_OFFSET)

    /** Pin will function as a OSPI peripheral pin */
#define     IOPORT_PERIPHERAL_OSPI   (0x1CUL << IOPORT_PRV_PFS_PSEL_OFFSET)

/** Options to configure pin functions  */

#define     IOPORT_CFG_PORT_DIRECTION_INPUT    0x00000000 ///< Sets the pin direction to input (default)
#define     IOPORT_CFG_PORT_DIRECTION_OUTPUT   0x00000004 ///< Sets the pin direction to output
#define     IOPORT_CFG_PORT_OUTPUT_LOW         0x00000000 ///< Sets the pin level to low
#define     IOPORT_CFG_PORT_OUTPUT_HIGH        0x00000001 ///< Sets the pin level to high
#define     IOPORT_CFG_PULLUP_ENABLE           0x00000010 ///< Enables the pin's internal pull-up
#define     IOPORT_CFG_PIM_TTL                 0x00000020 ///< Enables the pin's input mode
#define     IOPORT_CFG_NMOS_ENABLE             0x00000040 ///< Enables the pin's NMOS open-drain output
#define     IOPORT_CFG_PMOS_ENABLE             0x00000080 ///< Enables the pin's PMOS open-drain ouput
#define     IOPORT_CFG_DRIVE_MID               0x00000400 ///< Sets pin drive output to medium
#define     IOPORT_CFG_DRIVE_HS_HIGH           0x00000800 ///< Sets pin drive output to high along with supporting high speed
#define     IOPORT_CFG_DRIVE_MID_IIC           0x00000C00 ///< Sets pin to drive output needed for IIC on a 20mA port
#define     IOPORT_CFG_DRIVE_HIGH              0x00000C00 ///< Sets pin drive output to high
#define     IOPORT_CFG_EVENT_RISING_EDGE       0x00001000 ///< Sets pin event trigger to rising edge
#define     IOPORT_CFG_EVENT_FALLING_EDGE      0x00002000 ///< Sets pin event trigger to falling edge
#define     IOPORT_CFG_EVENT_BOTH_EDGES        0x00003000 ///< Sets pin event trigger to both edges
#define     IOPORT_CFG_IRQ_ENABLE              0x00004000 ///< Sets pin as an IRQ pin
#define     IOPORT_CFG_ANALOG_ENABLE           0x00008000 ///< Enables pin to operate as an analog pin
#define     IOPORT_CFG_PERIPHERAL_PIN          0x00010000  ///< Enables pin to operate as a peripheral pin


#endif /* INCLUDE_DT_BINDINGS_PINCTRL_RA_PINCTRL_H_ */
