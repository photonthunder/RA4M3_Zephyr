# RA4M3_Zephyr

This repository contains code that is not functional nor complete.

This repository contains an attempt to port the Renesas EK-RA4M3 to Zephyr.  My major holdup was in soc.c.  I did not fully understand what should be added in that file and what could be left out.

It does contain code for GPIO and UART, but obviously without any testing.   The ra4m3-pinctrl.dtsi file setups the different pinctrl options found in ek_ra4m3.dts.

Another challenge with the RA series is that the Renesas configurator will assign IRQ vector map, so in this version I was adding the generated code to the specific board under fsp_cfg and ra_gen.
