# Zephyr Driver for Qorvo/Decawave DW3000

This is a Zephyr module with a driver for Qorvo/Decawave DW3000. It contains the
official source release of the "dwt_uwb_driver" driver from Qorvo 
(version 08.02.02 from DW3_QM33_SDK_1.0.1.zip) and adds the necessary Zephyr
bindings for GPIO, SPI and DTS. We tried to add only the minimal code to drive
the DW3000, so it can be used in different projects and keep it as clean as
possible from Decawave example code, port abstractions and the general mess
around there. The driver files released from Qorvo have been modified to support
only one DW3000 chip per board and we removed the big IOCTL function which is
a unnecessary huge waste of space on embedded platforms.

There is a similar project https://github.com/br101/dw3000-decadriver-source which
contains the same driver and Zephyr support but also supports other platforms such
as ESP32 and the old NRF SDK. Most of the development will go there first.

The driver can be used by adding this repository as a zephyr module in
`west.yml`, or by adding the module to CMakeLists.txt, e.g.:

```
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/dw3000-decadriver/)
```

Then you only need to add a `decawave,dw3000` compatible device to your .dts, e.g.:

```
&spi0 {
	status = "okay";
	compatible = "nordic,nrf-spim";
	cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;

	pinctrl-0 = <&spi0_default>;
	pinctrl-1 = <&spi0_sleep>;
	pinctrl-names = "default", "sleep";

	dw3000@0 {
		compatible = "decawave,dw3000";
		spi-max-frequency = <1000000>;
		reg = <0>;
		reset-gpios = <&gpio0 9 GPIO_ACTIVE_LOW>;
		irq-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
		wakeup-gpios = <&gpio0 8 GPIO_ACTIVE_HIGH>;
		//spi-pol-gpios = <&gpio0 0 GPIO_ACTIVE_LOW>;
		//spi-pha-gpios = <&gpio0 0 GPIO_ACTIVE_LOW>;
	};
};

&pinctrl {
	spi0_default: spi0_default {
		group1 {
			psels = <NRF_PSEL(SPIM_SCK, 0, 28)>,
				<NRF_PSEL(SPIM_MOSI, 0, 31)>,
				<NRF_PSEL(SPIM_MISO, 0, 3)>;
		};
	};

	spi0_sleep: spi0_sleep {
		group1 {
			psels = <NRF_PSEL(SPIM_SCK, 0, 28)>,
				<NRF_PSEL(SPIM_MOSI, 0, 31)>,
				<NRF_PSEL(SPIM_MISO, 0, 3)>;
			low-power-enable;
		};
	};
};
```

And

```
CONFIG_DW3000=y
CONFIG_DW3000_CHIP_DW3000=y
CONFIG_SPI=y
CONFIG_GPIO=y
```

After that you can use the functions defined in `dw3000.h` and `deca_device_api.h`,
which would usually be like:

```
#include <dw3000.h>
...
dw3000_hw_init();
dw3000_hw_reset();
dw3000_hw_init_interrupt();
dw3000_spi_speed_fast();

int ret = dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);
if (ret < 0) {
	LOG_ERR("DWT Probe failed");
	return;
}

ret = dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID | DWT_READ_OTP_BAT
					 | DWT_READ_OTP_TMP);
```

There is a separate project which uses this driver for the Qorvo/Decawave DWS3000
examples here: https://github.com/br101/zephyr-dw3000-examples (may be out of date).

Thanks to https://github.com/foldedtoad/dwm3000 for an earlier Zephyr version of
the driver + example code.
