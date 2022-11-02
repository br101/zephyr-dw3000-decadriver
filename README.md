# Zephyr Driver for Qorvo/Decawave DW3000

This is a Zephyr module with a driver for Qorvo/Decawave DW3000. It contains the
official driver from Qorvo and adds the necessary Zephyr bindings for GPIO,
SPI and DTS. We tried to add only the minimal code to drive the DW3000, so it can
be used in different projects and keep it as clean as possible from Decawave
example code, port abstractions and the general mess around there.

* The 'master' branch uses the last release from Qorvo (DW3xxx_XR6.0C_24Feb2022.zip),
which unfortunately is a binary-only library and only available for NRF targets.

* There is an 'opensource' branch which contains the last open source release from 
Qorvo (DWS3000_Release_v1.1 / DW3000_API_C0_rev4p0), but this is older and not well
tested any more.

The driver can be used by adding this repository as a zephyr module in `west.yml`, or
by adding the module to CMakeLists.txt, e.g.:

```
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/dw3000-decadriver/)
```

Then you only need to add a `decawave,dw3000` compatible device to your .dts, e.g.:

```
&spi0 {
	status = "okay";
	compatible = "nordic,nrf-spim";
	sck-pin = <2>;
	mosi-pin = <20>;
	miso-pin = <3>;
	cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;

	dw3000@0 {
		compatible = "decawave,dw3000";
		label = "DW3000";
		spi-max-frequency = <1000000>;
		reg = <0>;
		reset-gpios = <&gpio0 9 GPIO_ACTIVE_LOW>;
		irq-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
		wakeup-gpios = <&gpio0 8 GPIO_ACTIVE_HIGH>;
		//spi-pol-gpios = <&gpio0 0 GPIO_ACTIVE_LOW>;
		//spi-pha-gpios = <&gpio0 0 GPIO_ACTIVE_LOW>;
	};
};
```

And

```
CONFIG_DW3000=y
CONFIG_SPI=y
CONFIG_GPIO=y
```

After that you can use the functions defined in `dw3000.h` and `deca_device_api.h`,
which would usually be like: 

```
dw3000_hw_init();
dw3000_hw_reset();
dw3000_hw_init_interrupt();
dw3000_spi_speed_fast();

int ret = dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);
if (ret < 0) {
	LOG_ERR("DWT Probe failed");
	return;
}
```

There is a separate project which uses this driver for the Qorvo/Decawave DWS3000 
examples here: https://github.com/br101/zephyr-dw3000-examples

Thanks to https://github.com/foldedtoad/dwm3000 for an earlier Zephyr version of
the driver + example code.
