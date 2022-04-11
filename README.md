# Zephyr Driver for Qorvo/Decawave DW3000

This is a Zephyr module which implements a driver for Qorvo/Decawave DW3000.
It contains not much more than the `decadriver/` from the DWS3000_Release_v1.1 
(DW3000_API_C0_rev4p0) and the necessary Zephyr bindings for GPIO, SPI and DTS.

The main idea is that this module contains only the minimal code to drive the 
DW3000, so it can be used in different projects - and keep it clean from Decawave
example code and the port abstractions and genral mess around there.

It can be used by adding this as a zephyr module in `west.yml`, or by adding the module
to CMakeLists.txt, e.g.:

```
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/dw3000-decadriver/)
```

Then you only need to add a `decawave,dw3000` compatible device to your .dts, e.g.:

```
&spi0 {
	status = "okay";
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
		irq-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;
		wakeup-gpios = <&gpio0 8 GPIO_ACTIVE_LOW>;
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

And then you can use the functions defined in `dw3000.h` and `deca_device_api.h`.

There is a separate project which uses this driver for running the
Qorvo/Decawave DWS3000 examples here:
https://github.com/br101/zephyr-dw3000-examples

Thanks to https://github.com/foldedtoad/dwm3000
