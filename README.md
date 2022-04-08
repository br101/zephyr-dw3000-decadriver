# Zephyr Driver for Qorvo/Decawave DW3000

This is a Zephyr module which implements a driver of Qorvo/Decawave DW3000.
It contains not much more than the `decadriver/` from the DWS3000_Release_v1.1 
(DW3000_API_C0_rev4p0) and the necessary Zephyr bindings for GPIO, SPI and DTS.

It can be used by adding this as a zephyr module in `west.yml`. Then you need to
add a `decawave,dw3000` compatible device to your .dts, e.g.:

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
		wakeup-gpios = <&gpio0 1 GPIO_ACTIVE_LOW>;
		spi-pol-gpios = <&gpio0 0 GPIO_ACTIVE_LOW>;
		spi-pha-gpios = <&gpio0 0 GPIO_ACTIVE_LOW>;
	};
};
```

And 

```
CONFIG_DW3000=y
CONFIG_SPI=y
CONFIG_GPIO=y
```
