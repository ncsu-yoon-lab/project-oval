# Project OVAL PCB

## Overview
The electronics system is designed and developed with the use of [KiCAD](https://www.kicad.org/) to incorporate the entire electrical system into a printed circuit board (PCB). The PCB will be manufactured by [JLCPCB](https://jlcpcb.com/). Developing a custom PCB will provide consistent connections and further adjustments as new revisions are made on the PCB.

## Next Steps
* Schematic needs to be checked
* Test ATTiny85 with I2C and PPM generation
* Figure out wiring for back LEDs
* Power switch for ESC
* Make the ESC directly attach to the PCB
* Remove the electromagnets
* Include a voltage sensor

## Components
Below are the descriptions of all the components considered and incorporated into the PCB

### Power
The power for the whole system is sourced from two [4s LiPo batteries](https://www.amazon.com/dp/B07Y1PYHKC?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_2). Turning the power on and off is directly controlled by a [DPDT switch](https://www.amazon.com/Twidec-Rocker-Toggle-Position-Waterproof/dp/B07LBMMN2Y/ref=sr_1_3?dib=eyJ2IjoiMSJ9.H1GVn0sV9nroy_uFaZHPdd5zl96OmoEBumcDbs2H2JQsCQWccp83BUf9UMrdTw5yn2Gh-R7XMM7Rhy0J8cPQUunQYQQI-I8DpZY4GHWhhZfa_H08-9tdhQhasKEfKwsORwEHOc4Qq9w2f_awsDMn9SciTHHxJvrF6eQ4TpXpnHJhAdd_uB5lI0awRb96oNr2xs-qYPh7MeJv7FwMkOd6xVZB6cMgUIFhQ-2nyexbUGw.bw0kXIxRu6rnZ9yNmepIGsJ9vjd-tjA7nIPwPtc036g&dib_tag=se&keywords=double%2Bpole%2Bdouble%2Bthrow%2Bswitch&qid=1748457096&sr=8-3&th=1). The power then goes through a [buck converter](https://www.amazon.com/WWZMDiB-Converter-Adjustable-Regulator-Protection/dp/B0B825HRB9/ref=sr_1_3?crid=X0NHTXOBZQBN&dib=eyJ2IjoiMSJ9.OVZkkMvTynFktJV0_FpqOz9G-79GCf5LbvPXwQiyANJTXUIdp-s9YyWXiubRIaMcQTqaOvUStJel0N4QgSzktCbwemDFsNzsKFPukLzzg9BAYQ7lPAzqoRV9t5gccvsWanqf6VVNeu8hhikXhyIUT0Np_qtc4yjpI7uwknCNYr7hk-_KrTu2k3UAdGd2GWSzxy316hXX1D9F4PrttX0l6_WxYWf5SUisVcA2T02Gyiw._XYZ6-QOanhcWZ5D29KLau23OnrpKjCr3EAjNbOWu8I&dib_tag=se&keywords=buck+converter+high+current&qid=1748453761&sprefix=buck+converter+high+current%2Caps%2C113&sr=8-3) to convert the output of the batteries to 12VDC and a max of 20A. This power will go to several features: cooling system, NVIDIA Jetson Orin AGX, monitoring LED, and the external 12VDC plug on the panel. A DC to DC converter is used to then convert the 12VDC to 5VDC to power the front LED and the touch screen.

### Charging
The batteries are expected to remain inside the electronics box at all times and are only able to be charged and discharged when the system is turned off via the [DPDT switch](https://www.amazon.com/Twidec-Rocker-Toggle-Position-Waterproof/dp/B07LBMMN2Y/ref=sr_1_3?dib=eyJ2IjoiMSJ9.H1GVn0sV9nroy_uFaZHPdd5zl96OmoEBumcDbs2H2JQsCQWccp83BUf9UMrdTw5yn2Gh-R7XMM7Rhy0J8cPQUunQYQQI-I8DpZY4GHWhhZfa_H08-9tdhQhasKEfKwsORwEHOc4Qq9w2f_awsDMn9SciTHHxJvrF6eQ4TpXpnHJhAdd_uB5lI0awRb96oNr2xs-qYPh7MeJv7FwMkOd6xVZB6cMgUIFhQ-2nyexbUGw.bw0kXIxRu6rnZ9yNmepIGsJ9vjd-tjA7nIPwPtc036g&dib_tag=se&keywords=double%2Bpole%2Bdouble%2Bthrow%2Bswitch&qid=1748457096&sr=8-3&th=1). The batteries will be ran in parallel to extend the battery life of the 2 4s LiPo batteries to last an hour of driving at half speed.

### Motor Control
Each motor will be driven via an [ATTiny85](https://www.microchip.com/en-us/product/attiny85). The two ATTiny85s are an AVR that will be on an I2C bus with the NVIDIA Jetson Orin AGX as the parent to receive a command on the throttle. The ATTiny85 will separately output a PPM signal to the ESCs to control the throttle of the motors. Currently untested.

### Access Panel
Inside the tailgate of the system there will be an access panel to all the features inside the electronics box to avoid having to remove the electronics box at any time. The access panel will include charging ports for battery 1 and 2, USB A and USB C ports to the Orin, pins to the Orin, OLED screen displaying battery life, LED displaying power on/off, display port to the Orin, and the toggle switch for powering the system, and a DC jack to access 12VDC if needed for testing.
