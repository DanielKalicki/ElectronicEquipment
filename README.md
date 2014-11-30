Electronic equipment
===================

Devices in this project:

1. Power Supply - 0-12V digital power supply, with low voltage capability. Based on Dave Johnes uSupply (link:  http://www.eevblog.com/projects/usupply/). This device is control by microcontroller Atmega16 and can be control by RS232 interface.
2. Waveform generator based on AD9833 - the output signal can be selected as sinus, triangle or square wave. DDS ic can generate frequencies up to 10MHz, with very small step size. The output signal amplitude is controlled by a potentiometer.
3. DC/DC Step up converter
	3.1 evaluation board for LTC3862
4. Programmable filter - this device uses AD9245 high speed ADC to aquire the input signal which is then filtered digitally using Silicon Labs SIM3U167 processor. Output waveform is generated using AD9117 DAC. Used microcontroller can be clocked at 80MHz, allowing the user to select high cutoff frequency.
	4.1 RevA - ADC module layout based on AD9244.
