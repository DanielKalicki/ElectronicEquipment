Electronic equipment
===================

Devices in this project:

1. Power Supply - 0-12V digital power supply, with low voltage capability. Based on Dave Johnes uSupply (link:  http://www.eevblog.com/projects/usupply/). This device is control by microcontroller Atmega16 and can receive and send commands using RS232 interface.
2. Waveform generator based on AD9833 - the output signal can be selected as sinus, triangle or square wave. DDS ic can generate frequencies up to 10MHz, with very small step size. The output signal amplitude is controlled by a potentiometer.
3. DC/DC Step up converter - this project consists only of an evaluation board for LTC3862
4. Decade resistor box - the correct resistor value is set using bistable relays (this relay retains it state without power). Resistors are organized in 2R values this configuration reduces the number of needed resistors to 6 (or five 2R and one 1R). This devices is controlled using an AVR Atmega8 microcontroller with 4094 shift register to increase the number of output pins.
5. Electronic DC Load - this device can act as constant current sink, constant power and resistance. It uses 16-bit AD5065 DAC to set the constant current value, and the input voltage and current is read back using AD7710 ADC with adjustable signal input gain. The current value is measured using a 10mOhm shunt resistor with a AD8552 low offset voltage (1uV) operational amplifier.
6. Programmable filter - This device uses AD9245 high speed ADC to aquire the input signal which is then filtered digitally using Silicon Labs SIM3U167 processor. Output waveform is generated using AD9117 DAC. Used microcontroller can be clocked at 80MHz, allowing the user to select high cutoff frequency filters.
