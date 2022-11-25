---
layout: article
title: CO2 Monitor
excerpt: I built a device for monitoring the CO2 concentration of indoor air. It may be used e.g. in the office to remind one when it is time to ventilate the room. 
---
I built a device for monitoring the CO2 concentration of indoor air. It may be used e.g. in the office to remind one when it is time to ventilate the room.

<figure>
    <img src="/assets/images/co2_front.JPG" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>

<figure>
    <img src="/assets/images/co2_back.JPG" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>

<figure>
    <img src="/assets/images/co2_open.JPG" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>

## Parts Used
I used commonly available electronic parts for this build, the housing is a matter of some wood working and not discussed here. Electronically, the device consists of three main parts. As you can see in the figure above, on the bottom there is an ESP32 microcontroller board, namely the [ESP32 Dev Kit C V4](https://www.az-delivery.de/en/products/esp-32-dev-kit-c-v4) by AZ-Delivery from which I removed the pre-soldered pin headers. On top, there is an [MH-Z19B](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf) CO2 sensor. The CO2 concentration is displayed on a [128 x 64 pixels 0.96 inch OLED display](https://www.az-delivery.de/en/products/0-96zolldisplay), which can be seen on the left in the above figure.

The microcontroller board is held in place in the wooden housing with the help of double sided sticky tape. The sensor is held in place by some hot glue. Through the front window of the wooden housing, only part of the display is visible. Double sided sticky tape was used on the parts of the display which are covered by the front part of the housing, and additionally some hot glue came into use.

The device is powered via a micro USB cable plugged into the microcontroller board (via this cable the microcontroller is also programmed).

## Wiring

<figure>
    <img src="/assets/images/co2_wiring.png" alt="MISSING IMAGE" style="width:100%"/>
    <figcaption></figcaption>
</figure>

The sensor needs 5V and is supplied directly from the corresponding pins of the microcontroller board. The ESP32 is actually a 3.3V microcontroller. Fortunately, the interface level of the MH-Z19B pins for UART is 3.3V (though the MH-Z19B tolerates 5V on its UART pins). I did not find a [fritzing](https://fritzing.org/) model of the ESP32 Dev Kit C V4 by AZ-Delivery, but the ESP32 board which I used in the above wiring diagram has the same pinout as the ESP32 Dev Kit C V4.

## Code
All the code can be found in the github repository [https://github.com/conrad-gst/co2-monitor](https://github.com/conrad-gst/co2-monitor). The microcontroller is programmed in MicroPython. The microcontroller requests a measurement from the sensor every three seconds and updates the screen accordingly. The microcontroller and the sensor communicate via UART, whereas I2C is used for the communication between the microcontroller and the display. The main program running on the microcontroller is listed below.

#### main.py
<font size="3">
{% highlight python linenos %}
from machine import Pin, SoftI2C
from time import sleep
import mhz19b
import ssd1306_new
import consolaB
import writer

mhz_sensor = mhz19b.MHZ19BSensor(rx_sensor=17, tx_sensor=16) 
# connect RX and TX of the sensor to GPIO17 and GPIO16 of the microcontroller

i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
display_width = 128
display_height = 64
display = ssd1306_new.SSD1306_I2C(display_width, display_height, i2c)
display.contrast(100)
font_writer = writer.Writer(display, consolaB)

sleep(1)
n = 0
while True:
    try:
        co2 = mhz_sensor.measure()
        if co2 == None:
            raise Exception()
            
        display.fill(0)
        
        text1 = "{co2:d}".format(co2=co2)
        text2 = "ppm"
        text1_width = font_writer.stringlen(text1)
        text2_width = font_writer.stringlen(text2)
        space = 4 # space between text1 and text2 in pixel
        text_width = text1_width + space + text2_width
        font_writer.set_textpos(int((display_width - text_width) / 2), 20)
        font_writer.printstring(text1)
        font_writer.set_textpos(int((display_width - text_width) / 2 + 
        text1_width + space), 20)
        font_writer.printstring(text2)
        display.show()
        sleep(3)
    except Exception as e:
        display.fill(0)
        display.text("Reading sensor",0,20)
        display.text("failed.",0,35)
        display.show()
        sleep(0.5)
{% endhighlight %}
</font>

In line 8 of the main program, the class MHZ19BSensor is instantiated, i.e. a sensor object is created. The constructor of this class takes the microcontroller pins to which the RX and the TX pins of the sensor are connected (compare with the wiring diagram above). In line 11, an object of the class SoftI2C (which comes with the machine module) is created (instead of pin numbers for scl (serial clock) and sda (serial data), Pin objects have to be provided here). This I2C object is needed in line 14, where the class SSD1306_I2C is instantiated (SSD1306 is a very common driver chip for OLED displays, mine uses this chip). Finally, the Writer class is instantiated. I use this class in order to be able to use consolasB as font (the standard font coming with the SSD1306 class is very small). Then, in line 20, an endless loop is entered. A measurement is taken in line 22, in line 26, the display is cleared. The purpose of lines 28 to 38 is to display the new sensor reading centered on the display. After a delay of three seconds, the whole cycle repeats.

The following code for communicating with the MH-Z19B sensor is based on [https://github.com/artem-smotrakov/esp32-weather-google-sheets/blob/master/src/weather.py](https://github.com/artem-smotrakov/esp32-weather-google-sheets/blob/master/src/weather.py) by [artem-smotrakov](https://github.com/artem-smotrakov) (see also [https://github.com/carlesfg/MicroPython/blob/CO2-Sensor/MHZ19B.py](https://github.com/carlesfg/MicroPython/blob/CO2-Sensor/MHZ19B.py)).

#### mhz19b.py
<font size="3">
{% highlight python linenos %}
import time
from machine import UART

class MHZ19BSensor:

    # constructor
    def __init__(self, rx_sensor, tx_sensor):
        self.uart = UART(2, baudrate=9600, rx=tx_sensor, tx=rx_sensor, 
        bits=8, parity=None, stop=1, timeout = 1000, timeout_char = 1000)
        self.data = bytearray(9)
        self.request_data = b'\xff\x01\x86\x00\x00\x00\x00\x00\x79'
        self.measure() # dummy reading since the first reading usually fails
        
    # measure CO2
    def measure(self):
        # send a read command to the sensor
        self.uart.write(self.request_data)

        # read and validate the data
        self.uart.readinto(self.data,9)
        if self.is_valid():
            co2 = (self.data[2] << 8) + self.data[3]
            return co2
        else:
            self.reset()
            return None
        
    # check data returned by the sensor
    def is_valid(self):
        if self.data[0] != 0xFF or self.data[1] != 0x86:
            return False
        i = 1
        checksum = 0x00
        while i < 8:
            checksum += self.data[i] % 256
            i += 1
        checksum = ~checksum & 0xFF
        checksum += 1
        return checksum == self.data[8]
    
    def reset(self):
        self.uart.read(self.uart.any())

{% endhighlight %}
</font>

In the constructor of this class, an UART object is created. Furthermore, the bytearray self.data is declared, which is used as a buffer for data received via UART from the sensor. According to the [datasheet](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf) of the MH-Z19B, in order to request a reading from the sensor, a total of 9 Bytes has to be sent namely \xff\x01\x86\x00\x00\x00\x00\x00\x79, which is stored in self.request_data. The sensor responds with 9 Bytes, the actual data, i.e. the CO2 concentration is contained in Byte2 and Byte3 and calculated via concentration = Byte2 * 256 + Byte3 (in the above code, this calculation is performed in line 22 where the multiplication by 256 is replaced by a bit-shift). Byte8 is a checksum, this Byte must coincide with the checksum calculated from the remaining Bytes in the lines 33 to 38. (In the [datasheet](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf), a snippet of  C-code which demonstrates the calculation of the checksum is provided.)

The sensor accepts several other commands, e.g. for calibrating the sensor. These commands are not used here. The sensor readings are reasonable after all. Outdoors, it reads about 400 ppm. Ideally, indoor air should not exceed a CO2 concentration of 1000 ppm. Since the main purpose of this device is to remind one when it is time to ventilate the room, it does not really matter if readings are off by e.g. 50 ppm.

For controlling the OLED display, the following code from [https://github.com/micropython/micropython](https://github.com/micropython/micropython) has been used.

#### ssd1306.py
<font size="3">
{% highlight python linenos %}
# MicroPython SSD1306 OLED driver, I2C and SPI interfaces

from micropython import const
import framebuf


# register definitions
SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xA4)
SET_NORM_INV = const(0xA6)
SET_DISP = const(0xAE)
SET_MEM_ADDR = const(0x20)
SET_COL_ADDR = const(0x21)
SET_PAGE_ADDR = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_IREF_SELECT = const(0xAD)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CHARGE_PUMP = const(0x8D)

# Subclassing FrameBuffer provides support for graphics primitives
# http://docs.micropython.org/en/latest/pyboard/library/framebuf.html
class SSD1306(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP,  # display off
            # address setting
            SET_MEM_ADDR,
            0x00,  # horizontal
            # resolution and layout
            SET_DISP_START_LINE,  # start at line 0
            SET_SEG_REMAP | 0x01,  # column addr 127 mapped to SEG0
            SET_MUX_RATIO,
            self.height - 1,
            SET_COM_OUT_DIR | 0x08,  # scan from COM[N] to COM0
            SET_DISP_OFFSET,
            0x00,
            SET_COM_PIN_CFG,
            0x02 if self.width > 2 * self.height else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV,
            0x80,
            SET_PRECHARGE,
            0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL,
            0x30,  # 0.83*Vcc
            # display
            SET_CONTRAST,
            0xFF,  # maximum
            SET_ENTIRE_ON,  # output follows RAM contents
            SET_NORM_INV,  # not inverted
            SET_IREF_SELECT,
            0x30,  # enable internal IREF during display on
            # charge pump
            SET_CHARGE_PUMP,
            0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01,  # display on
        ):  # on
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def rotate(self, rotate):
        self.write_cmd(SET_COM_OUT_DIR | ((rotate & 1) << 3))
        self.write_cmd(SET_SEG_REMAP | (rotate & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width != 128:
            # narrow displays use centred columns
            col_offset = (128 - self.width) // 2
            x0 += col_offset
            x1 += col_offset
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b"\x40", None]  # Co=0, D/C#=1
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80  # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        self.i2c.writevto(self.addr, self.write_list)


class SSD1306_SPI(SSD1306):
    def __init__(self, width, height, spi, dc, res, cs, external_vcc=False):
        self.rate = 10 * 1024 * 1024
        dc.init(dc.OUT, value=0)
        res.init(res.OUT, value=0)
        cs.init(cs.OUT, value=1)
        self.spi = spi
        self.dc = dc
        self.res = res
        self.cs = cs
        import time

        self.res(1)
        time.sleep_ms(1)
        self.res(0)
        time.sleep_ms(10)
        self.res(1)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(buf)
        self.cs(1)
{% endhighlight %}
</font>

 Since the display is only used for displaying the current CO2 concentration, I wanted a large font which is well readable also from some distance. Therefore, I used the [micropython-font-to-py](https://github.com/peterhinch/micropython-font-to-py) project by [peterhinch](https://github.com/peterhinch). In this project, a command line tool called [font_to_py.py](https://github.com/peterhinch/micropython-font-to-py/blob/master/font_to_py.py) is provided, which is capable of converting a font file (.ttf-file or .otf-file, such files can be downloaded for free for a lot of fonts) into a python file which can then be used in conjunction with [writer.py](https://github.com/peterhinch/micropython-font-to-py/blob/master/writer/old_versions/writer_fw_compatible.py) to display text in this font. I have used CONSOLAB.ttf which I got from [https://www.fonts100.com/font+21195_Consolas.html](https://www.fonts100.com/font+21195_Consolas.html) and specified the height to be 25 pixels. The command line call for producing the corresponding micropython file is as follows:

<font size="3">
{% highlight shell %}
python3 font_to_py.py -x CONSOLAB.ttf 25 consolaB.py
{% endhighlight %}
</font>

As mentioned above, all the code can be cloned from my github repository [https://github.com/conrad-gst/co2-monitor](https://github.com/conrad-gst/co2-monitor).