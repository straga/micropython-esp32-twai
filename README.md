# micropython-esp32-twai

This repository adds TWAI/CAN(USER_C_MODULES) support to MicroPython for the ESP32 family.
Use user_module, that draft probe. But works.


## Example

```
import esp
esp.osdebug(esp.LOG_INFO)
import uasyncio as asyncio
import CAN

dev = CAN(0, extframe=False, tx=4, rx=5, mode=CAN.NORMAL, baudrate=500000, auto_restart=False)

from scrivo import logging
log = logging.getLogger("CAN")
log.setLevel(logging.DEBUG)

# - identifier of can packet (int)
# - extended packet (bool)
# - rtr packet (bool)
# - data frame (0..8 bytes)

async def run():
    i = 0
    while True:
        # log.debug("-")
        if dev.any():
            data = dev.recv()
            log.debug(f"id:{hex(data[0])}, ex:{data[1]}, rtr:{data[2]}, data:{data[3]}")
        await asyncio.sleep(0.01)

        # i += 1
        # if i == 10:
        #     log.debug("send ACK")
        #     dev.send([0,0,0,0,0,0,0,0], 0x305)
        #     i = 0

loop = asyncio.get_event_loop()
loop.create_task(run())
```

Grab data from BMS <-> Inverter
```
DEBUG:CAN:id:0x35e, ex:False, rtr:False, data:b'PYLON   '
DEBUG:CAN:id:0x359, ex:False, rtr:False, data:b'\x00\x00\x00\x00\x03\x00\x00\x00'
DEBUG:CAN:id:0x351, ex:False, rtr:False, data:b'(\x02\xe8\x03\xdc\x05\xe0\x01'
DEBUG:CAN:id:0x355, ex:False, rtr:False, data:b'a\x00d\x00'
DEBUG:CAN:id:0x356, ex:False, rtr:False, data:b'x\x14\xfd\xff\xa0\x00'
DEBUG:CAN:id:0x35c, ex:False, rtr:False, data:b'\xc0\x00'
DEBUG:CAN:id:0x70,  ex:False, rtr:False, data:b'\xa0\x00\x96\x00G\x01G\x01'
DEBUG:CAN:id:0x371, ex:False, rtr:False, data:b'\x01\x00\x02\x00\x0f\x00\x06\x00'
DEBUG:CAN:id:0x35e, ex:False, rtr:False, data:b'PYLON   '
DEBUG:CAN:id:0x359, ex:False, rtr:False, data:b'\x00\x00\x00\x00\x03\x00\x00\x00'
DEBUG:CAN:id:0x351, ex:False, rtr:False, data:b'(\x02\xe8\x03\xdc\x05\xe0\x01
```


```
convert from b'(\x02\xf4\x01\xe8\x03\xe0\x01' to human readable.
get bytes description from:
Byte 0 - Byte 1, Battery charge voltage, Unit: 0.1V, 16 bits unsigned int
Byte 2 - Byte 3, Charge current limit, Unit: 0.1A, 16 bits signed int, 2`s complement
Byte 4 - Byte 5, Discharge current limit, Unit: 0.1A, 16 bits signed int, 2`s complement
Byte 6 - Byte 7, None

# data = b'(\x02\xf4\x01\xe8\x03\xe0\x01'
# #convert to int cut first 2 bytes
# data = int.from_bytes(data[2:], 'little', signed=False)

```





## DIY

Read this section if you want to include the ESP32 TWAI/CAN support to MicroPython from scratch. To do that follow these steps:
  
- Note: The steps below now also work for MicroPython "esp32 with PSRAM", "1.22.0-preview" , ESP-IDF v5.1.2

1. Clone the MicroPython repository:
    ```
    git clone --recursive https://github.com/micropython/micropython.git
    ```
  
2. git clone  https://github.com/vostraga/micropython-esp32-twai.git
    Copy the files and folders inside the root folder where `micropython`.
    Add to board - mpconfigboard.h
    #define MODULE_CAN_ENABLED               (1)

3. Compile the firmware by typing following commands:
    ```
    cd micropython/ports/esp32
    make USER_C_MODULES=../../../../cmodules/micropython-esp32-twai/src/micropython.cmake BOARD=STRAGA_CORE_SPIRAM all

    idf.py -D MICROPY_BOARD=STRAGA_CORE_SPIRAM -B build-STRAGA_CORE_SPIRAM -p /dev/tty.wchusbserial112330 flash
    
    ```
    Note that the folder `micropython-camera-driver` should be in the "cmodules" folder level as the `micropython`. Otherwise, you'll need to change the path (`../../../../cmodules/micropython-esp32-twai/src`) to the `micropython.cmake` file.

