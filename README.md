# micropython-esp32-twai

This repository adds TWAI/CAN(USER_C_MODULES) support to MicroPython for the ESP32 family.
Use user_module, that draft probe. But works.
From:
https://github.com/micropython/micropython/pull/12331

## DIY use scr_can_v2

Read this section if you want to include the ESP32 TWAI/CAN support to MicroPython from scratch. To do that follow these steps:
  
- Note: The steps below now also work for MicroPython "esp32", "1.25" with ESP-IDF v5.4.1

1. Clone the MicroPython repository:
    ```bash
    git clone --recursive https://github.com/micropython/micropython.git
    ```
  
2. Clone this repository:
    ```bash
    git clone https://github.com/vostraga/micropython-esp32-twai.git
    ```
    Copy the files and folders inside the root folder where `micropython`.

3. Directory structure should look like this:
    ```
    /
    ├── micropython/       # MicroPython repository
    │   └── port/
    │       └── esp32/     # build point
    │
    └── cmodules/
        └── micropython-esp32-twai/
            ├── src/       # First version of CAN module
            └── src_can_v2/  # Second version with improvements
    ```

## Build

### SETUP ESP-IDF v5.4.1

```bash
# Set environment variables
export IDF_TOOLS_PATH="$HOME/tools/esp_idf_v5.4.1/esp_tool" IDF_PATH="$HOME/tools/esp/v5.4.1/esp-idf"

# Create directories
mkdir -p "$HOME/Documents/dev_iot/opt/upy/tools/esp_idf_v5.4.1/esp_tool"
cd "$HOME/Documents/dev_iot/opt/upy/tools/esp_idf_v5.4.1"

# Clone ESP-IDF repository
git clone -j8 -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf

# Install and export
./install.sh
. ./export.sh
```

### USE ESP-IDF v5.4.1

```bash
export IDF_TOOLS_PATH="$HOME/Documents/dev_iot/opt/upy/tools/esp_idf_v5.4.1/esp_tool" IDF_PATH="$HOME/Documents/dev_iot/opt/upy/tools/esp_idf_v5.4.1/esp-idf" && . "$IDF_PATH/export.sh"
```

### Build MicroPython

Read official documentation for complete instructions. Basic steps:

```bash
# Go to micropython repository
cd micropython

# Build mpy-cross
make -C mpy-cross

# Go to ESP32 port
cd ports/esp32

# Delete lock files if present
rm *.lock

# Update submodules
make submodules
```

### Build with TWAI/CAN Support

For example, to build for ESP32-S3 with octal PSRAM (go to **build point**):

```bash
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 -D MICROPY_BOARD_VARIANT=SPIRAM_OCT -D USER_C_MODULES="../../../../cmodules/micropython-esp32-twai/src_can_v2/micropython.cmake" -B build_ESP32_GENERIC_S3_SPIRAM_OCT build
```

## Usage Example

For testing, connect pin 4 and pin 5 together for loopback mode.

### REPL Example

```sh
MicroPython v1.25.0 on 2025-05-09; PicoW_S3 with ESP32-S3
Type "help()" for more information.
>>>
>>> import CAN
>>> dev = CAN(0, extframe=False, tx=5, rx=4, mode=CAN.LOOPBACK, bitrate=50000, auto_restart=False)
CAN: TIMING
CAN: timing brp=0
CAN: timing tseg_1=15
CAN: timing tseg_2=4
CAN: timing sjw=3
CAN: timing triple_sampling=0
CAN: BRP_MIN=2, BRP_MAX=16384
CAN: bitrate 50000kb
CAN: Mode 1
CAN: Loopback flag 1
>>> dev
CAN(tx=5, rx=4, bitrate=50000, mode=NO_ACK, loopback=1, extframe=0)
```

### Async Example with Sender and Receiver

```python
import asyncio
import CAN

dev = CAN(0, extframe=False, tx=5, rx=4, mode=CAN.LOOPBACK, bitrate=50000, auto_restart=False)


# - identifier of can packet (int)
# - extended packet (bool)
# - rtr packet (bool)
# - data frame (0..8 bytes)

async def reader():
    while True:
        if dev.any():
            data = dev.recv()
            print(f"RECEIVED: id:{hex(data[0])}, ex:{data[1]}, rtr:{data[2]}, data:{data[3]}")
        await asyncio.sleep(0.01)

async def sender():
    counter = 0
    while True:
        # Send a message once per second
        msg_id = 0x123  # CAN message identifier
        # Use list of bytes instead of bytes object
        msg_data = [counter & 0xFF, (counter >> 8) & 0xFF]
        
        # Correct parameter order: data first, then ID
        dev.send(msg_data, msg_id)  # data, id
        
        print(f"SENT: id:{hex(msg_id)}, data:{msg_data}")
        counter += 1
        await asyncio.sleep(1)  # Send message once per second

async def main():
    # Start both tasks concurrently
    read_task = asyncio.create_task(reader())
    send_task = asyncio.create_task(sender())
    
    # Wait for both tasks (this will run forever)
    await asyncio.gather(read_task, send_task)

# Run the example
loop = asyncio.get_event_loop()
loop.run_until_complete(main())
```

### Output Example

```sh
SENT: id:0x123, data:[0, 0]
RECEIVED: id:0x123, ex:False, rtr:False, data:b'\x00\x00'
SENT: id:0x123, data:[1, 0]
RECEIVED: id:0x123, ex:False, rtr:False, data:b'\x01\x00'
SENT: id:0x123, data:[2, 0]
RECEIVED: id:0x123, ex:False, rtr:False, data:b'\x02\x00'
SENT: id:0x123, data:[3, 0]
RECEIVED: id:0x123, ex:False, rtr:False, data:b'\x03\x00'
SENT: id:0x123, data:[4, 0]
RECEIVED: id:0x123, ex:False, rtr:False, data:b'\x04\x00'
```


## Technical Notes

### ESP32 TWAI Timing Compatibility

This module uses a universal timing configuration approach to support all ESP32 variants (ESP32, ESP32-C3, ESP32-S2, ESP32-S3). The original ESP32 has a different `twai_timing_config_t` structure (5 fields) compared to newer variants (7 fields with additional clock source parameters). 

Our solution:
- **ESP32**: Manual timing calculations based on 40MHz APB (Advanced Peripheral Bus) clock frequency
- **ESP32-C3/S2/S3**: Uses ESP-IDF timing macros with conditional compilation  
- **Supported bitrates**: 1k, 5k, 10k, 12.5k, 16k, 20k, 25k, 50k, 100k, 125k, 250k, 500k, 800k, 1000k bps

The timing calculation formula: `Bitrate = APB_CLK_FREQ / (BRP * (1 + tseg_1 + tseg_2))`  
Where APB_CLK_FREQ = 40MHz for ESP32, BRP = Baud Rate Prescaler, tseg_1/tseg_2 = Time segments



## Info
esp32
CONFIG_XTAL_FREQ_40=y
CONFIG_XTAL_FREQ=40