# V53L7CX-Library (ESP-IDF)

VL53L7CX Time-of-Flight (8x8 multizone) driver as an ESP-IDF component, using the ESP-IDF v5.5+ **new I2C master driver** (`driver/i2c_master.h`).

This project is a clean ESP32 port of ST’s Ultra Lite Driver (ULD) for the VL53L7CX, with a robust ESP-IDF platform layer and safer firmware upload (chunked writes).

> Not compatible with Arduino framework.

## Highlights

- ESP-IDF **v5.5.x** compatible (ESP32 classic tested)
- Uses **new I²C master API** (`i2c_new_master_bus`, `i2c_master_bus_add_device`, `i2c_master_transmit_receive`, `i2c_master_multi_buffer_transmit`)
- ULD-compatible API: `vl53l7cx_is_alive()`, `vl53l7cx_init()`, `vl53l7cx_start_ranging()`, `vl53l7cx_get_ranging_data()`, …
- Firmware upload **chunking** for robustness on ESP32
- Component-style layout (`include/`, `src/`, `examples/`)

## Requirements

- ESP-IDF v5.5.x (or newer)
- VL53L7CX connected over I²C
- External pullups on SDA/SCL are recommended (e.g. 2.2k–4.7k to 3V3), depending on your wiring length.

## Wiring (ESP32 classic example)

Default example uses:
- SDA = GPIO 21
- SCL = GPIO 22
- I²C clock = 1 MHz

Power:
- 3V3 and GND (check your module’s AVDD/IOVDD requirements)

## I²C address note (0x52 vs 0x29)

ST ULD refers to the default address as **0x52** (8-bit form).
ESP-IDF `i2c_device_config_t.device_address` expects the **7-bit** address (**0x29**).

This component keeps `VL53L7CX_DEFAULT_I2C_ADDRESS` as **0x52** (ST style), and shifts right (`>> 1`) when configuring ESP-IDF.

## Installation

### Option A — Use as local component

Clone this repository into your project:

```
your_project/
  components/
    V53L7CX-Library/
  main/
  CMakeLists.txt
```

Include in code:

```c
#include "vl53l7cx_api.h"
```

### Option B — Use ESP-IDF Component Manager

Add to `idf_component.yml`:

```yaml
dependencies:
  grrtzm/v53l7cx:
    git: https://github.com/Grrtzm/V53L7CX-Library.git
```

## Examples

- `examples/ranging_basic`: minimal example demonstrating init, ranging, and data readout.

Build example:

```
idf.py set-target esp32
idf.py build flash monitor
```

## Common issues

### Stack overflow in main task

Increase stack size via:

```
idf.py menuconfig
Component config → ESP System Settings → Main task stack size
```

### GPIO “not usable” warnings

Usually caused by:
- Duplicate I²C bus initialization
- SDA/SCL pins used elsewhere as GPIO

Ensure `i2c_new_master_bus()` is called only once.

## Licensing / attribution

- Sensor driver logic based on **ST VL53L7CX Ultra Lite Driver (STSW-IMG036)**.
- This repository provides an ESP-IDF-specific platform adaptation.
