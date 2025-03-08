# MULoc: Towards Millimeter-Accurate Localization for Unlimited UWB Tags via Anchor Overhearing

*Junqi Ma, Fusang Zhang, Beihong Jin, Siheng Li, and Zhi Wang*

Thanks for your interest in MULoc. This repository contains source code and sample datasets of the paper "MULoc: Towards Millimeter-Accurate Localization for Unlimited UWB Tags via Anchor Overhearing" (accepted by IEEE INFOCOM 2025).

**Abstract**: Recent years have seen rapid advancements in ultra-wideband (UWB)-based localization systems. However, most existing solutions offer only centimeter-level accuracy and support a limited number of UWB tags, which fails to meet the growing demands of emerging sensing applications (e.g., virtual reality). This paper presents MULoc, the first system that can localize an unlimited number of UWB tags with millimeter-level accuracy. At the core of MULoc is the innovative use of UWB phase, which can provide finer-grained distance measurement than traditional time-of-flight (ToF) estimates. To accurately obtain phase estimates from unsynchronized devices, we introduce a novel localization scheme called anchor overhearing (AO) and eliminate raw signal errors through a signal-difference-based technique. For precise tag localization, we resolve phase ambiguity by combining a fusion-based filtering method and frequency hopping. We implement MULoc on commercial UWB modules. Extensive experiments demonstrate that our system achieves a median localization error of 0.47 mm and 90-th percentile error of 1.02 cm, reducing the error of traditional method by 91.12%.

## Introduction

This repository contains three main components:

1. **UWB Firmware Code**
2. **ESP32 Firmware Code**
3. **MATLAB Algorithm Code**

### 1. UWB Firmware Code

A Keil uVision project that includes firmware code for MULoc Anchors and Tags. 

### 2. ESP32-S3 Firmware Code

An ESP32-S3 project designed to forward data output by UWB tags to a PC via a WiFi connection. It requires the ESP toolchain for compilation. 

### 3. MATLAB Code

Signal processing algorithms written in MATLAB.

## Usage

### 1. Compilation of UWB Firmware Code

To compile this project, your environment must have the following installed:

- Keil uVision 5
- ARM Compiler 5.06
- STM32F103T8 library functions

The compiled binary files can be directly run on Jiulin X1 UWB modules. The most important files in this project are:

- `anchor_main.c`
- `tag_main.c`
- `bphere_uwb.c`
- `bphere_uwb.h`

These files describe the MULoc anchor scheduling process. If you want to port MULoc to your own UWB module, you only need to modify these three files and integrate them into your project.

### 2. Data Collection

#### Using ESP32-S3

1. Connect the ESP32-S3 and your PC to the same LAN.
2. Configure the PC's IP address and UDP port in the ESP32-S3 firmware.
3. Collect UWB raw data by setting up a UDP server on the PC:

```bash
python ./utils/udp_receive.py
```

#### Without ESP32-S3

You can also connect UWB tags directly to your PC via USB and collect data by reading the serial port:

```bash
python ./utils/serial_receive.py
```

For each data collection, you will get multiple files:

- `data.txt`: Raw data in byte format
- `data_str.txt`: Raw data parsed into string format
- `data_tag.txt`: Tag overhearing data
- `data_anchor_i.txt`: Anchor overhearing data for Anchor i

### 3. Signal Processing

We provide two sample datasets in the `./matlab/sample_data` directory. You can calculate the UWB tag positions by running `data_analyse.m` in MATLAB.
