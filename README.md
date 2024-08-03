## MULoc

The source code for the MULoc system (submitted to INFOCOM25 for peer review).

All content related to the authors' information has been removed from the code to comply with the double-blind review policy of INFOCOM. A detailed version of the code will be made available immediately upon acceptance of the paper.

The system consists of three main parts: UWB firmware code, ESP32 firmware code, and MATLAB algorithm code.

### 1. UWB Firmware Code

This is a KEIL UVISION project that includes firmware code for MULoc Anchors and Tags. Compiled .hex files are also provided directly in the root directory.

### 2. ESP32 Firmware Code

An ESP32 project functions to forward data outputted by UWB tags to a PC through WiFi connection. It requires the ESP toolchain to compile. Compiled .bin files are also provided directly in the root directory.

### 3. MATLAB Algorithm Code

Offline processing code for the signal processing pipeline, used for calculating tag positions.

