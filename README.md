# STM32H7 Dual-Core FDCAN Base Project

This project is a dual-core embedded application designed for the **STM32H745** microcontroller (Cortex-M7 + Cortex-M4). It serves as a foundational project for setting up **FDCAN communication**, inter-core synchronization via Hardware Semaphores (HSEM), and basic GPIO/UART functionality.

## Hardware Requirements
* **Development Board:** STM32 Nucleo-H745ZI-Q (or similar STM32H745 board).
* **CAN Transceiver:** Required on pins PD0/PD1 to connect to a physical CAN bus.
* **Cabling:** Micro-USB for programming/debugging.

## Project Architecture

The application is split into two sub-projects, one for each core. The Cortex-M7 acts as the system master, handling the high-speed peripherals and clock configuration, while the Cortex-M4 acts as a secondary processor.

### 1. Cortex-M7 (CM7) - Master Core
**Responsibility:** System Initialization & Communication.
* **System Clock:** Configures the main PLL and system clocks.
* **Boot Synchronization:** Uses Hardware Semaphore 0 (HSEM) to safely boot the CM4 core after system initialization is complete.
* **FDCAN1:** * Configured on **PD0 (RX)** and **PD1 (TX)**.
  * Sets up a global filter to reject non-matching frames.
  * Prepares a Tx Header (Standard ID: `0x111`).
  * *Note: Initialization supports Classic and FD frame formats.*
* **UART3:** Configured (115200 baud) for debug output.

### 2. Cortex-M4 (CM4) - Slave Core
**Responsibility:** Auxiliary Tasks.
* **Boot Wait:** Waits in STOP mode until notified by the CM7 via HSEM that the system is ready.
* **Heartbeat:** Toggles **LD3 (GPIO PE1)** every 100ms to indicate the core is running.
* **UART3:** Initialization included but currently unused in the main loop.

## Pin Configuration

| Pin  | Peripheral | Function                        |
| :--- | :---       | :---                            |
| **PD0** | FDCAN1     | CAN RX                          |
| **PD1** | FDCAN1     | CAN TX                          |
| **PB0** | GPIO       | LD1 (Green LED)                 |
| **PB14** | GPIO       | LD2 (Red LED)                   |
| **PE1** | GPIO       | LD3 (Yellow LED - CM4 Heartbeat)|
| **PC13** | GPIO       | User Button (B1)                |
| **PD8** | USART3     | ST-Link VCP RX                  |
| **PD9** | USART3     | ST-Link VCP TX                  |

## Software & Tools
* **IDE:** STM32CubeIDE (Project files generated via CubeMX).
* **Library:** STM32CubeH7 MCU Package (HAL Drivers).

## How to Build and Run

1. **Import:** * Open STM32CubeIDE.
   * Select `File` > `Import` > `General` > `Existing Projects into Workspace`.
   * Select the root directory of this repository (`stm32_h7_canbase`).
   * Ensure both `stm32_h7_canbase_CM4` and `stm32_h7_canbase_CM7` projects are selected.

2. **Debug Configuration:**
   * This is a dual-core device. You must create a **Debug Configuration** that loads both `.elf` files.
   * Usually, you start the **CM7** debug session first. The settings should be configured to "Download" both binaries or setup a specific "Startup Group" in Eclipse.

3. **Execution Flow:**
   * Upon reset, CM7 starts.
   * CM7 configures clocks and power.
   * CM7 releases HSEM_ID_0.
   * CM4 wakes up, initializes its peripherals, and begins blinking LD3.
   * CM7 initializes FDCAN and UART, then enters its main loop.

## Current Status
* **CM7:** FDCAN init logic is present with defined Tx Headers (`0x111`), but the `while(1)` loop is currently empty (no active transmission implemented yet).
* **CM4:** Active LED blinking confirms the core is alive and sync worked.

---
*Note: If using the Nucleo board without an external CAN transceiver module, FDCAN initialization may fail or hang if loopback mode is not enabled.*
