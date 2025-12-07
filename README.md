# 🚜 Sistema de Navegación Autónoma para Tractor (STM32H7 + ESP32)

Este repositorio contiene el firmware y la documentación técnica para el **Sistema de Seguimiento de Trayectoria de Precisión (WPP)**. El proyecto implementa un vehículo autónomo capaz de navegar por *waypoints* utilizando fusión de sensores, algoritmos de control geométrico y un sistema operativo en tiempo real.

---

## 📋 Descripción del Proyecto

El objetivo principal es controlar un tractor a escala para que siga rutas predefinidas con alta precisión. El sistema utiliza una arquitectura distribuida donde un **STM32H7** actúa como el controlador central de movimiento y un **ESP32-C3** funciona como un *gateway* de sensores inalámbricos.

### Funcionalidades Clave
* **Navegación Autónoma:** Implementación del algoritmo **Pure Pursuit** para el seguimiento suave de curvas y trayectorias.
* **Fusión de Sensores (Sensor Fusion):** Combinación de odometría relativa (Encoders) y posicionamiento absoluto (Cámara JD) para corregir la deriva en tiempo real.
* **Control de Crucero Adaptativo:** Ajuste dinámico de la velocidad en función del ángulo de giro para evitar derrapes.
* **Arquitectura RTOS:** Uso de **FreeRTOS** para gestionar tareas críticas (Control) y no críticas (Telemetría) sin bloqueos.
* **Conectividad Industrial:** Comunicación robusta entre módulos mediante el protocolo **CAN Bus (FD)**.

---

## 🛠️ Arquitectura del Sistema

El sistema se divide en dos nodos principales comunicados vía CAN Bus:

### 1. Controlador Central (STM32H745)
* **Núcleo:** Cortex-M7 a 480 MHz.
* **Responsabilidad:** Ejecuta el bucle de control, calcula la odometría y genera señales PWM.
* **Software:** `main.c`, `motion.c`, `freertos.c`.
* **Periféricos:**
    * `FDCAN1`: Recepción de datos de sensores.
    * `TIM2`: PWM para Motor (ESC).
    * `TIM13`: PWM para Servo (Dirección).
    * `UART3`: Debugging.

### 2. Gateway de Sensores (ESP32-C3)
* **Responsabilidad:** Puente de comunicaciones y adquisición de datos.
* **Funciones:**
    * Lectura de IMU (BNO055) vía I2C.
    * Lectura de Encoders vía Interrupciones.
    * Recepción de coordenadas de la Cámara JD vía **Bluetooth LE**.
    * Transmisión de todos los datos al bus CAN.

---

## 📡 Protocolo de Comunicación (CAN Bus)

El sistema utiliza tramas estándar para la interoperabilidad:

| ID (Hex) | Origen | Descripción | Datos (Payload) |
| :---: | :---: | :--- | :--- |
| **0x30** | ESP32 | Fusión de Sensores | `[Yaw (float)]` `[Encoder Ticks (int32)]` |
| **0x35** | Cámara | Posición Absoluta | `[X (int16)]` `[Y (int16)]` `[Angle (int16)]` |
| **0x111** | STM32 | Estado / Heartbeat | Status bytes |

---

## 🚀 Instalación y Uso

### Requisitos
* **IDE:** STM32CubeIDE (v1.10 o superior).
* **Hardware:** Kit de desarrollo STM32H7, ESP32-C3 Super Mini, Chasis de tractor, Sensores.

### Pasos para compilar
1.  Clonar el repositorio:
    ```bash
    git clone [https://github.com/JocelynVelarde/stm_test_repo.git](https://github.com/JocelynVelarde/stm_test_repo.git)
    ```
2.  Abrir `stm32_h7_canbase/.project` en STM32CubeIDE.
3.  Seleccionar la configuración de compilación para el núcleo **CM7**.
4.  Compilar el proyecto (Hammer icon).
5.  Flashear el microcontrolador.

---

## 👥 Autores y Roles

Este proyecto fue desarrollado por el equipo de **RoBorregos**, aprovechando las especialidades técnicas de cada miembro:

* **Héctor:**
    * *Rol:* **Navegación y Control.**
    * *Aportes:* Desarrollo de algoritmos de odometría, lógica Pure Pursuit y pruebas de integración de sensores.
* **Jocelyn:**
    * *Rol:* **Arquitectura de Software y Comunicaciones.**
    * *Aportes:* Implementación de FreeRTOS, protocolo CAN, Gateway ESP32 y estándares de código (GitHub).
* **Daniel:**
    * *Rol:* **Hardware y Electrónica.**
    * *Aportes:* Diseño y manufactura de la PCB, integración eléctrica y validación de componentes.
* **Melanie:**
    * *Rol:* **Gestión y Mecánica.**
    * *Aportes:* Diseño mecánico del chasis, montaje físico y gestión del cronograma del proyecto (Gantt).

---

## 📄 Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulta el archivo `LICENSE` para más detalles.
