# Robot Firmware RP2040 Port Plan

## Ziele

- Ersetze den STM32F4 + PlatformIO-Stack in `src/robot_firmware` durch Firmware, die auf dem Raspberry Pi Pico (RP2040) läuft.
- Erhalte micro-ROS-Integrationen (Motor‑Command‑Subscriber, IMU/Motor/Batterie‑Publisher, `get_cpu_id`‑Service) für Drop‑in‑Kompatibilität mit dem bestehenden ROS‑2‑Bringup.
- Unterstütze das digitale Board‑Pinout in `PINMAP.md` und nutze bewährte Komponenten aus `firmware/mecabridge_pico`, soweit sinnvoll.

## Toolchain & Buildsystem

- SDK: Raspberry Pi Pico SDK; TinyUSB, hardware PWM, PIO, I2C, SPI, Watchdog usw. werden über CMake‑Optionen und target_link_libraries eingebunden (z. B. PICO_ENABLE_TINYUSB). Hinweis: `pico_sdk_init()` initialisiert das SDK, aktiviert aber nicht automatisch alle Extras — CMake‑Flags müssen gesetzt werden.
- CMake: Struktur analog zu `firmware/mecabridge_pico` (Top‑Level `CMakeLists.txt`, `pico_sdk_import.cmake`, `micro_ros_pico_sdk_import.cmake`).
- Abhängigkeiten:
  - `micro_ros_pico` (USB CDC oder UART Transport konfigurierbar).
  - Optional `FreeRTOS` (abhängig von Executor-/Callback‑Design); bevorzugt ist zunächst ein kooperativer Loop mit rclc‑Timern zur Reduzierung der Komplexität.
  - `hardware_pwm`, `hardware_adc`, `hardware_dma`, `hardware_pio`.
  - ICM‑20948 IMU Treiber (Projekt‑Definition: ICM‑20948 — nicht BNO055).
  - VL53L0X Treiber (I2C).
  - INA3221 Überwachungs‑Integration (I2C).
  - WS2812/NeoPixel via PIO (pico-extras `ws2812` als Ausgangspunkt).

## Modulaufteilung (Zielstruktur)

| Modul               | Aufgabe                                                     | Portierungsstrategie                                                                                                           |
| ------------------- | ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| hardware_cfg.h      | Compile‑time Konstanten: Pins, I2C‑Busse, PWM‑Slices, Enums | Generieren aus `PINMAP.md`; eindeutige #defines für GPIO, PWM‑Slices, I2C‑Ports; Benennung an ROS‑Joint‑Konvention anpassen.   |
| motors.[h           | cpp]                                                        | Steuerung von 4 TB6612FNG‑Motoren (PWM + Richtung), Servos und ESCs                                                            | TB6612FNG: PWM + Dir‑GPIO API; ESCs/Servos über hardware_pwm; PWM‑Slice/Zuweisung in `hardware_cfg.h`; PID optional.        |
| encoder_reader.[hpp | cpp]                                                        | Lesen der Hall‑Encoder (Tick‑Zähler, Geschwindigkeit, Richtung)                                                                | Primär GPIO‑IRQ‑basiertes Zählen für Hall‑Sensoren; nur bei tatsächlicher quadrature‑Hardware PIO‑Quadrature/SMs verwenden. |
| ImuLib_cfg.[h       | cpp]                                                        | ICM‑20948 über I2C, publish IMU‑Daten (`sensor_msgs/Imu`)                                                                      | Implementierung mit `hardware/i2c` für ICM‑20948 (Projektvorgabe); kein BNO055; Sampling + Timestamp für rclc‑Publishes.    |
| PixelLedLib_cfg.[h  | cpp]                                                        | WS2812/NeoPixel Status‑LEDs                                                                                                    | PIO‑basierter Treiber (pico‑extras `ws2812` als Referenz); einfache Animationen + direkte Farbsteuerung.                    |
| UartLib.[h          | cpp]                                                        | Powerboard / Peripherie UART‑Protokoll (optional)                                                                              | `hardware/uart` (+ DMA optional) mit Feature‑Flag `ENABLE_POWERBOARD`; Frame‑Parsing und Timeout/Safety integrieren.        |
| micro_ros_cfg.[h    | cpp]                                                        | micro‑ROS Entities, Callbacks, Timer                                                                                           | RAII‑Klasse (z. B. `MicroRosSupport`) zur Verwaltung von Publishern/Subscribern/Services; Timer‑basierte Periodik nutzen.   |
| firmware_main.cpp   | System‑Init, Hauptschleife                                  | Systeminit (pico_sdk_init), HW‑Inits, Micro‑ROS Start; kooperativer Loop + rclc‑Executor; Watchdog + Safe‑Stop implementieren. |

## Scheduling & Nebenläufigkeit

- Periodische Aufgaben über `rclc_executor` Timer und Subscriber‑Callbacks.
- Schnelle Motorregelung (z. B. PID) kann auf Core1 oder per hochfrequentem Timer laufen. Vorschlag:
  - Core0: micro‑ROS Executor / Kommunikation.
  - Core1: Motor‑Control‑Loop (1 kHz) für Regler‑Updates und Safety‑Snapshots.
- Gemeinsame Daten über `critical_section_t`, atomare Variablen oder Ringbuffer schützen.

## Kommunikation & Safety

- Default Transport: USB CDC (TinyUSB). UART Transport optional per Macro.
- Watchdog/Agent‑Ping: bei Verbindungsverlust LED‑Signal + Stop aller Motoren.
- Behalte Topic‑Namen bei, die im Projekt verwendet werden: `_motors_cmd`, `_motors_response`, `/imu/data_raw`, `/battery/status`, `get_cpu_id`.

## Hardware‑Mapping (Auszug aus `PINMAP.md`, zu verifizieren)

- Motor‑PWM: z. B. GP2/GP3/GP4/GP5 → TB6612 PWM Inputs (Slice‑Zuordnung im Header prüfen).
- ESCs (Brushless für Nerf‑Launcher): z. B. GP14/GP15 (PWM).
- Richtungs‑Pins (TB6612): GP17..GP22 + GP26/27/28 (Belegung final in `PINMAP.md`).
- Encoder (Hall): GP6..GP13 als Eingänge für Hall‑Sensoren — meist single/dual‑edge GPIO‑Interrupts; implementiere IRQ‑zähler oder PIO‑Sampler je nach Signaltyp.
- I2C (IMU + VL53L0X + INA3221): wähle freie GPIOs; Beispiel I2C1 auf GP12/GP13 ist ein Vorschlag — gegen PCB prüfen, Konflikte mit Encoder vermeiden.
- LED‑Strip: GP0 oder dedizierter Pad.

## Offene Fragen / Risiken (angepasst an Projekt.md)

1. IMU‑Typ: Projekt.md spezifiziert ICM‑20948 — BNO055‑Referenzen entfernt. Implementierung muss ICM‑20948‑Register/Init verwenden.
2. Encoder‑Typ: Projekt.md nennt Hall‑Encoder — quadrature‑Annahmen in ursprünglichem Plan passen ggf. nicht. Klären: single‑channel Hall vs. quadrature‑encoder.
3. UART‑Ressourcen: RP2040 hat zwei Hardware‑UARTs (UART0, UART1). Entscheiden, welcher Port für SBC vs. Powerboard reserviert wird.
4. Pin‑Konflikte: I2C ↔ Encoder‑Pins (z. B. GP12/GP13) müssen mit PCB‑Team gelöst werden.
5. Safety‑Timeout: Sicherheitsstopp Timeout spezifizieren (siehe Spezifikation — momentan offen).

## Sofortige nächste Schritte (konkret, zu Projekt.md passend)

1. Erzeuge Pico‑Projektgerüst in `src/robot_firmware/pico/` mit CMake‑Vorlage (TinyUSB + micro‑ROS als Optionen).
2. Portiere micro‑ROS Node‑Scaffold (`MicrorosNode`) mit stubbed Publisher/Subscriber/Service‑Signaturen gemäß Feature‑Spec.
3. Erstelle `hardware_cfg.h`‑Template aus `PINMAP.md` (Platzhalter für Pins) und markiere Konfliktstellen.
4. Implementiere Basistreiber‑Stubs: TB6612 PWM+Dir, Hall‑Encoder Reader, ICM‑20948 I2C Reader, VL53L0X, INA3221, Servo/ESC PWM.
5. Testplan: Unit/Bench‑Tests für PWM‑Polung, Encoder‑Zählung, IMU‑Lesen; integriere in `just build-firmware`.

## Korrekturen gegenüber ursprünglichem Plan (kurz)

- BNO055 → ICM‑20948 (Projekt.md).
- Quadrature‑Encoder‑Annahme ersetzt durch Hall‑Encoder‑Leser; PIO‑Quadrature nur bei tatsächlicher Quadrature‑Hardware.
- `pico_sdk_init()`‑Hinweis präzisiert: Extras via CMake aktivieren.
- Ergänzt: VL53L0X, INA3221 und Nerf‑Launcher Servos/ESCs, da in Projekt.md genannt.
