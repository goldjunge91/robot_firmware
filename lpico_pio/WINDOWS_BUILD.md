# Windows Build-Anleitung für micro-ROS

## Problem und Lösung

**Problem**: `micro_ros_platformio` funktioniert nicht auf Windows native, da es POSIX-Tools benötigt.

**Lösung**: Verwende die precompiled `micro_ros_arduino` Library für Windows-Entwicklung.

## Aktueller Build-Status

Die Konfiguration wurde für Windows-native Entwicklung optimiert:

### platformio.ini Konfiguration

```ini
lib_deps = 
    mirs240x/micro_ros_arduino@^2.0.7-humble  ; ✓ Windows-kompatibel
    ; https://github.com/micro-ROS/micro_ros_platformio  ; Nur Linux/WSL
```

### Header-Includes angepasst

```cpp
#include <micro_ros_arduino.h>  // Statt micro_ros_platformio.h
```

## Build-Kommandos

```bash
# Clean build (empfohlen nach Konfigurationsänderungen)
pio run --target clean

# Build
pio run

# Upload
pio run --target upload

# Monitor
pio device monitor --baud 115200
```

## Alternative Entwicklungsoptionen

### Option 1: Windows native (aktuelle Konfiguration)

- ✓ Einfach zu verwenden
- ✓ Keine zusätzlichen Tools erforderlich  
- ⚠ Precompiled Library (begrenzte Anpassungsmöglichkeiten)

### Option 2: WSL (Windows Subsystem for Linux)

```bash
# In WSL installieren:
sudo apt update
sudo apt install python3-pip
pip3 install platformio

# Dann in WSL:
cd /mnt/c/GIT/my_steel-robot_ws/src/robot_firmware/lpico_pio
pio run
```

### Option 3: Docker

```bash
# Docker Container für PlatformIO
docker run -it --rm -v ${PWD}:/workspace -w /workspace platformio/platformio-core pio run
```

## Troubleshooting

### Build-Fehler beheben

1. **Clean Build**:

   ```bash
   pio run --target clean
   rm -rf .pio/
   pio run
   ```

2. **Library Cache leeren**:

   ```bash
   pio lib uninstall micro_ros_platformio
   pio lib install mirs240x/micro_ros_arduino@^2.0.7-humble
   ```

3. **Dependency-Probleme**:

   ```bash
   pio pkg uninstall --all
   pio pkg install
   ```

### Häufige Fehlermeldungen

#### "Der Befehl '.' ist entweder falsch geschrieben"

- **Ursache**: `micro_ros_platformio` versucht POSIX-Commands auszuführen
- **Lösung**: Verwende `micro_ros_arduino` Library (bereits konfiguriert)

#### "micro_ros_platformio.h not found"

- **Ursache**: Header-Include passt nicht zur Library
- **Lösung**: Verwende `#include <micro_ros_arduino.h>` (bereits angepasst)

#### "undefined reference to micro_ros_*"

- **Ursache**: Library nicht korrekt gelinkt
- **Lösung**: Clean build durchführen

## Verifikation

Nach erfolgreichem Build sollten Sie sehen:

```
Building in release mode
Linking .pio/build/pico/firmware.elf
Building .pio/build/pico/firmware.uf2
========================= [SUCCESS] Took X.XX seconds =========================
```

## Nächste Schritte

1. **Build testen**: `pio run`
2. **Upload testen**: `pio run --target upload`
3. **micro-ROS Agent starten** (siehe README_micro_ros.md)
4. **ROS2 Topics testen**

## Support

Bei weiteren Problemen:

- Überprüfen Sie die PlatformIO-Logs mit `-v` Flag
- Konsultieren Sie die micro-ROS Arduino Dokumentation
- Testen Sie mit einem einfachen Beispiel zuerst
