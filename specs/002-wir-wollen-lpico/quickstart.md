# Quickstart Guide for lpico_pio Firmware

```markdown

# Quickstart Guide — micro_ros_raspberrypi_pico_sdk

Kurzanleitung zum Bauen, Flashen und Testen der Firmware, basierend auf dem
`micro_ros_raspberrypi_pico_sdk` Projekt (dieses Repository / Feature nutzt
`micro_ros_raspberrypi_pico_sdk` als Projekt-Ordner für den Firmware-Build).

## Voraussetzungen

- Raspberry Pi Pico (RP2040) mit angeschlossener Robot-Hardware.
- Raspberry Pi Pico SDK (pico-sdk) und eine CMake-fähige Toolchain vorhanden.
- `micro_ros_raspberrypi_pico_sdk` Projekt-Ordner im Workspace (Projekt-Root für diese Firmware).
- Terminal (PowerShell / bash) und, für micro-ROS-Tests, ein laufender micro-ROS Agent auf dem Host.

> Hinweis: Für die meisten Entwicklungs-Workflows empfehlen wir USB CDC (TinyUSB)
> als primären micro-ROS Transport. Falls USB nicht verfügbar, kann UART als Fallback genutzt werden.

## Build (CMake)

1. Öffne ein Terminal und wechsle in das Firmware-Projektverzeichnis:

  cd <pfad-zum-repo>/micro_ros_raspberrypi_pico_sdk

2. Erstelle das Build-Verzeichnis und wechsle hinein:

  mkdir build
  cd build

3. Konfiguriere das Projekt mit CMake (verwende ggf. `-G Ninja` wenn Ninja installiert ist):

  cmake ..

4. Baue das Projekt:

  cmake --build . --config Release

Das Build erzeugt ein UF2-Image (z. B. `lpico_pio.uf2` oder `firmware.uf2`) im `build/` Verzeichnis — der genaue Name hängt vom CMake-Target ab.

## Flashen

1. Halte die `BOOTSEL`-Taste auf dem Pico gedrückt und verbinde das Gerät per USB mit dem Rechner.
2. Der Pico wird als USB-Mass‑Storage-Laufwerk angezeigt.
3. Kopiere die erzeugte UF2-Datei (`*.uf2`) in das Pico-Laufwerk.
4. Der Pico startet neu und läuft mit der neuen Firmware.

## Micro-ROS + Test

1. Starte auf dem Host einen micro-ROS Agent (z. B. `micro_ros_agent`), konfiguriert für den gewählten Transport (USB CDC / UART).
2. Verbinde einen ROS2-Client (auf dem Host) mit dem Agenten und sende die Kontraktnachrichten/Services, die in `specs/002-wir-wollen-lpico/contracts/` definiert sind.
3. Überprüfe Logausgaben über die serielle Konsole (USB CDC) und validiere, dass Topics/Services korrekt publiziert/bedient werden.

## Abgleich mit Plan und Tasks

- Diese Quickstart-Anleitung wurde auf die Nutzung von `micro_ros_raspberrypi_pico_sdk`
  als Projektordner abgestimmt, wie in `plan.md` beschrieben.
- Die Tasks in `specs/002-wir-wollen-lpico/tasks.md` erwarten, dass das Build/Flash-Verfahren
  über das `micro_ros_raspberrypi_pico_sdk` Projekt läuft. Prüfe bitte `tasks.md` auf
  konkrete Task‑IDs, die Build-Targets oder erwartete UF2-Namen referenzieren und passe ggf. die Targets dort an.

# Quickstart Guide — micro_ros_raspberrypi_pico_sdk

Kurzanleitung zum Bauen, Flashen und Testen der Firmware, basierend auf dem
`micro_ros_raspberrypi_pico_sdk` Projekt (dieses Repository / Feature nutzt
`micro_ros_raspberrypi_pico_sdk` als Projekt-Ordner für den Firmware-Build).

## Voraussetzungen

- Raspberry Pi Pico (RP2040) mit angeschlossener Robot-Hardware.
- Raspberry Pi Pico SDK (pico-sdk) und eine CMake-fähige Toolchain vorhanden.
- `micro_ros_raspberrypi_pico_sdk` Projekt-Ordner im Workspace (Projekt-Root für diese Firmware).
- Terminal (PowerShell / bash) und, für micro-ROS-Tests, ein laufender micro-ROS Agent auf dem Host.

> Hinweis: Für die meisten Entwicklungs-Workflows empfehlen wir USB CDC (TinyUSB)
> als primären micro-ROS Transport. Falls USB nicht verfügbar, kann UART als Fallback genutzt werden.

## Build (CMake)

1. Öffne ein Terminal und wechsle in das Firmware-Projektverzeichnis.

```powershell
cd <pfad-zum-repo>/micro_ros_raspberrypi_pico_sdk
```

1. Erstelle das Build-Verzeichnis und wechsle hinein.

```powershell
mkdir build
cd build
```

1. Konfiguriere das Projekt mit CMake (verwende ggf. `-G Ninja` wenn Ninja installiert ist).

```powershell
cmake ..
```

1. Baue das Projekt.

```powershell
cmake --build . --config Release
```

Das Build erzeugt ein UF2-Image (z. B. `lpico_pio.uf2` oder `firmware.uf2`) im `build/` Verzeichnis — der genaue Name hängt vom CMake-Target ab.

## Flashen

1. Halte die `BOOTSEL`-Taste auf dem Pico gedrückt und verbinde das Gerät per USB mit dem Rechner.
2. Der Pico wird als USB-Mass‑Storage-Laufwerk angezeigt.
3. Kopiere die erzeugte UF2-Datei (`*.uf2`) in das Pico-Laufwerk.
4. Der Pico startet neu und läuft mit der neuen Firmware.

## Micro-ROS + Test

1. Starte auf dem Host einen micro-ROS Agent (z. B. `micro_ros_agent`), konfiguriert für den gewählten Transport (USB CDC / UART).
2. Verbinde einen ROS2-Client (auf dem Host) mit dem Agenten und sende die Kontraktnachrichten/Services, die in `specs/002-wir-wollen-lpico/contracts/` definiert sind.
3. Überprüfe Logausgaben über die serielle Konsole (USB CDC) und validiere, dass Topics/Services korrekt publiziert/bedient werden.

## Abgleich mit Plan und Tasks

- Diese Quickstart-Anleitung wurde auf die Nutzung von `micro_ros_raspberrypi_pico_sdk` als Projektordner abgestimmt, wie in `plan.md` beschrieben.
- Die Tasks in `specs/002-wir-wollen-lpico/tasks.md` erwarten, dass das Build/Flash-Verfahren über das `micro_ros_raspberrypi_pico_sdk` Projekt läuft. Prüfe bitte `tasks.md` auf konkrete Task‑IDs, die Build-Targets oder erwartete UF2-Namen referenzieren und passe ggf. die Targets dort an.

## Troubleshooting (kurz)

- Keine seriellen Logs: Prüfe, ob das Gerät korrekt als USB-CDC verbunden ist oder ob die Firmware TinyUSB initialisiert.
- Buildfehler wegen fehlender `pico-sdk`: Setze die Umgebungsvariable `PICO_SDK_PATH` auf den lokalen pico-sdk-Ordner oder installiere pico-sdk im Workspace.
