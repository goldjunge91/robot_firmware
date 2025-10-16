#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Nur die Pins, die wir für den Test brauchen
const uint8_t kMotor0In1 = 2u;
const uint8_t kMotor0In2 = 3u;

int main() {
    // Richtungspins als Ausgänge initialisieren
    gpio_init(kMotor0In1);
    gpio_set_dir(kMotor0In1, GPIO_OUT);
    gpio_init(kMotor0In2);
    gpio_set_dir(kMotor0In2, GPIO_OUT);

    // Motor auf "vorwärts" stellen
    gpio_put(kMotor0In1, 1);
    gpio_put(kMotor0In2, 0);

    // Endlosschleife, damit das Programm nicht endet
    while (true) {
        tight_loop_contents();
    }
    return 0;
}