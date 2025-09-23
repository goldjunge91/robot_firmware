#include <stdio.h>
#include "pico/stdlib.h"

void usb_init(void) {
    stdio_init_all();
}

void usb_send_string(const char* s) {
    printf("%s", s);
}

void usb_get_string(char* buf, int len) {
    fgets(buf, len, stdin);
}
