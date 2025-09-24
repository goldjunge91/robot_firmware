#include <stdio.h>
#include "pico/stdlib.h"
#include "usb_interface.h"

void usb_interface_init(void) {
    stdio_init_all();
}

void usb_interface_task(void) {
    // USB CDC task is handled by stdio
}

int usb_interface_get_command(char* buffer, int len) {
    return scanf("%s", buffer);
}

void usb_interface_send_response(const char* response) {
    printf("%s\n", response);
}