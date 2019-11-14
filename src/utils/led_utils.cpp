#include <led_utils.h>

void blink_builtin() {
    bool led_status = (digitalRead(LED_BUILTIN));
    if(led_status) {
        digitalWrite(LED_BUILTIN, 0);
    } else {
        digitalWrite(LED_BUILTIN, 1);
    }
}
