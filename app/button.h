#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include <stdbool.h>

void button_init(void);
bool button_poll(uint32_t now_us);

typedef void (*button_cb_t)(void);

void button_onUp(button_cb_t cb);
void button_onDown(button_cb_t cb);
void button_onShort(button_cb_t cb);
void button_onLong(button_cb_t cb);
    
#endif
