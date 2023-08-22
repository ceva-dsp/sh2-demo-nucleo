#include <stdio.h>
#include <stdbool.h>

#include "button.h"

#include "stm32f4xx_hal.h"

// Use pin C13 for button input
#define BUTTON_GPIO_PORT GPIOC
#define BUTTON_GPIO_PIN  GPIO_PIN_13

#define DEBOUNCE_INTERVAL_US (10000)
#define LONG_PRESS_US (500000)

// Forward declarations
static void _onUp(void);
static void _onDown(void);
static void _onShort(void);
static void _onLong(void);

static bool button_initialized = false;

// Callbacks and their defaults
static button_cb_t onUp = _onUp;
static button_cb_t onDown = _onDown;
static button_cb_t onShort = _onShort;
static button_cb_t onLong = _onLong;


void button_init(void) {
    button_initialized = true;
    
    GPIO_InitTypeDef GPIO_InitStruct;
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
  
    /*Configure GPIO pins : Debug */
    GPIO_InitStruct.Pin = BUTTON_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);
}

void button_onUp(button_cb_t cb) {
    onUp = cb;
}

void button_onDown(button_cb_t cb) {
    onDown = cb;
}

void button_onShort(button_cb_t cb) {
    onShort = cb;
}

void button_onLong(button_cb_t cb) {
    onLong = cb;
}

static bool button_read(void) {
    return HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN);
}

static void _onUp(void) {
    // Default event handler -- do nothing
}

static void _onDown(void) {
    // Default event handler -- do nothing
}

static void _onShort(void) {
    // Default event handler -- do nothing
}

static void _onLong(void) {
    // Default event handler -- do nothing
}

bool button_poll(uint32_t now_us) {
    if (!button_initialized) {
        button_init();
    }
    
    // Called every loop to check on the button.
    static bool debounced_state = false;
    static uint32_t transition_time_us = 0;
    static bool reported_long = false;

    uint32_t since_transition_us = (uint32_t)(now_us - transition_time_us);

    if (since_transition_us >= DEBOUNCE_INTERVAL_US) {
        // Read the button, why not?
        bool button_state = button_read();

        if (button_state != debounced_state) {
            // This is a new transition
            
            // Set new debounced state
            debounced_state = button_state;
            
            // Set transition_time_us
            transition_time_us = now_us;
            
            // If new state is up ...
            if (debounced_state == true) {
                // button is up
                if (since_transition_us < LONG_PRESS_US) {
                    // report a short press, onShort
                    (*onShort)();
                }
                // report the onUp event
                (*onUp)();
            }
            else {
                // button is newly down
                reported_long = false;
                
                // report the onDown event
                (*onDown)();
            }
        }
        else {
            // Button continues in the same state
            if ((debounced_state == false) &&
                !reported_long &&
                (since_transition_us >= LONG_PRESS_US)) {
                // report the onLong event
                reported_long = true;
                (*onLong)();
            }
        }
    }

    return debounced_state;
}
