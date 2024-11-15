#include "led.h"
#include "fsl_gpio.h"
#include "millis.h"  // Dependiendo de la biblioteca de temporización que utilices

// Almacena el estado actual de los LEDs para cada tira
static uint8_t led_state[8] = {0};
static led_t leds[8] = {0};  // Asumiendo que tienes un arreglo de LEDs

// Inicializa el MAX7219 y apaga todos los LEDs
void LEDS_Init(void)
{

	MAX7219_Init(SPI0, CLOCK_GetFreq(kCLOCK_Fro12M));
    LEDS_ClearAll();  // Apagar todos los LEDs al iniciar
}

// Enciende o apaga un LED específico en una tira
void LEDS_SetLED(led_t *led, bool state) {
    if (led->strip_id < 8 && led->led_id < 8) {
        // Modificar el estado del LED en led_state
        if (state) {
            led_state[led->strip_id] |= (1 << led->led_id);  // Enciende el LED específico
        } else {
            led_state[led->strip_id] &= ~(1 << led->led_id); // Apaga el LED específico
        }

        // Actualizar el estado en el MAX7219
        MAX7219_SendData(MAX7219_REG_DIGIT0 + led->strip_id, led_state[led->strip_id]);
    }
}

// Apaga todos los LEDs en todas las tiras
void LEDS_ClearAll(void) {
    for (uint8_t i = 0; i < 8; i++) {
        led_state[i] = 0x00;  // Apagar cada LED en el arreglo de estado
        MAX7219_SendData(MAX7219_REG_DIGIT0 + i, 0x00);  // Apagar cada tira en el MAX7219
    }
}

// Actualiza el estado de los LEDs que están configurados para titilar
void LEDS_Update(void) {
    uint32_t current_time = millis();  // Debes usar un temporizador para obtener el tiempo actual

    for (uint8_t i = 0; i < 8; i++) {
        // Si el LED tiene un intervalo de parpadeo configurado
        led_t *led = &leds[i];  // Asumiendo que tienes un arreglo de leds

        if (led->blink_interval > 0 && (current_time - led->last_blink_time) >= led->blink_interval) {
            // Cambiar el estado del LED
            led->state = !led->state;

            // Actualizar el estado en MAX7219
            LEDS_SetLED(led, led->state);

            // Actualizar el tiempo del último parpadeo
            led->last_blink_time = current_time;
        }
    }
}

// Establece el intervalo de parpadeo para un LED específico
void LEDS_SetBlinkInterval(led_t *led, uint32_t interval_ms) {
    led->blink_interval = interval_ms;
    led->last_blink_time = millis();  // Configura el tiempo actual
}
