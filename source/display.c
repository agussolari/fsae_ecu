#include <display.h>
#include "fsl_gpio.h"
#include "millis.h"  // Dependiendo de la biblioteca de temporización que utilices

// Almacena el estado actual de los LEDs para cada tira
static uint8_t led_state[8] = {0};
static led_t leds_strip[8][8] = {0};  // Asumiendo que tienes un arreglo de LEDs

// Inicializa el MAX7219 y apaga todos los LEDs
void LEDS_Init(void)
{

	MAX7219_Init();  // Inicializar el MAX7219
    LEDS_ClearAll();  // Apagar todos los LEDs al iniciar

    // Inicializar los LEDs
	for (uint8_t i = 0; i < 8; i++) {
		for (uint8_t j = 0; j < 8; j++) {
			leds_strip[i][j].strip_id = i;
			leds_strip[i][j].led_id = j;
			leds_strip[i][j].state = false;
			leds_strip[i][j].blink_interval = 0;
			leds_strip[i][j].last_blink_time = 0;
		}
	}
}

// Enciende o apaga un LED específico en una tira
void LEDS_SetLED(led_t *led, bool state)
{
	// Actualizar el estado del LED
	led->state = state;

	// Actualizar el estado en MAX7219
	if (state) {
		led_state[led->strip_id] |= (1 << led->led_id);  // Encender el LED
	} else {
		led_state[led->strip_id] &= ~(1 << led->led_id);  // Apagar el LED
	}

	MAX7219_SendData(MAX7219_REG_DIGIT0 + led->strip_id,
			led_state[led->strip_id]);  // Actualizar el estado en MAX7219
}

// Apaga todos los LEDs en todas las tiras
void LEDS_ClearAll(void)
{
	// Apagar todos los LEDs en todas las tiras
	for (uint8_t i = 0; i < 8; i++) {
		led_state[i] = 0;
		MAX7219_SendData(MAX7219_REG_DIGIT0 + i, 0);  // Apagar todos los LEDs
	}

}

// Actualiza el estado de los LEDs que están configurados para titilar
void LEDS_Update(void) {
    uint32_t current_time = millis();  // Debes usar un temporizador para obtener el tiempo actual
    // Iterar sobre todos los LEDs

        for (uint8_t i = 0; i < 8; i++) {
		for (uint8_t j = 0; j < 8; j++) {
			led_t *led = &leds_strip[i][j];
			if (led->blink_interval > 0) {
				// Verificar si es tiempo de parpadear
				if ((current_time - led->last_blink_time) >= led->blink_interval) {
					// Alternar el estado del LED
					LEDS_SetLED(led, !led->state);
					led->last_blink_time = current_time; // Actualizar el tiempo de parpadeo
				}
			}
		}
        }
}

// Establece el intervalo de parpadeo para un LED específico
void LEDS_SetBlinkInterval(led_t *led, uint32_t interval_ms) {
    led->blink_interval = interval_ms;
    led->last_blink_time = millis();  // Configura el tiempo actual
}
