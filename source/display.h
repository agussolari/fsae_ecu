#ifndef DISP_H_
#define DISP_H_

#include <stdint.h>
#include <stdbool.h>
#include "max7219.h"

// Estructura para representar cada LED
typedef struct {
    uint8_t strip_id;     // ID de la tira de LEDs (0-7)
    uint8_t led_id;       // ID del LED dentro de la tira (0-7)
    bool state;           // Estado del LED (encendido o apagado)
    uint32_t blink_interval;  // Intervalo de parpadeo en ms
    uint32_t last_blink_time; // Último tiempo de parpadeo
} led_t;

extern led_t leds_strip[8][8]; // Asumiendo que tienes un arreglo de LEDs

// Inicializa el MAX7219 y apaga todos los LEDs
void LEDS_Init(void);

// Enciende o apaga un LED específico en una tira
void LEDS_SetLED(led_t *led, bool state);

// Apaga todos los LEDs en todas las tiras
void LEDS_ClearAll(void);

// Función que se llama periódicamente para controlar los LEDs que titilan
void LEDS_Update(void);

// Establece el intervalo de parpadeo para un LED específico
void LEDS_SetBlinkInterval(led_t *led, uint32_t interval_ms);

#endif // DISP_H_
