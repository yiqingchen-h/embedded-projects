#include "LED.h"
void Led_On(unsigned char LED)
{
    switch (LED) {
        case LED_RED:
             DL_GPIO_setPins(LED_PORT, LED_Red_PIN);
             break;
        case LED_BLUE:
            DL_GPIO_setPins(LED_PORT, LED_Blue_PIN);
             break;
        case LED_GREEN:
            DL_GPIO_setPins(LED_PORT, LED_Green_PIN);
             break;
        default:
            break;
    }
}

void Led_Off(unsigned char LED)
{
    switch (LED) {
        case LED_RED:
             DL_GPIO_clearPins(LED_PORT, LED_Red_PIN);
             break;
        case LED_BLUE:
            DL_GPIO_clearPins(LED_PORT, LED_Blue_PIN);
             break;
        case LED_GREEN:
            DL_GPIO_clearPins(LED_PORT, LED_Green_PIN);
             break;
        default:
            break;
    }
}


