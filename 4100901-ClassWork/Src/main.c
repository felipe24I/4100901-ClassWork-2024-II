#include <stdint.h>

#include "systick.h"
#include "gpio.h"
#include "uart.h"

extern uint8_t button_pressed;

void check_button_event(void)
{
    if (button_pressed == 1) {
        UART_send_string(USART2, "Single press detected\r\n");
        button_pressed = 0;
    } else if (button_pressed == 2) {
        UART_send_string(USART2, "Double press detected\r\n");
        button_pressed = 0;
    }
}

int main(void)
{
    configure_systick_and_start();
    configure_gpio();
    
    UART_Init(USART2);

    UART_send_string(USART2, "Hello World, from main!\r\n");

    extern uint8_t rx_byte;

    uint32_t hearbeat_tick = 0;
    while (1) {
        if (systick_GetTick() - hearbeat_tick > 500) {
            hearbeat_tick = systick_GetTick();
            check_button_event(); // Check for button press events every 500 ms
            gpio_toggle_led();
        }
        if (rx_byte != 0) {
            uint8_t command = rx_byte;
            UART_send_string(USART2, "Command received: ");
            UART_send_char(USART2, command);
            UART_send_string(USART2, "\r\n");
            rx_byte = 0; // Reset command
        }
        
        // TODO: Run the FSM here       
    }
}

