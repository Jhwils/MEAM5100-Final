// /* Just echos back on UART whatever is sent, adds a "." between each char
//  * uses pins PD2=RX and PD3=TX of ATmega32U4
//  * 
//  */

// #include "MEAM_general.h"
// #include "uart.h"

// int main(void) {
//   uint8_t c;
//   _clockdivide(0);          
//   uart_init(115200);
//   while (1) {
//     if (uart_available()) {
//       uart_putchar('.');    // put '.'

//       c = uart_getchar(); // get a char
//       uart_putchar(c);    // put a char
//     }
//   }
// }





