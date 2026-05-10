#ifndef UART_DEFINES_H__
#define UART_DEFINES_H__

#ifndef UART_START_BYTE
#define UART_START_BYTE 0xA5 /**< Start byte of packet */
#endif

#ifndef UART_MAX_RX_BUFFER_SIZE
#define UART_MAX_RX_BUFFER_SIZE 50 /**< Maximum size of receive buffer */
#endif

#ifndef UART_MAX_TX_BUFFER_SIZE
#define UART_MAX_TX_BUFFER_SIZE 50 /**< Maximum size of transmit buffer */
#endif

#ifndef UART_CONNCETION_TIMEOUT
#define UART_CONNCETION_TIMEOUT 100 /**< Timeout for UART connection */
#endif

#endif // UART_DEFINES_H__
