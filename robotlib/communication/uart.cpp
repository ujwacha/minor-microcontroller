/***********************************************************************************************
 * @file uart.cpp
 * @brief Implementation file for the UART class providing communication functionality.
 * @author Robotics Team, IOE Pulchowk Campus
 * @date 2023
 **********************************************************************************************/

#include <memory.h>
#include <stdio.h>

#include "uart.hpp"
#include "../crypto/easy_crc.h"
// #include "../crypto/crc8.hpp"

UART::UART() : huart(nullptr) {}

UART::UART(UART_HandleTypeDef *_huart, UARTMode _mode, uint16_t _rt_size, uint16_t _t_size)
    : huart(_huart), mode(_mode)
{
    if (_mode == UART_RECEIVING || _mode == UART_BOTH)
    {
        r_size = _rt_size;
    }
    if (_mode == UART_TRANSMITTING || _mode == UART_BOTH)
    {
        t_size = _t_size > 0 ? _t_size : _rt_size;
    }
}

bool UART::init()
{
    if (huart == nullptr)
        return false;

    is_waiting_for_start_byte = true;
    is_transmitting = false;

    if (mode == UART_RECEIVING || mode == UART_BOTH)
    {
        HAL_UART_Receive_DMA(huart, receive_buffer, 1);
    }

    return true;
}

/**
 * This function receives data over UART. It performs the following steps:
 * - Checks whether the start byte has been received.
 *   - If the start byte is received, proceeds to receive the data payload along with the CRC.
 *   - If the start byte is not received, continues waiting for it.
 * - Once the data is received, validates the CRC.
 * - Copies the data to the provided buffer and updates the reception status accordingly.
 *
 * @attention This function assumes that the UART has been initialized in RECEIVING mode or BOTH
 *            prior to calling this function. If the UART is initialized in a different mode,
 *            the behavior of this function may not be as expected.
 */
UARTReceiveStatus UART::process_receive_callback()
{
    /* Know the current tick */
    uint32_t current_tick = HAL_GetTick();

    /* Check whether received start byte or data */
    if (is_waiting_for_start_byte)
    {
        /* If it is start byte, check its value */
        if (receive_buffer[0] == UART_START_BYTE)
        {
            /* Reset start byte flag for receiving data */
            is_waiting_for_start_byte = false;

            /* Trigger DMA for receiving data */
            HAL_UART_Receive_DMA(huart, receive_buffer + 1, r_size + 1);

            /* Update received_status for start byte matched */
            received_status = UART_START_BYTE_MATCHED;
        }
        else /* Start byte failed */
        {
            /* Increase start byte error count for testing*/
            start_byte_error_count++;
            /* Call user defined error callback */
            UART_RxErrorCallBack(huart, UART_START_BYTE_ERROR);
            /* Again try to receive start byte */
            HAL_UART_Receive_DMA(huart, receive_buffer, 1);

            /* Update received_status for start byte failure */
            received_status = UART_START_BYTE_ERROR;
        }
    }
    else /* Receive data */
    {
        /* Calculate CRC for received data */
        // uint8_t hash = CRC8::Instance.get_hash(receive_buffer + 1, r_size);

        uint8_t hash = calculate_cr8x_fast(receive_buffer + 1,
                                           r_size);

        //        uint8_t hash = CRC8::Instance.get_hash(receive_buffer, r_size + 1); // To make it work for our standard

        /* Compare with received Hash */
        if (hash == receive_buffer[r_size + 1])
        {
            /* Put data from receive buffer to receive data*/
            memcpy(receive_data, receive_buffer + 1, r_size);

            /* Update received_status for CRC Matched */
            received_status = UART_CRC_MATCHED;

            /* Update receive_seq, receive_cplt_period and last_receive_cplt_tick as new data is received in user memory */
            if (receive_seq == 65535)
                receive_seq = 0;
            else
                receive_seq++;

            receive_cplt_period = current_tick - last_receive_cplt_tick;
            last_receive_cplt_tick = current_tick;
        }
        else /* Hash didn't match */
        {
            /* Increase check error count for testing*/
            check_error_count++;
            /* Call user defined error callback */
            UART_RxErrorCallBack(huart, UART_CRC_ERROR);
            /* Update received_status for CRC failure */
            received_status = UART_CRC_ERROR;
        }

        /* Set start byte flag to receive start byte again for the next packet */
        is_waiting_for_start_byte = true;

        /* Trigger DMA for receiving start byte */
        HAL_UART_Receive_DMA(huart, receive_buffer, 1);
    }

    /* Update last_receive_tick to track receive call */
    last_receive_tick = current_tick;

    /* Return received_status */
    return received_status;
}

void UART::process_transmit_callback()
{
    /* Update transmit_cplt_period */
    uint32_t now = HAL_GetTick();
    transmit_cplt_period = now - last_transmit_cplt_tick;
    last_transmit_cplt_tick = now;
    is_transmitting = false;
}

/**
 * This function receives data over UART. It checks whether the data has been received
 * and copies the data to the provided buffer.
 */
bool UART::get_received_data(uint8_t *r_data)
{
    if (receive_seq != prev_receive_seq)
    {
        /* Copy the data*/
        memcpy(r_data, receive_data, r_size);
        /*Update prev_receive_seq so we can use it in next receive*/
        prev_receive_seq = receive_seq;

        return true;
    }

    return false;
}

/**
 * This function transmits data over UART. It prepares the data packet by loading
 * the start byte, copying the transmitted data, calculating and loading the CRC,
 * then transmitting the data using DMA. It also updates the transmit tick.
 *
 * @note This function assumes that the UART communication has been initialized
 *       prior to calling this function.
 */
bool UART::transmit(uint8_t *t_data)
{
    /* Load start byte */
    transmit_buffer[0] = UART_START_BYTE;

    /* Load tx data */
    memcpy(transmit_buffer + 1, t_data, t_size);

    /* Calculate and Load CRC */
    // transmit_buffer[t_size + 1] = CRC8::Instance.get_hash(transmit_buffer + 1, t_size);
    // transmit_buffer[t_size + 1] = CRC8::Instance.get_hash(transmit_buffer, t_size + 1); // for our standard

    transmit_buffer[t_size + 1] = calculate_cr8x_fast(t_data,
                                                      t_size);

    /* Transmit tx data using DMA */
    if (HAL_UART_Transmit_DMA(huart, transmit_buffer, t_size + 2) == HAL_OK)
    {
        /* Update transmit tick */
        last_transmit_tick = HAL_GetTick();
        /* Update is_transmitting flag */
        is_transmitting = true;
        return true;
    }

    /* Increase transmit error count for testing*/
    transmit_error_count++;
    /* Call user defined error callback */
    UART_TxErrorCallBack(huart);

    return false;
}

USART_TypeDef *UART::get_uart_instance()
{
    return huart->Instance;
}

bool UART::connected()
{
    return (HAL_GetTick() - last_receive_cplt_tick < UART_CONNCETION_TIMEOUT);
}

uint32_t UART::get_last_receive_tick()
{
    return last_receive_tick;
}

uint32_t UART::get_last_transmit_tick()
{
    return last_transmit_tick;
}

uint32_t UART::get_last_receive_cplt_tick()
{
    return last_receive_cplt_tick;
}

uint32_t UART::get_receive_cplt_period()
{
    return receive_cplt_period;
}

float UART::get_receive_cplt_frequency()
{
    if (receive_cplt_period == 0)
        return 0.0f;

    return 1000.0f / receive_cplt_period;
}

uint32_t UART::get_receive_throughput(uint32_t time)
{
    if (receive_cplt_period == 0 || time == 0)
        return 0;

    return (r_size + 2) * time / receive_cplt_period;
}

uint32_t UART::get_receive_seq()
{
    return receive_seq;
}

uint32_t UART::get_transmit_cplt_period()
{
    return transmit_cplt_period;
}

float UART::get_transmit_cplt_frequency()
{
    if (transmit_cplt_period == 0)
        return 0.0f;

    return 1000.0f / transmit_cplt_period;
}

uint32_t UART::get_transmit_throughput(uint32_t time)
{
    if (transmit_cplt_period == 0 || time == 0)
        return 0;

    return (t_size + 2) * time / transmit_cplt_period;
}

void UART::print_uart_status()
{
    printf("========================================\n");
    printf("UART Status\n");
    printf("========================================\n");
    printf("Receive Status:\n");
    printf("----------------------------------------\n");
    printf("is_waiting_for_start_byte:: %d\n", is_waiting_for_start_byte);
    printf("Receive Seq:: %hu\n", receive_seq);
    printf("Start Byte Error Count:: %lu\n", start_byte_error_count);
    printf("Check Error Count:: %lu\n", check_error_count);
    printf("Last Receive Tick:: %lu\n", last_receive_tick);
    printf("Last Transmit Tick:: %lu\n", last_transmit_tick);
    printf("Last Receive Cplt Tick:: %lu\n", last_receive_cplt_tick);
    printf("Last Transmit Cplt Tick:: %lu\n", last_transmit_cplt_tick);
    printf("Receive Cplt Period:: %lu\n", receive_cplt_period);
    printf("Receive Cplt Frequency:: %.2f\n", get_receive_cplt_frequency());
    printf("Receive Throughput:: %lu\n", get_receive_throughput());
    printf("----------------------------------------\n");
    printf("Transmit Status:\n");
    printf("----------------------------------------\n");
    printf("is_transmitting:: %d\n", is_transmitting);
    printf("Transmit Cplt Period:: %lu\n", transmit_cplt_period);
    printf("Transmit Cplt Frequency:: %.2f\n", get_transmit_cplt_frequency());
    printf("Transmit Throughput:: %lu\n", get_transmit_throughput());
    printf("========================================\n\n");
}

void UART::print_receive_buffer()
{
    printf("UART RX Data::");
    for (int i = 0; i <= r_size + 1; ++i)
    {
        printf(" %02x", receive_buffer[i]);
    }
    printf("\n");
}

void UART::print_transmit_buffer()
{
    printf("UART TX Data::");
    for (int i = 0; i <= t_size + 1; ++i)
    {
        printf("%02x ", transmit_buffer[i]);
    }
    printf("\n");
}

/*Weakly defined, so user can have its own implementation and donot throw error if not defined*/
__weak void UART::UART_RxErrorCallBack(UART_HandleTypeDef *huart, UARTReceiveStatus status)
{
    (void)huart;
    (void)status;
}

/*Weakly defined, so user can have its own implementation and donot throw error if not defined*/
__weak void UART::UART_TxErrorCallBack(UART_HandleTypeDef *huart)
{
    (void)huart;
}
