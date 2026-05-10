#include <memory.h>
#include <stdio.h>

#include "I2C.hpp"
#include "crc8.hpp"

bool I2C::init()
{
    if (hi2c == nullptr)
        return false;

    is_head = true;
    is_transmitting = false;

    if (mode == I2CMode::I2C_MODE_RX || mode == I2CMode::I2C_MODE_BOTH)
    {
        if (role == I2C_MASTER)
        {
            if (HAL_I2C_Master_Receive_DMA(hi2c, address, rx_data, rx_size) != HAL_OK)
                return false;
        }
        else
        {
            if (HAL_I2C_Slave_Receive_DMA(hi2c, rx_data, rx_size) != HAL_OK)
                return false;
        }
    }

    return true;
}

I2CReceiveStatus I2C::rx_callback()
{
    /* Know the current tick */
    uint32_t current_tick = HAL_GetTick();

    /* Check whether received head or data */
    if (is_head)
    {
        /* If it is start byte, check its value */
        if (rx_buffer[0] == I2C_HEAD)
        {
            /* Reset start byte flag for receiving data */
            is_head = false;

            /* Trigger DMA for receiving data */
            if (role == I2C_MASTER)
                HAL_I2C_Master_Receive_DMA(hi2c, address, rx_buffer, rx_size + 1);
            else
                HAL_I2C_Slave_Receive_DMA(hi2c, rx_buffer, rx_size + 1);

            /* Update received_status for start byte matched */
            received_status = I2C_HEAD_MATCHED;
        }
        else /* Start byte failed */
        {
            /* Increase start byte error count for testing*/
            head_error_count++;
            /* Call user defined error callback */
            I2C_RxErrorCallBack(hi2c, I2C_HEAD_ERROR);
            /* Again try to receive start byte */
            if (role == I2C_MASTER)
                HAL_I2C_Master_Receive_DMA(hi2c, address, rx_buffer, 1);
            else
                HAL_I2C_Slave_Receive_DMA(hi2c, rx_buffer, 1);

            /* Update received_status for start byte failure */
            received_status = I2C_HEAD_ERROR;
        }
    }
    else /* Receive data */
    {
        /* Calculate CRC for received data */
        uint8_t hash = CRC8::Instance.get_hash(rx_buffer + 1, rx_size);

        /* Compare with received Hash */
        if (hash == rx_buffer[rx_size + 1])
        {
            /* Put data from receive buffer to receive data*/
            memcpy(rx_data, rx_buffer + 1, rx_size);

            /* Update received_status for CRC Matched */
            received_status = I2C_CRC_MATCHED;

            /* Update receive_seq, receive_cplt_period and last_receive_cplt_tick as new data is received in user memory */
            if (rx_seq == 65535)
                rx_seq = 0;
            else
                rx_seq++;

            rx_cplt_period = current_tick - last_rx_cplt_tick;
            last_rx_cplt_tick = current_tick;
        }
        else /* Hash didn't match */
        {
            /* Increase check error count for testing*/
            hash_error_count++;
            /* Call user defined error callback */
            I2C_RxErrorCallBack(hi2c, I2C_CRC_ERROR);
            /* Update received_status for CRC failure */
            received_status = I2C_CRC_ERROR;
        }

        /* Set start byte flag to receive start byte again for the next packet */
        is_head = true;

        /* Trigger DMA for receiving start byte */
        if (role == I2C_MASTER)
            HAL_I2C_Master_Receive_DMA(hi2c, address, rx_buffer, 1);
        else
            HAL_I2C_Slave_Receive_DMA(hi2c, rx_buffer, 1);
    }

    /* Update last_receive_tick to track receive call */
    last_rx_tick = current_tick;

    /* Return received_status */
    return received_status;
}

void I2C::tx_callback()
{
    /* Update transmit_cplt_period */
    uint32_t now = HAL_GetTick();
    tx_cplt_period = now - last_tx_cplt_tick;
    last_tx_cplt_tick = now;
    is_transmitting = false;
}

void I2C::transmit(uint8_t* data)
{
    tx_buffer[0] = I2C_HEAD;
    memcpy(tx_buffer + 1, data, tx_size);
    tx_buffer[tx_size + 1] = CRC8::Instance.get_hash(data, tx_size);
    if (role == I2C_MASTER)
        HAL_I2C_Master_Transmit_DMA(hi2c, address, tx_buffer, tx_size + 2);
    else
        HAL_I2C_Slave_Transmit_DMA(hi2c, tx_buffer, tx_size + 2);
}

void I2C::print_i2c_status()
{
    printf("========================================\n");
    printf("I2C Status\n");
    printf("========================================\n");
    printf("Receive Status:\n");
    printf("----------------------------------------\n");
    printf("is_head:: %d\n", is_head);
    printf("Rx Seq:: %hu\n", rx_seq);
    printf("Head Error Count:: %lu\n", head_error_count);
    printf("Hash Error Count:: %lu\n", hash_error_count);
    printf("Last Rx Tick:: %lu\n", last_rx_tick);
    printf("Last Tx Tick:: %lu\n", last_tx_tick);
    printf("Last Rx Cplt Tick:: %lu\n", last_rx_cplt_tick);
    printf("Last Tx Cplt Tick:: %lu\n", last_tx_cplt_tick);
    printf("Rx Cplt Period:: %lu\n", rx_cplt_period);
    printf("Rx Cplt Freq:: %.2f\n", get_rx_cplt_freq());
    printf("Rx Throughput:: %f\n", get_rx_throughput());
    printf("----------------------------------------\n");
    printf("Transmit Status:\n");
    printf("----------------------------------------\n");
    printf("is_transmitting:: %d\n", is_transmitting);
    printf("Tx Cplt Period:: %lu\n", tx_cplt_period);
    printf("Tx Cplt Freq:: %.2f\n", get_tx_cplt_freq());
    printf("Tx Throughput:: %f\n", get_tx_throughput());
    printf("========================================\n\n");
}

void I2C::print_rx_buffer()
{
    printf("========================================\n");
    printf("Receive Buffer\n");
    printf("========================================\n");
    for (uint16_t i = 0; i < rx_size + 2; i++)
    {
        printf("%02X ", rx_buffer[i]);
    }
    printf("\n========================================\n\n");
}

void I2C::print_tx_buffer()
{
    printf("========================================\n");
    printf("Transmit Buffer\n");
    printf("========================================\n");
    for (uint16_t i = 0; i < tx_size + 2; i++)
    {
        printf("%02X ", tx_buffer[i]);
    }
    printf("\n========================================\n\n");
}
