#include <memory.h>
#include <stdio.h>
#include "robotlib/maths/math.hpp"
#include "robotlib/crypto/crc8.hpp"
#include "uart_pid.hpp"

// clang-format off
#define START_INDEX         0
#define ID_INDEX            1
#define LEN_INDEX           2
#define ID_CRC_INDEX        3
#define DATA_START_INDEX    4
#define DATA_END_INDEX      (3 + receive_buffer[LEN_INDEX])
#define DATA_CRC_INDEX      (1 + DATA_END_INDEX)
// clang-format on

UART_PID::UART_PID(UART_HandleTypeDef *_huart, UART_PID_Mode _mode = UART_PID_RECEIVING)
    : huart(_huart), mode(_mode), last_receive_tick(0), last_transmit_tick(0)
{
}

void UART_PID::init()
{
    if (mode == UART_PID_RECEIVING || mode == UART_PID_BOTH)
    {
        HAL_UART_Receive_DMA(huart, receive_buffer, 1);
    }

    last_receive_tick = HAL_GetTick();
    last_transmit_tick = HAL_GetTick();
}

UART_PID_ReceivedStatus UART_PID::receive(uint8_t &id, uint8_t &len)
{
    bool do_receive_start_byte = false;

    if (receiving_state == UART_PID_WAITING_FOR_START_BYTE)
    {
        if (receive_buffer[START_INDEX] == UART_START_BYTE)
        {
            HAL_UART_Receive_DMA(huart, receive_buffer + ID_INDEX, 3);
            receiving_state = UART_PID_RECEIVING_ID;
            received_status = UART_PID_START_BYTE_MATCHED;
        }
        else
        {
            received_status = UART_PID_START_BYTE_DIDNT_MATCH;
            do_receive_start_byte = true;
            printf("Start byte error\n");
        }
    }
    else if (receiving_state == UART_PID_RECEIVING_ID)
    {
        uint8_t hash = CRC8::Instance.get_hash(receive_buffer + ID_INDEX, 2);
        if (hash == receive_buffer[ID_CRC_INDEX])
        {
            HAL_UART_Receive_DMA(huart, receive_buffer + DATA_START_INDEX, receive_buffer[LEN_INDEX] + 1);
            id = receive_buffer[ID_INDEX];
            len = receive_buffer[LEN_INDEX];
            receiving_state = UART_PID_RECEIVING_DATA;
            received_status = UART_PID_ID_HASH_MATCHED;
            printf("Packet id received, id:%d len:%d hash:%02x\n", (int)id, (int)len, (int)hash);
        }
        else
        {
            received_status = UART_PID_ID_HASH_DIDNT_MATCH;
            do_receive_start_byte = true;
            printf("Id Hash Error: %02x\n", (int)hash);
        }
    }
    else if (receiving_state == UART_PID_RECEIVING_DATA)
    {
        uint8_t hash = CRC8::Instance.get_hash(receive_buffer + DATA_START_INDEX, receive_buffer[LEN_INDEX]);
        if (hash == receive_buffer[DATA_CRC_INDEX])
        {
            id = receive_buffer[ID_INDEX];
            len = receive_buffer[LEN_INDEX];
            received_status = UART_PID_RECV_CPLT;
            last_receive_tick = HAL_GetTick();
            printf("Data received Received id:%d len:%d\n", (int)id, (int)len);
        }
        else
        {
            printf("data hash error :%02x\n", (int)hash);
            received_status = UART_PID_DATA_HASH_DIDNT_MATCH;
        }
        do_receive_start_byte = true;
    }

    if (do_receive_start_byte == true)
    {
        HAL_UART_Receive_DMA(huart, receive_buffer, 1);
        receiving_state = UART_PID_WAITING_FOR_START_BYTE;
    }

    return received_status;
}

void UART_PID::transmit(uint8_t *data, uint8_t len, uint8_t id)
{
    transmit_buffer[0] = UART_START_BYTE;
    transmit_buffer[1] = id;
    transmit_buffer[2] = len;
    transmit_buffer[3] = CRC8::Instance.get_hash(transmit_buffer + ID_INDEX, 2);
    memcpy(transmit_buffer + 4, data, len);
    transmit_buffer[4 + len] = CRC8::Instance.get_hash(transmit_buffer + 4, len);

    HAL_UART_Transmit_DMA(huart, transmit_buffer, len + 5);
    last_transmit_tick = HAL_GetTick();
}

void UART_PID::get_received_data(uint8_t *r_data)
{
    memcpy(r_data, receive_buffer + DATA_START_INDEX, receive_buffer[LEN_INDEX]);
}