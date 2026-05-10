#ifndef UART_PACKET_ID_HPP__
#define UART_PACKET_ID_HPP__

#include "usart.h"
#include "uart_def.h"

/**
 * @brief Enumeration defining different states of receiving a byte over UART
 */
enum UART_PID_ReceivingState
{
  UART_PID_WAITING_FOR_START_BYTE, /**< Waiting for the start byte */
  UART_PID_RECEIVING_ID,           /**< Receiving ID */
  UART_PID_RECEIVING_DATA          /**< Receiving data */
};

/**
 * @brief Enumeration defining different status of received packet
 */
enum UART_PID_ReceivedStatus
{
  UART_PID_START_BYTE_MATCHED,
  UART_PID_START_BYTE_DIDNT_MATCH,
  UART_PID_ID_HASH_MATCHED,
  UART_PID_ID_HASH_DIDNT_MATCH,
  UART_PID_RECV_CPLT,
  UART_PID_DATA_HASH_DIDNT_MATCH
};

/**
 * @brief Enumeration defining different modes of UART_PID
 */
enum UART_PID_Mode
{
  UART_PID_RECEIVING,
  UART_PID_TRANSMITTING,
  UART_PID_BOTH
};

/**
 * @class UART_PID
 * @brief Class to handle UART communication with packet ID
 */
class UART_PID
{
public:
  /**
   * @brief Default constructor for UART_PID class.
   * @warning Be careful, do not call init() if uart handle is not set
   */
  UART_PID() = default;

  /**
   * @brief Constructor for the UART_PID class.
   * @param _huart UART_HandleTypeDef pointer to the UART handle
   * @param _mode UART_PID_Mode mode of the UART_PID
   */
  UART_PID(UART_HandleTypeDef *_huart, const UART_PID_Mode _mode);

  /**
   * @brief Default destructor for UART_PID class.
   */
  ~UART_PID() = default;

  /**
   * @brief Initializes the UART_PID object.
   */
  void init();

  /**
   * @brief Receives data over UART.
   * @param id ID of the received packet
   * @param len Length of the received packet
   * @return UART_PID_ReceivedStatus status of the received packet
   */
  UART_PID_ReceivedStatus receive(uint8_t &id, uint8_t &len);

  /**
   * @brief Gets the received data.
   * @param data Pointer to the received data
   */
  void get_received_data(uint8_t *);

  /**
   * @brief Transmits data over UART.
   * @param data Pointer to the data to be transmitted
   * @param len Length of the data to be transmitted
   * @param id ID of the packet to be transmitted
   */
  void transmit(uint8_t *data, uint8_t len, uint8_t id);
  

  UART_HandleTypeDef *huart;               /**< UART_HandleTypeDef pointer to the UART handle */
  UART_PID_Mode mode = UART_PID_RECEIVING; /**< UART_PID_Mode mode of the UART_PID */

  uint32_t last_receive_tick = 0;  /**< Milliseconds since last receive cplt */
  uint32_t last_transmit_tick = 0; /**< Milliseconds since last transmit cplt */

  uint8_t receive_buffer[UART_MAX_RX_BUFFER_SIZE];  /**< Receive buffer */
  uint8_t transmit_buffer[UART_MAX_TX_BUFFER_SIZE]; /**< Transmit buffer */

  UART_PID_ReceivingState receiving_state = UART_PID_WAITING_FOR_START_BYTE; /**< UART_PID_ReceivingState state of receiving a byte over UART */
  UART_PID_ReceivedStatus received_status;                                   /**< UART_PID_ReceivedStatus status of received packet */
};

#endif // UART_PID_HPP__