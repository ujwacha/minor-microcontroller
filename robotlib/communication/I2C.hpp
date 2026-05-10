#ifndef I2C_HPP
#define I2C_HPP

#include "processor_type.h"

#ifndef I2C_HEAD
#define I2C_HEAD 0xA5 /**< Start byte of I2C packet */
#endif

#ifndef I2C_RX_BUFFER_SIZE
#define I2C_RX_BUFFER_SIZE 50 /**< Maximum size of I2C buffer */
#endif
#ifndef I2C_TX_BUFFER_SIZE
#define I2C_TX_BUFFER_SIZE 50 /**< Maximum size of I2C buffer */
#endif

#if I2C_RX_BUFFER_SIZE < 10 || I2C_TX_BUFFER_SIZE < 10
#error "I2C buffer size cannot be less than 10"
#endif

enum I2CRole
{
    I2C_MASTER, /**< I2C in master mode */
    I2C_SLAVE   /**< I2C in slave mode */
};

enum I2CMode
{
    I2C_MODE_RX,  /**< I2C in receiving mode */
    I2C_MODE_TX,  /**< I2C in transmitting mode */
    I2C_MODE_BOTH /**< I2C in both receiving and transmitting mode */
};

enum I2CReceiveStatus
{
    I2C_HEAD_MATCHED, /**< Head of packet matched */
    I2C_HEAD_ERROR,   /**< Head of packet failed to match */
    I2C_CRC_MATCHED,         /**< CRC hash matched */
    I2C_CRC_ERROR,           /**< CRC hash failed to match */
    I2C_INTERRUPT_CRASHED    /**< I2C Interrupt crashed while receiving*/
};

class I2C
{
public:
    I2C() : hi2c(nullptr) {}
    I2C(I2C_HandleTypeDef *_hi2c, I2CRole _role, uint8_t _address, uint16_t _rx_size, uint16_t _tx_size)
        : hi2c(_hi2c), role(_role), address(_address), rx_size(_rx_size), tx_size(_tx_size) {}
    I2C(const I2C &) = default;
    I2C &operator=(const I2C &) = default;
    ~I2C() = default;

    bool init();
    bool connected();
    void transmit(uint8_t *data);
    bool get_received_data(uint8_t *data);
    I2CReceiveStatus rx_callback();
    void tx_callback();
    I2C_HandleTypeDef *get_hadle() { return hi2c; }

    void switch_role(I2CRole _role) { role = _role; }
    void switch_mode(I2CMode _mode) { mode = _mode; }
    void switch_slave(uint8_t _address) { address = _address; }

    uint16_t get_rx_seq() { return rx_seq; }
    uint16_t get_prev_rx_seq() { return prev_rx_seq; }
    uint32_t get_last_rx_tick() { return last_rx_tick; }
    uint32_t get_last_tx_tick() { return last_tx_tick; }
    uint32_t get_last_rx_cplt_tick() { return last_rx_cplt_tick; }
    uint32_t get_last_tx_cplt_tick() { return last_tx_cplt_tick; }
    uint32_t get_rx_cplt_period() { return rx_cplt_period; }
    uint32_t get_tx_cplt_period() { return tx_cplt_period; }
    uint32_t get_head_error_count() { return head_error_count; }
    uint32_t get_hash_error_count() { return hash_error_count; }
    uint32_t get_tx_error_count() { return tx_error_count; }

    float get_rx_cplt_freq() { return rx_cplt_period > 0 ? 1000.0f / rx_cplt_period : 0; }
    float get_tx_cplt_freq() { return rx_cplt_period > 0 ? 1000.0f / tx_cplt_period : 0; }
    float get_rx_throughput(uint32_t time = 1000) { return rx_cplt_period > 0 && time > 0 ? (rx_size + 2) * time / rx_cplt_period : 0; }
    float get_tx_throughput(uint32_t time = 1000) { return tx_cplt_period > 0 && time > 0 ? (tx_size + 2) * time / tx_cplt_period : 0; }

    void print_i2c_status();
    void print_rx_buffer();
    void print_tx_buffer();

    __weak static void I2C_RxErrorCallBack(I2C_HandleTypeDef *hi2c, I2CReceiveStatus status) { (void)hi2c; (void)status; }
    __weak static void I2C_TxErrorCallBack(I2C_HandleTypeDef *hi2c) { (void)hi2c; }

private:
    I2C_HandleTypeDef *hi2c; /**< I2C handler */
    I2CRole role;            /** Role of I2C */
    uint8_t address;         /**< Address of SLave to be used if it is master */
    I2CMode mode;            /**< Mode of I2C */
    uint16_t rx_size;        /**< Sizes for receiving data */
    uint16_t tx_size;        /**< Sizes for tarnsmitting data */

    uint8_t rx_buffer[I2C_RX_BUFFER_SIZE];   /**< Array to receive data */
    uint8_t tx_buffer[I2C_TX_BUFFER_SIZE];   /**< Array to transmit start byte, data, hash */
    uint8_t rx_data[I2C_RX_BUFFER_SIZE - 2]; /**< Array to receive data */

    bool is_transmitting;
    bool is_head;
    I2CReceiveStatus received_status; /**< last receive status */
    uint16_t rx_seq = 0;              /**< Sequence number of received data */
    uint16_t prev_rx_seq = 0;         /**< Previous sequence number of received data */

    uint32_t last_rx_tick = 0;      /**< last receive function call tick  */
    uint32_t last_tx_tick = 0;      /**< last  transmit function call tick */
    uint32_t last_rx_cplt_tick = 0; /**< last receive complete tick after crc matched */
    uint32_t last_tx_cplt_tick = 0; /**< last transmit complete tick after transmit cplt callback */
    uint32_t rx_cplt_period = 0;    /**< Period of last receive complete */
    uint32_t tx_cplt_period = 0;    /**< Period of last transmit complete */

    uint32_t head_error_count = 0; /**< Count of start byte errors */
    uint32_t hash_error_count = 0; /**< Count of CRC hash errors */
    uint32_t tx_error_count = 0;   /**< Count of transmit errors */
};

#endif // I2C_HPP
