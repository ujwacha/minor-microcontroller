#ifndef BNO08_SPI_HPP
#define BNO08_SPI_HPP

// Registers
#include <stdint.h>

#include "spi.h"

enum Registers {
  CHANNEL_COMMAND = 0,
  CHANNEL_EXECUTABLE = 1,
  CHANNEL_CONTROL = 2,
  CHANNEL_REPORTS = 3,
  CHANNEL_WAKE_REPORTS = 4,
  CHANNEL_GYRO = 5
};

#pragma pack(push, 1)
struct ImuData {
  volatile float yaw;
  volatile float pitch;
  volatile float roll;
  volatile float accel_x;
  volatile float accel_y;
  volatile float accel_z;
};
#pragma pack(pop)

// All the ways we can configure or talk to the BNO080, figure 34, page 36
// reference manual These are used for low level communication with the sensor,
// on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// All the different sensors and features we can get reports from
// These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

// Record IDs from figure 29, page 29 reference manual
// These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

// Command IDs from section 6.4, page 42
// These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128
#define MAX_METADATA_SIZE 9

class Bno {
 private:
  SPI_HandleTypeDef *hspi;

  GPIO_TypeDef *CS_PORT;
  uint16_t CS_PIN;

  GPIO_TypeDef *INT_PORT;
  uint16_t INT_PIN;

  GPIO_TypeDef *RST_PORT;
  uint16_t RST_PIN;

  // Each packet has a header of 4 bytes
  uint8_t shtp_header[4];
  uint8_t shtp_data[MAX_PACKET_SIZE];
  // There are 6 com channels. Each channel has its own seqnum
  uint8_t sequence_number[6] = {0, 0, 0, 0, 0, 0};
  // Commands have a seqNum as well. These are inside command packet,
  // the header uses its own seqNum per channel
  uint8_t command_sequence_number = 0;
  // There is more than 10 words in a metadata
  // record but we'll stop at Q point 3
  uint32_t meta_data[MAX_METADATA_SIZE];

  // These are the raw sensor values pulled from the user requested Input Report
  uint16_t raw_accel_x, raw_accel_y, raw_accel_z, accel_accuracy;
  uint16_t raw_linear_accel_x, raw_linear_accel_y, raw_linear_accel_z,
      linear_accel_accuracy;
  uint16_t raw_gyro_x, raw_gyro_y, raw_gyro_z, gyro_accuracy;
  uint16_t raw_mag_x, raw_mag_y, raw_mag_z, mag_accuracy;
  uint16_t raw_quat_I, raw_quat_J, raw_quat_K, raw_quat_Real,
      raw_quat_radian_accuracy, quatAccuracy;
  uint16_t step_count;
  uint32_t time_stamp;
  uint8_t stability_classifier;
  uint8_t activity_clasifier;
  uint8_t *_activity_confidences;  // Array that store the confidences of the 9
                                   // possible activities
  uint8_t calibration_status;      // Byte R0 of ME Calibration Response

  volatile float quaternions[4];

  ImuData bno_data;

  // These Q values are defined in the datasheet but can also be obtained by
  // querying the meta data records See the read metadata example for more info
  int16_t rotation_vector_Q1 = 14;
  int16_t accelerometer_Q1 = 8;
  int16_t linear_accelerometer_Q1 = 8;
  int16_t gyro_Q1 = 9;
  int16_t magnetometer_Q1 = 4;

  uint8_t wait_for_rst_low();

  uint8_t receive_packet();

  uint8_t send_byte(uint8_t data);

  uint8_t send_packet(uint8_t channel_number, uint8_t data_size);

  void set_feature_command(uint8_t report_id, uint32_t micros_between_reports,
                           uint32_t specific_config);

  void parse_input_report();

  void parse_command_report();

  float get_quat_radian_accuracy();
  uint8_t get_quat_accuracy();
  float get_accel_x();
  float get_accel_y();
  float get_accel_z();
  uint8_t get_accel_accuracy();
  float get_linear_accel_x();
  float get_linear_accel_y();
  float get_linear_accel_z();
  uint8_t get_linear_accel_accuracy();
  float get_gyro_x();
  float get_gyro_y();
  float get_gyro_z();
  uint8_t get_gyro_accuracy();
  float get_mag_x();
  float get_max_y();
  float get_mag_z();
  uint8_t get_mag_accuracy();
  uint16_t get_step_count();
  uint8_t get_stability_classifier();
  uint8_t get_activity_classifier();
  uint32_t get_time_stamp();
  int16_t get_Q1(uint16_t record_id);
  int16_t get_Q2(uint16_t record_id);
  int16_t get_Q3(uint16_t record_id);
  float get_resolution(uint16_t record_id);
  float get_range(uint16_t record_id);

  float q_to_float(int16_t fixed_point_value, uint8_t q_point);

  uint32_t read_FSR_word(uint16_t record_id, uint8_t word_number);
  void FSR_read_request(uint16_t record_id, uint16_t read_offset,
                        uint16_t block_size);
  int read_FSR_data(uint16_t record_id, uint8_t start_location,
                    uint8_t words_to_read);

  void send_command(uint8_t command);
  void send_calibrate_command(uint8_t thing_to_calibrate);
  void request_calibration_status();
  float get_quat_I();
  float get_quat_J();
  float get_quat_K();
  float get_quat_Real();

  uint16_t timeout = 10;

  void quaternion_update();

 public:
  Bno() = default;

  Bno(SPI_HandleTypeDef *_hspi, GPIO_TypeDef *_cs_port, uint16_t _cs_pin,
      GPIO_TypeDef *_int_port, uint16_t _int_pin, GPIO_TypeDef *_rst_port,
      uint16_t _rst_pin);

  ~Bno() = default;

  bool init();

  SPI_HandleTypeDef *get_spi_instance();

  uint8_t data_available();

  void update();

  ImuData get_data() { return bno_data; }

  void enable_rotation_vector(uint16_t micros_between_reports);

  void enable_game_rotation_vector(uint16_t time_between_reports);

  void enable_magnetometer(uint16_t time_between_reports);

  void enable_gyro(uint16_t time_between_reports);

  void enable_accelerometer(uint16_t time_between_reports);

  void enable_linear_accelerometer(uint16_t time_between_reports);

  void enable_step_counter(uint16_t time_between_reports);

  void enable_stability_classifier(uint16_t time_between_reports);

  void calibrate_magnetometer();

  int calibration_complete();

  void save_calibratoin();

  void calibrate_gyro();

  void calibrate_all();

  void calibrate_accelerometer();

  void calibrate_planar_acelerometer();

  void end_calibration();

  void handle_interrupt_callback(); 
};

#endif
