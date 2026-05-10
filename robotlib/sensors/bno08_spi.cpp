#include <math.h>

#include <cstdint>

#include "../maths/math.hpp"
#include "bno08_spi.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"

Bno::Bno(SPI_HandleTypeDef *_spih, GPIO_TypeDef *_CS_PORT, uint16_t _CS_PIN,
         GPIO_TypeDef *_INT_PORT, uint16_t _INT_PIN, GPIO_TypeDef *_RST_PORT,
         uint16_t _RST_PIN) {
  hspi = _spih;
  CS_PORT = _CS_PORT;
  CS_PIN = _CS_PIN;
  INT_PORT = _INT_PORT;
  INT_PIN = _INT_PIN;
  RST_PORT = _RST_PORT;
  RST_PIN = _RST_PIN;
}

bool Bno::init() {
  HAL_SPI_Init(hspi);
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);
  wait_for_rst_low();

  wait_for_rst_low();
  receive_packet();

  wait_for_rst_low();
  receive_packet();

  shtp_data[0] = SHTP_REPORT_PRODUCT_ID_REQUEST;
  shtp_data[1] = 0;

  send_packet(CHANNEL_CONTROL, 2);

  wait_for_rst_low();
  receive_packet();

  return true;
}

uint8_t Bno::wait_for_rst_low() {
  for (uint32_t counter = 0; counter < 0xFFFFF; counter++) {
    if (HAL_GPIO_ReadPin(INT_PORT, INT_PIN) == 0) {
      return 1;
    }
  }
  return 0;
}

uint8_t Bno::receive_packet() {
  uint8_t incoming;

  if (HAL_GPIO_ReadPin(INT_PORT, INT_PIN) == 1) {
    return 0;
  }

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

  uint8_t to_transmit[4] = {0, 0, 0, 0};
  HAL_SPI_TransmitReceive(hspi, to_transmit, shtp_header, 4, timeout);

  uint16_t data_size = ((uint16_t)shtp_header[1] << 8 | shtp_header[0]);
  data_size &= 0x7FFF;

  if (data_size == 0) {
    return 0;
  }
  data_size -= 4;

  for (uint16_t i = 0; i < data_size; i++) {
    incoming = send_byte(0xFF);
    if (i < MAX_PACKET_SIZE) {
      shtp_data[i] = incoming;
    }
  }

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
  return 1;
}

uint8_t Bno::send_byte(uint8_t data) {
  uint8_t rx_data;

  HAL_SPI_TransmitReceive(hspi, &data, &rx_data, 1, timeout);

  return rx_data;
}

uint8_t Bno::send_packet(uint8_t channel_number, uint8_t data_size) {
  uint8_t packet_size = data_size + 4;

  if (wait_for_rst_low() == 0) return 0;

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

  uint8_t to_transmit[4];
  to_transmit[0] = packet_size;
  to_transmit[1] = 0;
  to_transmit[2] = channel_number;
  to_transmit[3] = sequence_number[channel_number]++;

  HAL_SPI_Transmit(hspi, to_transmit, 4, timeout);
  HAL_SPI_Transmit(hspi, shtp_data, data_size, timeout);

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

  return 1;
}

void Bno::enable_rotation_vector(uint16_t micros_between_reports) {
  set_feature_command(SENSOR_REPORTID_ROTATION_VECTOR, micros_between_reports,
                      0);
}

void Bno::set_feature_command(uint8_t report_id,
                              uint32_t micros_between_reports,
                              uint32_t specific_config) {
  shtp_data[0] = SHTP_REPORT_SET_FEATURE_COMMAND;  // Set feature command.
                                                   // Reference page 55
  shtp_data[1] = report_id;  // Feature Report ID. 0x01 = Accelerometer, 0x05 =
                             // Rotation vector
  shtp_data[2] = 0;          // Feature flags
  shtp_data[3] = 0;          // Change sensitivity (LSB)
  shtp_data[4] = 0;          // Change sensitivity (MSB)
  shtp_data[5] =
      (micros_between_reports >> 0) &
      0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
  shtp_data[6] = (micros_between_reports >> 8) & 0xFF;   // Report interval
  shtp_data[7] = (micros_between_reports >> 16) & 0xFF;  // Report interval
  shtp_data[8] =
      (micros_between_reports >> 24) & 0xFF;  // Report interval (MSB)
  shtp_data[9] = 0;                           // Batch Interval (LSB)
  shtp_data[10] = 0;                          // Batch Interval
  shtp_data[11] = 0;                          // Batch Interval
  shtp_data[12] = 0;                          // Batch Interval (MSB)
  shtp_data[13] =
      (specific_config >> 0) & 0xFF;  // Sensor-specific config (LSB)
  shtp_data[14] = (specific_config >> 8) & 0xFF;   // Sensor-specific config
  shtp_data[15] = (specific_config >> 16) & 0xFF;  // Sensor-specific config
  shtp_data[16] =
      (specific_config >> 24) & 0xFF;  // Sensor-specific config (MSB)

  // Transmit packet on channel 2, 17 bytes
  send_packet(CHANNEL_CONTROL, 17);
}

uint8_t Bno::data_available() {
  if (HAL_GPIO_ReadPin(INT_PORT, INT_PIN) == 1) return 0;

  if (receive_packet() == 1) {
    if (shtp_header[2] == CHANNEL_REPORTS &&
        shtp_data[0] == SHTP_REPORT_BASE_TIMESTAMP) {
      parse_input_report();
      return 1;
    } else if (shtp_header[2] == CHANNEL_CONTROL) {
      parse_command_report();
      return 1;
    }
  }
  return 0;
}

void Bno::parse_input_report() {
  uint16_t dataLength = ((uint16_t)shtp_header[1] << 8 | shtp_header[0]);
  dataLength &= 0x7FFF;
  dataLength -= 4;

  time_stamp = ((uint32_t)shtp_data[4] << (8 * 3)) | (shtp_data[3] << (8 * 2)) |
               (shtp_data[2] << (8 * 1)) | (shtp_data[1] << (8 * 0));

  uint8_t status = shtp_data[7] & 0x03;  // Get status bits
  uint16_t data1 = (uint16_t)shtp_data[10] << 8 | shtp_data[9];
  uint16_t data2 = (uint16_t)shtp_data[12] << 8 | shtp_data[11];
  uint16_t data3 = (uint16_t)shtp_data[14] << 8 | shtp_data[13];
  uint16_t data4 = 0;
  uint16_t data5 = 0;

  if (dataLength > 14) {
    data4 = (uint16_t)shtp_data[16] << 8 | shtp_data[15];
  }
  if (dataLength > 16) {
    data5 = (uint16_t)shtp_data[18] << 8 | shtp_data[17];
  }

  // Store these generic values to their proper global variable
  switch (shtp_data[5]) {
    case SENSOR_REPORTID_ACCELEROMETER: {
      accel_accuracy = status;
      raw_accel_x = data1;
      raw_accel_y = data2;
      raw_accel_z = data3;
      break;
    }
    case SENSOR_REPORTID_LINEAR_ACCELERATION: {
      accel_accuracy = status;
      raw_linear_accel_x = data1;
      raw_linear_accel_y = data2;
      raw_linear_accel_z = data3;
      break;
    }
    case SENSOR_REPORTID_GYROSCOPE: {
      gyro_accuracy = status;
      raw_gyro_x = data1;
      raw_gyro_y = data2;
      raw_gyro_z = data3;
      break;
    }
    case SENSOR_REPORTID_MAGNETIC_FIELD: {
      mag_accuracy = status;
      raw_mag_x = data1;
      raw_mag_y = data2;
      raw_mag_z = data3;
      break;
    }
    case SENSOR_REPORTID_ROTATION_VECTOR:
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {
      quatAccuracy = status;
      raw_quat_I = data1;
      raw_quat_J = data2;
      raw_quat_K = data3;
      raw_quat_Real = data4;
      raw_quat_radian_accuracy =
          data5;  // Only available on rotation vector, not game rot vector
      break;
    }
    case SENSOR_REPORTID_STEP_COUNTER: {
      step_count = data3;  // Bytes 8/9
      break;
    }
    case SENSOR_REPORTID_STABILITY_CLASSIFIER: {
      stability_classifier = shtp_data[5 + 4];  // Byte 4 only
      break;
    }
    case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER: {
      activity_clasifier = shtp_data[5 + 5];  // Most likely state

      // Load activity classification confidences into the array
      for (uint8_t x = 0; x < 9;
           x++)  // Hardcoded to max of 9. TODO - bring in array size
        _activity_confidences[x] =
            shtp_data[11 + x];  // 5 bytes of timestamp, byte 6 is first
                                // confidence byte
      break;
    }
    case SHTP_REPORT_COMMAND_RESPONSE: {
      // printf("!");
      // The BNO080 responds with this report to command requests. It's up to
      // use to remember which command we issued.
      uint8_t command =
          shtp_data[5 + 2];  // This is the Command byte of the response

      if (command == COMMAND_ME_CALIBRATE) {
        // printf("ME Cal report found!");
        calibration_status =
            shtp_data[5 + 5];  // R0 - Status (0 = success, non-zero = fail)
      }
      break;
    }
    default: {
      // This sensor report ID is unhandled.
      // See reference manual to add additional feature reports as needed
    }
  }
}

void Bno::parse_command_report(void) {
  if (shtp_data[0] == SHTP_REPORT_COMMAND_RESPONSE) {
    // The BNO080 responds with this report to command requests. It's up to
    // use to remember which command we issued.
    uint8_t command = shtp_data[2];  // This is the Command byte of the response

    if (command == COMMAND_ME_CALIBRATE) {
      calibration_status =
          shtp_data[5];  // R0 - Status (0 = success, non-zero = fail)
    }
  } else {
    // This sensor report ID is unhandled.
    // See reference manual to add additional feature reports as needed
  }

  // TODO additional feature reports may be strung together. Parse them all.
}

// Given a register value and a Q point, convert to float
// See https://en.wikipedia.org/wiki/Q_(number_format)
float Bno::q_to_float(int16_t fixedPointValue, uint8_t qPoint) {
  return fixedPointValue * powf(2.0, qPoint * -1);
}

// Return the rotation vector quaternion I
// float Bno::get_quat_I() { return q_to_float(raw_quat_I, rotation_vector_Q1);
// }

// // Return the rotation vector quaternion J
// float Bno::get_quat_J() { return q_to_float(raw_quat_J, rotation_vector_Q1);
// }

// // Return the rotation vector quaternion K
// float Bno::get_quat_K() { return q_to_float(raw_quat_K, rotation_vector_Q1);
// }

// // Return the rotation vector quaternion Real
// float Bno::get_quat_Real() {
//   return q_to_float(raw_quat_Real, rotation_vector_Q1);
// }

float Bno::get_quat_I() { return (int16_t)raw_quat_I / 16384.0f; }
float Bno::get_quat_J() { return (int16_t)raw_quat_J / 16384.0f; }
float Bno::get_quat_K() { return (int16_t)raw_quat_K / 16384.0f; }
float Bno::get_quat_Real() { return (int16_t)raw_quat_Real / 16384.0f; }

// Return the rotation vector accuracy
float Bno::get_quat_radian_accuracy() {
  return q_to_float(raw_quat_radian_accuracy, rotation_vector_Q1);
}

// Return the acceleration component
uint8_t Bno::get_quat_accuracy() { return (quatAccuracy); }

// Return the acceleration component
float Bno::get_accel_x() { return q_to_float(raw_accel_x, accelerometer_Q1); }

// Return the acceleration component
float Bno::get_accel_y() { return q_to_float(raw_accel_y, accelerometer_Q1); }

// Return the acceleration component
float Bno::get_accel_z() { return q_to_float(raw_accel_z, accelerometer_Q1); }

// Return the acceleration component
uint8_t Bno::get_accel_accuracy() { return (accel_accuracy); }

// linear acceleration, i.e. minus gravity

// Return the acceleration component
float Bno::get_linear_accel_x() {
  return q_to_float(raw_linear_accel_x, linear_accelerometer_Q1);
}

// Return the acceleration component
float Bno::get_linear_accel_y() {
  return q_to_float(raw_linear_accel_y, linear_accelerometer_Q1);
}

// Return the acceleration component
float Bno::get_linear_accel_z() {
  return q_to_float(raw_linear_accel_z, linear_accelerometer_Q1);
}

// Return the acceleration component
uint8_t Bno::get_linear_accel_accuracy() { return (linear_accel_accuracy); }

// Return the gyro component
float Bno::get_gyro_x() { return q_to_float(raw_gyro_x, gyro_Q1); }

// Return the gyro component
float Bno::get_gyro_y() { return q_to_float(raw_gyro_y, gyro_Q1); }

// Return the gyro component
float Bno::get_gyro_z() { return q_to_float(raw_gyro_z, gyro_Q1); }

// Return the gyro component
uint8_t Bno::get_gyro_accuracy() { return (gyro_accuracy); }

// Return the magnetometer component
float Bno::get_mag_x() { return q_to_float(raw_mag_x, magnetometer_Q1); }

// Return the magnetometer component
float Bno::get_max_y() { return q_to_float(raw_mag_y, magnetometer_Q1); }

// Return the magnetometer component
float Bno::get_mag_z() { return q_to_float(raw_mag_z, magnetometer_Q1); }

// Return the mag component
uint8_t Bno::get_mag_accuracy() { return (mag_accuracy); }

// Return the step count
uint16_t Bno::get_step_count() { return (step_count); }

// Return the stability classifier
uint8_t Bno::get_stability_classifier() { return (stability_classifier); }

// Return the activity classifier
uint8_t Bno::get_activity_classifier() { return (activity_clasifier); }

// Return the time stamp
uint32_t Bno::get_time_stamp() { return (time_stamp); }

// Given a record ID, read the Q1 value from the metaData record in the FRS (ya,
// it's complicated) Q1 is used for all sensor data calculations
int16_t Bno::get_Q1(uint16_t recordID) {
  // Q1 is always the lower 16 bits of word 7
  return read_FSR_word(recordID, 7) & 0xFFFF;  // Get word 7, lower 16 bits
}

// Given a record ID, read the Q2 value from the metaData record in the FRS
// Q2 is used in sensor bias
int16_t Bno::get_Q2(uint16_t recordID) {
  // Q2 is always the upper 16 bits of word 7
  return read_FSR_word(recordID, 7) >> 16;  // Get word 7, upper 16 bits
}

// Given a record ID, read the Q3 value from the metaData record in the FRS
// Q3 is used in sensor change sensitivity
int16_t Bno::get_Q3(uint16_t recordID) {
  // Q3 is always the upper 16 bits of word 8
  return read_FSR_word(recordID, 8) >> 16;  // Get word 8, upper 16 bits
}

// Given a record ID, read the resolution value from the metaData record in the
// FRS for a given sensor
float Bno::get_resolution(uint16_t recordID) {
  // The resolution Q value are 'the same as those used in the sensor's input
  // report' This should be Q1.
  int16_t Q = get_Q1(recordID);

  // Resolution is always word 2
  uint32_t value = read_FSR_word(recordID, 2);  // Get word 2

  return q_to_float(value, Q);
}

// Given a record ID, read the range value from the metaData record in the FRS
// for a given sensor
float Bno::get_range(uint16_t recordID) {
  // The resolution Q value are 'the same as those used in the sensor's input
  // report' This should be Q1.
  int16_t Q = get_Q1(recordID);

  // Range is always word 1
  uint32_t value = read_FSR_word(recordID, 1);  // Get word 1

  return q_to_float(value, Q);
}

// Given a record ID and a word number, look up the word data
// Helpful for pulling out a Q value, range, etc.
// Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data
// for example)
uint32_t Bno::read_FSR_word(uint16_t recordID, uint8_t wordNumber) {
  if (read_FSR_data(recordID, wordNumber, 1) ==
      1)                    // Get word number, just one word in length from FRS
    return (meta_data[0]);  // Return this one word

  return (0);  // Error
}

// Ask the sensor for data from the Flash Record System
// See 6.3.6 page 40, FRS Read Request
void Bno::FSR_read_request(uint16_t recordID, uint16_t readOffset,
                           uint16_t blockSize) {
  shtp_data[0] = SHTP_REPORT_FRS_READ_REQUEST;  // FRS Read Request
  shtp_data[1] = 0;                             // Reserved
  shtp_data[2] = (readOffset >> 0) & 0xFF;      // Read Offset LSB
  shtp_data[3] = (readOffset >> 8) & 0xFF;      // Read Offset MSB
  shtp_data[4] = (recordID >> 0) & 0xFF;        // FRS Type LSB
  shtp_data[5] = (recordID >> 8) & 0xFF;        // FRS Type MSB
  shtp_data[6] = (blockSize >> 0) & 0xFF;       // Block size LSB
  shtp_data[7] = (blockSize >> 8) & 0xFF;       // Block size MSB

  // Transmit packet on channel 2, 8 bytes
  send_packet(CHANNEL_CONTROL, 8);
}

// Given a sensor or record ID, and a given start/stop bytes, read the data from
// the Flash Record System (FRS) for this sensor Returns true if metaData array
// is loaded successfully Returns false if failure
int Bno::read_FSR_data(uint16_t recordID, uint8_t startLocation,
                       uint8_t wordsToRead) {
  uint8_t spot = 0;

  // First we send a Flash Record System (FRS) request
  FSR_read_request(
      recordID, startLocation,
      wordsToRead);  // From startLocation of record, read a # of words

  // Read bytes until FRS reports that the read is complete
  while (1) {
    // Now we wait for response
    while (1) {
      uint8_t counter = 0;
      while (receive_packet() == 0) {
        if (counter++ > 100) return (0);  // Give up
        HAL_Delay(1);
      }

      // We have the packet, inspect it for the right contents
      // See page 40. Report ID should be 0xF3 and the FRS types should match
      // the thing we requested
      if (shtp_data[0] == SHTP_REPORT_FRS_READ_RESPONSE)
        if (((uint16_t)shtp_data[13] << 8 | shtp_data[12]) == recordID)
          break;  // This packet is one we are looking for
    }

    uint8_t dataLength = shtp_data[1] >> 4;
    uint8_t frsStatus = shtp_data[1] & 0x0F;

    uint32_t data0 = (uint32_t)shtp_data[7] << 24 |
                     (uint32_t)shtp_data[6] << 16 |
                     (uint32_t)shtp_data[5] << 8 | (uint32_t)shtp_data[4];
    uint32_t data1 = (uint32_t)shtp_data[11] << 24 |
                     (uint32_t)shtp_data[10] << 16 |
                     (uint32_t)shtp_data[9] << 8 | (uint32_t)shtp_data[8];

    // Record these words to the metaData array
    if (dataLength > 0) {
      meta_data[spot++] = data0;
    }
    if (dataLength > 1) {
      meta_data[spot++] = data1;
    }

    if (spot >= MAX_METADATA_SIZE) {
      // printf("metaData array over run. Returning.");
      return (1);  // We have run out of space in our array. Bail.
    }

    if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7) {
      return (1);  // FRS status is read completed! We're done!
    }
  }
}

// Sends the packet to enable the rotation vector
void Bno::enable_game_rotation_vector(uint16_t time_between_reports) {
  set_feature_command(SENSOR_REPORTID_GAME_ROTATION_VECTOR,
                      time_between_reports, 0);
}

// Sends the packet to enable the accelerometer
void Bno::enable_accelerometer(uint16_t time_between_reports) {
  set_feature_command(SENSOR_REPORTID_ACCELEROMETER, time_between_reports, 0);
}

// Sends the packet to enable the accelerometer
void Bno::enable_linear_accelerometer(uint16_t time_between_reports) {
  set_feature_command(SENSOR_REPORTID_LINEAR_ACCELERATION, time_between_reports,
                      0);
}

// Sends the packet to enable the gyro
void Bno::enable_gyro(uint16_t time_between_reports) {
  set_feature_command(SENSOR_REPORTID_GYROSCOPE, time_between_reports, 0);
}

// Sends the packet to enable the magnetometer
void Bno::enable_magnetometer(uint16_t time_between_reports) {
  set_feature_command(SENSOR_REPORTID_MAGNETIC_FIELD, time_between_reports, 0);
}

// Sends the packet to enable the step counter
void Bno::enable_step_counter(uint16_t time_between_reports) {
  set_feature_command(SENSOR_REPORTID_STEP_COUNTER, time_between_reports, 0);
}

// Sends the packet to enable the Stability Classifier
void Bno::enable_stability_classifier(uint16_t time_between_reports) {
  set_feature_command(SENSOR_REPORTID_STABILITY_CLASSIFIER,
                      time_between_reports, 0);
}

// Sends the commands to begin calibration of the accelerometer
void Bno::Bno::calibrate_accelerometer() {
  send_calibrate_command(CALIBRATE_ACCEL);
}

// Sends the commands to begin calibration of the gyro
void Bno::calibrate_gyro() { send_calibrate_command(CALIBRATE_GYRO); }

// Sends the commands to begin calibration of the magnetometer
void Bno::calibrate_magnetometer() { send_calibrate_command(CALIBRATE_MAG); }

// Sends the commands to begin calibration of the planar accelerometer
void Bno::calibrate_planar_acelerometer() {
  send_calibrate_command(CALIBRATE_PLANAR_ACCEL);
}

// See 2.2 of the Calibration Procedure document 1000-4044
void Bno::calibrate_all() { send_calibrate_command(CALIBRATE_ACCEL_GYRO_MAG); }

void Bno::end_calibration() {
  send_calibrate_command(CALIBRATE_STOP);  // Disables all calibrations
}

// See page 51 of reference manual - ME Calibration Response
// Byte 5 is parsed during the readPacket and stored in calibrationStatus
int Bno::calibration_complete() {
  if (calibration_status == 0) return (1);
  return (0);
}

// Tell the sensor to do a command
// See 6.3.8 page 41, Command request
// The caller is expected to set P0 through P8 prior to calling
void Bno::send_command(uint8_t command) {
  shtp_data[0] = SHTP_REPORT_COMMAND_REQUEST;  // Command Request
  // Increments automatically each function call
  shtp_data[1] = command_sequence_number++;
  shtp_data[2] = command;  // Command

  // Caller must set these
  /*shtpData[3] = 0; //P0
  shtpData[4] = 0; //P1
  shtpData[5] = 0; //P2
  shtpData[6] = 0;
  shtpData[7] = 0;
  shtpData[8] = 0;
  shtpData[9] = 0;
  shtpData[10] = 0;
  shtpData[11] = 0;*/

  // Transmit packet on channel 2, 12 bytes
  send_packet(CHANNEL_CONTROL, 12);
}

// This tells the BNO080 to begin calibrating
// See page 50 of reference manual and the 1000-4044 calibration doc
void Bno::send_calibrate_command(uint8_t thing_to_calibrate) {
  /*shtpData[3] = 0; //P0 - Accel Cal Enable
  shtpData[4] = 0; //P1 - Gyro Cal Enable
  shtpData[5] = 0; //P2 - Mag Cal Enable
  shtpData[6] = 0; //P3 - Subcommand 0x00
  shtpData[7] = 0; //P4 - Planar Accel Cal Enable
  shtpData[8] = 0; //P5 - Reserved
  shtpData[9] = 0; //P6 - Reserved
  shtpData[10] = 0; //P7 - Reserved
  shtpData[11] = 0; //P8 - Reserved*/

  for (uint8_t x = 3; x < 12; x++)  // Clear this section of the shtpData array
    shtp_data[x] = 0;

  if (thing_to_calibrate == CALIBRATE_ACCEL)
    shtp_data[3] = 1;
  else if (thing_to_calibrate == CALIBRATE_GYRO)
    shtp_data[4] = 1;
  else if (thing_to_calibrate == CALIBRATE_MAG)
    shtp_data[5] = 1;
  else if (thing_to_calibrate == CALIBRATE_PLANAR_ACCEL)
    shtp_data[7] = 1;
  else if (thing_to_calibrate == CALIBRATE_ACCEL_GYRO_MAG) {
    shtp_data[3] = 1;
    shtp_data[4] = 1;
    shtp_data[5] = 1;
  } else if (thing_to_calibrate == CALIBRATE_STOP) {
  }
  // Do nothing, bytes are set to zero

  // Make the internal calStatus variable non-zero (operation failed) so that
  // user can test while we wait
  calibration_status = 1;

  // Using this shtpData packet, send a command
  send_command(COMMAND_ME_CALIBRATE);
}

// Request ME Calibration Status from BNO080
// See page 51 of reference manual
void Bno::request_calibration_status() {
  /*shtpData[3] = 0; //P0 - Reserved
  shtpData[4] = 0; //P1 - Reserved
  shtpData[5] = 0; //P2 - Reserved
  shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
  shtpData[7] = 0; //P4 - Reserved
  shtpData[8] = 0; //P5 - Reserved
  shtpData[9] = 0; //P6 - Reserved
  shtpData[10] = 0; //P7 - Reserved
  shtpData[11] = 0; //P8 - Reserved*/

  for (uint8_t x = 3; x < 12; x++)  // Clear this section of the shtpData array
    shtp_data[x] = 0;

  shtp_data[6] = 0x01;  // P3 - 0x01 - Subcommand: Get ME Calibration

  // Using this shtpData packet, send a command
  send_command(COMMAND_ME_CALIBRATE);
}

// This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
// See page 49 of reference manual and the 1000-4044 calibration doc
void Bno::save_calibratoin() {
  /*shtpData[3] = 0; //P0 - Reserved
  shtpData[4] = 0; //P1 - Reserved
  shtpData[5] = 0; //P2 - Reserved
  shtpData[6] = 0; //P3 - Reserved
  shtpData[7] = 0; //P4 - Reserved
  shtpData[8] = 0; //P5 - Reserved
  shtpData[9] = 0; //P6 - Reserved
  shtpData[10] = 0; //P7 - Reserved
  shtpData[11] = 0; //P8 - Reserved*/

  for (uint8_t x = 3; x < 12; x++)  // Clear this section of the shtpData array
    shtp_data[x] = 0;

  // Using this shtpData packet, send a command
  send_command(COMMAND_DCD);  // Save DCD command
}

void Bno::update() {
  quaternions[0] = get_quat_Real();
  quaternions[1] = get_quat_I();
  quaternions[2] = get_quat_J();
  quaternions[3] = get_quat_K();
  quaternion_update();
  bno_data.accel_x = get_accel_x();
  bno_data.accel_y = get_accel_y();
  bno_data.accel_z = get_accel_z();
}

void Bno::quaternion_update() {
  // float sqr = quaternions[0] * quaternions[0];
  // float sqi = quaternions[1] * quaternions[1];
  // float sqj = quaternions[2] * quaternions[2];
  // float sqk = quaternions[3] * quaternions[3];
  float sqr = powf(quaternions[0], 2.0f);
  float sqi = powf(quaternions[1], 2.0f);
  float sqj = powf(quaternions[2], 2.0f);
  float sqk = powf(quaternions[3], 2.0f);

  bno_data.yaw = atan2f(2.0f * (quaternions[1] * quaternions[2] +
                                quaternions[3] * quaternions[0]),
                        (sqi - sqj - sqk + sqr)) *
                 RAD2DEG;
  bno_data.pitch = asinf(-2.0f *
                         (quaternions[1] * quaternions[3] -
                          quaternions[2] * quaternions[0]) /
                         (sqi + sqj + sqk + sqr)) *
                   RAD2DEG;
  bno_data.roll = atan2f(2.0f * (quaternions[2] * quaternions[3] +
                                 quaternions[1] * quaternions[0]),
                         (-sqi - sqj + sqk + sqr)) *
                  RAD2DEG;
}

void Bno::handle_interrupt_callback() {
  if (receive_packet() == 1) {
    if (shtp_header[2] == CHANNEL_REPORTS &&
        shtp_data[0] == SHTP_REPORT_BASE_TIMESTAMP) {
      parse_input_report();
    } else if (shtp_header[2] == CHANNEL_CONTROL) {
      parse_command_report();
    }
  }
  update();
}

#pragma GCC diagnostic pop
