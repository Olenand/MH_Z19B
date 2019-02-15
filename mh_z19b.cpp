#include <Arduino.h>
#include <HardwareSerial.h>  // Only for debugging purposes

#include "mh_z19b.h"


const uint8_t SensorMH_Z19B::command_read_ppm[]             = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
const uint8_t SensorMH_Z19B::command_calibrate_zero_point[] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
const uint8_t SensorMH_Z19B::command_set_abc[]              = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
const uint8_t SensorMH_Z19B::command_unset_abc[]            = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};


/** MH-Z19B checksum. See datasheet */
uint8_t get_checksum(uint8_t * packet) {
  uint8_t checksum = 0x00;
  for (int i = 1; i < 8; ++i) {
    checksum += packet[i];
  }
  checksum = 0xff - checksum;
  checksum += 1;
  return checksum;
}


SensorMH_Z19B::SensorMH_Z19B(uint8_t tx_pin, uint8_t rx_pin)
    : sensor_port(tx_pin, rx_pin),
      is_started(false),
      response(new uint8_t[9]),
      timeout_set_millis(0),
      cur_timeout(0)
  {}


SensorMH_Z19B::~SensorMH_Z19B() {
  delete [] response;
}


bool SensorMH_Z19B::is_ready() const {
  unsigned long cur_millis = millis();
// DEBUG
//  Serial.print("cur_millis: ");
//  Serial.print(cur_millis);
//  Serial.print("\t");
//  Serial.print("timeout_set_millis: ");
//  Serial.print(timeout_set_millis);
//  Serial.print("\t");
//  Serial.print("cur_timeout: ");
//  Serial.println(cur_timeout);
  return is_started && (cur_millis - timeout_set_millis > cur_timeout);
}


void SensorMH_Z19B::start() {
  if (!is_started) {
    sensor_port.begin(9600);
    set_timeout(START_TIMEOUT_MS);
    is_started = true;
    // Serial.println("start(): sensor started.");
  }
}


int SensorMH_Z19B::read_ppm(int & ppm) {
  // Check running status
  if (!is_ready()) {
    // Serial.println("read_ppm(): sensor is not ready!!!");
    return 1;
  }

  // Clear serial buffer
  flush_sensor_port_in();

  // Send a request to sensor
  sensor_port.write(command_read_ppm, 9);

  // Get the response
  memset(response, 0, 9);
  sensor_port.readBytes(response, 9);

// DEBUG
//  Serial.print("Response: ");
//  for (size_t i = 0; i < 9; ++i) {
//    Serial.print(" ");
//    Serial.print(response[i], HEX);
//  }
//  Serial.println("");
  
  // Set timeout directly after response
  set_timeout(READ_PPM_TIMEOUT_MS);

  // Compute the checksum
  uint8_t checksum = get_checksum(response);

  // Parse the answer
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == checksum) ) {
    // Serial.println("Wrong checksum!!! Got " + String(response[8]) + " expected " + String(checksum));
    return 2;
  } else {
    int responseHigh = static_cast<int>(response[2]);
    int responseLow = static_cast<int>(response[3]);
    ppm = (256*responseHigh) + responseLow;
    return 0;
  }
}


int SensorMH_Z19B::set_abc(bool value) {
  // Check running status
  if (!is_ready()) {
    // Serial.println("set_abc(): sensor is not ready!!!");
    return 1;
  }

  // Clear serial buffer
  flush_sensor_port_in();

  // Set/unset ABC
  if (value) {
    // Set
    sensor_port.write(command_set_abc, 9);
  }
  else {
    // Unset
    sensor_port.write(command_unset_abc, 9);
  }

  // Set timeout
  set_timeout(SET_ABC_TIMEOUT_MS);
  return 0;
}


int SensorMH_Z19B::calibrate_zero_point() {
  // Check running status
  if (!is_ready()) {
    // Serial.println("calibrate_zero_point(): sensor is not ready!!!");
    return 1;
  }

  // Clear serial buffer
  flush_sensor_port_in();
  
  // Send calibration command
  sensor_port.write(command_calibrate_zero_point, 9);

// DEBUG
//  // Read the response (never mind that it is not used)
//  memset(response, 0, 9);
//  sensor_port.readBytes(response, 9);
//
//  Serial.print("Response: ");
//  for (size_t i = 0; i < 9; ++i) {
//    Serial.print(" ");
//    Serial.print(response[i], HEX);
//  }
//  Serial.println("");
  

  // Set timeout
  set_timeout(CALIBRATE_ZERO_POINT_TIMEOUT_MS);
  return 0;
}


void SensorMH_Z19B::set_timeout(unsigned long timeout) {
  timeout_set_millis = millis();
  cur_timeout = timeout;
}


void SensorMH_Z19B::flush_sensor_port_in() {
  while (sensor_port.available()) {
    sensor_port.read();
  }
}
