#ifndef _MH_Z19B_H
#define _MH_Z19B_H

#include <SoftwareSerial.h>


// Ignore all incoming requests during these timeouts
// MH-Z19B is observed to send some crap via UART during first 20 seconds of operation
// So one should wait at least for 25 seconds and clear buffer after that (any IO operation
// should call flush_sensor_port_in() internally)
#define START_TIMEOUT_MS 100000  // Accordint to datasheet, it should be 3 min.
#define READ_PPM_TIMEOUT_MS 10000
#define SET_ABC_TIMEOUT_MS 10000
#define CALIBRATE_ZERO_POINT_TIMEOUT_MS 30000


class SensorMH_Z19B {
  public:
    SensorMH_Z19B(uint8_t tx_pin, uint8_t rx_pin);

    ~SensorMH_Z19B();

    void start();

    /** Returns true is the timeout has expired and the functions read_ppm(), etc. can be called */
    bool is_ready() const;

    /** Reads current ppm to ppm argument. Returs 0 if ppm was read without errors */
    int read_ppm(int & ppm);

    /** Set/unset Automatic Baseline Correction */
    int set_abc(bool value);

    int calibrate_zero_point();

  private:
    void set_timeout(unsigned long timeout);

    // Read everything from the serial port
    void flush_sensor_port_in();

    // Sensor state and timeout
    bool is_started;
    unsigned long timeout_set_millis;
    unsigned long cur_timeout;

    SoftwareSerial sensor_port;

    // Array with commands
    const static uint8_t command_read_ppm[];
    const static uint8_t command_calibrate_zero_point[];
    const static uint8_t command_set_abc[];
    const static uint8_t command_unset_abc[];
    // A buffer for the response
    uint8_t * response;
};


#endif // _MH_Z19B_H

