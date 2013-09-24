#ifndef MSPSA430_H
#define MSPSA430_H

#include <vector>

#include <sys/types.h>

#include "mspsa430_frame.h"
#include "mspsa430_calibration.h"


extern "C" {
    #include "mspsa430_lld.h"
}

namespace gr {
    namespace mspsa430 {
        class mspsa430 {
            public:
                mspsa430(mspsa430_lld_t *lld);
                ~mspsa430();

                void connect(std::string device, int speed);
                void disconnect();

                // High-level
                std::string get_info();
                std::string get_calibration_data_hr();

                void setup();
                void load_calibration_data();

                // General
                std::string get_identity();
                uint32_t get_hw_serial_number();
                void hw_reset();
                void blink_led();
                uint16_t get_core_version();
                void get_last_error();
                void sync();
                // Flash operation
                ssize_t flash_read(uint16_t address, uint8_t *buffer, uint16_t length);
                // Spectrum measurement
                uint16_t get_spec_version();
                void set_freq_start(uint32_t frequency);
                void set_freq_stop(uint32_t frequency);
                void set_freq_step(uint32_t frequency);
                void set_frequency(uint32_t frequency);
                void set_rx_filter_bandwidth();
                void set_dac();
                void set_gain();
                void set_freq_intermediate();
                void init_spec_measurement();
                std::vector<int8_t> get_spectrum_no_init();
                // Production
                uint16_t get_prod_version();
                void set_prod_fw_init();
                void get_temp();
                uint32_t get_hw_id();
                void set_hw_id();
                uint32_t get_boot_count();
                /*
                void set_freq_out();
                void set_freq_xtal();
                void get_freq_xtal();
                void get_sweep_edc();
                void get_chip_tlv();
                */

            protected:
                void crc16_add(uint16_t *crc16, uint8_t c);
                uint16_t checksum(uint8_t *buffer, size_t length);

                size_t pack(mspsa430_frame *f, uint8_t *data, size_t length);
                mspsa430_frame *unpack(mspsa430_frame *f, uint8_t *data, size_t length);

                ssize_t send(mspsa430_frame *f);
                ssize_t recv(mspsa430_frame *f);
                int8_t send_and_confirm(mspsa430_frame *f);

                mspsa430_lld_t *lld;

                struct calibration_data calibration_data;
        };
    }
}

#endif
