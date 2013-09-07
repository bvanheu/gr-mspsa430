#ifndef MSPSA430_H
#define MSPSA430_H

#include <sys/types.h>

#include "mspsa430_frame.h"
extern "C" {
    #include "mspsa430_lld.h"
}

class mspsa430 {
    public:
        mspsa430(mspsa430_lld_t *lld);
        ~mspsa430();

        void connect(std::string device, int speed);
        void disconnect();

        /*
        // General
        void get_idn;
        */
        void get_hw_serial_number();
        void blink_led();
        /*
        void get_core_version();
        void get_last_error();
        void sync();
        // Spectrum measurement
        void get_spec_version();
        void set_freq_start();
        void set_freq_stop();
        void set_freq_step();
        void set_freq_bandwidth();
        void set_dac();
        void set_gain();
        void set_freq_intermediate();
        void set_init_param();
        void get_spectrum_no_init();
        // Production
        void get_prod_version();
        void set_prod_fw_init();
        void get_temp();
        void get_hw_id();
        void set_hw_id();
        void get_boot_count();
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
        int8_t send_and_confirm(mspsa430_frame *f);
        ssize_t send(mspsa430_frame *f);
        ssize_t recv(mspsa430_frame *f);


        mspsa430_lld_t *lld;
};

#endif
