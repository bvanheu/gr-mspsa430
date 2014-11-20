#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mspsa430_command.h"
#include "mspsa430_frame.h"
#include "mspsa430_calibration.h"
#include "mspsa430.h"
#include "mspsa430_lld.h"

#define FRAME_MAX_LENGTH 256

namespace gr {
    namespace mspsa430 {

        mspsa430::mspsa430(mspsa430_lld_t *lld) {
            this->lld = lld;
        }

        void mspsa430::connect(std::string device, int speed) {
            if (mspsa430_lld_open(this->lld, device.c_str(), speed)) {
                // FIXME - Error
            }
        }

        void mspsa430::disconnect() {
            mspsa430_lld_close(this->lld);
        }

        void mspsa430::crc16_add(uint16_t *crc16, uint8_t c) {
            uint16_t crc = *crc16;
            crc  = (uint8_t)(crc>>8)|(crc<<8);
            crc ^=  c;
            crc ^= (uint8_t)(crc & 0xff)>>4;
            crc ^= (crc << 8) << 4;
            crc ^= ((crc & 0xff) << 4)<< 1;
            *crc16 = crc;
        }

        uint16_t mspsa430::checksum(uint8_t *data, size_t length) {
            uint16_t crc = 0x2A;
            uint8_t i;

            for (i=0; i<length; i++) {
                this->crc16_add(&crc, data[i]);
            }

            return crc;
        }

        size_t mspsa430::pack(mspsa430_frame *f, uint8_t *data, size_t length) {
            uint16_t checksum;

            data[0] = 0x2A;
                data[1] = f->length;
                data[2] = f->command;
                memcpy(&data[3], f->data, f->length);

                checksum = this->checksum(&data[1], f->length + 2);

                data[f->length + 3] = checksum >> 8 & 0xFF;
                data[f->length + 4] = checksum & 0xFF;

                return f->length + 5;
            }

        mspsa430_frame *mspsa430::unpack(mspsa430_frame *f, uint8_t *data, size_t length) {
            if (data[0] != 0x2A) {
                std::cout <<  __PRETTY_FUNCTION__ << ": error: no start-of-frame" << std::endl;
                throw 1;
            }

            f->length = data[1];
            f->command = data[2];

            memcpy(f->data, &data[3], data[1]);

            // TODO - checksum validation

            return f;
        }

        int8_t mspsa430::send_and_confirm(mspsa430_frame *f) {
            mspsa430_frame frame;

            this->send(f);
            this->recv(&frame);

            if (f->command != frame.command
                    || frame.length != 0x0) {
                    std::cout <<  __PRETTY_FUNCTION__ << ": error: unable to confirm the message sent" << std::endl;
                    throw 1;
            }

            return 0;
        }

        ssize_t mspsa430::send(mspsa430_frame *f) {
            uint8_t data[256];
            size_t bytes;

            bytes = this->pack(f, data, 256);

            return mspsa430_lld_write(this->lld, data, bytes);
        }

        ssize_t mspsa430::recv(mspsa430_frame *f) {
            uint8_t data[512];
            ssize_t bytes;
            uint16_t total_bytes = 0;

            // Frame start-of-frame
            bytes = mspsa430_lld_read(this->lld, &data[0], 1);
            if (bytes != 1) {
                std::cout <<  __PRETTY_FUNCTION__ << ": error: no start-of-frame" << std::endl;
                throw -1;
            }
            total_bytes += bytes;

            // Frame length
            bytes = mspsa430_lld_read(this->lld, &data[1], 1);
            if (bytes != 1) {
                std::cout <<  __PRETTY_FUNCTION__ << ": error: no frame length" << std::endl;
                throw -1;
            }
            total_bytes += bytes;

            // Frame command
            bytes = mspsa430_lld_read(this->lld, &data[2], 1);
            if (bytes != 1) {
                std::cout <<  __PRETTY_FUNCTION__ << ": error: no frame command" << std::endl;
                throw -1;
            }
            total_bytes += bytes;

            bytes = mspsa430_lld_read(this->lld, &data[3], data[1]);
            while (bytes != data[1]) {
                // FIXME - easy fix to read all the data coming, completely wrong, the lld layer should handle this.
                bytes += mspsa430_lld_read(this->lld, &data[3+bytes], data[1]-bytes);
            }
            /*
            if (bytes != data[1]) {
                std::cout <<  __PRETTY_FUNCTION__ << ": error: incomplete data, recv " << bytes << " bytes should have " << (int)data[1] << " bytes" << std::endl;
                throw -1;
            }
            */
            total_bytes += bytes;

            // Frame checksum
            bytes = mspsa430_lld_read(this->lld, &data[4 + data[1]], 2);
            if (bytes != 2) {
                std::cout <<  __PRETTY_FUNCTION__ << ": error: no checksum" << std::endl;
                throw -1;
            }
            total_bytes += bytes;

            this->unpack(f, data, total_bytes);

            return total_bytes;
        }

        // High level commands

        std::string mspsa430::get_info() {
            std::string info;
            std::string identity;
            uint32_t hw_serial_number;
            uint16_t core_version;
            uint16_t spectrum_version;
            uint32_t boot_count = 0;
            uint32_t hw_id = 0;
            uint16_t prod_version = 0;

            identity = this->get_identity();
            hw_serial_number = this->get_hw_serial_number();
            core_version = this->get_core_version();
            spectrum_version = this->get_spec_version();
            boot_count = this->get_boot_count();
            hw_id = this->get_hw_id();
            prod_version = this->get_prod_version();

            info.append(identity);
            info.append("\n");

            info.append("Hardware s/n: ");
            info.append(std::to_string(hw_serial_number));
            info.append("\n");

            info.append("Hardware id: ");
            info.append(std::to_string(hw_id));
            info.append("\n");

            info.append("Core version: ");
            info.append(std::to_string(core_version));
            info.append("\n");

            info.append("Spectrum version: ");
            info.append(std::to_string(spectrum_version));
            info.append("\n");

            info.append("Production version: ");
            info.append(std::to_string(prod_version));
            info.append("\n");

            info.append("Boot count: ");
            info.append(std::to_string(boot_count));
            info.append("\n");

            return info;
        }

        std::string mspsa430::get_calibration_data_hr() {
            std::string info;

            info.append("Format version: ");
            info.append(std::to_string(this->calibration_data.format_version));
            info.append("\n");

            info.append("Calibration date: ");
            info.append((const char *)this->calibration_data.calibration_date);
            info.append("\n");

            info.append("Software version: ");
            info.append(std::to_string(this->calibration_data.software_version));
            info.append("\n");

            info.append("Production side: ");
            info.append(std::to_string(this->calibration_data.prod_side));
            info.append("\n");

            info.append("\nCalibration frequency range\n");
            info.append("---------------------------\n");
            for (uint8_t i=0; i<3; i++) {
                info.append("\t" + std::to_string(this->calibration_data.freq_range[i].start) + " Hz - " + std::to_string(this->calibration_data.freq_range[i].stop) + " Hz @ " + std::to_string(this->calibration_data.freq_range[i].samples) + " samples\n");
            }

            info.append("\nReference level gain (level @ gain)\n");
            info.append("-----------------------------------\n");
            for (uint8_t i=0; i<8; i++) {
                info.append("\t" + std::to_string(this->calibration_data.ref_gain_table[i].reference_level) + " @ " + std::to_string(this->calibration_data.ref_gain_table[i].gain) + "\n");
            }

            info.append("Hardware id: ");
            info.append(std::to_string(this->calibration_data.hardware_id));
            info.append("\n");

            info.append("USB s/n: ");
            info.append((const char *)this->calibration_data.usb_serial_number);
            info.append("\n");

            info.append("XTAL frequency: " + std::to_string(this->calibration_data.xtal_freq) + " Hz (" + std::to_string(this->calibration_data.xtal_ppm) + " ppm)\n");

            info.append("\nTemperature\n");
            info.append("-----------\n");
            info.append("\tStart: ");
            for (uint8_t i=0; i<6; i++) {
                info.append(std::to_string(this->calibration_data.dev_temp_start[i]) +  ", ");
            }
            info.append("\n");
            info.append("\tStop: ");
            for (uint8_t i=0; i<6; i++) {
                info.append(std::to_string(this->calibration_data.dev_temp_stop[i]) +  ", ");
            }
            info.append("\n");

            /*
            info.append("\nDC select and gain level per frequency range\n");
            info.append("--------------------------------------------\n");
            for (uint8_t freq_i=0; freq_i<3; freq_i++) {
                for (uint8_t gain_i=0; gain_i<8; gain_i++) {
                    info.append(std::to_string(this->calibration_data.coeff_freq_gain[freq_i][gain_i].dc_select) + " - ");
                    for (uint8_t value_i=0; value_i<8; value_i++) {
                        info.append("[" + std::to_string(this->calibration_data.coeff_freq_gain[freq_i][gain_i].values[value_i]) + "] ");
                    }
                    info.append("\n");
                }
                info.append("\n\n");
            }
            */

            return info;
        }

        void mspsa430::setup() {
            uint32_t core_version = 0;
            uint32_t spectrum_version = 0;

            // Initialize spectrum measurement
            this->init_spec_measurement();

            // Validate firmware version
            core_version = this->get_core_version();
            spectrum_version = this->get_spec_version();
            if (core_version == 0xFFFF || core_version < 0x0209) {
                throw 1;
            }
            if (spectrum_version == 0xFFFF || spectrum_version < 0x0204) {
                throw 2;
            }

            // Load calibration data
            this->load_calibration_data();
        }


#define FLASH_CALIBRATION_DATA_START 0xD400
#define FLASH_CALIBRATION_DATA_END 0xEBFF
#define PROGTYPE_CALC 62

        void mspsa430::load_calibration_data() {
            uint8_t flash[65536];
            ssize_t bytes;

            bytes = this->flash_read(FLASH_CALIBRATION_DATA_START + sizeof(struct program_header), flash, sizeof(struct calibration_data));
            if (bytes < 0)
				return;//TODO: error

            this->calibration_data.format_version = be16toh(*(uint16_t *)flash);
            memcpy(this->calibration_data.calibration_date, (const uint8_t *)&flash[2], 16);
            this->calibration_data.calibration_date[15] = 0;
            this->calibration_data.software_version = be16toh(*(uint16_t *)&flash[0x12]);
            this->calibration_data.prod_side = flash[0x14];
            for (uint8_t i=0; i<3; i++) {
                this->calibration_data.freq_range[i].start = be32toh(((uint32_t *)&flash[0x15])[i*3+0]);
                this->calibration_data.freq_range[i].stop = be32toh(((uint32_t *)&flash[0x15])[i*3+1]);
                this->calibration_data.freq_range[i].samples = be32toh(((uint32_t *)&flash[0x15])[i*3+2]);
            }
            for (uint8_t i=0; i<8; i++) {
                this->calibration_data.ref_gain_table[i].reference_level = *(int8_t *)&flash[0x39 + i * 2 + 0];
                this->calibration_data.ref_gain_table[i].gain = flash[0x40 + i * 2 + 1];
            }
            this->calibration_data.hardware_id = be32toh(*(uint32_t *)&flash[0x49]);
            memcpy(this->calibration_data.usb_serial_number, (const uint8_t *)&flash[0x4D], 16);

            // Device crystal information
            this->calibration_data.xtal_freq = be32toh(*(uint32_t *)&flash[0x5D]);
            this->calibration_data.xtal_ppm = be16toh(*(uint16_t *)&flash[0x61]);

            // Device start/stop temperature calibration
            for (uint8_t i=0; i<6; i++) {
                this->calibration_data.dev_temp_start[i] = flash[0x63 + i];
            }
            for (uint8_t i=0; i<6; i++) {
                this->calibration_data.dev_temp_stop[i] = flash[0x69 + i];
            }


            // Device calibration coefficient and dc select data per freq range and gain level
            //
            // freq range 0
            //      [uint8_t][double][double][double][double][double][double][double][double]
            //      ... 8 times
            // freq range 1
            //      [uint8_t][double][double][double][double][double][double][double][double]
            //      ... 8 times
            // freq range 2
            //      [uint8_t][double][double][double][double][double][double][double][double]
            //      ... 8 times
            for (uint8_t freq_i=0; freq_i<3; freq_i++) {
                for (uint8_t gain_i=0; gain_i<8; gain_i++) {
                    this->calibration_data.coeff_freq_gain[freq_i][gain_i].dc_select = flash[0x6F + (freq_i * 520) + (gain_i * 65)];
                    for (uint8_t value_i=0; value_i<8; value_i++) {
                        uint8_t *pointer = &flash[0x6F + (freq_i * 520) + (gain_i * 65 + 1) + value_i * 8];
                        double temp = 0;
                        uint8_t *dest = (uint8_t *)&temp;

                        dest[0] = pointer[7];
                        dest[1] = pointer[6];
                        dest[2] = pointer[5];
                        dest[3] = pointer[4];
                        dest[4] = pointer[3];
                        dest[5] = pointer[2];
                        dest[6] = pointer[1];
                        dest[7] = pointer[0];

                        // FIXME - the value sometimes doesn't make sense...
                        this->calibration_data.coeff_freq_gain[freq_i][gain_i].values[value_i] = temp;
                    }
                }
            }
        }

        //
        // General commands
        //

        std::string mspsa430::get_identity() {
            mspsa430_frame f = mspsa430_frame(CMD_GETIDN);
            std::string identity;

            this->send_and_confirm(&f);
            this->recv(&f);

            // Null terminate the string
            f.data[f.length] = 0x00;

            identity.append((const char *)f.data);

            return identity;
        }

        uint32_t mspsa430::get_hw_serial_number() {
            mspsa430_frame f = mspsa430_frame(CMD_GETHWSERNR);

            this->send_and_confirm(&f);
            this->recv(&f);

            return be32toh(*(uint32_t *)f.data);
        }

        void mspsa430::hw_reset() {
            mspsa430_frame f = mspsa430_frame();
            f.command = CMD_HWRESET;
            f.length = 0x00;

            this->send_and_confirm(&f);
        }

        void mspsa430::blink_led() {
            mspsa430_frame f = mspsa430_frame();
            f.command = CMD_BLINKLED;
            f.length = 0x00;

            this->send_and_confirm(&f);
        }

        uint16_t mspsa430::get_core_version() {
            mspsa430_frame f = mspsa430_frame(CMD_GETCOREVER);

            this->send_and_confirm(&f);
            this->recv(&f);

            return be16toh(*(uint16_t *)f.data);
        }

        void mspsa430::get_last_error() {
            mspsa430_frame f = mspsa430_frame();
            f.command = CMD_GETLASTERROR;
            f.length = 0x00;

            this->send_and_confirm(&f);
            this->recv(&f);

            for (int i=0; i<f.length; i++) {
                std::cout << std::hex << (int)f.data[i];
            }
            std::cout << std::endl;
        }

        void mspsa430::sync() {
            mspsa430_frame f = mspsa430_frame();
            f.command = CMD_SYNC;
            f.length = 0x00;

            this->send_and_confirm(&f);
        }

        //
        // Flash operation
        //

        ssize_t mspsa430::flash_read(uint16_t address, uint8_t *buffer, uint16_t length) {
            mspsa430_frame frame_command;
            mspsa430_frame frame_data;

            uint16_t block_count;
            uint16_t block_index;
            uint16_t block_length;
            uint16_t addr_current;
            uint16_t byte_index;
            ssize_t byte_read = 0;

            frame_command.command = CMD_FLASH_READ;
            frame_command.length = 0x04;

            block_count = length / 255;

            for (block_index=0; block_index <= block_count; block_index++) {
                block_length = 255;
                if (length < 256) {
                    block_length = length;
                }
                addr_current = address + (block_index * 255);
                frame_command.data[0] = addr_current >> 8;
                frame_command.data[1] = addr_current & 0xFF;
                frame_command.data[2] = block_length >> 8;
                frame_command.data[3] = block_length & 0x00FF;

                this->send_and_confirm(&frame_command);
                byte_read += this->recv(&frame_data);

                for (byte_index=0; byte_index < frame_data.length; byte_index++) {
                    buffer[(block_index * 256) + byte_index] = frame_data.data[byte_index];
                    length--;
                }
            }

            return byte_read;
        }

        //
        // Spectrum measurement
        //

        uint16_t mspsa430::get_spec_version() {
            mspsa430_frame f = mspsa430_frame(CMD_GETSPECVER);

            this->send_and_confirm(&f);
            this->recv(&f);

            return be16toh(*(uint16_t *)f.data);
        }

        void mspsa430::set_freq_start(uint32_t frequency) {
            mspsa430_frame f = mspsa430_frame();
            f.command = CMD_SETFSTART;
            f.length = 0x04;
            memcpy(f.data, (uint8_t *)&frequency, 4);

            this->send_and_confirm(&f);
        }

        void mspsa430::set_freq_stop(uint32_t frequency) {
            mspsa430_frame f = mspsa430_frame();
            f.command = CMD_SETFSTOP;
            f.length = 0x04;
            memcpy(f.data, (uint8_t *)&frequency, 4);

            this->send_and_confirm(&f);
        }

        void mspsa430::set_freq_step(uint32_t frequency) {
            mspsa430_frame f = mspsa430_frame();
            f.command = CMD_SETFSTEP;
            f.length = 0x04;
            memcpy(f.data, (uint8_t *)&frequency, 4);

            this->send_and_confirm(&f);
        }

        void mspsa430::set_frequency(uint32_t frequency) {
        }

        void mspsa430::set_freq_intermediate() {
        }

        void mspsa430::init_spec_measurement() {
            mspsa430_frame f = mspsa430_frame(CMD_INITPARAMETER);
            this->send_and_confirm(&f);
        }

        std::vector<int8_t> mspsa430::get_spectrum_no_init() {
            mspsa430_frame f = mspsa430_frame(CMD_GETSPECNOINIT);
            std::vector<int8_t> spectrum;

            this->send_and_confirm(&f);
            this->recv(&f);

            spectrum.resize(f.length);

            for (int i=0; i<f.length; i++) {
                spectrum[i] = (int8_t)f.data[i];
            }

            return spectrum;
        }

        //
        // Production
        //

        uint16_t mspsa430::get_prod_version() {
            mspsa430_frame f = mspsa430_frame(CMD_GETPRODVER);

            this->send_and_confirm(&f);
            this->recv(&f);

            return be16toh(*(uint16_t *)f.data);
        }

        void mspsa430::set_prod_fw_init() {
        }

        void mspsa430::get_temp() {
            mspsa430_frame f = mspsa430_frame(CMD_GETTEMP);

            this->send_and_confirm(&f);
            this->recv(&f);

            for (int i=0; i<f.length; i++) {
                std::cout << std::hex << (int)f.data[i];
            }
            std::cout << std::endl;
        }

        uint32_t mspsa430::get_hw_id() {
            mspsa430_frame f = mspsa430_frame(CMD_GETHWID);

            this->send_and_confirm(&f);
            this->recv(&f);

            return be32toh(*(uint32_t *)f.data);
        }

        void mspsa430::set_hw_id() {
        }

        uint32_t mspsa430::get_boot_count() {
            mspsa430_frame f = mspsa430_frame(CMD_GETBOOTCNT);

            this->send_and_confirm(&f);
            this->recv(&f);

            return be32toh(*(uint32_t *)f.data);
        }

        void f(int8_t i) {
            std::cout << ' ' << (int)i;
        }
    }
}

    /*
    int main(int argc, char *argv[]) {
        std::string information;
        std::string calibration_data;
        std::vector<int8_t> spectrum;
        ssize_t bytes;

        mspsa430_lld_t lld;
        gr::mspsa430::mspsa430 *m = new gr::mspsa430::mspsa430(&lld);

		std::cout << "connect " << std::endl;
        m->connect("/dev/ttyACM0", 921600);

		std::cout << "get_info " << std::endl;
        information = m->get_info();
        std::cout << information << std::endl;

		std::cout << "setup " << std::endl;
        m->setup();

		std::cout << "get_spectrum_no_init " << std::endl;
        spectrum = m->get_spectrum_no_init();
        for (int i=0; i<spectrum.size(); i++) {
            std::cout << (int)spectrum.at(i) << ", ";
        }
        std::cout << std::endl;

 		std::cout << "disconnect " << std::endl;
		m->disconnect();

        return 0;
    }
    */
