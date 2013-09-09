#include <iostream>
#include <sstream>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mspsa430_command.h"
#include "mspsa430_frame.h"
#include "mspsa430_calibration.h"

extern "C" {
    #include "mspsa430_lld.h"
}

#include "mspsa430.h"

#define FRAME_MAX_LENGTH 256

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
    uint16_t total_bytes;

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

void mspsa430::get_info(std::string *info) {
    std::string identification;
    std::string hw_serial_number;
    std::string core_version;
    std::string spec_version;

    this->get_idn(&identification);
    this->get_hw_serial_number(&hw_serial_number);
    this->get_core_version(&core_version);
    this->get_spec_version(&spec_version);

    info->append(identification);
    info->append("\n");

    info->append("Hardware s/n: 0x");
    info->append(hw_serial_number);
    info->append("\n");

    info->append("Core version: 0x");
    info->append(core_version);
    info->append("\n");

    info->append("Spectrum version: 0x");
    info->append(spec_version);
    info->append("\n");
}

//
// General commands
//

void mspsa430::get_idn(std::string *idn) {
    mspsa430_frame f = mspsa430_frame();
    f.command = CMD_GETIDN;
    f.length = 0x00;

    this->send_and_confirm(&f);
    this->recv(&f);

    idn->append((const char *)f.data);
}

void mspsa430::get_hw_serial_number(std::string *hw_serial_number) {
    std::stringstream ss;

    mspsa430_frame f = mspsa430_frame();
    f.command = CMD_GETHWSERNR;
    f.length = 0x00;

    this->send_and_confirm(&f);
    this->recv(&f);

    for (int i=0; i<f.length; i++) {
        ss << std::hex << (int)f.data[i];
    }

    hw_serial_number->append(ss.str());
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

void mspsa430::get_core_version(std::string *core_version) {
    std::stringstream ss;

    mspsa430_frame f = mspsa430_frame();
    f.command = CMD_GETCOREVER;
    f.length = 0x00;

    this->send_and_confirm(&f);
    this->recv(&f);

    for (int i=0; i<f.length; i++) {
        ss << std::hex << (int)f.data[i];
    }

    core_version->append(ss.str());
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

void mspsa430::flash_read(uint16_t address, uint8_t *buffer, uint16_t length) {
    mspsa430_frame frame_command;
    mspsa430_frame frame_data;

    uint16_t block_count;
    uint16_t block_index;
    uint16_t block_length;
    uint16_t addr_current;
    uint16_t byte_index;

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
        this->recv(&frame_data);

        for (byte_index=0; byte_index < frame_data.length; byte_index++) {
            buffer[(block_index * 256) + byte_index] = frame_data.data[byte_index];
            length--;
        }
    }
}

//
// Spectrum measurement
//

void mspsa430::get_spec_version(std::string *spectrum_version) {
    std::stringstream ss;

    mspsa430_frame f = mspsa430_frame();
    f.command = CMD_GETSPECVER;
    f.length = 0x00;

    this->send_and_confirm(&f);
    this->recv(&f);

    for (int i=0; i<f.length; i++) {
        ss << std::hex << (int)f.data[i];
    }

    spectrum_version->append(ss.str());
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

void mspsa430::get_spectrum_no_init() {
    mspsa430_frame f = mspsa430_frame();
    f.command = CMD_GETSPECNOINIT;
    f.length = 0x00;

    this->send_and_confirm(&f);
    this->recv(&f);

    std::cout << "Length: " << (int)f.length << std::endl;

    for (int i=0; i<f.length; i++) {
        std::cout << std::hex << (int)f.data[i];
    }
    std::cout << std::endl;
}

#define FLASH_CALIBRATION_DATA_START 0xD400
#define FLASH_CALIBRATION_DATA_END 0xEBFF
#define PROGTYPE_CALC 62

struct program_header {
    uint16_t mem_start_addr;
    uint16_t mem_length;
    uint16_t mem_type;
    uint16_t type_division;
    uint16_t crc16;
};

int main(int argc, char *argv[]) {
    std::string information;
    uint8_t flash[65536];
    struct program_header ph;
    struct calibration_data cd;

    mspsa430_lld_t lld;
    mspsa430 *m = new mspsa430(&lld);

    m->connect("/dev/ttyACM0", 921600);

    m->get_info(&information);
    std::cout << information;
    std::cout << "------------------------------------" << std::endl;

    m->flash_read(0xD400, (uint8_t *)&ph, sizeof(struct program_header));

    std::cout << "Memory start address: 0x" << std::hex << ph.mem_start_addr << std::endl;
    std::cout << "Memory length: 0x" << std::hex << ph.mem_length << std::endl;
    std::cout << "Memory type: 0x" << std::hex << ph.mem_type << std::endl;
    std::cout << "Type division: 0x" << std::hex << ph.type_division << std::endl;
    std::cout << "CRC16: 0x" << std::hex << ph.crc16 << std::endl;
    std::cout << "------------------------------------" << std::endl;

    m->flash_read(0xD400 + sizeof(struct program_header), (uint8_t *)&cd, sizeof(struct calibration_data));

    std::cout << "Format version: 0x" << std::hex << cd.format_version << std::endl;
    std::cout << "Software version: 0x" << std::hex << cd.software_version << std::endl;
    std::cout << "Prod side: 0x" << std::hex << cd.prod_side << std::endl;

    std::cout << "Calibration frequency range:" << std::endl;
    for (uint8_t i=0; i<3; i++) {
        std::cout << "\tStart: " << std::dec << cd.freq_range[i].start <<  std::endl;
        std::cout << "\tStop: " << cd.freq_range[i].stop <<  std::endl;
        std::cout << "\tSamples: " << cd.freq_range[i].samples << std::endl;
    }

    std::cout << "Calibration date: " << cd.calibration_date << std::endl;
    std::cout << "USB s/n: " << cd.usb_serial_number << std::endl;

    m->disconnect();

    return 0;
}
