#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mspsa430_command.h"
#include "mspsa430_frame.h"

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
            std::cout <<  __PRETTY_FUNCTION__ << ": error" << std::endl;
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
    uint8_t data[256];
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
    if (bytes != data[1]) {
        std::cout <<  __PRETTY_FUNCTION__ << ": error: incomplete data, recv " << bytes << " should have " << f->length << " bytes" << std::endl;
        throw -1;
    }
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

void mspsa430::blink_led() {
    mspsa430_frame f = mspsa430_frame();
    f.command = CMD_BLINKLED;
    f.length = 0x00;

    this->send_and_confirm(&f);
}

void mspsa430::get_hw_serial_number() {
    mspsa430_frame f = mspsa430_frame();
    f.command = CMD_GETHWSERNR;
    f.length = 0x00;

    this->send_and_confirm(&f);
    this->recv(&f);

    for (int i=0; i<f.length; i++) {
        std::cout << std::hex << (int)f.data[i];
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[]) {

    std::cout << "--------------------" << std::endl;
    std::cout << "| MSP-SA430-SUB1GHZ" << std::endl;
    std::cout << "--------------------" << std::endl;

    mspsa430_lld_t lld;

    mspsa430 *m = new mspsa430(&lld);

    m->connect("/dev/ttyACM0", 921600);

    std::cout << "blink led...";
    m->blink_led();
    std::cout << "done" << std::endl;

    std::cout << "hw serial number...";
    m->get_hw_serial_number();
    std::cout << "done" << std::endl;

    return 0;
}
