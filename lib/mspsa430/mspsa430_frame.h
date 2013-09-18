#ifndef MSPSA430_FRAME_H
#define MSPSA430_FRAME_H

#include <stdint.h>
#include <string.h>

class mspsa430_frame {
    public:
        uint8_t command;
        uint8_t length;
        uint8_t data[256];

        mspsa430_frame() {
        };

        mspsa430_frame(uint8_t command) {
            this->command = command;
            this->length = 0;
        };

        mspsa430_frame(uint8_t command, uint8_t *data, uint8_t length) {
            this->command = command;
            memcpy(this->data, data, length);
            this->length = length;
        };
};

#endif
