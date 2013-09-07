#ifndef MSPSA430_FRAME_H
#define MSPSA430_FRAME_H

class mspsa430_frame {
    public:
        uint8_t command;
        uint8_t length;
        uint8_t data[256];
};

#endif
