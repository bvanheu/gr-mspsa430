#ifndef MSPSA430_LLD
#define MSPSA430_LLD

#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif
    struct mspsa430_lld {
        char *device;
        int speed;
        int fd;
    };

    typedef struct mspsa430_lld mspsa430_lld_t;

    int8_t mspsa430_lld_open(mspsa430_lld_t *m, const char *device, int speed);
    void mspsa430_lld_close(mspsa430_lld_t *m);
    ssize_t mspsa430_lld_read(mspsa430_lld_t *m, uint8_t *buffer, size_t buflen);
    ssize_t mspsa430_lld_write(mspsa430_lld_t *m, uint8_t *buffer, size_t buflen);
#ifdef __cplusplus
}
#endif

#endif
