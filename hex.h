#ifndef HEX_H
#define HEX_H 1

typedef int (*record_00_handler_t)(uint32_t addr, uint8_t *data, uint8_t len, void *ctx);
typedef int (*record_04_handler_t)(uint16_t addr, void *ctx);

int read_hexfile(const char *filename, record_00_handler_t cb_00, record_04_handler_t cb_04, void *ctx);

#endif

