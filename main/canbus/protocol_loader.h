#pragma once

#include <stdint.h>

#define MAX_PROTOCOLS 8
#define MAX_FRAMES    64
#define MAX_SIGNALS   16
#define CAN_ID_MAX    2048

typedef enum {
    // Byte order used when reconstructing multi-byte signal values from a CAN
    // payload.
    ENDIAN_BIG,
    ENDIAN_LITTLE
} endian_t;

typedef struct {
    // Decoded signal description from protocol JSON.  target is NULL when the
    // JSON names a signal the dash does not currently display.
    float *target;
    uint8_t offset;
    uint8_t len;
    float scale;
    float offset_val;
    endian_t endian;
} can_signal_t;

typedef struct {
    // All signals carried by one CAN identifier.
    uint32_t id;
    int signal_count;
    can_signal_t signals[MAX_SIGNALS];
} can_frame_def_t;

typedef struct {
    // One ECU protocol definition loaded from embedded JSON.
    char name[32];
    int bitrate;
    int frame_count;
    can_frame_def_t frames[MAX_FRAMES];
} can_protocol_t;

extern can_protocol_t *active_protocol;
extern can_frame_def_t *frame_lookup[CAN_ID_MAX];

// Load all embedded protocol JSON strings and build frame_lookup.
void protocol_loader_init(void);

// Observe one CAN ID and lock active_protocol once there is enough evidence.
void protocol_detect(uint32_t id);
