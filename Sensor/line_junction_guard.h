#ifndef LINE_JUNCTION_GUARD_H
#define LINE_JUNCTION_GUARD_H

#include <stdint.h>

typedef struct {
    uint8_t enabled;
    uint8_t active;
    uint8_t has_valid_measure;
    float held_measure;
} LineJunctionGuardState;

typedef struct {
    float measure;
    uint8_t sync_pid_history;
} LineJunctionGuardOutput;

void LineJunctionGuard_Reset(LineJunctionGuardState *state);
void LineJunctionGuard_SetEnabled(LineJunctionGuardState *state, uint8_t enabled);
LineJunctionGuardOutput LineJunctionGuard_Update(LineJunctionGuardState *state,
                                                 float raw_measure,
                                                 uint8_t line_count,
                                                 uint8_t led_count,
                                                 float target);

#endif /* LINE_JUNCTION_GUARD_H */
