#include "line_junction_guard.h"

#define JUNCTION_NORMAL_LED_MAX 4u

void LineJunctionGuard_Reset(LineJunctionGuardState *state)
{
    state->enabled = 0u;
    state->active = 0u;
    state->has_valid_measure = 0u;
    state->held_measure = 0.0f;
}

void LineJunctionGuard_SetEnabled(LineJunctionGuardState *state, uint8_t enabled)
{
    enabled = enabled ? 1u : 0u;

    if (enabled && !state->enabled)
    {
        state->active = 0u;
        state->has_valid_measure = 0u;
        state->held_measure = 0.0f;
    }

    state->enabled = enabled;
}

LineJunctionGuardOutput LineJunctionGuard_Update(LineJunctionGuardState *state,
                                                 float raw_measure,
                                                 uint8_t line_count,
                                                 uint8_t led_count,
                                                 float target)
{
    LineJunctionGuardOutput output = {raw_measure, 0u};
    uint8_t ambiguous = (line_count > 1u || led_count > JUNCTION_NORMAL_LED_MAX) ? 1u : 0u;

    if (!state->enabled)
    {
        if (state->active)
            output.sync_pid_history = 1u;
        state->active = 0u;
        return output;
    }

    if (ambiguous)
    {
        output.measure = state->has_valid_measure ? state->held_measure : target;
        if (!state->active)
            output.sync_pid_history = 1u;
        state->active = 1u;
        return output;
    }

    if (state->active)
        output.sync_pid_history = 1u;

    state->active = 0u;
    state->has_valid_measure = 1u;
    state->held_measure = raw_measure;
    return output;
}
