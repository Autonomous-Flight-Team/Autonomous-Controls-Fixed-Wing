#ifndef SURFACE_DEMAND_H_
#define SURFACE_DEMAND_H_

// What the plane should do, in normalized units, before mixing.
// Produced by control/ (future) and consumed by the mixer.
struct SurfaceDemand {
    float roll = 0.0f;     // [-1, 1], +right wing down
    float pitch = 0.0f;    // [-1, 1], +nose up
    float yaw = 0.0f;      // [-1, 1], +nose right
    float throttle = 0.0f; // [0, 1]
    float flaps = 0.0f;    // [0, 1]
};

#endif // SURFACE_DEMAND_H_
