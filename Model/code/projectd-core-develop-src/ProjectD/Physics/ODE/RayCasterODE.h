#pragma once

#include "Physics/ODE/PhysicsEngineODE.h"

namespace D {

// RayCasterODE.h
struct RayParams {
    vec3f pos;
    vec3f dir;
    float length;
    int firstContact;   // 0/1
    int backfaceCull;   // 0/1
    int closestHit;     // 0/1
};

struct RayCasterODE : public IRayCaster, PhysicsChildODE {
    RayCasterODE(PhysicsEngineODEPtr core, float length);
    ~RayCasterODE();

    RayCastHit rayCast(const vec3f& pos, const vec3f& dir) override;

    // NEW:
    RayParams getState() const;
    void setState(const RayParams& p);
    void reset(float length = 100.0f);   // 简易 reset

    dxGeom* id = nullptr;
};

}
