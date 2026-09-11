// boat_sim.h
// Real-hull boat rigid body for the CFD jet-ski. Header-only, pure ISO C++20,
// allocation-free, NaN-safe, no platform headers. Drops into the MSVC/D3D12
// project as a single #include. Verified on Linux with boat_sim_test.cpp.
//
// Replaces the 0.9m box collider in main.cpp with a collider that spans the
// real ~3.0 m hull, adds planing lift + trim (Froude-gated), and emits a
// jetski::VehicleState that feeds the rider rig directly.
//
// Frame: world Y up, +Z forward (bow), +X right, meters. Hull-local frame is
// the same convention as the jetski rig.
#pragma once
#include "jetski_asset.h"   // jetski::VehicleState (exact layout)
#include <cmath>
#include <cfloat>

namespace boat {

struct Vec3 { float x, y, z; };
inline Vec3 vadd(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline Vec3 vsub(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline Vec3 vmul(Vec3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
inline Vec3 vdiv(Vec3 a, float s) { return { a.x / s, a.y / s, a.z / s }; }
inline float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 vcross(Vec3 a, Vec3 b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline float vlen(Vec3 a) { return std::sqrt(vdot(a, a)); }
inline Vec3 vnorm(Vec3 a) { float l = vlen(a); return l > 1e-8f ? vdiv(a, l) : Vec3{0,1,0}; }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float smoothstepf(float e0, float e1, float x) {
    float t = clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

struct Quat { float x, y, z, w; };
inline Quat qmul(Quat a, Quat b) {
    return { a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
             a.w*b.y - a.x*b.w + a.y*b.z + a.z*b.x,
             a.w*b.z + a.x*b.w - a.y*b.x + a.z*b.y,
             a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z };
}
inline Quat qfromAxis(Vec3 axis, float ang) {
    Vec3 a = vnorm(axis); float s = std::sin(ang * 0.5f);
    return { a.x*s, a.y*s, a.z*s, std::cos(ang * 0.5f) };
}
inline Quat qnorm(Quat q) {
    float l = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    return l > 1e-8f ? Quat{ q.x/l, q.y/l, q.z/l, q.w/l } : Quat{0,0,0,1};
}
// Columns of the returned basis are the hull X,Y,Z axes in world space.
inline void qaxes(Quat q, Vec3& X, Vec3& Y, Vec3& Z) {
    X = { 1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.w*q.z), 2*(q.x*q.z-q.w*q.y) };
    Y = { 2*(q.x*q.y-q.w*q.z),   1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z+q.w*q.x) };
    Z = { 2*(q.x*q.z+q.w*q.y),   2*(q.y*q.z-q.w*q.x),   1-2*(q.x*q.x+q.y*q.y) };
}
inline Quat qrotate(Quat q, Vec3 local) {
    // rotate a vector by the quaternion
    Quat p{ local.x, local.y, local.z, 0 };
    Quat c = qmul(qmul(q, p), Quat{-q.x, -q.y, -q.z, q.w});
    return Quat{ c.x, c.y, c.z, 0 };
}

// ---------------------------------------------------------------------------
// Configuration (real jet-ski scale, meters / kg / N).
// ---------------------------------------------------------------------------
struct Config {
    float length      = 3.0f;   // hull length (z)
    float beam        = 1.2f;   // hull beam (x)
    float flatArea    = 2.2f;   // effective planing/buoyant bottom area (m^2)
    float maxSubmerse = 0.28f;  // max per-sample submersion depth (m) — shallow draft
    float mass        = 400.0f; // ski + rider (kg)
    float maxThrust   = 3600.0f;// engine force at full throttle (N)
    float C_lin       = 150.0f; // linear drag coeff
    float C_quad      = 18.0f;  // quadratic drag coeff
    float liftCoeff   = 0.12f;  // planing lift coefficient (V-deadrise)
    float liftCap     = 2.5f;   // max planing lift as a multiple of weight
    float trimCoeff   = 1.6f;   // bow-up trim torque scale (N*m per v^2)
    float steerTorque = 1400.0f;// yaw torque per unit steer (N*m)
    float restoreK    = 2200.0f;// righting torque toward upright (N*m per sin)
    float angDamp     = 3.2f;   // angular velocity damping (1/s, exp decay)
    float froudeLen   = 2.0f;   // reference length for the Froude number (m)
    float rho         = 1000.0f;
    float g           = 9.81f;
    // Buoyancy samples (hull-local, at the bottom). 3 (z) x 2 (x). weights sum to 1.
    struct S { Vec3 p; float w; };
    S samples[6] = {
        //            port(-x) / starboard(+x),   z, weight
        { { -0.40f, 0.0f, -1.20f }, 0.10f },  // stern  port
        { {  0.40f, 0.0f, -1.20f }, 0.10f },  // stern  starboard
        { { -0.40f, 0.0f,  0.00f }, 0.30f },  // mid    port
        { {  0.40f, 0.0f,  0.00f }, 0.30f },  // mid    starboard
        { { -0.40f, 0.0f,  1.20f }, 0.10f },  // bow    port
        { {  0.40f, 0.0f,  1.20f }, 0.10f },  // bow    starboard
    };
    int nSamples = 6;
};

class Boat {
public:
    explicit Boat(const Config& c = Config{}) : cfg_(c) { reset(0, 0, 0); }

    void reset(float x, float y, float z) {
        pos_ = { x, y, z };
        vel_ = { 0, 0, 0 };
        ang_ = { 0, 0, 0 };
        q_   = { 0, 0, 0, 1 };
    }
    void setConfig(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    int nSamples() const { return cfg_.nSamples; }
    Vec3 sampleLocal(int i) const { return cfg_.samples[i].p; }
    Vec3 sampleWorld(int i) const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        Vec3 s = cfg_.samples[i].p;
        return vadd(pos_, vadd(vadd(vmul(X, s.x), vmul(Y, s.y)), vmul(Z, s.z)));
    }

    Vec3 pos() const { return pos_; }
    Vec3 vel() const { return vel_; }
    Vec3 angVel() const { return ang_; }
    float speed() const { return vlen(vel_); }
    float froude() const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        float vh = std::sqrt(vel_.x*vel_.x + vel_.z*vel_.z);
        return vh / std::sqrt(cfg_.g * cfg_.froudeLen);
    }
    // Trim = pitch, bow-up positive (radians).
    float trim() const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        return std::atan2(Z.y, Z.z);
    }
    float roll() const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        return std::atan2(-X.y, X.x);
    }
    // How wetted the hull is, 0 (dry/planing) .. 1 (fully submerged samples).
    float wettedFrac() const { return lastWet_; }
    float lastLift() const { return lastLift_; }
    float lastFr() const { return lastFr_; }

    // Advance the boat. waterH[i] = world-space water height at sample i (the
    // caller combines Gerstner + wake field). dt should be a substep (e.g. 1/120).
    void step(float dt, const float* waterH) {
        const Config& c = cfg_;
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        (void)Y;

        Vec3 F = { 0.0f, -c.g * c.mass, 0.0f };   // gravity
        Vec3 T = { 0, 0, 0 };

        // Engine thrust along the hull forward axis.
        F = vadd(F, vmul(Z, c.maxThrust * throttle_));

        // Wetted fraction from sample submersion (drives planing + drag).
        float wetSum = 0.0f;
        for (int i = 0; i < c.nSamples; ++i) {
            float depth = waterH[i] - sampleWorldY(i);
            wetSum += c.samples[i].w * clampf(depth / c.maxSubmerse, 0.0f, 1.0f);
        }
        lastWet_ = clampf(wetSum, 0.0f, 1.0f);

        // Buoyancy: each sample is a bottom patch of area flatArea*w_i. The lever
        // arm is relative to the center of mass (NOT the world position), or a
        // forward-driving boat injects a huge spurious torque as pos_ grows.
        for (int i = 0; i < c.nSamples; ++i) {
            float depth = waterH[i] - sampleWorldY(i);
            float d = clampf(depth, 0.0f, c.maxSubmerse);
            float area = c.flatArea * c.samples[i].w;
            float by = c.rho * c.g * area * d;
            Vec3 s = cfg_.samples[i].p;
            Vec3 r = vadd(vadd(vmul(X, s.x), vmul(Y, s.y)), vmul(Z, s.z));  // rel to COM
            Vec3 Fs = { 0, by, 0 };
            // Point-surface drag on the moving sample.
            Vec3 vpt = vadd(vel_, vcross(ang_, r));
            float sp = vlen(vpt);
            Fs = vsub(Fs, vmul(vpt, (60.0f + 20.0f * sp) * c.samples[i].w));
            F = vadd(F, Fs);
            T = vadd(T, vcross(r, Fs));
        }

        // Global (viscous) drag, reduced when planing (less wetted area).
        float sp = speed();
        float wetDragFactor = 0.35f + 0.65f * lastWet_;
        F = vsub(F, vmul(vel_, (c.C_lin + c.C_quad * sp) * wetDragFactor));

        // Planing lift: Froude-gated dynamic pressure on the wetted flat.
        float Fr = froude();
        lastFr_ = Fr;
        float planing = smoothstepf(0.50f, 1.10f, Fr);
        float q = 0.5f * c.rho * sp * sp;
        // Lift scales with wetted fraction: it vanishes as the hull rises out of
        // the water, which is what creates the ride-height equilibrium (the hull
        // settles at the draft where lift + buoyancy = weight). A constant floor
        // here would let the hull climb indefinitely.
        float lift = planing * q * (c.flatArea * lastWet_) * c.liftCoeff;
        lift = std::min(lift, c.liftCap * c.g * c.mass);   // never launch it
        lastLift_ = lift;
        F = vadd(F, vmul(Y, lift));                          // lift along hull up (at COM: no torque)
        // Bow-up trim torque (planing pressure pitches the nose up). A +X torque
        // pitches the bow DOWN (right-hand rule), so bow-UP is the -X direction.
        T = vadd(T, vmul(X, -planing * sp * sp * c.trimCoeff));

        // Righting: a stable restoring torque toward upright (world up). This is
        // what keeps a game boat from somersaulting; it balances the planing
        // trim into a small steady bow-up angle instead of relying on the (weak)
        // geometric buoyancy moment alone.
        T = vadd(T, vmul(vcross(Y, Vec3{0, 1, 0}), c.restoreK));

        // Steering yaw torque (needs speed to bite, like a real rudder/steer).
        float speedFactor = clampf(sp / 5.0f, 0.0f, 1.0f);
        T = vadd(T, vmul(Y, steer_ * c.steerTorque * speedFactor));

        // Integrate (semi-implicit Euler).
        vel_ = vadd(vel_, vmul(F, dt / c.mass));
        if (vlen(vel_) > 30.0f) vel_ = vmul(vnorm(vel_), 30.0f);
        pos_ = vadd(pos_, vmul(vel_, dt));

        // Angular: I is a box approximation about the hull center.
        Vec3 I = { c.mass / 12.0f * (0.7f*0.7f + c.length*0.5f*c.length*0.5f),
                   c.mass / 12.0f * (c.beam*0.5f*c.beam*0.5f + c.length*0.5f*c.length*0.5f),
                   c.mass / 12.0f * (c.beam*0.5f*c.beam*0.5f + 0.7f*0.7f) };
        Vec3 tb = { vdot(T, X), vdot(T, Y), vdot(T, Z) };
        Vec3 wb = { vdot(ang_, X), vdot(ang_, Y), vdot(ang_, Z) };
        Vec3 Iw = { wb.x*I.x, wb.y*I.y, wb.z*I.z };
        Vec3 gyro = vcross(wb, Iw);
        Vec3 alpha = { (tb.x - gyro.x)/I.x, (tb.y - gyro.y)/I.y, (tb.z - gyro.z)/I.z };
        wb = vadd(wb, vmul(alpha, dt));
        wb = vmul(wb, std::exp(-c.angDamp * dt));      // strong angular damping
        if (vlen(wb) > 4.0f) wb = vmul(vnorm(wb), 4.0f);
        ang_ = vadd(vadd(vmul(X, wb.x), vmul(Y, wb.y)), vmul(Z, wb.z));
        q_ = qnorm(qmul(q_, qfromAxis(vnorm(safeVec(ang_)), vlen(ang_) * dt)));
    }

    // Control inputs (0..1 / -1..1). Set between steps.
    void setControls(float throttle, float steer) {
        throttle_ = clampf(throttle, 0.0f, 1.0f);
        steer_    = clampf(steer, -1.0f, 1.0f);
    }
    float throttle() const { return throttle_; }
    float steer() const { return steer_; }

    // Fill the rider rig's state. supportH/supportN are the water height/normal
    // directly under the hull center (caller combines Gerstner + wake).
    void vehicleState(float time, float supportH, Vec3 supportN, jetski::VehicleState& out) const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        out.time = time;
        out.throttle = throttle_;
        out.steer = steer_;
        out.hullPosition[0] = pos_.x; out.hullPosition[1] = pos_.y; out.hullPosition[2] = pos_.z;
        // row-major 3x3, columns = X,Y,Z (matches jetski: world = B * local)
        out.hullBasis[0*3+0] = X.x; out.hullBasis[0*3+1] = Y.x; out.hullBasis[0*3+2] = Z.x;
        out.hullBasis[1*3+0] = X.y; out.hullBasis[1*3+1] = Y.y; out.hullBasis[1*3+2] = Z.y;
        out.hullBasis[2*3+0] = X.z; out.hullBasis[2*3+1] = Y.z; out.hullBasis[2*3+2] = Z.z;
        out.hullVelocity[0] = vel_.x; out.hullVelocity[1] = vel_.y; out.hullVelocity[2] = vel_.z;
        out.hullAngularVel[0] = ang_.x; out.hullAngularVel[1] = ang_.y; out.hullAngularVel[2] = ang_.z;
        out.supportHeight = supportH;
        out.supportNormal[0] = supportN.x; out.supportNormal[1] = supportN.y; out.supportNormal[2] = supportN.z;
    }

private:
    Vec3 localToWorldVec(Vec3 local) const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        return vadd(pos_, vadd(vadd(vmul(X, local.x), vmul(Y, local.y)), vmul(Z, local.z)));
    }
    float sampleWorldY(int i) const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        Vec3 s = cfg_.samples[i].p;
        return pos_.y + X.y*s.x + Y.y*s.y + Z.y*s.z;
    }
    static Vec3 safeVec(Vec3 v) {
        if (std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z)) return v;
        return {0,0,0};
    }
    Config cfg_;
    Vec3 pos_{0,0,0}, vel_{0,0,0}, ang_{0,0,0};
    Quat q_{0,0,0,1};
    float throttle_ = 0.0f, steer_ = 0.0f;
    float lastWet_ = 0.0f, lastLift_ = 0.0f, lastFr_ = 0.0f;
};

}  // namespace boat
