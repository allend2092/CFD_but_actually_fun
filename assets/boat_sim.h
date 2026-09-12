// boat_sim.h
// Real-hull boat rigid body for the CFD jet-ski. Header-only, pure ISO C++20,
// allocation-free, NaN-safe, no platform headers. Drops into the MSVC/D3D12
// project as a single #include. Verified on Linux with the turn test harness.
//
// HYBRID MODEL (2026-09-12): linear motion is a full rigid body — gravity,
// engine thrust, per-sample buoyancy, point-surface + viscous drag, planing
// lift, deck slam, lateral grip — integrated at 120 Hz. Attitude is KINEMATIC:
//   yaw   = steer-driven constant turn rate (integrates through 360 deg and on)
//   roll  = spring-damper toward a bank-into-turn target
//   pitch = spring-damper toward level (ride height comes from the heave physics)
// The old 6-DOF buoyancy-TORQUE attitude was structurally unstable past ~88 deg
// of cumulative heading change (pitch-roll coupling through the sample lever
// arms). It could not be tamed by righting moments, angular damping, gyro
// scaling, tumble locks, or controller gains, so the attitude DOFs are driven
// by bounded controllers that are stable by construction.
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
    // Hamilton product: (w1,v1)*(w2,v2) = (w1w2 - v1.v2,  w1 v2 + w2 v1 + v1 x v2)
    return { a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,   // x
             a.w*b.y + a.y*b.w + a.z*b.x - a.x*b.z,   // y
             a.w*b.z + a.z*b.w + a.x*b.y - a.y*b.x,   // z
             a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z }; // w
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

// ---------------------------------------------------------------------------
// Configuration (real jet-ski scale, meters / kg / N).
// ---------------------------------------------------------------------------
struct Config {
    float length      = 3.0f;   // hull length (z)
    float beam        = 1.2f;   // hull beam (x)
    float flatArea    = 2.2f;   // effective planing/buoyant bottom area (m^2)
    float maxSubmerse = 0.35f;  // buoyancy saturation depth (m)
    float mass        = 400.0f; // ski + rider (kg)
    float maxThrust   = 3600.0f;// engine force at full throttle (N)
    float C_lin       = 150.0f; // linear drag coeff
    float C_quad      = 18.0f;  // quadratic drag coeff
    float dragScale   = 1.0f;   // scale on the point-surface drag
    // Lateral grip: the hull tracks its nose in a turn. Pure velocity damping
    // on the slip component (always stabilizing); without it the kinematic yaw
    // would slide wide like a skid on ice.
    float latGrip     = 1200.0f;// N per (m/s) of lateral slip
    float liftCoeff   = 0.12f;  // planing lift coefficient (V-deadrise)
    float liftCap     = 2.5f;   // max planing lift as a multiple of weight
    // --- Kinematic attitude (hybrid model) ---
    // Yaw: holding a turn key rotates the heading at ~maxYawRate rad/s,
    // continuously through 360 deg and beyond (no heading error to saturate).
    float maxYawRate = 0.35f;   // rad/s full-lock turn rate (~20 deg/s)
    // Roll: spring-damper (units 1/s^2, 1/s) toward bank-into-turn.
    // +steer = right turn => positive roll (hull right side down).
    float bankMax = 0.16f;      // lean into the turn (~9 deg)
    float rollK = 30.0f;        // 1/s^2 (natural freq ~5.5 rad/s: a lazy bank)
    float rollDamp = 11.0f;     // 1/s (slightly over critical: no overshoot)
    // Pitch: spring-damper toward level trim; the heave physics sets ride height.
    float pitchK = 30.0f;       // 1/s^2
    float pitchDamp = 11.0f;    // 1/s
    // Turn authority vs speed: full at steerSpeedDiv m/s, but keep a floor so the
    // bow can still be aimed at low speed (a jetski can pivot).
    float steerSpeedDiv = 4.0f; // speed (m/s) at which turn authority is full
    float steerSpeedFloor = 0.35f; // min turn authority fraction at standstill
    // Deck reaction: waterproof concave deck traps air; water over the deck
    // meets a much stiffer, damped load than the bottom hull (deck slam).
    float deckHeight = 0.45f;   // bottom plane -> rider deck (m)
    float deckStiff = 8.0f;     // stiffness multiplier vs the bottom spring
    float deckDamp = 900.0f;    // N per (m/s) of deck descending into water
    float froudeLen   = 2.0f;   // reference length for the Froude number (m)
    float rho         = 1000.0f;
    float g           = 9.81f;
    // Buoyancy samples (hull-local, at the bottom). 3 (z) x 2 (x). weights sum to 1.
    struct S { Vec3 p; float w; };
    S samples[6] = {
        //            port(-x) / starboard(+x),   z, weight   (sum = 1.0)
        { { -0.45f, 0.0f, -1.35f }, 0.12f },  // stern  port
        { {  0.45f, 0.0f, -1.35f }, 0.12f },  // stern  starboard
        { { -0.45f, 0.0f,  0.00f }, 0.26f },  // mid    port
        { {  0.45f, 0.0f,  0.00f }, 0.26f },  // mid    starboard
        { { -0.45f, 0.0f,  1.35f }, 0.12f },  // bow    port
        { {  0.45f, 0.0f,  1.35f }, 0.12f },  // bow    starboard
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
        yaw_ = 0.0f; roll_ = 0.0f; pitch_ = 0.0f;
        rollRate_ = 0.0f; pitchRate_ = 0.0f;
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
    // Trim = pitch, bow-up positive (radians). asin of the forward-axis elevation
    // is yaw-agnostic, so it stays correct through 360 deg of heading.
    float trim() const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        return std::asin(clampf(Z.y, -1.0f, 1.0f));
    }
    // Roll, right-side-down positive. asin of the right-axis drop is yaw-agnostic.
    float roll() const {
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);
        return std::asin(clampf(-X.y, -1.0f, 1.0f));
    }
    // Kinematic attitude state (radians) — exact values the controllers track.
    float kinYaw() const { return yaw_; }
    float kinRoll() const { return roll_; }
    float kinPitch() const { return pitch_; }
    // How wetted the hull is, 0 (dry/planing) .. 1 (fully submerged samples).
    float wettedFrac() const { return lastWet_; }
    float lastLift() const { return lastLift_; }
    float lastFr() const { return lastFr_; }

    // Advance the boat. waterH[i] = world-space water height at sample i (the
    // caller combines Gerstner + wake field). dt should be a substep (e.g. 1/120).
    void step(float dt, const float* waterH) {
        const Config& c = cfg_;
        Vec3 X, Y, Z; qaxes(q_, X, Y, Z);

        Vec3 F = { 0.0f, -c.g * c.mass, 0.0f };   // gravity

        // Engine thrust along the hull forward axis.
        F = vadd(F, vmul(Z, c.maxThrust * throttle_));

        // Wetted fraction from sample submersion (drives planing + drag).
        float wetSum = 0.0f;
        for (int i = 0; i < c.nSamples; ++i) {
            float depth = waterH[i] - sampleWorldY(i);
            wetSum += c.samples[i].w * clampf(depth / c.maxSubmerse, 0.0f, 1.0f);
        }
        lastWet_ = clampf(wetSum, 0.0f, 1.0f);

        // Buoyancy + point-surface drag + deck slam, per bottom patch. Forces
        // only: the attitude is kinematic now, so the old lever-arm torques
        // (the source of the 88-deg pitch-roll tumble) are gone.
        for (int i = 0; i < c.nSamples; ++i) {
            float depth = waterH[i] - sampleWorldY(i);
            float d = clampf(depth, 0.0f, c.maxSubmerse);
            float area = c.flatArea * c.samples[i].w;
            float by = c.rho * c.g * area * d;
            Vec3 s = cfg_.samples[i].p;
            Vec3 r = vadd(vadd(vmul(X, s.x), vmul(Y, s.y)), vmul(Z, s.z));  // rel to COM
            // Point-surface drag on the moving sample.
            Vec3 vpt = vadd(vel_, vcross(ang_, r));
            float vsp = vlen(vpt);
            Vec3 Fs = { 0, by, 0 };
            Fs = vsub(Fs, vmul(vpt, (40.0f + 14.0f * vsp) * c.dragScale * c.samples[i].w));
            F = vadd(F, Fs);
            // Deck slam: once water reaches the deck plane, a much stiffer and
            // damped reaction catches it. Bottom may immerse; the deck resists.
            const float deckY = sampleWorldY(i) + Y.y * c.deckHeight;
            const float over = waterH[i] - deckY;
            if (over > 0.0f) {
                const float ov = std::min(over, 0.5f);
                const Vec3 rDeck = vadd(r, vmul(Y, c.deckHeight));
                const Vec3 vDeck = vadd(vel_, vcross(ang_, rDeck));
                const float slamV = std::max(0.0f, -vDeck.y);   // deck moving down
                const float fDeck = (c.rho * c.g * c.flatArea * c.deckStiff * ov
                    + c.deckDamp * slamV) * c.samples[i].w;
                F = vadd(F, Vec3{ 0.0f, fDeck, 0.0f });
            }
        }

        // Global (viscous) drag, reduced when planing (less wetted area).
        float sp = speed();
        float wetDragFactor = 0.35f + 0.65f * lastWet_;
        F = vsub(F, vmul(vel_, (c.C_lin + c.C_quad * sp) * wetDragFactor));

        // Lateral grip: bleed off velocity that points sideways to the hull so
        // the boat tracks its nose while the (kinematic) heading rotates.
        {
            const float vfwd = vdot(vel_, Z);
            const Vec3 vlat = vsub(vel_, vmul(Z, vfwd));
            F = vsub(F, vmul(vlat, c.latGrip));
        }

        // Planing lift: Froude-gated dynamic pressure on the wetted flat.
        float Fr = froude();
        lastFr_ = Fr;
        float planing = smoothstepf(0.50f, 1.10f, Fr);
        float dynQ = 0.5f * c.rho * sp * sp;
        // Lift scales with wetted fraction: it vanishes as the hull rises out of
        // the water, which is what creates the ride-height equilibrium (the hull
        // settles at the draft where lift + buoyancy = weight).
        float lift = planing * dynQ * (c.flatArea * lastWet_) * c.liftCoeff;
        lift = std::min(lift, c.liftCap * c.g * c.mass);   // never launch it
        lastLift_ = lift;
        F = vadd(F, vmul(Y, lift));                          // lift along hull up (at COM: no torque)

        // Linear integration (semi-implicit Euler).
        vel_ = vadd(vel_, vmul(F, dt / c.mass));
        if (vlen(vel_) > 30.0f) vel_ = vmul(vnorm(vel_), 30.0f);
        pos_ = vadd(pos_, vmul(vel_, dt));

        // --- Kinematic attitude: bounded, stable by construction ---
        const float speedFactor = clampf(c.steerSpeedFloor + (1.0f - c.steerSpeedFloor) * (sp / c.steerSpeedDiv), 0.0f, 1.0f);
        const float yawRate = steer_ * c.maxYawRate * speedFactor;
        yaw_ += yawRate * dt;
        if (yaw_ > 3.14159265f) yaw_ -= 6.28318531f;   // keep float precision over long turns
        if (yaw_ < -3.14159265f) yaw_ += 6.28318531f;

        const float rollTarget = steer_ * c.bankMax * speedFactor;  // + = bank right (into a right turn)
        rollRate_ += ((rollTarget - roll_) * c.rollK - rollRate_ * c.rollDamp) * dt;
        roll_ += rollRate_ * dt;

        const float pitchTarget = 0.0f;   // level trim; ride height comes from the heave physics
        pitchRate_ += ((pitchTarget - pitch_) * c.pitchK - pitchRate_ * c.pitchDamp) * dt;
        pitch_ += pitchRate_ * dt;

        if (!std::isfinite(roll_) || !std::isfinite(pitch_) || !std::isfinite(yaw_)) {
            roll_ = pitch_ = yaw_ = 0.0f;
            rollRate_ = pitchRate_ = 0.0f;
        }
        roll_  = clampf(roll_, -1.2f, 1.2f);
        pitch_ = clampf(pitch_, -1.0f, 1.0f);

        // Compose the hull orientation: yaw about world Y, then pitch about the
        // hull fore-aft axis, then roll about the hull forward axis.
        q_ = qnorm(qmul(qmul(qfromAxis(Vec3{ 0, 1, 0 }, yaw_),
                            qfromAxis(Vec3{ 1, 0, 0 }, -pitch_)),
                        qfromAxis(Vec3{ 0, 0, 1 }, -roll_)));

        // Pseudo angular velocity from the kinematic rates (body frame), used
        // for point-surface drag velocities and the rider rig.
        Vec3 Xn, Yn, Zn; qaxes(q_, Xn, Yn, Zn);
        ang_ = vadd(vsub(vmul(Yn, yawRate), vmul(Xn, pitchRate_)), vmul(Zn, -rollRate_));
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
    Config cfg_;
    Vec3 pos_{0,0,0}, vel_{0,0,0}, ang_{0,0,0};
    Quat q_{0,0,0,1};
    float throttle_ = 0.0f, steer_ = 0.0f;
    float lastWet_ = 0.0f, lastLift_ = 0.0f, lastFr_ = 0.0f;
    float yaw_ = 0.0f, roll_ = 0.0f, pitch_ = 0.0f;
    float rollRate_ = 0.0f, pitchRate_ = 0.0f;
};

}  // namespace boat
