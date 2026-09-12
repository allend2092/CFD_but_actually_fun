// jetski_internal.h
// Internal shared types for the jetski module. NOT part of the public contract
// (jetski_asset.h). Included only by jetski_asset.cpp, jetski_ik.cpp, and the
// test harness. No platform headers.
#pragma once
#include "jetski_asset.h"   // public: AssetVertex, MeshTopology, VehicleState, fwd Rig
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <vector>

namespace jetski {
namespace detail {

// ---------------------------------------------------------------------------
// Minimal 3D math. World conventions: meters, Y-up, +Z forward, +X right.
// All functions are allocation-free and NaN-safe (see safeNormalize/clampf).
// ---------------------------------------------------------------------------
struct Vec3 { float x = 0.0f, y = 0.0f, z = 0.0f; };

constexpr float kEps = 1e-6f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

inline Vec3 vadd(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline Vec3 vsub(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline Vec3 vmul(Vec3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
inline float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 vcross(Vec3 a, Vec3 b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline float vlen(Vec3 a) { return std::sqrt(std::max(0.0f, vdot(a, a))); }

// Returns unit(a); if |a| < 1e-8 returns `fallback` (never NaN/Inf).
inline Vec3 safeNormalize(Vec3 a, Vec3 fallback) {
    const float l = vlen(a);
    if (l < 1e-8f) return fallback;
    return { a.x / l, a.y / l, a.z / l };
}
inline Vec3 safeNormalize(Vec3 a) { return safeNormalize(a, Vec3{0.0f, 0.0f, 1.0f}); }

// Row-major 3x3, world <- local (matches VehicleState::hullBasis layout).
struct Mat3 { float m[3][3]; };

inline Mat3 mat3Identity() {
    Mat3 r{}; r.m[0][0] = 1.0f; r.m[1][1] = 1.0f; r.m[2][2] = 1.0f; return r;
}
inline Vec3 m3mulv(Mat3 M, Vec3 v) {
    return {
        M.m[0][0] * v.x + M.m[0][1] * v.y + M.m[0][2] * v.z,
        M.m[1][0] * v.x + M.m[1][1] * v.y + M.m[1][2] * v.z,
        M.m[2][0] * v.x + M.m[2][1] * v.y + M.m[2][2] * v.z,
    };
}
inline Mat3 m3mul(Mat3 A, Mat3 B) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += A.m[i][k] * B.m[k][j];
            r.m[i][j] = s;
        }
    return r;
}
// Rotation about a unit axis by angle (radians). Rodrigues.
inline Mat3 mat3AxisAngle(Vec3 axis, float angle) {
    Vec3 a = safeNormalize(axis, Vec3{0.0f, 1.0f, 0.0f});
    const float c = std::cos(angle), s = std::sin(angle), t = 1.0f - c;
    Mat3 r{};
    r.m[0][0] = t * a.x * a.x + c;
    r.m[0][1] = t * a.x * a.y - s * a.z;
    r.m[0][2] = t * a.x * a.z + s * a.y;
    r.m[1][0] = t * a.x * a.y + s * a.z;
    r.m[1][1] = t * a.y * a.y + c;
    r.m[1][2] = t * a.y * a.z - s * a.x;
    r.m[2][0] = t * a.x * a.z - s * a.y;
    r.m[2][1] = t * a.y * a.z + s * a.x;
    r.m[2][2] = t * a.z * a.z + c;
    return r;
}
constexpr float toRadians(float deg) { return deg * (3.14159265358979323846f / 180.0f); }

// Load a row-major 3x3 from a flat[9] (hullBasis) into a Mat3.
inline Mat3 mat3FromFlat(const float* f) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = f[i * 3 + j];
    return r;
}

// A joint's world transform: origin (world position) + basis (world <- local).
struct JointXf { Vec3 origin; Mat3 basis; };

// ---------------------------------------------------------------------------
// Skeleton. Fixed joint set; indices match the JointXf array in PoseResult.
// ---------------------------------------------------------------------------
enum Joint : int {
    J_Hull = 0,
    J_Grips,       // handlebar grips; rotates with steer
    J_Hips,        // pelvis root; low-pass filtered vertical lag
    J_Torso,       // spine; lean pivot at the hips
    J_Head,        // neck; vestibular counter-rotation
    J_ShoulderL, J_ShoulderR,
    J_ElbowL,  J_ElbowR,
    J_HandL,   J_HandR,      // IK targets (grips)
    J_HipL,    J_HipR,       // leg roots
    J_KneeL,   J_KneeR,
    J_FootL,   J_FootR,      // IK targets (footwells)
    J_Count
};

// Joint angles reported for the test harness (radians).
enum JointAngle : int {
    JA_SpineRoll = 0, JA_SpinePitch,
    JA_HipL, JA_HipR,
    JA_KneeL, JA_KneeR,
    JA_ElbowL, JA_ElbowR,
    JA_ShoulderL, JA_ShoulderR,
    JA_NeckRoll, JA_NeckPitch,
    JA_Count
};

// Low-pass filter state for the hips (lives in the Rig, mutated in rig_pose).
struct FilterState {
    float filteredHullY = 0.0f;  // smoothed hull vertical position
    float lastTime = -1.0f;      // previous state.time, for dt
};

// (Rig is the public forward-declared jetski::Rig; its complete definition
// lives at the bottom of this header, in namespace jetski.)

// Full posed joint graph + reported angles.
struct PoseResult {
    JointXf joint[J_Count];
    float angles[JA_Count];
};

// Advances the hip low-pass filter using this frame's state. Defensive: clamps
// dt, ignores non-finite input, and handles the first frame.
void advanceFilter(FilterState& f, float time, float hullY);

// Computes the full pose (joint transforms + angles) WITHOUT advancing the
// filter. `f` is read as-is. Deterministic and allocation-free.
void computePose(const VehicleState& state, const FilterState& f, PoseResult& out);

// Static rider pose: the rest pose (identity hull, neutral filter, zero
// throttle/steer) rigidly attached to the live hull. No IK, no filter lag,
// no lean/squat/steer — the rider moves exactly with the jet-ski. Deterministic
// and allocation-free. Safe at any heading (unlike computePose, whose
// atan2-based hull roll/pitch extraction degenerates near +/-180 deg yaw).
void computeStaticPose(const VehicleState& state, PoseResult& out);

// Debug only (test harness): joint angles / world joint positions for a state,
// using the Rig's CURRENT filter state (as advanced by the most recent
// rig_pose call). Does not advance the filter. out must have JA_Count floats
// (angles) or J_Count*3 floats (positions, xyz per joint).
void rig_debug_jointAngles(const Rig& r, const VehicleState& s, float* out);
void rig_debug_jointPositions(const Rig& r, const VehicleState& s, float* out);

// Two-bone analytic IK. Returns the joint (elbow/knee) world position for
// root R reaching target T with bone lengths L1 (upper), L2 (lower). The end
// effector is placed at T. Bend plane is defined by (bendHint, T-R); bendSign
// selects the side. Always finite for finite input (dist + acos clamped).
Vec3 twoBoneIK(Vec3 R, Vec3 T, float L1, float L2, Vec3 bendHint, float bendSign);

// Builds an orthonormal basis whose local -Z points along `boneDir` (i.e. a
// limb modeled from (0,0,0) to (0,0,-len) in local space points along boneDir
// in world). upHint disambiguates the roll of the frame.
Mat3 boneBasis(Vec3 boneDir, Vec3 upHint);

}  // namespace detail

// Complete definition of the publicly forward-declared jetski::Rig.
// Implementation detail (the public header only forward-declares it); it lives
// here so the test harness and integrators can instantiate it. `filter` is
// mutable so rig_pose can advance the hip low-pass through a const Rig&.
struct Rig {
    std::vector<uint32_t>           indices;    // all part indices, concatenated
    std::vector<AssetVertex>        rest;       // rest-pose vertices (one per vertex)
    std::vector<MeshTopology::Part> partRecs;   // one per part (topo.parts points here)
    std::vector<int>                vertexJoint;// per-vertex joint index (for posing)
    MeshTopology                    topo{};       // points into indices/partRecs
    mutable detail::FilterState     filter;     // hip low-pass state (lives in the Rig)
};

}  // namespace jetski
