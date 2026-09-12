// jetski_ik.cpp
// Pose solver: low-pass hip filter, two-bone analytic IK, joint graph.
// Pure ISO C++20, allocation-free, NaN-safe. No platform headers.
#include "jetski_internal.h"
#include "jetski_asset.h"   // for VehicleState definition

#include <cmath>

namespace jetski {
namespace detail {

// ---------------------------------------------------------------------------
// Rest-pose dimensions (meters), hull-local frame: origin at the waterline
// center of the hull, +Z forward, +Y up, +X right. Real-world scale.
// ---------------------------------------------------------------------------
namespace {

constexpr Vec3 HIPS_LOCAL   = { 0.00f, 0.55f, -0.15f };  // pelvis, hull frame
constexpr Vec3 SHOULDER_L   = { -0.16f, 0.50f, 0.12f };  // shoulder L, torso frame
constexpr Vec3 SHOULDER_R   = {  0.16f, 0.50f, 0.12f };  // shoulder R, torso frame
constexpr Vec3 NECK_LOCAL   = { 0.00f, 0.56f, 0.14f };   // head origin, torso frame
constexpr Vec3 LEGR_ROOT_L  = { -0.10f, 0.00f, 0.02f };  // leg root L, hips frame
constexpr Vec3 LEGR_ROOT_R  = {  0.10f, 0.00f, 0.02f };  // leg root R, hips frame
constexpr Vec3 GRIP_CENTER  = { 0.00f, 0.92f, 0.40f };  // handlebar pivot, hull frame
constexpr Vec3 GRIP_L       = { -0.20f, 0.00f, 0.00f }; // grip L, grips frame
constexpr Vec3 GRIP_R       = {  0.20f, 0.00f, 0.00f }; // grip R, grips frame
constexpr Vec3 FOOT_L       = { -0.22f, 0.15f, 0.35f }; // footwell L, hull frame
constexpr Vec3 FOOT_R       = {  0.22f, 0.15f, 0.35f }; // footwell R, hull frame

constexpr float ARM_UPPER = 0.37f;   // tuned: arms reach the grips through the
constexpr float ARM_LOWER = 0.35f;   // full steer sweep + lean without over-extending
constexpr float LEG_UPPER = 0.42f;
constexpr float LEG_LOWER = 0.42f;

constexpr float MAX_LEAN      = toRadians(12.0f);   // lateral lean from steer
constexpr float MAX_PITCHBACK = toRadians(14.0f);   // lean back from throttle
constexpr float ROLL_COUNTER  = 0.60f;              // torso counterbalances this fraction of the ACTUAL hull roll
constexpr float STEER_LEAN_FRAC = 0.40f;            // intentional steer-lean fraction (of MAX_LEAN), on top of counterbalance
constexpr float SQUAT_DEPTH   = 0.07f;              // hip drop under full throttle (m) — rider gets low into acceleration
constexpr float HEAD_COUNTER  = 0.75f;              // vestibular correction ratio
constexpr float STEER_MAX     = toRadians(14.0f);   // max grip rotation (about Y)
constexpr float FILTER_TAU    = 0.25f;              // hip low-pass time constant

inline bool isFiniteVec(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
inline Vec3 finiteVec(Vec3 v, Vec3 fb) { return isFiniteVec(v) ? v : fb; }
inline Mat3 finiteMat3(Mat3 M) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!std::isfinite(M.m[i][j])) return mat3Identity();
    return M;
}
inline float angleBetween(Vec3 a, Vec3 b) {
    Vec3 na = safeNormalize(a), nb = safeNormalize(b);
    return std::acos(clampf(vdot(na, nb), -1.0f, 1.0f));
}

}  // namespace

// ---------------------------------------------------------------------------
// Low-pass filter for the hips: tracks hull vertical motion with a lag so the
// knees/elbows read as shock absorbers while hands/feet stay planted.
// ---------------------------------------------------------------------------
void advanceFilter(FilterState& f, float time, float hullY)
{
    if (!std::isfinite(time) || !std::isfinite(hullY)) return;  // ignore garbage
    if (f.lastTime < 0.0f) {  // first frame: seed, no jump
        f.filteredHullY = hullY;
        f.lastTime = time;
        return;
    }
    float dt = time - f.lastTime;
    if (dt <= 0.0f) dt = 1.0f / 60.0f;  // non-positive dt: assume 60 Hz
    if (dt > 0.1f)   dt = 0.1f;         // clamp large jumps (tab-switch, etc.)
    const float k = 1.0f - std::exp(-dt / FILTER_TAU);
    f.filteredHullY += (hullY - f.filteredHullY) * k;
    f.lastTime = time;
}

// ---------------------------------------------------------------------------
// Two-bone analytic IK (closed form, no iteration).
// ---------------------------------------------------------------------------
Vec3 twoBoneIK(Vec3 R, Vec3 T, float L1, float L2, Vec3 bendHint, float bendSign)
{
    Vec3 d = vsub(T, R);
    float dist = vlen(d);
    float minD = std::fabs(L1 - L2) + 1e-4f;
    float maxD = (L1 + L2) - 1e-4f;
    if (minD > maxD) { minD = 1e-4f; maxD = (L1 > L2 ? L1 : L2); }
    const float D = clampf(dist, minD, maxD);

    float cosA = (L1 * L1 + D * D - L2 * L2) / (2.0f * L1 * D);
    cosA = clampf(cosA, -1.0f, 1.0f);
    const float A = std::acos(cosA);

    Vec3 dir  = safeNormalize(d, Vec3{0.0f, 0.0f, 1.0f});
    Vec3 axis = safeNormalize(vcross(bendHint, dir), Vec3{0.0f, 1.0f, 0.0f});
    Mat3 rot  = mat3AxisAngle(axis, A * bendSign);
    Vec3 upperDir = m3mulv(rot, dir);
    return vadd(R, vmul(upperDir, L1));
}

// ---------------------------------------------------------------------------
// Bone frame: local -Z points along boneDir (this joint -> next joint).
// ---------------------------------------------------------------------------
Mat3 boneBasis(Vec3 boneDir, Vec3 upHint)
{
    Vec3 localZ = vmul(safeNormalize(boneDir, Vec3{0.0f, 0.0f, 1.0f}), -1.0f);
    Vec3 localX = safeNormalize(vcross(upHint, localZ), Vec3{1.0f, 0.0f, 0.0f});
    Vec3 localY = vcross(localZ, localX);
    if (vdot(localY, upHint) < 0.0f) { localX = vmul(localX, -1.0f); localY = vmul(localY, -1.0f); }
    Mat3 r{};
    r.m[0][0] = localX.x; r.m[1][0] = localX.y; r.m[2][0] = localX.z;
    r.m[0][1] = localY.x; r.m[1][1] = localY.y; r.m[2][1] = localY.z;
    r.m[0][2] = localZ.x; r.m[1][2] = localZ.y; r.m[2][2] = localZ.z;
    return r;
}

// ---------------------------------------------------------------------------
// Full pose.
// ---------------------------------------------------------------------------
void computePose(const VehicleState& s, const FilterState& f, PoseResult& out)
{
    const float throttle = std::isfinite(s.throttle) ? clampf(s.throttle, 0.0f, 1.0f) : 0.0f;
    const float steer    = std::isfinite(s.steer)    ? clampf(s.steer, -1.0f, 1.0f)    : 0.0f;

    Mat3 hullBasis = finiteMat3(mat3FromFlat(s.hullBasis));
    Vec3 hullPos   = finiteVec({ s.hullPosition[0], s.hullPosition[1], s.hullPosition[2] },
                               Vec3{0.0f, 0.0f, 0.0f});

    // Hull joint.
    out.joint[J_Hull].origin = hullPos;
    out.joint[J_Hull].basis  = hullBasis;

    // Hull roll/pitch (world) from the basis columns (local axes in world).
    Vec3 lx = { hullBasis.m[0][0], hullBasis.m[1][0], hullBasis.m[2][0] };  // local X in world
    Vec3 lz = { hullBasis.m[0][2], hullBasis.m[1][2], hullBasis.m[2][2] };  // local Z in world
    const float hullRoll  = std::atan2(-lx.y, lx.x);  // roll about forward Z
    const float hullPitch = std::atan2(lz.y, lz.z);   // pitch about lateral X (nose up +)

    // Hips: rest position on the hull, with a low-pass vertical lag (shock
    // absorber) plus a squat under throttle — the rider drops low into the
    // acceleration, bending the knees (the leg IK reaches the planted feet from
    // the lowered hips).
    Vec3 hipsRest = vadd(hullPos, m3mulv(hullBasis, HIPS_LOCAL));
    const float lag = f.filteredHullY - hullPos.y;
    Vec3 hipsPos = hipsRest;
    hipsPos.y += lag;
    const float squat = throttle * SQUAT_DEPTH;
    hipsPos = vadd(hipsPos, m3mulv(hullBasis, Vec3{0.0f, -squat, 0.0f}));
    out.joint[J_Hips].origin = hipsPos;
    out.joint[J_Hips].basis  = hullBasis;  // pelvis aligned with the hull

    // Torso: counterbalances the ACTUAL hull roll (keeps the upper body steadier
    // than the bank — the rider shifts weight to stay upright), plus a modest
    // intentional lean into the steer, and leans back under throttle.
    const float leanRoll  = -hullRoll * ROLL_COUNTER - steer * MAX_LEAN * STEER_LEAN_FRAC;
    const float leanPitch = -throttle * MAX_PITCHBACK;
    Mat3 torsoBasis = m3mul(hullBasis,
        m3mul(mat3AxisAngle(Vec3{0, 0, 1}, leanRoll),
              mat3AxisAngle(Vec3{1, 0, 0}, leanPitch)));
    out.joint[J_Torso].origin = hipsPos;
    out.joint[J_Torso].basis  = torsoBasis;

    // Shoulders.
    Vec3 shoulderLPos = vadd(hipsPos, m3mulv(torsoBasis, SHOULDER_L));
    Vec3 shoulderRPos = vadd(hipsPos, m3mulv(torsoBasis, SHOULDER_R));

    // Head: vestibular counter-rotation toward world-up (not hull-up).
    const float headRoll  = -(hullRoll  + leanRoll)  * HEAD_COUNTER;
    const float headPitch = -(hullPitch + leanPitch) * HEAD_COUNTER;
    Mat3 headBasis = m3mul(torsoBasis,
        m3mul(mat3AxisAngle(Vec3{0, 0, 1}, headRoll),
              mat3AxisAngle(Vec3{1, 0, 0}, headPitch)));
    Vec3 headPos = vadd(hipsPos, m3mulv(torsoBasis, NECK_LOCAL));
    out.joint[J_Head].origin = headPos;
    out.joint[J_Head].basis  = headBasis;

    // Grips: rotate with steer about the local vertical (Y) steering column.
    Mat3 gripBasis = m3mul(hullBasis, mat3AxisAngle(Vec3{0, 1, 0}, steer * STEER_MAX));
    Vec3 gripCenter = vadd(hullPos, m3mulv(hullBasis, GRIP_CENTER));
    out.joint[J_Grips].origin = gripCenter;
    out.joint[J_Grips].basis  = gripBasis;

    // IK targets (world).
    Vec3 handLTarget = vadd(gripCenter, m3mulv(gripBasis, GRIP_L));
    Vec3 handRTarget = vadd(gripCenter, m3mulv(gripBasis, GRIP_R));
    Vec3 footLTarget = vadd(hullPos, m3mulv(hullBasis, FOOT_L));
    Vec3 footRTarget = vadd(hullPos, m3mulv(hullBasis, FOOT_R));

    // Leg roots.
    Vec3 hipLPos = vadd(hipsPos, m3mulv(hullBasis, LEGR_ROOT_L));
    Vec3 hipRPos = vadd(hipsPos, m3mulv(hullBasis, LEGR_ROOT_R));
    out.joint[J_HipL].origin = hipLPos; out.joint[J_HipL].basis = hullBasis;
    out.joint[J_HipR].origin = hipRPos; out.joint[J_HipR].basis = hullBasis;

    // Arms: two-bone IK shoulder -> hand. bendHint = world up, bendSign +1 =>
    // elbow drops (gravity) in the sagittal plane.
    Vec3 elbowL = twoBoneIK(shoulderLPos, handLTarget, ARM_UPPER, ARM_LOWER,
                            Vec3{0.0f, 1.0f, 0.0f}, +1.0f);
    Vec3 elbowR = twoBoneIK(shoulderRPos, handRTarget, ARM_UPPER, ARM_LOWER,
                            Vec3{0.0f, 1.0f, 0.0f}, +1.0f);
    out.joint[J_ShoulderL].origin = shoulderLPos;
    out.joint[J_ShoulderR].origin = shoulderRPos;
    out.joint[J_ShoulderL].basis = boneBasis(vsub(elbowL, shoulderLPos), Vec3{0, 1, 0});
    out.joint[J_ShoulderR].basis = boneBasis(vsub(elbowR, shoulderRPos), Vec3{0, 1, 0});
    out.joint[J_ElbowL].origin = elbowL;
    out.joint[J_ElbowR].origin = elbowR;
    out.joint[J_ElbowL].basis = boneBasis(vsub(handLTarget, elbowL), Vec3{0, 1, 0});
    out.joint[J_ElbowR].basis = boneBasis(vsub(handRTarget, elbowR), Vec3{0, 1, 0});
    out.joint[J_HandL].origin = handLTarget; out.joint[J_HandL].basis = gripBasis;
    out.joint[J_HandR].origin = handRTarget; out.joint[J_HandR].basis = gripBasis;

    // Legs: two-bone IK hip -> foot. bendHint = world up, bendSign -1 => knee
    // rises forward (+Z), the seated posture.
    Vec3 kneeL = twoBoneIK(hipLPos, footLTarget, LEG_UPPER, LEG_LOWER,
                           Vec3{0.0f, 1.0f, 0.0f}, -1.0f);
    Vec3 kneeR = twoBoneIK(hipRPos, footRTarget, LEG_UPPER, LEG_LOWER,
                           Vec3{0.0f, 1.0f, 0.0f}, -1.0f);
    out.joint[J_KneeL].origin = kneeL;
    out.joint[J_KneeR].origin = kneeR;
    out.joint[J_HipL].basis = boneBasis(vsub(kneeL, hipLPos), Vec3{0, 1, 0});
    out.joint[J_HipR].basis = boneBasis(vsub(kneeR, hipRPos), Vec3{0, 1, 0});
    out.joint[J_KneeL].basis = boneBasis(vsub(footLTarget, kneeL), Vec3{0, 1, 0});
    out.joint[J_KneeR].basis = boneBasis(vsub(footRTarget, kneeR), Vec3{0, 1, 0});
    out.joint[J_FootL].origin = footLTarget; out.joint[J_FootL].basis = hullBasis;
    out.joint[J_FootR].origin = footRTarget; out.joint[J_FootR].basis = hullBasis;

    // Reported joint angles (radians).
    const Vec3 worldDown = { 0.0f, -1.0f, 0.0f };
    const Vec3 torsoDown = m3mulv(torsoBasis, Vec3{0.0f, -1.0f, 0.0f});
    out.angles[JA_SpineRoll]  = leanRoll;
    out.angles[JA_SpinePitch] = leanPitch;
    out.angles[JA_HipL]       = angleBetween(vsub(kneeL, hipLPos), worldDown);
    out.angles[JA_HipR]       = angleBetween(vsub(kneeR, hipRPos), worldDown);
    out.angles[JA_KneeL]      = angleBetween(vsub(kneeL, hipLPos), vsub(footLTarget, kneeL));
    out.angles[JA_KneeR]      = angleBetween(vsub(kneeR, hipRPos), vsub(footRTarget, kneeR));
    out.angles[JA_ElbowL]     = angleBetween(vsub(elbowL, shoulderLPos), vsub(handLTarget, elbowL));
    out.angles[JA_ElbowR]     = angleBetween(vsub(elbowR, shoulderRPos), vsub(handRTarget, elbowR));
    out.angles[JA_ShoulderL]  = angleBetween(vsub(elbowL, shoulderLPos), torsoDown);
    out.angles[JA_ShoulderR]  = angleBetween(vsub(elbowR, shoulderRPos), torsoDown);
    out.angles[JA_NeckRoll]   = headRoll;
    out.angles[JA_NeckPitch]  = headPitch;
}

// ---------------------------------------------------------------------------
// Static rider pose: rigid copy of the rest pose attached to the hull.
// ---------------------------------------------------------------------------
void computeStaticPose(const VehicleState& s, PoseResult& out)
{
    const Mat3 hullBasis = finiteMat3(mat3FromFlat(s.hullBasis));
    const Vec3 hullPos = finiteVec({ s.hullPosition[0], s.hullPosition[1], s.hullPosition[2] },
                                   Vec3{ 0.0f, 0.0f, 0.0f });

    // Rest pose in the hull-local frame: IK pose at identity hull with a
    // neutral filter (zero lag) and zero throttle/steer.
    VehicleState rest{};
    rest.hullBasis[0] = 1.0f; rest.hullBasis[4] = 1.0f; rest.hullBasis[8] = 1.0f;
    FilterState restFilter{};   // filteredHullY = 0, hullPos.y = 0 -> zero lag
    PoseResult restPose{};
    computePose(rest, restFilter, restPose);

    // Rigidly attach every joint to the live hull frame.
    for (int j = 0; j < J_Count; ++j) {
        out.joint[j].origin = vadd(hullPos, m3mulv(hullBasis, restPose.joint[j].origin));
        out.joint[j].basis  = m3mul(hullBasis, restPose.joint[j].basis);
    }
    for (int a = 0; a < JA_Count; ++a) out.angles[a] = (a == JA_SpineRoll || a == JA_SpinePitch) ? 0.0f : restPose.angles[a];
}

}  // namespace detail
}  // namespace jetski
