// jetski_test.cpp
// Standalone test harness (runnable on Linux, no D3D12/Windows).
// Scripts a 10-second sequence, asserts invariants every frame, writes OBJ
// snapshots at t=0.0/0.7/1.3 s, and prints min/max joint angles.
// Pure ISO C++20. Exits nonzero on any assertion failure.
#include "jetski_asset.h"
#include "jetski_internal.h"

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <span>

using namespace jetski;

namespace {

int g_fail = 0;
void check(bool cond, const char* msg, int line)
{
    if (!cond) {
        std::printf("  FAIL (line %d): %s\n", line, msg);
        g_fail = 1;
    }
}
#define CHECK(c) check((c), #c, __LINE__)

float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float smoothstep_(float x) { x = clampf_(x, 0.0f, 1.0f); return x * x * (3.0f - 2.0f * x); }

const float kTwoPi = 6.283185307179586f;

float steerAt(double t)
{
    // Right turn 1.0-3.0 (ramp by 1.3), ease to center by 4.5; left turn
    // 5.0-7.0, ease to center by 8.5. A full steering sweep.
    if (t < 1.0)  return 0.0f;
    if (t < 3.0)  return 0.85f * smoothstep_(float(t - 1.0) / 0.3f);
    if (t < 4.5)  return 0.85f * (1.0f - smoothstep_(float(t - 3.0) / 1.5f));
    if (t < 5.0)  return 0.0f;
    if (t < 7.0)  return -0.85f * smoothstep_(float(t - 5.0) / 0.3f);
    if (t < 8.5)  return -0.85f * (1.0f - smoothstep_(float(t - 7.0) / 1.5f));
    return 0.0f;
}

float throttleAt(double t)
{
    if (t < 2.0) return 0.0f;
    if (t < 5.0) return smoothstep_(float(t - 2.0) / 3.0f);
    return 1.0f;
}

// Builds a VehicleState for the scripted 10 s sequence.
VehicleState makeState(double t)
{
    VehicleState s{};
    s.time = float(t);
    const float steer = steerAt(t);
    const float throttle = throttleAt(t);
    const float bounceEnv = smoothstep_(float(t - 0.2) / 0.6f);  // waves ramp in

    const float heave = bounceEnv * (0.18f * std::sin(float(t) * kTwoPi * 0.8f)
                                     + 0.06f * std::sin(float(t) * kTwoPi * 0.33f + 1.0f))
                      + 0.02f * std::sin(float(t) * kTwoPi * 0.25f);
    const float roll  = -0.30f * steer
                      + bounceEnv * 0.10f * std::sin(float(t) * kTwoPi * 0.6f + 0.3f);
    const float pitch = 0.10f * throttle
                      + bounceEnv * 0.07f * std::sin(float(t) * kTwoPi * 0.5f + 0.7f);

    s.throttle = throttle;
    s.steer = steer;
    s.hullPosition[0] = 0.0f;
    s.hullPosition[1] = heave;
    s.hullPosition[2] = 0.0f;

    const float cr = std::cos(roll), sr = std::sin(roll);
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    // M = Rz(roll) * Rx(-pitch)  (nose-up = +pitch, right bank = steer>0)
    s.hullBasis[0] = cr;      s.hullBasis[1] = -sr * cp;  s.hullBasis[2] = -sr * sp;
    s.hullBasis[3] = sr;      s.hullBasis[4] =  cr * cp;  s.hullBasis[5] =  cr * sp;
    s.hullBasis[6] = 0.0f;    s.hullBasis[7] = -sp;       s.hullBasis[8] =  cp;

    s.hullVelocity[0] = 0.0f; s.hullVelocity[1] = 0.0f; s.hullVelocity[2] = 0.0f;
    s.hullAngularVel[0] = 0.0f; s.hullAngularVel[1] = 0.0f; s.hullAngularVel[2] = roll;
    s.supportHeight = heave;
    s.supportNormal[0] = 0.0f; s.supportNormal[1] = 1.0f; s.supportNormal[2] = 0.0f;
    return s;
}

const char* kJointAngleNames[detail::JA_Count] = {
    "spineRoll", "spinePitch", "hipL", "hipR", "kneeL", "kneeR",
    "elbowL", "elbowR", "shoulderL", "shoulderR", "neckRoll", "neckPitch"
};

}  // namespace

int main()
{
    std::printf("jetski_test: creating rig...\n");
    Rig rig;
    if (!rig_create(rig)) {
        std::printf("FAIL: rig_create returned false\n");
        return 1;
    }
    const MeshTopology& topo = rig_topology(rig);
    std::printf("  parts=%zu vertices=%zu indices=%zu\n",
                topo.partCount, topo.vertexCount, topo.indexCount);
    for (std::size_t p = 0; p < topo.partCount; ++p)
        std::printf("    part %-12s firstIndex=%u indexCount=%u\n",
                    topo.parts[p].name, topo.parts[p].firstIndex, topo.parts[p].indexCount);

    std::vector<AssetVertex> buf(topo.vertexCount);
    std::vector<float> angles(detail::JA_Count);
    std::vector<float> joints(detail::J_Count * 3);

    float amn[detail::JA_Count], amx[detail::JA_Count];
    for (int i = 0; i < detail::JA_Count; ++i) { amn[i] = 1e30f; amx[i] = -1e30f; }

    const double T = 10.0;
    const double dt = 1.0 / 60.0;
    int frames = 0;

    // Snapshot targets (t, label)
    struct Snap { double t; const char* label; };
    const Snap snaps[3] = {
        { 0.0, "idle" }, { 0.7, "bounce" }, { 1.3, "turn" }
    };
    bool snapDone[3] = { false, false, false };

    std::printf("jetski_test: running %.0f s @ 60 Hz...\n", T);
    for (double t = 0.0; t < T + 1e-9; t += dt, ++frames)
    {
        const VehicleState s = makeState(t);
        rig_pose(rig, s, std::span<AssetVertex>(buf.data(), buf.size()));

        // --- Invariants, every frame ---
        CHECK(buf.size() == topo.vertexCount);
        for (std::size_t i = 0; i < buf.size(); ++i) {
            for (int k = 0; k < 3; ++k) {
                CHECK(std::isfinite(buf[i].position[k]));
                CHECK(std::isfinite(buf[i].normal[k]));
                CHECK(std::isfinite(buf[i].color[k]));
            }
            const float nl = std::sqrt(buf[i].normal[0] * buf[i].normal[0]
                                     + buf[i].normal[1] * buf[i].normal[1]
                                     + buf[i].normal[2] * buf[i].normal[2]);
            CHECK(std::fabs(nl - 1.0f) < 1e-3f);
        }

        // --- Joint angle min/max ---
        detail::rig_debug_jointAngles(rig, s, angles.data());
        for (int i = 0; i < detail::JA_Count; ++i) {
            CHECK(std::isfinite(angles[i]));
            if (angles[i] < amn[i]) amn[i] = angles[i];
            if (angles[i] > amx[i]) amx[i] = angles[i];
        }

        // --- OBJ snapshots ---
        for (int k = 0; k < 3; ++k) {
            if (!snapDone[k] && std::fabs(t - snaps[k].t) < dt * 0.5) {
                char path[128];
                std::snprintf(path, sizeof(path), "jetski_t%04.2f_%s.obj", snaps[k].t, snaps[k].label);
                const bool ok = export_obj(topo, std::span<const AssetVertex>(buf.data(), buf.size()), path);
                CHECK(ok);
                if (ok) std::printf("  [t=%.2f] wrote %s\n", t, path);
                snapDone[k] = true;

                // Debug: key joint positions for an anatomy check.
                detail::rig_debug_jointPositions(rig, s, joints.data());
                auto jp = [&](int j) {
                    std::printf("    joint %-4d pos=(%+.3f, %+.3f, %+.3f)\n",
                                j, joints[j * 3], joints[j * 3 + 1], joints[j * 3 + 2]);
                };
                jp(detail::J_ShoulderL); jp(detail::J_ElbowL); jp(detail::J_HandL);
                jp(detail::J_HipL);      jp(detail::J_KneeL);  jp(detail::J_FootL);
                jp(detail::J_Head);
            }
        }
    }

    std::printf("jetski_test: %d frames processed.\n", frames);
    std::printf("\n=== Min/max joint angles (radians) over %.0f s ===\n", T);
    std::printf("%-14s %12s %12s\n", "joint", "min", "max");
    for (int i = 0; i < detail::JA_Count; ++i)
        std::printf("%-14s %12.4f %12.4f\n", kJointAngleNames[i], amn[i], amx[i]);

    rig_destroy(rig);

    if (g_fail) {
        std::printf("\njetski_test: FAILED\n");
        return 1;
    }
    std::printf("\njetski_test: PASSED\n");
    return 0;
}
