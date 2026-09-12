// jetski_asset.h
// Pure ISO C++20. No platform headers, no dependencies.
#pragma once
#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>

namespace jetski {

// 36 bytes. The renderer will adapt its vertex layout to THIS, not vice versa.
struct AssetVertex {
    float position[3];  // meters
    float normal[3];    // unit length, outward-facing
    float color[3];     // albedo 0..1; vertex colors, no textures ever
};

struct MeshTopology {
    const uint32_t* indices;   // immutable after rig_create
    std::size_t     indexCount;
    std::size_t     vertexCount;  // immutable; posing never changes counts
    struct Part { const char* name; uint32_t firstIndex; uint32_t indexCount; };
    const Part* parts; std::size_t partCount;  // e.g. hull, seat, rider, visor
};

struct VehicleState {
    float time;              // seconds
    float throttle;          // 0..1
    float steer;             // -1 left .. +1 right
    float hullPosition[3];   // world space, meters
    float hullBasis[9];      // row-major 3x3 rotation, world <- local
    float hullVelocity[3];
    float hullAngularVel[3];
    float supportHeight;     // wave height under hull, meters
    float supportNormal[3];  // wave normal under hull
};

struct Rig;  // opaque

bool  rig_create(Rig& out);                 // builds topology + scratch buffers
void  rig_destroy(Rig&);
const MeshTopology& rig_topology(const Rig&);

// Writes posed vertices into caller memory. out.size() must equal
// topology.vertexCount. No allocations, no exceptions, deterministic.
void  rig_pose(const Rig&, const VehicleState&, std::span<AssetVertex> out);

// Static-rider pose: the rider is a rigid copy of its rest pose attached to
// the hull (no IK, no hip filter, no lean/squat/steer). Same contract as
// rig_pose; does NOT advance the hip filter, so switching back to rig_pose
// resumes smoothly.
void  rig_pose_static(const Rig&, const VehicleState&, std::span<AssetVertex> out);

// Debug/inspection only (used by the test harness and by humans):
bool  export_obj(const MeshTopology&, std::span<const AssetVertex>, const char* path);

}  // namespace jetski
