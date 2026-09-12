// jetski_asset.cpp
// Public API (rig_create/destroy/topology/pose) + procedural geometry.
// Pure ISO C++20. No platform headers. No global mutable state.
#include "jetski_asset.h"
#include "jetski_internal.h"

#include <vector>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace jetski {

using detail::Vec3;
using detail::safeNormalize;

// ---------------------------------------------------------------------------
// Rest-pose dimensions shared with the pose solver (jetski_ik.cpp).
// Hull-local frame: origin at the waterline center, +Z forward, +Y up, +X right.
// ---------------------------------------------------------------------------
namespace {

constexpr float ARM_UPPER = 0.37f, ARM_LOWER = 0.35f;  // match jetski_ik.cpp
constexpr float LEG_UPPER = 0.42f, LEG_LOWER = 0.42f;  // match jetski_ik.cpp

// Hull cross-section stations, stern (-Z) to bow (+Z).
struct Station { float z, halfW, depth, deckY; };
constexpr int kNS = 11;
const Station kStations[kNS] = {
    { -1.50f, 0.42f, 0.45f, 0.18f },
    { -1.20f, 0.50f, 0.50f, 0.20f },
    { -0.90f, 0.56f, 0.52f, 0.22f },
    { -0.60f, 0.60f, 0.50f, 0.22f },
    { -0.30f, 0.60f, 0.48f, 0.22f },
    {  0.00f, 0.58f, 0.46f, 0.24f },
    {  0.30f, 0.54f, 0.42f, 0.26f },
    {  0.60f, 0.48f, 0.36f, 0.30f },
    {  0.90f, 0.40f, 0.30f, 0.34f },
    {  1.20f, 0.30f, 0.22f, 0.40f },
    {  1.50f, 0.06f, 0.10f, 0.50f },
};

// Vertex colors (albedo 0..1).
const Vec3 COL_HULL      = { 0.05f, 0.26f, 0.32f };
const Vec3 COL_DECK      = { 0.10f, 0.36f, 0.42f };
const Vec3 COL_SEAT      = { 0.16f, 0.16f, 0.19f };
const Vec3 COL_HANDLEBAR = { 0.22f, 0.22f, 0.24f };
const Vec3 COL_GRIP      = { 0.06f, 0.06f, 0.06f };
const Vec3 COL_WETSUIT   = { 0.10f, 0.16f, 0.38f };
const Vec3 COL_ACCENT    = { 0.85f, 0.30f, 0.10f };
const Vec3 COL_SKIN      = { 0.78f, 0.58f, 0.46f };
const Vec3 COL_VISOR     = { 0.45f, 0.72f, 0.90f };
const Vec3 COL_BOOT      = { 0.09f, 0.09f, 0.10f };
const Vec3 COL_GLOVE     = { 0.08f, 0.08f, 0.09f };

// ---------------------------------------------------------------------------
// Primitive builders. Each appends to v (vertices, in the part's local frame)
// and idx (local indices, 0-based within the part). Normals are unit + outward.
// ---------------------------------------------------------------------------
void buildBox(const Vec3& c, const Vec3& h, const Vec3& col,
              std::vector<AssetVertex>& v, std::vector<uint32_t>& idx)
{
    const float x0 = c.x - h.x, x1 = c.x + h.x;
    const float y0 = c.y - h.y, y1 = c.y + h.y;
    const float z0 = c.z - h.z, z1 = c.z + h.z;
    struct Face { Vec3 n; Vec3 p[4]; };
    const Face faces[6] = {
        { { 1, 0, 0}, { { x1, y0, z0}, { x1, y1, z0}, { x1, y1, z1}, { x1, y0, z1} } },
        { {-1, 0, 0}, { { x0, y0, z1}, { x0, y1, z1}, { x0, y1, z0}, { x0, y0, z0} } },
        { { 0, 1, 0}, { { x0, y1, z1}, { x1, y1, z1}, { x1, y1, z0}, { x0, y1, z0} } },
        { { 0,-1, 0}, { { x0, y0, z0}, { x1, y0, z0}, { x1, y0, z1}, { x0, y0, z1} } },
        { { 0, 0, 1}, { { x1, y0, z1}, { x1, y1, z1}, { x0, y1, z1}, { x0, y0, z1} } },
        { { 0, 0,-1}, { { x0, y0, z0}, { x0, y1, z0}, { x1, y1, z0}, { x1, y0, z0} } },
    };
    for (int fi = 0; fi < 6; ++fi) {
        const uint32_t base = (uint32_t)v.size();
        const Vec3 n = safeNormalize(faces[fi].n);
        for (int i = 0; i < 4; ++i) {
            AssetVertex av{};
            av.position[0] = faces[fi].p[i].x; av.position[1] = faces[fi].p[i].y; av.position[2] = faces[fi].p[i].z;
            av.normal[0] = n.x; av.normal[1] = n.y; av.normal[2] = n.z;
            av.color[0] = col.x; av.color[1] = col.y; av.color[2] = col.z;
            v.push_back(av);
        }
        idx.push_back(base + 0); idx.push_back(base + 1); idx.push_back(base + 2);
        idx.push_back(base + 0); idx.push_back(base + 2); idx.push_back(base + 3);
    }
}

void buildCylinder(const Vec3& b, const Vec3& t, float r, const Vec3& col, int nSides,
                   std::vector<AssetVertex>& v, std::vector<uint32_t>& idx)
{
    Vec3 axis = safeNormalize(detail::vsub(t, b), Vec3{0, 1, 0});
    Vec3 up = (std::fabs(axis.y) > 0.9f) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 x = safeNormalize(detail::vcross(up, axis), Vec3{1, 0, 0});
    Vec3 y = detail::vcross(axis, x);
    const float kTwoPi = 6.283185307179586f;
    const uint32_t ring0 = (uint32_t)v.size();
    for (int i = 0; i < nSides; ++i) {
        const float a = kTwoPi * (float)i / (float)nSides;
        const Vec3 dir = detail::vadd(detail::vmul(x, std::cos(a)), detail::vmul(y, std::sin(a)));
        AssetVertex av{};
        av.position[0] = b.x + dir.x * r; av.position[1] = b.y + dir.y * r; av.position[2] = b.z + dir.z * r;
        av.normal[0] = dir.x; av.normal[1] = dir.y; av.normal[2] = dir.z;
        av.color[0] = col.x; av.color[1] = col.y; av.color[2] = col.z;
        v.push_back(av);
    }
    const uint32_t ring1 = (uint32_t)v.size();
    for (int i = 0; i < nSides; ++i) {
        const float a = kTwoPi * (float)i / (float)nSides;
        const Vec3 dir = detail::vadd(detail::vmul(x, std::cos(a)), detail::vmul(y, std::sin(a)));
        AssetVertex av{};
        av.position[0] = t.x + dir.x * r; av.position[1] = t.y + dir.y * r; av.position[2] = t.z + dir.z * r;
        av.normal[0] = dir.x; av.normal[1] = dir.y; av.normal[2] = dir.z;
        av.color[0] = col.x; av.color[1] = col.y; av.color[2] = col.z;
        v.push_back(av);
    }
    for (int i = 0; i < nSides; ++i) {
        const int j = (i + 1) % nSides;
        const uint32_t a = ring0 + (uint32_t)i, bb = ring0 + (uint32_t)j;
        const uint32_t c = ring1 + (uint32_t)j, d = ring1 + (uint32_t)i;
        idx.push_back(a); idx.push_back(c); idx.push_back(bb);
        idx.push_back(a); idx.push_back(d); idx.push_back(c);
    }
    // Base cap
    {
        const uint32_t center = (uint32_t)v.size();
        AssetVertex av{};
        av.position[0] = b.x; av.position[1] = b.y; av.position[2] = b.z;
        av.normal[0] = -axis.x; av.normal[1] = -axis.y; av.normal[2] = -axis.z;
        av.color[0] = col.x; av.color[1] = col.y; av.color[2] = col.z;
        v.push_back(av);
        for (int i = 0; i < nSides; ++i) {
            const int j = (i + 1) % nSides;
            idx.push_back(center); idx.push_back(ring0 + (uint32_t)j); idx.push_back(ring0 + (uint32_t)i);
        }
    }
    // Tip cap
    {
        const uint32_t center = (uint32_t)v.size();
        AssetVertex av{};
        av.position[0] = t.x; av.position[1] = t.y; av.position[2] = t.z;
        av.normal[0] = axis.x; av.normal[1] = axis.y; av.normal[2] = axis.z;
        av.color[0] = col.x; av.color[1] = col.y; av.color[2] = col.z;
        v.push_back(av);
        for (int i = 0; i < nSides; ++i) {
            const int j = (i + 1) % nSides;
            idx.push_back(center); idx.push_back(ring1 + (uint32_t)i); idx.push_back(ring1 + (uint32_t)j);
        }
    }
}

// A limb segment: cylinder from (0,0,0) to (0,0,-len) in the joint's local
// frame (local -Z points along the bone toward the next joint).
void buildLimb(float len, float r, const Vec3& col, int nSides,
               std::vector<AssetVertex>& v, std::vector<uint32_t>& idx)
{
    buildCylinder(Vec3{0, 0, 0}, Vec3{0, 0, -len}, r, col, nSides, v, idx);
}

// Lofted U hull: sides + bottom, open on top. Outward analytic normals.
void buildHull(std::vector<AssetVertex>& v, std::vector<uint32_t>& idx, const Vec3& col)
{
    constexpr int NP = 9;
    const uint32_t base = (uint32_t)v.size();
    const float kPi = 3.14159265358979323846f;
    for (int s = 0; s < kNS; ++s) {
        const Station& S = kStations[s];
        for (int i = 0; i < NP; ++i) {
            const float th = kPi * (float)i / (float)(NP - 1);
            const float x = -S.halfW * std::cos(th);
            const float y = S.deckY - (S.deckY + S.depth) * std::sin(th);
            const Vec3 n = safeNormalize(Vec3{ -(S.deckY + S.depth) * std::cos(th),
                                               -S.halfW * std::sin(th), 0.0f });
            AssetVertex av{};
            av.position[0] = x; av.position[1] = y; av.position[2] = S.z;
            av.normal[0] = n.x; av.normal[1] = n.y; av.normal[2] = n.z;
            av.color[0] = col.x; av.color[1] = col.y; av.color[2] = col.z;
            v.push_back(av);
        }
    }
    for (int s = 0; s < kNS - 1; ++s)
        for (int i = 0; i < NP - 1; ++i) {
            const uint32_t a = base + (uint32_t)(s * NP + i);
            const uint32_t b = base + (uint32_t)(s * NP + i + 1);
            const uint32_t c = base + (uint32_t)((s + 1) * NP + i);
            const uint32_t d = base + (uint32_t)((s + 1) * NP + i + 1);
            idx.push_back(a); idx.push_back(c); idx.push_back(b);
            idx.push_back(b); idx.push_back(c); idx.push_back(d);
        }
}

// Flat top deck at deckY(z), spanning the beam. Heightfield normal (up, tilting
// with the bow rise).
void buildDeck(std::vector<AssetVertex>& v, std::vector<uint32_t>& idx, const Vec3& col)
{
    constexpr int NX = 3;
    const float fx[NX] = { -1.0f, 0.0f, 1.0f };
    float zs[kNS], dy[kNS], dydz[kNS];
    for (int s = 0; s < kNS; ++s) { zs[s] = kStations[s].z; dy[s] = kStations[s].deckY; }
    for (int s = 0; s < kNS; ++s) {
        const int a = (s > 0) ? s - 1 : s, b = (s < kNS - 1) ? s + 1 : s;
        dydz[s] = (a == b) ? 0.0f : (dy[b] - dy[a]) / (zs[b] - zs[a]);
    }
    const uint32_t base = (uint32_t)v.size();
    for (int s = 0; s < kNS; ++s) {
        const float hwv = kStations[s].halfW;
        const Vec3 n = safeNormalize(Vec3{0.0f, 1.0f, -dydz[s]});
        for (int i = 0; i < NX; ++i) {
            AssetVertex av{};
            av.position[0] = fx[i] * hwv; av.position[1] = dy[s]; av.position[2] = zs[s];
            av.normal[0] = n.x; av.normal[1] = n.y; av.normal[2] = n.z;
            av.color[0] = col.x; av.color[1] = col.y; av.color[2] = col.z;
            v.push_back(av);
        }
    }
    for (int s = 0; s < kNS - 1; ++s)
        for (int i = 0; i < NX - 1; ++i) {
            const uint32_t a = base + (uint32_t)(s * NX + i);
            const uint32_t b = base + (uint32_t)(s * NX + i + 1);
            const uint32_t c = base + (uint32_t)((s + 1) * NX + i);
            const uint32_t d = base + (uint32_t)((s + 1) * NX + i + 1);
            idx.push_back(a); idx.push_back(c); idx.push_back(b);
            idx.push_back(b); idx.push_back(c); idx.push_back(d);
        }
}

}  // namespace

// The complete Rig type is defined in jetski_internal.h (namespace jetski).

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------
bool rig_create(Rig& out)
{
    out = Rig{};
    std::vector<AssetVertex>& v = out.rest;
    std::vector<uint32_t>& idx = out.indices;

    auto addPart = [&](const char* name, int joint,
                       const std::vector<AssetVertex>& pv, const std::vector<uint32_t>& pidx) {
        const uint32_t firstVertex = (uint32_t)v.size();
        const uint32_t firstIndex  = (uint32_t)idx.size();
        for (const auto& a : pv) v.push_back(a);
        for (auto i : pidx) idx.push_back(i + firstVertex);
        out.partRecs.push_back({ name, firstIndex, (uint32_t)pidx.size() });
        for (size_t k = 0; k < pv.size(); ++k) out.vertexJoint.push_back(joint);
    };

    std::vector<AssetVertex> pv;
    std::vector<uint32_t> pidx;

    // -- Watercraft (hull frame) --
    pv.clear(); pidx.clear(); buildHull(pv, pidx, COL_HULL);
    addPart("hull", detail::J_Hull, pv, pidx);

    pv.clear(); pidx.clear(); buildDeck(pv, pidx, COL_DECK);
    addPart("deck", detail::J_Hull, pv, pidx);

    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.40f, -0.15f}, Vec3{0.22f, 0.17f, 0.32f}, COL_SEAT, pv, pidx);
    addPart("seat", detail::J_Hull, pv, pidx);

    pv.clear(); pidx.clear();
    buildCylinder(Vec3{0.0f, 0.20f, 0.40f}, Vec3{0.0f, 0.92f, 0.40f}, 0.03f, COL_HANDLEBAR, 10, pv, pidx);
    addPart("handlebar", detail::J_Hull, pv, pidx);

    // Grips (grips frame: origin at the handlebar pivot, rotate with steer).
    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.20f, 0.025f, 0.025f}, COL_HANDLEBAR, pv, pidx);
    buildBox(Vec3{-0.20f, 0.0f, 0.0f}, Vec3{0.05f, 0.035f, 0.035f}, COL_GRIP, pv, pidx);
    buildBox(Vec3{ 0.20f, 0.0f, 0.0f}, Vec3{0.05f, 0.035f, 0.035f}, COL_GRIP, pv, pidx);
    addPart("grips", detail::J_Grips, pv, pidx);

    // -- Rider core --
    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.02f, 0.0f}, Vec3{0.16f, 0.10f, 0.16f}, COL_WETSUIT, pv, pidx);
    addPart("pelvis", detail::J_Hips, pv, pidx);

    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.26f, 0.06f}, Vec3{0.15f, 0.27f, 0.13f}, COL_WETSUIT, pv, pidx);
    addPart("torso", detail::J_Torso, pv, pidx);

    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.10f, 0.03f}, Vec3{0.10f, 0.11f, 0.11f}, COL_SKIN, pv, pidx);
    addPart("head", detail::J_Head, pv, pidx);

    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.11f, 0.13f}, Vec3{0.10f, 0.05f, 0.02f}, COL_VISOR, pv, pidx);
    addPart("visor", detail::J_Head, pv, pidx);

    // -- Arms (limb joints: local -Z along the bone) --
    pv.clear(); pidx.clear(); buildLimb(ARM_UPPER, 0.05f, COL_WETSUIT, 8, pv, pidx);
    addPart("armUpperL", detail::J_ShoulderL, pv, pidx);
    pv.clear(); pidx.clear(); buildLimb(ARM_UPPER, 0.05f, COL_WETSUIT, 8, pv, pidx);
    addPart("armUpperR", detail::J_ShoulderR, pv, pidx);
    pv.clear(); pidx.clear(); buildLimb(ARM_LOWER, 0.045f, COL_ACCENT, 8, pv, pidx);
    addPart("armLowerL", detail::J_ElbowL, pv, pidx);
    pv.clear(); pidx.clear(); buildLimb(ARM_LOWER, 0.045f, COL_ACCENT, 8, pv, pidx);
    addPart("armLowerR", detail::J_ElbowR, pv, pidx);

    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.0f, 0.02f}, Vec3{0.04f, 0.035f, 0.06f}, COL_GLOVE, pv, pidx);
    addPart("handL", detail::J_HandL, pv, pidx);
    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, 0.0f, 0.02f}, Vec3{0.04f, 0.035f, 0.06f}, COL_GLOVE, pv, pidx);
    addPart("handR", detail::J_HandR, pv, pidx);

    // -- Legs --
    pv.clear(); pidx.clear(); buildLimb(LEG_UPPER, 0.07f, COL_WETSUIT, 8, pv, pidx);
    addPart("legUpperL", detail::J_HipL, pv, pidx);
    pv.clear(); pidx.clear(); buildLimb(LEG_UPPER, 0.07f, COL_WETSUIT, 8, pv, pidx);
    addPart("legUpperR", detail::J_HipR, pv, pidx);
    pv.clear(); pidx.clear(); buildLimb(LEG_LOWER, 0.06f, COL_ACCENT, 8, pv, pidx);
    addPart("legLowerL", detail::J_KneeL, pv, pidx);
    pv.clear(); pidx.clear(); buildLimb(LEG_LOWER, 0.06f, COL_ACCENT, 8, pv, pidx);
    addPart("legLowerR", detail::J_KneeR, pv, pidx);

    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, -0.03f, 0.06f}, Vec3{0.05f, 0.04f, 0.13f}, COL_BOOT, pv, pidx);
    addPart("footL", detail::J_FootL, pv, pidx);
    pv.clear(); pidx.clear();
    buildBox(Vec3{0.0f, -0.03f, 0.06f}, Vec3{0.05f, 0.04f, 0.13f}, COL_BOOT, pv, pidx);
    addPart("footR", detail::J_FootR, pv, pidx);

    // -- Topology --
    out.topo.indices    = idx.data();
    out.topo.indexCount = idx.size();
    out.topo.vertexCount = v.size();
    out.topo.parts      = out.partRecs.data();
    out.topo.partCount  = out.partRecs.size();
    return true;
}

void rig_destroy(Rig& r)
{
    r = Rig{};
}

const MeshTopology& rig_topology(const Rig& r)
{
    return r.topo;
}

void rig_pose(const Rig& r, const VehicleState& s, std::span<AssetVertex> out)
{
    // r.filter is mutable, so this is well-defined through a const Rig&.
    detail::advanceFilter(r.filter, s.time, s.hullPosition[1]);
    detail::PoseResult pose{};
    detail::computePose(s, r.filter, pose);

    const std::size_t n = (out.size() < r.vertexJoint.size()) ? out.size() : r.vertexJoint.size();
    for (std::size_t i = 0; i < n; ++i) {
        const detail::JointXf& xf = pose.joint[r.vertexJoint[i]];
        const AssetVertex& rv = r.rest[i];
        AssetVertex& ov = out[i];
        const Vec3 lp{ rv.position[0], rv.position[1], rv.position[2] };
        const Vec3 wp = detail::vadd(xf.origin, detail::m3mulv(xf.basis, lp));
        ov.position[0] = wp.x; ov.position[1] = wp.y; ov.position[2] = wp.z;
        const Vec3 ln{ rv.normal[0], rv.normal[1], rv.normal[2] };
        const Vec3 wn = detail::safeNormalize(detail::m3mulv(xf.basis, ln), ln);
        ov.normal[0] = wn.x; ov.normal[1] = wn.y; ov.normal[2] = wn.z;
        ov.color[0] = rv.color[0]; ov.color[1] = rv.color[1]; ov.color[2] = rv.color[2];
    }
}

void rig_pose_static(const Rig& r, const VehicleState& s, std::span<AssetVertex> out)
{
    // Static rider: rigid rest pose attached to the hull. Does NOT advance the
    // hip filter, so switching back to rig_pose resumes without a jump.
    detail::PoseResult pose{};
    detail::computeStaticPose(s, pose);

    const std::size_t n = (out.size() < r.vertexJoint.size()) ? out.size() : r.vertexJoint.size();
    for (std::size_t i = 0; i < n; ++i) {
        const detail::JointXf& xf = pose.joint[r.vertexJoint[i]];
        const AssetVertex& rv = r.rest[i];
        AssetVertex& ov = out[i];
        const Vec3 lp{ rv.position[0], rv.position[1], rv.position[2] };
        const Vec3 wp = detail::vadd(xf.origin, detail::m3mulv(xf.basis, lp));
        ov.position[0] = wp.x; ov.position[1] = wp.y; ov.position[2] = wp.z;
        const Vec3 ln{ rv.normal[0], rv.normal[1], rv.normal[2] };
        const Vec3 wn = detail::safeNormalize(detail::m3mulv(xf.basis, ln), ln);
        ov.normal[0] = wn.x; ov.normal[1] = wn.y; ov.normal[2] = wn.z;
        ov.color[0] = rv.color[0]; ov.color[1] = rv.color[1]; ov.color[2] = rv.color[2];
    }
}

// Debug only (test harness). Reads the Rig's current filter state (as advanced
// by the most recent rig_pose call) without advancing it. Defined in
// namespace detail to match the declarations in jetski_internal.h.
namespace detail {

void rig_debug_jointAngles(const Rig& r, const VehicleState& s, float* out)
{
    PoseResult pose{};
    computePose(s, r.filter, pose);
    for (int i = 0; i < JA_Count; ++i) out[i] = pose.angles[i];
}

void rig_debug_jointPositions(const Rig& r, const VehicleState& s, float* out)
{
    PoseResult pose{};
    computePose(s, r.filter, pose);
    for (int j = 0; j < J_Count; ++j) {
        out[j * 3 + 0] = pose.joint[j].origin.x;
        out[j * 3 + 1] = pose.joint[j].origin.y;
        out[j * 3 + 2] = pose.joint[j].origin.z;
    }
}

}  // namespace detail

}  // namespace jetski
