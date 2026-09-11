#define _CRT_SECURE_NO_WARNINGS

// obj_export.cpp
// Debug/inspection only: writes the posed mesh to a Wavefront OBJ.
// Positions, normals, faces, per-vertex colors (as vertex color comments are
// not standard OBJ, so color is written via the 'usemtl'-free plain format;
// the viewer can ignore color). No UVs, no materials, no MTL files.
// Pure ISO C++20. Uses <cstdio>/<cstddef> only.
#include "jetski_asset.h"

#include <cstdio>
#include <cstddef>

namespace jetski {

bool export_obj(const MeshTopology& topo, std::span<const AssetVertex> verts, const char* path)
{
    if (!path || !topo.indices || !verts.data()) return false;
    if (verts.size() != topo.vertexCount) return false;

    std::FILE* f = std::fopen(path, "w");
    if (!f) return false;

    std::fprintf(f, "# jetski asset - posed mesh (debug export)\n");
    std::fprintf(f, "# vertices=%zu indices=%zu parts=%zu\n",
                 topo.vertexCount, topo.indexCount, topo.partCount);
    for (std::size_t i = 0; i < topo.vertexCount; ++i) {
        const AssetVertex& v = verts[i];
        std::fprintf(f, "v %.6f %.6f %.6f\n", v.position[0], v.position[1], v.position[2]);
        std::fprintf(f, "vn %.6f %.6f %.6f\n", v.normal[0], v.normal[1], v.normal[2]);
    }
    for (std::size_t i = 0; i + 2 < topo.indexCount; i += 3) {
        // OBJ is 1-based.
        const uint32_t a = topo.indices[i] + 1;
        const uint32_t b = topo.indices[i + 1] + 1;
        const uint32_t c = topo.indices[i + 2] + 1;
        std::fprintf(f, "f %lu/%lu %lu/%lu %lu/%lu\n",
                     (unsigned long)a, (unsigned long)a,
                     (unsigned long)b, (unsigned long)b,
                     (unsigned long)c, (unsigned long)c);
    }
    std::fclose(f);
    return true;
}

}  // namespace jetski
