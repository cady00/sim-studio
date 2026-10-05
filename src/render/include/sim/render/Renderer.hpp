#pragma once

#include "sim/core/Math.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/core/Pick.hpp"
#include "sim/core/Scene.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {

// Renderer-agnostic submission types. The widget owns input and the camera.
// A backend only uploads meshes, draws instances and lines, and can read the
// color buffer. OpenGL is the backend that ships; a Vulkan or D3D12 class
// implements the same interface without changing this header.

struct RenderMaterial {
    Vec3 color{0.7, 0.7, 0.7};
    float highlight = 0;
    // 1 is opaque. Below 1 the backend blends and skips the depth write.
    float opacity = 1;
    float metallic = 0;
    float roughness = 1;
};

struct RenderInstance {
    std::string meshKey;
    Mat4 world = Mat4::identity();
    RenderMaterial material;
    Aabb worldBounds;
    // Distance from the camera, millimeters. selectLodLevel uses this.
    float distanceMm = 0;
    int lodLevels = 1;
    int triangles = 0;
};

struct LineVertex {
    float x = 0;
    float y = 0;
    float z = 0;
    float r = 1;
    float g = 1;
    float b = 1;
};

struct LineBatch {
    std::vector<LineVertex> vertices;
    float width = 1;
    bool depthTest = true;
};

struct FrameParams {
    Mat4 viewProjection = Mat4::identity();
    Mat4 view = Mat4::identity();
    Mat4 projection = Mat4::identity();
    Vec3 eye;
    int width = 1;
    int height = 1;
    // When true, submitInstances drops boxes that sit fully outside the clip volume.
    bool frustumCull = true;
    Vec3 clearColor{0.10, 0.11, 0.14};
    // 0 leaves the key light unshadowed. 1 blacks out the key term in shadow. Fill and rim stay.
    float shadowStrength = 0.72f;
    // 0 disables the screen-space occlusion mix. 1 applies the full occlusion term.
    float aoStrength = 0.50f;
};

// Highest detail is 0. Coarse levels are for distant repeated parts on a long line.
inline int selectLodLevel(float distanceMm, int levels) {
    if (levels <= 1) {
        return 0;
    }
    if (distanceMm > 8000.0f) {
        return levels - 1;
    }
    if (distanceMm > 3000.0f) {
        return levels > 2 ? 1 : levels - 1;
    }
    return 0;
}

// True when the box overlaps the clip volume of a column-major view-projection matrix.
inline bool intersectsClipFrustum(const Mat4& viewProjection, const Aabb& box) {
    if (!box.valid) {
        return true;
    }
    const Vec3 corners[8] = {
        {box.min.x, box.min.y, box.min.z}, {box.max.x, box.min.y, box.min.z},
        {box.min.x, box.max.y, box.min.z}, {box.max.x, box.max.y, box.min.z},
        {box.min.x, box.min.y, box.max.z}, {box.max.x, box.min.y, box.max.z},
        {box.min.x, box.max.y, box.max.z}, {box.max.x, box.max.y, box.max.z},
    };
    int outside[6] = {};
    int counted = 0;
    for (const Vec3& corner : corners) {
        const Vec4 clip = transformPoint4(viewProjection, Vec4{corner.x, corner.y, corner.z, 1});
        if (std::abs(clip.w) < 1e-8) {
            continue;
        }
        ++counted;
        if (clip.x < -clip.w) {
            ++outside[0];
        }
        if (clip.x > clip.w) {
            ++outside[1];
        }
        if (clip.y < -clip.w) {
            ++outside[2];
        }
        if (clip.y > clip.w) {
            ++outside[3];
        }
        if (clip.z < -clip.w) {
            ++outside[4];
        }
        if (clip.z > clip.w) {
            ++outside[5];
        }
    }
    if (counted == 0) {
        return true;
    }
    for (int count : outside) {
        if (count == counted) {
            return false;
        }
    }
    return true;
}

// Low-resolution depth of already accepted bounds. A later box is dropped when its
// screen rectangle sits entirely behind that depth. The test uses the box, so a
// thin gap inside the silhouette can still be marked hidden.
struct OcclusionBuffer {
    int width = 1;
    int height = 1;
    std::vector<float> depth;

    void reset(int w, int h) {
        width = std::max(w, 1);
        height = std::max(h, 1);
        depth.assign(static_cast<std::size_t>(width * height), 1.0f);
    }
};

struct ScreenRect {
    bool valid = false;
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
    float nearest = 1.0f;
};

inline bool projectBounds(const Mat4& viewProjection, const Aabb& box, int width, int height, ScreenRect& rect) {
    rect = {};
    if (!box.valid || width <= 0 || height <= 0) {
        return false;
    }
    const Vec3 corners[8] = {
        {box.min.x, box.min.y, box.min.z}, {box.max.x, box.min.y, box.min.z},
        {box.min.x, box.max.y, box.min.z}, {box.max.x, box.max.y, box.min.z},
        {box.min.x, box.min.y, box.max.z}, {box.max.x, box.min.y, box.max.z},
        {box.min.x, box.max.y, box.max.z}, {box.max.x, box.max.y, box.max.z},
    };
    float minX = 1.0e9f;
    float minY = 1.0e9f;
    float maxX = -1.0e9f;
    float maxY = -1.0e9f;
    float nearest = 1.0f;
    int accepted = 0;
    for (const Vec3& corner : corners) {
        const Vec4 clip = transformPoint4(viewProjection, Vec4{corner.x, corner.y, corner.z, 1});
        if (clip.w <= 1e-4) {
            continue;
        }
        const float inv = static_cast<float>(1.0 / clip.w);
        const float ndcX = static_cast<float>(clip.x) * inv;
        const float ndcY = static_cast<float>(clip.y) * inv;
        const float ndcZ = static_cast<float>(clip.z) * inv;
        minX = std::min(minX, (ndcX * 0.5f + 0.5f) * static_cast<float>(width));
        maxX = std::max(maxX, (ndcX * 0.5f + 0.5f) * static_cast<float>(width));
        minY = std::min(minY, (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(height));
        maxY = std::max(maxY, (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(height));
        nearest = std::min(nearest, ndcZ);
        ++accepted;
    }
    if (accepted == 0 || maxX < 0 || maxY < 0 || minX >= width || minY >= height) {
        return false;
    }
    rect.valid = true;
    rect.x0 = std::clamp(static_cast<int>(std::floor(minX)), 0, width - 1);
    rect.y0 = std::clamp(static_cast<int>(std::floor(minY)), 0, height - 1);
    rect.x1 = std::clamp(static_cast<int>(std::ceil(maxX)), 0, width - 1);
    rect.y1 = std::clamp(static_cast<int>(std::ceil(maxY)), 0, height - 1);
    rect.nearest = nearest;
    return true;
}

inline bool hiddenByOcclusion(const OcclusionBuffer& buffer, const ScreenRect& rect) {
    if (!rect.valid || buffer.depth.empty()) {
        return false;
    }
    // Neighbors on the same surface sit within a few hundredths of NDC depth.
    // A box is hidden only when every pixel is already covered by something clearly closer.
    constexpr float kDepthGap = 0.05f;
    for (int y = rect.y0; y <= rect.y1; ++y) {
        for (int x = rect.x0; x <= rect.x1; ++x) {
            if (buffer.depth[static_cast<std::size_t>(y * buffer.width + x)] > rect.nearest - kDepthGap) {
                return false;
            }
        }
    }
    return true;
}

inline void rasterizeOccluder(OcclusionBuffer& buffer, const ScreenRect& rect) {
    if (!rect.valid) {
        return;
    }
    // The stored depth is the AABB rectangle, which is larger than the mesh.
    // Inset by one pixel so a grazing neighbor is not covered by that extra rim.
    const int x0 = rect.x0 + 1;
    const int y0 = rect.y0 + 1;
    const int x1 = rect.x1 - 1;
    const int y1 = rect.y1 - 1;
    if (x0 > x1 || y0 > y1) {
        return;
    }
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            float& depth = buffer.depth[static_cast<std::size_t>(y * buffer.width + x)];
            depth = std::min(depth, rect.nearest);
        }
    }
}

// Compute-shader source for frustum visibility. A 4.3 context can compile it.
// The software path in cullBounds is the one the viewport runs on every frame.
inline const char* gpuFrustumCullingShader() {
    return R"(#version 430
layout(local_size_x = 64) in;
layout(std430, binding = 0) readonly buffer Bounds { float b[]; };
layout(std430, binding = 1) writeonly buffer Visible { uint v[]; };
uniform mat4 uViewProj;
uniform uint uCount;
void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= uCount) return;
    uint base = i * 6u;
    vec3 bmin = vec3(b[base], b[base + 1u], b[base + 2u]);
    vec3 bmax = vec3(b[base + 3u], b[base + 4u], b[base + 5u]);
    uint outside[6] = uint[6](0u, 0u, 0u, 0u, 0u, 0u);
    uint counted = 0u;
    for (int c = 0; c < 8; ++c) {
        vec3 p = vec3((c & 1) == 0 ? bmin.x : bmax.x, (c & 2) == 0 ? bmin.y : bmax.y, (c & 4) == 0 ? bmin.z : bmax.z);
        vec4 clip = uViewProj * vec4(p, 1.0);
        if (abs(clip.w) < 1e-6) continue;
        counted += 1u;
        if (clip.x < -clip.w) outside[0] += 1u;
        if (clip.x > clip.w) outside[1] += 1u;
        if (clip.y < -clip.w) outside[2] += 1u;
        if (clip.y > clip.w) outside[3] += 1u;
        if (clip.z < -clip.w) outside[4] += 1u;
        if (clip.z > clip.w) outside[5] += 1u;
    }
    uint dropped = 0u;
    if (counted > 0u) {
        for (int p = 0; p < 6; ++p) if (outside[p] == counted) dropped = 1u;
    }
    v[i] = dropped == 0u ? 1u : 0u;
}
)";
}

inline Aabb transformedBounds(const Aabb& local, const Mat4& world) {
    Aabb out;
    if (!local.valid) {
        return out;
    }
    const Vec3 corners[8] = {
        {local.min.x, local.min.y, local.min.z}, {local.max.x, local.min.y, local.min.z},
        {local.min.x, local.max.y, local.min.z}, {local.max.x, local.max.y, local.min.z},
        {local.min.x, local.min.y, local.max.z}, {local.max.x, local.min.y, local.max.z},
        {local.min.x, local.max.y, local.max.z}, {local.max.x, local.max.y, local.max.z},
    };
    for (const Vec3& corner : corners) {
        out.expand(transformPoint(world, corner));
    }
    return out;
}

class IRenderer {
public:
    virtual ~IRenderer() = default;
    virtual const char* backendName() const = 0;
    virtual bool initialize(std::string& error) = 0;
    virtual void resize(int width, int height) = 0;
    virtual void shutdown() = 0;

    virtual void uploadMesh(const std::string& key, const MeshData& mesh) = 0;
    virtual void clearMeshes() = 0;

    virtual void beginFrame(const FrameParams& frame) = 0;
    // Groups by mesh key. Batches larger than one instance are drawn with instancing
    // on backends that support it. Culled instances are counted and skipped.
    virtual void submitInstances(const std::vector<RenderInstance>& instances) = 0;
    virtual void submitLines(const LineBatch& lines) = 0;
    virtual void endFrame() = 0;

    // RGBA8, tightly packed, bottom-left origin. Empty on failure.
    virtual bool captureRgba(std::vector<std::uint8_t>& rgba, int& width, int& height) = 0;

    virtual int lastSubmitted() const = 0;
    virtual int lastCulled() const = 0;
    virtual int lastDraws() const { return 0; }
    // -1 when this context cannot compile a compute shader. Otherwise the number kept.
    virtual int gpuFrustumCull(const Mat4&, const std::vector<Aabb>&) { return -1; }

    // CPU ray pick is the default. A GPU backend may override this later.
    virtual std::optional<PickHit> pick(const Scene& scene, const Ray& ray) const { return pickScene(scene, ray); }
};

std::unique_ptr<IRenderer> createRenderer();

}  // namespace sim
