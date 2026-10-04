#pragma once

#include "sim/core/Math.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/core/Pick.hpp"
#include "sim/core/Scene.hpp"

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
    Vec3 eye;
    int width = 1;
    int height = 1;
    // When true, submitInstances drops boxes that sit fully outside the clip volume.
    bool frustumCull = true;
    Vec3 clearColor{0.10, 0.11, 0.14};
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

    // CPU ray pick is the default. A GPU backend may override this later.
    virtual std::optional<PickHit> pick(const Scene& scene, const Ray& ray) const { return pickScene(scene, ray); }
};

std::unique_ptr<IRenderer> createRenderer();

}  // namespace sim
