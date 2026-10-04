#pragma once

#include "sim/render/Renderer.hpp"

#include <QOpenGLFunctions_3_3_Core>

#include <unordered_map>

namespace sim {

class OpenGLRenderer final : public IRenderer, protected QOpenGLFunctions_3_3_Core {
public:
    const char* backendName() const override { return "opengl"; }
    bool initialize(std::string& error) override;
    void resize(int width, int height) override;
    void shutdown() override;
    void uploadMesh(const std::string& key, const MeshData& mesh) override;
    void clearMeshes() override;
    void beginFrame(const FrameParams& frame) override;
    void submitInstances(const std::vector<RenderInstance>& instances) override;
    void submitLines(const LineBatch& lines) override;
    void endFrame() override;
    bool captureRgba(std::vector<std::uint8_t>& rgba, int& width, int& height) override;
    int lastSubmitted() const override { return submitted_; }
    int lastCulled() const override { return culled_; }
    int lastDraws() const override { return draws_; }

private:
    struct GpuMesh {
        unsigned vao = 0;
        unsigned vbo = 0;
        unsigned ebo = 0;
        int indexCount = 0;
    };

    bool compileProgram(unsigned& program, const char* vertex, const char* fragment, std::string& error);
    void drawInstances(const GpuMesh& mesh, const std::vector<float>& instances, bool blend);

    std::unordered_map<std::string, GpuMesh> meshes_;
    unsigned meshProgram_ = 0;
    unsigned lineProgram_ = 0;
    unsigned lineVao_ = 0;
    unsigned lineVbo_ = 0;
    unsigned instanceVbo_ = 0;
    FrameParams frame_;
    int submitted_ = 0;
    int culled_ = 0;
    int draws_ = 0;
    bool ready_ = false;
};

}  // namespace sim
