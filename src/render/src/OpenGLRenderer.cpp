#include "sim/render/OpenGLRenderer.hpp"

#include <algorithm>
#include <unordered_map>

namespace sim {
namespace {

const char* kMeshVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 iM0;
layout(location = 3) in vec4 iM1;
layout(location = 4) in vec4 iM2;
layout(location = 5) in vec4 iM3;
layout(location = 6) in vec4 iColor;
uniform mat4 uViewProj;
uniform vec3 uEye;
out vec3 vNormal;
out vec3 vWorld;
out vec3 vColor;
out float vHighlight;
void main() {
    mat4 model = mat4(iM0, iM1, iM2, iM3);
    vec4 world = model * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = normalize(mat3(model) * aNormal);
    vColor = iColor.rgb;
    vHighlight = iColor.a;
    gl_Position = uViewProj * world;
}
)";

const char* kMeshFragment = R"(#version 330 core
in vec3 vNormal;
in vec3 vWorld;
in vec3 vColor;
in float vHighlight;
uniform vec3 uEye;
out vec4 fragColor;
void main() {
    vec3 n = normalize(vNormal);
    vec3 lightA = normalize(vec3(0.35, -0.45, 0.82));
    vec3 lightB = normalize(vec3(-0.55, -0.15, 0.35));
    float diffuse = max(dot(n, lightA), 0.0) * 0.72 + max(dot(n, lightB), 0.0) * 0.22;
    vec3 viewDir = normalize(uEye - vWorld);
    float spec = pow(max(dot(n, normalize(lightA + viewDir)), 0.0), 32.0) * 0.22;
    vec3 color = vColor * (0.28 + diffuse) + vec3(spec);
    if (vHighlight > 0.5) {
        color = mix(color, vec3(0.58, 0.60, 0.98), 0.42);
    }
    fragColor = vec4(color, 1.0);
}
)";

const char* kLineVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

const char* kLineFragment = R"(#version 330 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";

void toGl(const Mat4& matrix, float out[16]) {
    for (int i = 0; i < 16; ++i) {
        out[i] = static_cast<float>(matrix.m[i]);
    }
}

}  // namespace

std::unique_ptr<IRenderer> createRenderer() { return std::make_unique<OpenGLRenderer>(); }

bool OpenGLRenderer::compileProgram(unsigned& program, const char* vertexSource, const char* fragmentSource,
                                    std::string& error) {
    const auto compile = [&](unsigned type, const char* source) -> unsigned {
        const unsigned shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        int ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[1024];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            error = log;
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    };
    const unsigned vertex = compile(GL_VERTEX_SHADER, vertexSource);
    const unsigned fragment = compile(GL_FRAGMENT_SHADER, fragmentSource);
    if (vertex == 0 || fragment == 0) {
        return false;
    }
    program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    int ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        error = log;
        glDeleteProgram(program);
        program = 0;
        return false;
    }
    return true;
}

bool OpenGLRenderer::initialize(std::string& error) {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glClearColor(0.10f, 0.11f, 0.14f, 1.0f);
    if (!compileProgram(meshProgram_, kMeshVertex, kMeshFragment, error) ||
        !compileProgram(lineProgram_, kLineVertex, kLineFragment, error)) {
        return false;
    }
    glGenVertexArrays(1, &lineVao_);
    glGenBuffers(1, &lineVbo_);
    glGenBuffers(1, &instanceVbo_);
    ready_ = true;
    return true;
}

void OpenGLRenderer::resize(int width, int height) { glViewport(0, 0, width, height); }

void OpenGLRenderer::clearMeshes() {
    for (auto& [key, mesh] : meshes_) {
        (void)key;
        glDeleteBuffers(1, &mesh.vbo);
        glDeleteBuffers(1, &mesh.ebo);
        glDeleteVertexArrays(1, &mesh.vao);
    }
    meshes_.clear();
}

void OpenGLRenderer::shutdown() {
    clearMeshes();
    if (instanceVbo_ != 0) {
        glDeleteBuffers(1, &instanceVbo_);
        instanceVbo_ = 0;
    }
    if (lineVbo_ != 0) {
        glDeleteBuffers(1, &lineVbo_);
        lineVbo_ = 0;
    }
    if (lineVao_ != 0) {
        glDeleteVertexArrays(1, &lineVao_);
        lineVao_ = 0;
    }
    if (meshProgram_ != 0) {
        glDeleteProgram(meshProgram_);
        meshProgram_ = 0;
    }
    if (lineProgram_ != 0) {
        glDeleteProgram(lineProgram_);
        lineProgram_ = 0;
    }
    ready_ = false;
}

void OpenGLRenderer::uploadMesh(const std::string& key, const MeshData& mesh) {
    if (!ready_ || mesh.empty()) {
        return;
    }
    std::vector<float> interleaved(mesh.vertexCount() * 6);
    for (std::size_t i = 0; i < mesh.vertexCount(); ++i) {
        interleaved[i * 6 + 0] = mesh.positions[i * 3 + 0];
        interleaved[i * 6 + 1] = mesh.positions[i * 3 + 1];
        interleaved[i * 6 + 2] = mesh.positions[i * 3 + 2];
        interleaved[i * 6 + 3] = mesh.normals[i * 3 + 0];
        interleaved[i * 6 + 4] = mesh.normals[i * 3 + 1];
        interleaved[i * 6 + 5] = mesh.normals[i * 3 + 2];
    }
    GpuMesh gpu;
    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);
    glGenBuffers(1, &gpu.ebo);
    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)), interleaved.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(std::uint32_t)),
                 mesh.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    gpu.indexCount = static_cast<int>(mesh.indices.size());
    glBindVertexArray(0);
    meshes_.insert_or_assign(key, gpu);
}

void OpenGLRenderer::beginFrame(const FrameParams& frame) {
    frame_ = frame;
    submitted_ = 0;
    culled_ = 0;
    glViewport(0, 0, frame.width, frame.height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRenderer::drawInstances(const GpuMesh& mesh, const std::vector<float>& instances) {
    const int count = static_cast<int>(instances.size() / 20);
    if (count <= 0) {
        return;
    }
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(instances.size() * sizeof(float)), instances.data(),
                 GL_DYNAMIC_DRAW);
    const auto stride = static_cast<int>(20 * sizeof(float));
    for (int column = 0; column < 4; ++column) {
        const unsigned location = static_cast<unsigned>(2 + column);
        glEnableVertexAttribArray(location);
        glVertexAttribPointer(location, 4, GL_FLOAT, GL_FALSE, stride,
                              reinterpret_cast<void*>(static_cast<std::size_t>(column) * 4 * sizeof(float)));
        glVertexAttribDivisor(location, 1);
    }
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(16 * sizeof(float)));
    glVertexAttribDivisor(6, 1);
    glDrawElementsInstanced(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, nullptr, count);
    for (unsigned location = 2; location <= 6; ++location) {
        glVertexAttribDivisor(location, 0);
        glDisableVertexAttribArray(location);
    }
    glBindVertexArray(0);
    submitted_ += count;
}

void OpenGLRenderer::submitInstances(const std::vector<RenderInstance>& instances) {
    if (!ready_) {
        return;
    }
    std::unordered_map<std::string, std::vector<float>> batches;
    for (const RenderInstance& instance : instances) {
        if (frame_.frustumCull && !intersectsClipFrustum(frame_.viewProjection, instance.worldBounds)) {
            ++culled_;
            continue;
        }
        const int lod = selectLodLevel(instance.distanceMm, instance.lodLevels);
        (void)lod;
        float model[16];
        toGl(instance.world, model);
        auto& batch = batches[instance.meshKey];
        batch.insert(batch.end(), model, model + 16);
        batch.push_back(static_cast<float>(instance.material.color.x));
        batch.push_back(static_cast<float>(instance.material.color.y));
        batch.push_back(static_cast<float>(instance.material.color.z));
        batch.push_back(instance.material.highlight);
    }
    float viewProj[16];
    toGl(frame_.viewProjection, viewProj);
    glUseProgram(meshProgram_);
    glUniformMatrix4fv(glGetUniformLocation(meshProgram_, "uViewProj"), 1, GL_FALSE, viewProj);
    glUniform3f(glGetUniformLocation(meshProgram_, "uEye"), static_cast<float>(frame_.eye.x),
                static_cast<float>(frame_.eye.y), static_cast<float>(frame_.eye.z));
    for (const auto& [key, batch] : batches) {
        const auto mesh = meshes_.find(key);
        if (mesh == meshes_.end()) {
            continue;
        }
        drawInstances(mesh->second, batch);
    }
}

void OpenGLRenderer::submitLines(const LineBatch& lines) {
    if (!ready_ || lines.vertices.empty() || lineProgram_ == 0) {
        return;
    }
    std::vector<float> data;
    data.reserve(lines.vertices.size() * 6);
    for (const LineVertex& vertex : lines.vertices) {
        data.push_back(vertex.x);
        data.push_back(vertex.y);
        data.push_back(vertex.z);
        data.push_back(vertex.r);
        data.push_back(vertex.g);
        data.push_back(vertex.b);
    }
    float viewProj[16];
    toGl(frame_.viewProjection, viewProj);
    if (!lines.depthTest) {
        glDisable(GL_DEPTH_TEST);
    }
    glUseProgram(lineProgram_);
    glUniformMatrix4fv(glGetUniformLocation(lineProgram_, "uViewProj"), 1, GL_FALSE, viewProj);
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data.size() * sizeof(float)), data.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glLineWidth(lines.width);
    glDrawArrays(GL_LINES, 0, static_cast<int>(lines.vertices.size()));
    glBindVertexArray(0);
    if (!lines.depthTest) {
        glEnable(GL_DEPTH_TEST);
    }
}

void OpenGLRenderer::endFrame() {}

bool OpenGLRenderer::captureRgba(std::vector<std::uint8_t>& rgba, int& width, int& height) {
    width = std::max(frame_.width, 1);
    height = std::max(frame_.height, 1);
    rgba.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    return !rgba.empty();
}

}  // namespace sim
