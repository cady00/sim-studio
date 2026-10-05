#include "sim/render/OpenGLRenderer.hpp"

#include <QOpenGLContext>

#include <algorithm>
#include <string>
#include <unordered_map>

#ifndef GL_COMPUTE_SHADER
#define GL_COMPUTE_SHADER 0x91B9
#endif
#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif
#ifndef GL_SHADER_STORAGE_BARRIER_BIT
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000u
#endif

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
layout(location = 7) in float iOpacity;
layout(location = 8) in vec2 iSurface;
uniform mat4 uViewProj;
uniform vec3 uEye;
out vec3 vNormal;
out vec3 vWorld;
out vec3 vColor;
out float vHighlight;
out float vOpacity;
out float vMetallic;
out float vRoughness;
void main() {
    mat4 uModel = mat4(iM0, iM1, iM2, iM3);
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    mat3 linear = mat3(uModel);
    float det = determinant(linear);
    mat3 normalMatrix = abs(det) > 1e-12 ? transpose(inverse(linear)) : linear;
    vNormal = normalMatrix * aNormal;
    vColor = iColor.rgb;
    vHighlight = iColor.a;
    vOpacity = iOpacity;
    vMetallic = iSurface.x;
    vRoughness = iSurface.y;
    gl_Position = uViewProj * world;
}
)";

const char* kMeshFragment = R"(#version 330 core
in vec3 vNormal;
in vec3 vWorld;
in vec3 vColor;
in float vHighlight;
in float vOpacity;
in float vMetallic;
in float vRoughness;
uniform vec3 uEye;
out vec4 fragColor;
void main() {
    vec3 geo = cross(dFdx(vWorld), dFdy(vWorld));
    vec3 n = dot(vNormal, vNormal) > 1e-8 ? normalize(vNormal) : normalize(geo);
    if (!gl_FrontFacing) {
        n = -n;
    }
    vec3 viewDir = normalize(uEye - vWorld);
    vec3 keyDir = normalize(vec3(0.42, -0.55, 0.72));
    vec3 fillDir = normalize(vec3(-0.78, 0.28, 0.46));
    vec3 rimDir = normalize(vec3(-0.12, 0.82, 0.28));
    float hemi = n.z * 0.5 + 0.5;
    vec3 ambient = mix(vec3(0.20, 0.21, 0.23), vec3(0.40, 0.44, 0.50), hemi);
    float ndotk = max(dot(n, keyDir), 0.0);
    float ndotf = max(dot(n, fillDir), 0.0);
    float ndotr = max(dot(n, rimDir), 0.0);
    vec3 lit = vColor * (ambient + vec3(0.62, 0.60, 0.54) * ndotk + vec3(0.16, 0.18, 0.22) * ndotf + vec3(0.07) * ndotr);
    float rough = clamp(vRoughness, 0.05, 1.0);
    float shininess = mix(128.0, 10.0, rough);
    float metal = clamp(vMetallic, 0.0, 1.0);
    vec3 halfKey = normalize(keyDir + viewDir);
    vec3 halfFill = normalize(fillDir + viewDir);
    float specKey = pow(max(dot(n, halfKey), 0.0), shininess);
    float specFill = pow(max(dot(n, halfFill), 0.0), max(shininess * 0.45, 4.0));
    vec3 specTint = mix(vec3(1.0), max(vColor, vec3(0.22)), metal);
    vec3 spec = specTint * (specKey * mix(0.42, 0.90, metal) + specFill * 0.14);
    vec3 color = lit + spec;
    float crease = smoothstep(0.18, 0.72, length(fwidth(n)));
    color *= mix(1.0, 0.58, crease);
    if (vHighlight > 0.5) {
        color = mix(color, vec3(0.239, 0.525, 0.910), 0.42);
    }
    fragColor = vec4(color, vOpacity);
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

const char* kBackgroundVertex = R"(#version 330 core
out vec2 vUv;
void main() {
    vec2 pos = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = pos;
    gl_Position = vec4(pos * 2.0 - 1.0, 0.0, 1.0);
}
)";

const char* kBackgroundFragment = R"(#version 330 core
in vec2 vUv;
uniform vec3 uClear;
out vec4 fragColor;
void main() {
    vec3 top = uClear + vec3(0.07, 0.08, 0.10);
    vec3 bottom = uClear * 0.62;
    fragColor = vec4(mix(bottom, top, smoothstep(0.0, 1.0, vUv.y)), 1.0);
}
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
        !compileProgram(lineProgram_, kLineVertex, kLineFragment, error) ||
        !compileProgram(backgroundProgram_, kBackgroundVertex, kBackgroundFragment, error)) {
        return false;
    }
    glGenVertexArrays(1, &backgroundVao_);
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
    if (backgroundVao_ != 0) {
        glDeleteVertexArrays(1, &backgroundVao_);
        backgroundVao_ = 0;
    }
    if (backgroundProgram_ != 0) {
        glDeleteProgram(backgroundProgram_);
        backgroundProgram_ = 0;
    }
    if (meshProgram_ != 0) {
        glDeleteProgram(meshProgram_);
        meshProgram_ = 0;
    }
    if (lineProgram_ != 0) {
        glDeleteProgram(lineProgram_);
        lineProgram_ = 0;
    }
    if (cullProgram_ != 0) {
        glDeleteProgram(cullProgram_);
        cullProgram_ = 0;
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
    draws_ = 0;
    glViewport(0, 0, frame.width, frame.height);
    glClearColor(static_cast<float>(frame.clearColor.x), static_cast<float>(frame.clearColor.y),
                 static_cast<float>(frame.clearColor.z), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawBackground();
}

void OpenGLRenderer::drawBackground() {
    if (backgroundProgram_ == 0 || backgroundVao_ == 0) {
        return;
    }
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glUseProgram(backgroundProgram_);
    glUniform3f(glGetUniformLocation(backgroundProgram_, "uClear"), static_cast<float>(frame_.clearColor.x),
                static_cast<float>(frame_.clearColor.y), static_cast<float>(frame_.clearColor.z));
    glBindVertexArray(backgroundVao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void OpenGLRenderer::drawInstances(const GpuMesh& mesh, const std::vector<float>& instances, bool blend) {
    const int count = static_cast<int>(instances.size() / 23);
    if (count <= 0 || meshProgram_ == 0 || instanceVbo_ == 0) {
        return;
    }
    if (blend) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(instances.size() * sizeof(float)), instances.data(), GL_DYNAMIC_DRAW);
    const int stride = 23 * static_cast<int>(sizeof(float));
    for (int column = 0; column < 4; ++column) {
        const unsigned location = static_cast<unsigned>(2 + column);
        glEnableVertexAttribArray(location);
        glVertexAttribPointer(location, 4, GL_FLOAT, GL_FALSE, stride,
                              reinterpret_cast<void*>(static_cast<std::uintptr_t>(column * 4 * sizeof(float))));
        glVertexAttribDivisor(location, 1);
    }
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(static_cast<std::uintptr_t>(16 * sizeof(float))));
    glVertexAttribDivisor(6, 1);
    glEnableVertexAttribArray(7);
    glVertexAttribPointer(7, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(static_cast<std::uintptr_t>(20 * sizeof(float))));
    glVertexAttribDivisor(7, 1);
    glEnableVertexAttribArray(8);
    glVertexAttribPointer(8, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(static_cast<std::uintptr_t>(21 * sizeof(float))));
    glVertexAttribDivisor(8, 1);
    glDrawElementsInstanced(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, nullptr, count);
    glBindVertexArray(0);
    if (blend) {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    submitted_ += count;
    ++draws_;
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
        std::string key = instance.meshKey;
        if (instance.lodLevels > 1) {
            const int lod = selectLodLevel(instance.distanceMm, instance.lodLevels);
            if (lod > 0) {
                const std::string lodKey = instance.meshKey + "#lod" + std::to_string(lod);
                if (meshes_.find(lodKey) != meshes_.end()) {
                    key = lodKey;
                }
            }
        }
        float model[16];
        toGl(instance.world, model);
        auto& batch = batches[key + (instance.material.opacity < 0.999f ? "#blend" : "")];
        batch.insert(batch.end(), model, model + 16);
        batch.push_back(static_cast<float>(instance.material.color.x));
        batch.push_back(static_cast<float>(instance.material.color.y));
        batch.push_back(static_cast<float>(instance.material.color.z));
        batch.push_back(instance.material.highlight);
        batch.push_back(instance.material.opacity);
        batch.push_back(instance.material.metallic);
        batch.push_back(instance.material.roughness);
    }
    float viewProj[16];
    toGl(frame_.viewProjection, viewProj);
    glUseProgram(meshProgram_);
    glUniformMatrix4fv(glGetUniformLocation(meshProgram_, "uViewProj"), 1, GL_FALSE, viewProj);
    glUniform3f(glGetUniformLocation(meshProgram_, "uEye"), static_cast<float>(frame_.eye.x),
                static_cast<float>(frame_.eye.y), static_cast<float>(frame_.eye.z));
    auto drawBatch = [&](const std::string& key, const std::vector<float>& batch) {
        const bool blend = key.size() >= 6 && key.compare(key.size() - 6, 6, "#blend") == 0;
        const std::string meshKey = blend ? key.substr(0, key.size() - 6) : key;
        const auto mesh = meshes_.find(meshKey);
        if (mesh == meshes_.end()) {
            return;
        }
        drawInstances(mesh->second, batch, blend);
    };
    for (const auto& [key, batch] : batches) {
        const bool blend = key.size() >= 6 && key.compare(key.size() - 6, 6, "#blend") == 0;
        if (!blend) {
            drawBatch(key, batch);
        }
    }
    for (const auto& [key, batch] : batches) {
        const bool blend = key.size() >= 6 && key.compare(key.size() - 6, 6, "#blend") == 0;
        if (blend) {
            drawBatch(key, batch);
        }
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

void OpenGLRenderer::endFrame() {
    // llvmpipe finishes the draw on the calling thread. Without this, the paint
    // timer stops at the draw call and the real raster shows up on the next swap.
    glFinish();
}

int OpenGLRenderer::gpuFrustumCull(const Mat4& viewProjection, const std::vector<Aabb>& bounds) {
    if (bounds.empty()) {
        return 0;
    }
    if (!ready_ || cullFailed_) {
        return -1;
    }
    QOpenGLContext* context = QOpenGLContext::currentContext();
    if (context == nullptr) {
        return -1;
    }
    GLint major = 0;
    GLint minor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    if (major < 4 || (major == 4 && minor < 3)) {
        return -1;
    }
    using DispatchFn = void (*)(GLuint, GLuint, GLuint);
    using BarrierFn = void (*)(GLbitfield);
    auto* dispatch = reinterpret_cast<DispatchFn>(context->getProcAddress("glDispatchCompute"));
    auto* barrier = reinterpret_cast<BarrierFn>(context->getProcAddress("glMemoryBarrier"));
    if (dispatch == nullptr || barrier == nullptr) {
        return -1;
    }
    if (cullProgram_ == 0) {
        const char* source = gpuFrustumCullingShader();
        const unsigned shader = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        int ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            glDeleteShader(shader);
            cullFailed_ = true;
            return -1;
        }
        cullProgram_ = glCreateProgram();
        glAttachShader(cullProgram_, shader);
        glLinkProgram(cullProgram_);
        glDeleteShader(shader);
        glGetProgramiv(cullProgram_, GL_LINK_STATUS, &ok);
        if (!ok || cullProgram_ == 0) {
            if (cullProgram_ != 0) {
                glDeleteProgram(cullProgram_);
            }
            cullProgram_ = 0;
            cullFailed_ = true;
            return -1;
        }
    }
    std::vector<float> packed(bounds.size() * 6);
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        const Aabb& box = bounds[i];
        packed[i * 6 + 0] = static_cast<float>(box.min.x);
        packed[i * 6 + 1] = static_cast<float>(box.min.y);
        packed[i * 6 + 2] = static_cast<float>(box.min.z);
        packed[i * 6 + 3] = static_cast<float>(box.max.x);
        packed[i * 6 + 4] = static_cast<float>(box.max.y);
        packed[i * 6 + 5] = static_cast<float>(box.max.z);
    }
    unsigned buffers[2] = {};
    glGenBuffers(2, buffers);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[0]);
    glBufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(packed.size() * sizeof(float)), packed.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, buffers[0]);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[1]);
    glBufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(bounds.size() * sizeof(unsigned)), nullptr, GL_DYNAMIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, buffers[1]);
    GLint previousProgram = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &previousProgram);
    glUseProgram(cullProgram_);
    float viewProj[16];
    toGl(viewProjection, viewProj);
    glUniformMatrix4fv(glGetUniformLocation(cullProgram_, "uViewProj"), 1, GL_FALSE, viewProj);
    glUniform1ui(glGetUniformLocation(cullProgram_, "uCount"), static_cast<unsigned>(bounds.size()));
    dispatch(static_cast<unsigned>((bounds.size() + 63) / 64), 1, 1);
    barrier(GL_SHADER_STORAGE_BARRIER_BIT);
    std::vector<unsigned> visible(bounds.size(), 0);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[1]);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, static_cast<GLsizeiptr>(visible.size() * sizeof(unsigned)), visible.data());
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    glDeleteBuffers(2, buffers);
    glUseProgram(static_cast<unsigned>(previousProgram));
    int kept = 0;
    for (unsigned flag : visible) {
        kept += flag != 0 ? 1 : 0;
    }
    return kept;
}

bool OpenGLRenderer::captureRgba(std::vector<std::uint8_t>& rgba, int& width, int& height) {
    width = std::max(frame_.width, 1);
    height = std::max(frame_.height, 1);
    rgba.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    return !rgba.empty();
}

}  // namespace sim
