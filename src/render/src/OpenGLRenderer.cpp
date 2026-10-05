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
uniform mat4 uView;
uniform mat4 uLightViewProj;
uniform sampler2DShadow uShadow;
uniform float uShadowStrength;
layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec4 fragNormal;
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
    float keyShade = 1.0;
    if (uShadowStrength > 0.001) {
        vec3 shifted = vWorld + keyDir * (1.6 + (1.0 - ndotk) * 5.0);
        vec4 sl = uLightViewProj * vec4(shifted, 1.0);
        vec3 p = sl.xyz / sl.w * 0.5 + 0.5;
        if (p.x > 0.0 && p.x < 1.0 && p.y > 0.0 && p.y < 1.0 && p.z > 0.0 && p.z < 1.0) {
            vec2 texel = 1.0 / vec2(textureSize(uShadow, 0));
            float sum = 0.0;
            for (int y = -1; y <= 1; ++y) {
                for (int x = -1; x <= 1; ++x) {
                    sum += texture(uShadow, vec3(p.xy + vec2(float(x), float(y)) * texel, p.z));
                }
            }
            keyShade = mix(1.0, sum / 9.0, clamp(uShadowStrength, 0.0, 1.0));
        }
    }
    vec3 lit = vColor * (ambient + vec3(0.62, 0.60, 0.54) * ndotk * keyShade + vec3(0.16, 0.18, 0.22) * ndotf +
                         vec3(0.07) * ndotr);
    float rough = clamp(vRoughness, 0.05, 1.0);
    float shininess = mix(128.0, 10.0, rough);
    float metal = clamp(vMetallic, 0.0, 1.0);
    vec3 halfKey = normalize(keyDir + viewDir);
    vec3 halfFill = normalize(fillDir + viewDir);
    float specKey = pow(max(dot(n, halfKey), 0.0), shininess);
    float specFill = pow(max(dot(n, halfFill), 0.0), max(shininess * 0.45, 4.0));
    vec3 specTint = mix(vec3(1.0), max(vColor, vec3(0.22)), metal);
    vec3 spec = specTint * (specKey * mix(0.42, 0.90, metal) * keyShade + specFill * 0.14);
    vec3 color = lit + spec;
    float crease = smoothstep(0.22, 0.85, length(fwidth(n)));
    color *= mix(1.0, 0.78, crease);
    if (vHighlight > 0.5) {
        color = mix(color, vec3(0.239, 0.525, 0.910), 0.42);
    }
    fragColor = vec4(color, vOpacity);
    fragNormal = vec4(normalize(mat3(uView) * n) * 0.5 + 0.5, 1.0);
}
)";

const char* kShadowVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 2) in vec4 iM0;
layout(location = 3) in vec4 iM1;
layout(location = 4) in vec4 iM2;
layout(location = 5) in vec4 iM3;
uniform mat4 uViewProj;
void main() {
    mat4 uModel = mat4(iM0, iM1, iM2, iM3);
    gl_Position = uViewProj * uModel * vec4(aPos, 1.0);
}
)";

const char* kShadowFragment = R"(#version 330 core
void main() {}
)";

const char* kAoFragment = R"(#version 330 core
in vec2 vUv;
uniform sampler2D uDepth;
uniform sampler2D uNormal;
uniform mat4 uProj;
uniform mat4 uInvProj;
uniform vec2 uTexel;
uniform float uRadius;
out vec4 fragColor;
vec3 viewAt(vec2 uv, float depth) {
    vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 view = uInvProj * clip;
    return view.xyz / view.w;
}
void main() {
    float depth = texture(uDepth, vUv).r;
    vec3 n = texture(uNormal, vUv).rgb * 2.0 - 1.0;
    if (depth > 0.9995 || dot(n, n) < 0.25) {
        fragColor = vec4(1.0);
        return;
    }
    n = normalize(n);
    vec3 pos = viewAt(vUv, depth);
    vec3 tangent = normalize(cross(abs(n.z) < 0.9 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0), n));
    vec3 bitangent = cross(n, tangent);
    float angle = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453) * 6.2831853;
    float cs = cos(angle);
    float sn = sin(angle);
    float occ = 0.0;
    float weight = 0.0;
    for (int i = 0; i < 16; ++i) {
        float h = float(i + 1) / 16.0;
        float spiral = float(i) * 2.399963;
        float ring = sqrt(h);
        vec2 disk = vec2(cos(spiral), sin(spiral)) * ring;
        disk = vec2(cs * disk.x - sn * disk.y, sn * disk.x + cs * disk.y);
        vec3 sampleView = pos + (tangent * disk.x + bitangent * disk.y + n * h) * uRadius;
        vec4 clip = uProj * vec4(sampleView, 1.0);
        vec2 suv = clip.xy / clip.w * 0.5 + 0.5;
        if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) {
            continue;
        }
        float sd = texture(uDepth, suv).r;
        vec3 hit = viewAt(suv, sd);
        float closer = hit.z - sampleView.z;
        float dist = abs(pos.z - hit.z);
        float range = smoothstep(uRadius, uRadius * 0.15, dist);
        float hitOccluder = smoothstep(1.5, 6.0, closer);
        occ += hitOccluder * range;
        weight += 1.0;
    }
    float ao = weight > 0.0 ? 1.0 - occ / weight : 1.0;
    ao = clamp(ao, 0.35, 1.0);
    fragColor = vec4(ao, ao, ao, 1.0);
}
)";

const char* kBlurFragment = R"(#version 330 core
in vec2 vUv;
uniform sampler2D uAo;
uniform sampler2D uDepth;
uniform vec2 uTexel;
uniform vec2 uDirection;
out vec4 fragColor;
void main() {
    float center = texture(uDepth, vUv).r;
    float sum = 0.0;
    float weight = 0.0;
    for (int i = -2; i <= 2; ++i) {
        vec2 uv = vUv + uDirection * uTexel * float(i);
        float d = texture(uDepth, uv).r;
        float w = 1.0 / (1.0 + abs(d - center) * 1800.0);
        if (i == 0) {
            w = 1.0;
        }
        sum += texture(uAo, uv).r * w;
        weight += w;
    }
    float ao = sum / max(weight, 1e-4);
    fragColor = vec4(ao, ao, ao, 1.0);
}
)";

const char* kCompositeFragment = R"(#version 330 core
in vec2 vUv;
uniform sampler2D uColor;
uniform sampler2D uDepth;
uniform sampler2D uAo;
uniform float uAoStrength;
out vec4 fragColor;
void main() {
    float depth = texture(uDepth, vUv).r;
    vec4 color = texture(uColor, vUv);
    if (depth > 0.9995 || color.a < 0.01) {
        discard;
    }
    float ao = texture(uAo, vUv).r;
    float shade = mix(1.0, ao, clamp(uAoStrength, 0.0, 1.0));
    fragColor = vec4(color.rgb * shade, color.a);
    gl_FragDepth = depth;
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

bool invertMat4(const Mat4& m, Mat4& out) {
    double a[4][8];
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            a[row][column] = m(column, row);
            a[row][column + 4] = row == column ? 1.0 : 0.0;
        }
    }
    for (int column = 0; column < 4; ++column) {
        int pivot = column;
        for (int row = column + 1; row < 4; ++row) {
            if (std::abs(a[row][column]) > std::abs(a[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(a[pivot][column]) < 1e-12) {
            return false;
        }
        if (pivot != column) {
            for (int k = 0; k < 8; ++k) {
                std::swap(a[column][k], a[pivot][k]);
            }
        }
        const double divisor = a[column][column];
        for (int k = 0; k < 8; ++k) {
            a[column][k] /= divisor;
        }
        for (int row = 0; row < 4; ++row) {
            if (row == column) {
                continue;
            }
            const double factor = a[row][column];
            for (int k = 0; k < 8; ++k) {
                a[row][k] -= factor * a[column][k];
            }
        }
    }
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            out(column, row) = a[row][column + 4];
        }
    }
    return true;
}

constexpr int kShadowSize = 2048;

bool endsWith(const std::string& key, const char* suffix) {
    const std::size_t length = std::char_traits<char>::length(suffix);
    return key.size() >= length && key.compare(key.size() - length, length, suffix) == 0;
}

bool blendKey(const std::string& key) { return endsWith(key, "#blend"); }

bool noCastKey(const std::string& key) { return endsWith(key, "#nocast"); }

std::string batchMeshKey(std::string key) {
    if (noCastKey(key)) {
        key.resize(key.size() - 7);
    }
    if (blendKey(key)) {
        key.resize(key.size() - 6);
    }
    return key;
}

bool batchBlends(std::string key) {
    if (noCastKey(key)) {
        key.resize(key.size() - 7);
    }
    return blendKey(key);
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
        !compileProgram(shadowProgram_, kShadowVertex, kShadowFragment, error) ||
        !compileProgram(aoProgram_, kBackgroundVertex, kAoFragment, error) ||
        !compileProgram(blurProgram_, kBackgroundVertex, kBlurFragment, error) ||
        !compileProgram(compositeProgram_, kBackgroundVertex, kCompositeFragment, error) ||
        !compileProgram(lineProgram_, kLineVertex, kLineFragment, error) ||
        !compileProgram(backgroundProgram_, kBackgroundVertex, kBackgroundFragment, error)) {
        return false;
    }
    shadowReady_ = ensureShadow();
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
    destroyTargets();
    if (shadowProgram_ != 0) {
        glDeleteProgram(shadowProgram_);
        shadowProgram_ = 0;
    }
    if (aoProgram_ != 0) {
        glDeleteProgram(aoProgram_);
        aoProgram_ = 0;
    }
    if (blurProgram_ != 0) {
        glDeleteProgram(blurProgram_);
        blurProgram_ = 0;
    }
    if (compositeProgram_ != 0) {
        glDeleteProgram(compositeProgram_);
        compositeProgram_ = 0;
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

void OpenGLRenderer::drawInstances(const GpuMesh& mesh, const std::vector<float>& instances, bool blend, bool tally) {
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
    if (tally) {
        submitted_ += count;
        ++draws_;
    }
}

void OpenGLRenderer::destroyTargets() {
    if (shadowFbo_ != 0) {
        glDeleteFramebuffers(1, &shadowFbo_);
        shadowFbo_ = 0;
    }
    if (shadowTex_ != 0) {
        glDeleteTextures(1, &shadowTex_);
        shadowTex_ = 0;
    }
    if (geomFbo_ != 0) {
        glDeleteFramebuffers(1, &geomFbo_);
        geomFbo_ = 0;
    }
    if (geomColor_ != 0) {
        glDeleteTextures(1, &geomColor_);
        geomColor_ = 0;
    }
    if (geomNormal_ != 0) {
        glDeleteTextures(1, &geomNormal_);
        geomNormal_ = 0;
    }
    if (geomDepth_ != 0) {
        glDeleteTextures(1, &geomDepth_);
        geomDepth_ = 0;
    }
    if (aoFbo_ != 0) {
        glDeleteFramebuffers(1, &aoFbo_);
        aoFbo_ = 0;
    }
    if (aoTex_ != 0) {
        glDeleteTextures(1, &aoTex_);
        aoTex_ = 0;
    }
    if (aoBlurFbo_ != 0) {
        glDeleteFramebuffers(1, &aoBlurFbo_);
        aoBlurFbo_ = 0;
    }
    if (aoBlurTex_ != 0) {
        glDeleteTextures(1, &aoBlurTex_);
        aoBlurTex_ = 0;
    }
    geomWidth_ = 0;
    geomHeight_ = 0;
    shadowReady_ = false;
}

bool OpenGLRenderer::ensureShadow() {
    if (shadowFbo_ != 0 && shadowTex_ != 0) {
        return shadowReady_;
    }
    glGenTextures(1, &shadowTex_);
    glBindTexture(GL_TEXTURE_2D, shadowTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kShadowSize, kShadowSize, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT,
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    const float border[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    GLint previous = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previous);
    glGenFramebuffers(1, &shadowFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowTex_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    shadowReady_ = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<unsigned>(previous));
    glBindTexture(GL_TEXTURE_2D, 0);
    return shadowReady_;
}

bool OpenGLRenderer::ensureGeometry(int width, int height) {
    if (width < 1 || height < 1) {
        return false;
    }
    if (geomFbo_ != 0 && geomWidth_ == width && geomHeight_ == height) {
        return true;
    }
    if (geomFbo_ != 0) {
        glDeleteFramebuffers(1, &geomFbo_);
        glDeleteTextures(1, &geomColor_);
        glDeleteTextures(1, &geomNormal_);
        glDeleteTextures(1, &geomDepth_);
        glDeleteFramebuffers(1, &aoFbo_);
        glDeleteTextures(1, &aoTex_);
        glDeleteFramebuffers(1, &aoBlurFbo_);
        glDeleteTextures(1, &aoBlurTex_);
        geomFbo_ = geomColor_ = geomNormal_ = geomDepth_ = 0;
        aoFbo_ = aoTex_ = aoBlurFbo_ = aoBlurTex_ = 0;
    }
    const auto colorTex = [this](unsigned& name, int w, int h, GLenum internal) {
        glGenTextures(1, &name);
        glBindTexture(GL_TEXTURE_2D, name);
        glTexImage2D(GL_TEXTURE_2D, 0, internal, w, h, 0, internal == GL_R8 ? GL_RED : GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    };
    GLint previous = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previous);
    colorTex(geomColor_, width, height, GL_RGBA8);
    colorTex(geomNormal_, width, height, GL_RGBA8);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glGenTextures(1, &geomDepth_);
    glBindTexture(GL_TEXTURE_2D, geomDepth_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    colorTex(aoTex_, width, height, GL_R8);
    colorTex(aoBlurTex_, width, height, GL_R8);
    glGenFramebuffers(1, &geomFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, geomFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, geomColor_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, geomNormal_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, geomDepth_, 0);
    const unsigned buffers[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    glDrawBuffers(2, buffers);
    const bool geomOk = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glGenFramebuffers(1, &aoFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, aoFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, aoTex_, 0);
    const bool aoOk = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glGenFramebuffers(1, &aoBlurFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, aoBlurFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, aoBlurTex_, 0);
    const bool blurOk = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<unsigned>(previous));
    glBindTexture(GL_TEXTURE_2D, 0);
    if (!geomOk || !aoOk || !blurOk) {
        return false;
    }
    geomWidth_ = width;
    geomHeight_ = height;
    return true;
}

Mat4 OpenGLRenderer::lightViewProjection(const Aabb& bounds) const {
    const Vec3 key = normalize(Vec3{0.42, -0.55, 0.72});
    const Vec3 center = bounds.center();
    const double span = std::max(bounds.diagonal(), 1.0);
    const Vec3 eye = center + key * span;
    Vec3 up{0, 0, 1};
    if (std::abs(dot(key, up)) > 0.92) {
        up = {0, 1, 0};
    }
    const Vec3 forward = normalize(center - eye);
    const Vec3 side = normalize(cross(forward, up));
    const Vec3 trueUp = cross(side, forward);
    Mat4 view = Mat4::identity();
    view(0, 0) = side.x;
    view(1, 0) = side.y;
    view(2, 0) = side.z;
    view(0, 1) = trueUp.x;
    view(1, 1) = trueUp.y;
    view(2, 1) = trueUp.z;
    view(0, 2) = -forward.x;
    view(1, 2) = -forward.y;
    view(2, 2) = -forward.z;
    view(3, 0) = -dot(side, eye);
    view(3, 1) = -dot(trueUp, eye);
    view(3, 2) = dot(forward, eye);
    double minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;
    bool any = false;
    for (int xi = 0; xi < 2; ++xi) {
        for (int yi = 0; yi < 2; ++yi) {
            for (int zi = 0; zi < 2; ++zi) {
                const Vec3 point = transformPoint(view, {xi ? bounds.max.x : bounds.min.x, yi ? bounds.max.y : bounds.min.y,
                                                         zi ? bounds.max.z : bounds.min.z});
                if (!any) {
                    minX = maxX = point.x;
                    minY = maxY = point.y;
                    minZ = maxZ = point.z;
                    any = true;
                } else {
                    minX = std::min(minX, point.x);
                    maxX = std::max(maxX, point.x);
                    minY = std::min(minY, point.y);
                    maxY = std::max(maxY, point.y);
                    minZ = std::min(minZ, point.z);
                    maxZ = std::max(maxZ, point.z);
                }
            }
        }
    }
    const double padXY = std::max(maxX - minX, maxY - minY) * 0.04 + 30.0;
    const double padZ = std::max(maxZ - minZ, 1.0) * 0.08 + 50.0;
    const double left = minX - padXY;
    const double right = maxX + padXY;
    const double bottom = minY - padXY;
    const double top = maxY + padXY;
    const double zNear = maxZ + padZ;
    const double zFar = minZ - padZ;
    Mat4 ortho = Mat4::identity();
    ortho(0, 0) = 2.0 / (right - left);
    ortho(3, 0) = -(right + left) / (right - left);
    ortho(1, 1) = 2.0 / (top - bottom);
    ortho(3, 1) = -(top + bottom) / (top - bottom);
    ortho(2, 2) = -2.0 / (zNear - zFar);
    ortho(3, 2) = -1.0 - ortho(2, 2) * zNear;
    return mul(ortho, view);
}

void OpenGLRenderer::submitInstances(const std::vector<RenderInstance>& instances) {
    if (!ready_) {
        return;
    }
    std::unordered_map<std::string, std::vector<float>> batches;
    Aabb shadowBounds;
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
        if (instance.material.opacity >= 0.999f && instance.worldBounds.valid) {
            shadowBounds.expand(instance.worldBounds);
        }
        float model[16];
        toGl(instance.world, model);
        const std::string suffix = std::string(instance.material.opacity < 0.999f ? "#blend" : "") +
                                   (instance.castShadow ? "" : "#nocast");
        auto& batch = batches[key + suffix];
        batch.insert(batch.end(), model, model + 16);
        batch.push_back(static_cast<float>(instance.material.color.x));
        batch.push_back(static_cast<float>(instance.material.color.y));
        batch.push_back(static_cast<float>(instance.material.color.z));
        batch.push_back(instance.material.highlight);
        batch.push_back(instance.material.opacity);
        batch.push_back(instance.material.metallic);
        batch.push_back(instance.material.roughness);
    }
    GLint widgetFbo = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &widgetFbo);
    const bool offscreen = shadowBounds.valid && ensureGeometry(frame_.width, frame_.height) &&
                           (frame_.shadowStrength > 0.001f || frame_.aoStrength > 0.001f);
    const bool doShadow = offscreen && shadowReady_ && frame_.shadowStrength > 0.001f;
    Mat4 light = Mat4::identity();
    if (doShadow) {
        light = lightViewProjection(shadowBounds);
        float lightMatrix[16];
        toGl(light, lightMatrix);
        glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
        glDrawBuffer(GL_NONE);
        glViewport(0, 0, kShadowSize, kShadowSize);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.6f, 2.4f);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glUseProgram(shadowProgram_);
        glUniformMatrix4fv(glGetUniformLocation(shadowProgram_, "uViewProj"), 1, GL_FALSE, lightMatrix);
        for (const auto& [key, batch] : batches) {
            if (batchBlends(key) || noCastKey(key)) {
                continue;
            }
            const auto mesh = meshes_.find(batchMeshKey(key));
            if (mesh == meshes_.end()) {
                continue;
            }
            drawInstances(mesh->second, batch, false, false);
        }
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    const bool ao = offscreen && frame_.aoStrength > 0.001f;
    if (offscreen) {
        glBindFramebuffer(GL_FRAMEBUFFER, geomFbo_);
        const unsigned buffers[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        glDrawBuffers(2, buffers);
        glViewport(0, 0, frame_.width, frame_.height);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    } else {
        glBindFramebuffer(GL_FRAMEBUFFER, static_cast<unsigned>(widgetFbo));
        glViewport(0, 0, frame_.width, frame_.height);
    }

    float viewProj[16];
    float view[16];
    float lightMatrix[16];
    toGl(frame_.viewProjection, viewProj);
    toGl(frame_.view, view);
    toGl(light, lightMatrix);
    glUseProgram(meshProgram_);
    glUniformMatrix4fv(glGetUniformLocation(meshProgram_, "uViewProj"), 1, GL_FALSE, viewProj);
    glUniformMatrix4fv(glGetUniformLocation(meshProgram_, "uView"), 1, GL_FALSE, view);
    glUniformMatrix4fv(glGetUniformLocation(meshProgram_, "uLightViewProj"), 1, GL_FALSE, lightMatrix);
    glUniform3f(glGetUniformLocation(meshProgram_, "uEye"), static_cast<float>(frame_.eye.x),
                static_cast<float>(frame_.eye.y), static_cast<float>(frame_.eye.z));
    glUniform1f(glGetUniformLocation(meshProgram_, "uShadowStrength"), doShadow ? frame_.shadowStrength : 0.0f);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowTex_);
    glUniform1i(glGetUniformLocation(meshProgram_, "uShadow"), 0);
    auto drawBatch = [&](const std::string& key, const std::vector<float>& batch, bool tally) {
        const bool blend = batchBlends(key);
        const auto mesh = meshes_.find(batchMeshKey(key));
        if (mesh == meshes_.end()) {
            return;
        }
        drawInstances(mesh->second, batch, blend, tally);
    };
    for (const auto& [key, batch] : batches) {
        if (!batchBlends(key)) {
            drawBatch(key, batch, true);
        }
    }

    if (offscreen) {
        Mat4 invProj;
        if (!invertMat4(frame_.projection, invProj)) {
            invProj = Mat4::identity();
        }
        float proj[16];
        float inv[16];
        toGl(frame_.projection, proj);
        toGl(invProj, inv);
        const float radius = static_cast<float>(std::clamp(shadowBounds.diagonal() * 0.012, 28.0, 110.0));
        if (ao) {
            glBindFramebuffer(GL_FRAMEBUFFER, aoFbo_);
            glViewport(0, 0, frame_.width, frame_.height);
            glDisable(GL_DEPTH_TEST);
            glUseProgram(aoProgram_);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, geomDepth_);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, geomNormal_);
            glUniform1i(glGetUniformLocation(aoProgram_, "uDepth"), 0);
            glUniform1i(glGetUniformLocation(aoProgram_, "uNormal"), 1);
            glUniformMatrix4fv(glGetUniformLocation(aoProgram_, "uProj"), 1, GL_FALSE, proj);
            glUniformMatrix4fv(glGetUniformLocation(aoProgram_, "uInvProj"), 1, GL_FALSE, inv);
            glUniform1f(glGetUniformLocation(aoProgram_, "uRadius"), radius);
            glBindVertexArray(backgroundVao_);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            auto blur = [&](unsigned target, unsigned source, float dirX, float dirY) {
                glBindFramebuffer(GL_FRAMEBUFFER, target);
                glUseProgram(blurProgram_);
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, source);
                glActiveTexture(GL_TEXTURE1);
                glBindTexture(GL_TEXTURE_2D, geomDepth_);
                glUniform1i(glGetUniformLocation(blurProgram_, "uAo"), 0);
                glUniform1i(glGetUniformLocation(blurProgram_, "uDepth"), 1);
                glUniform2f(glGetUniformLocation(blurProgram_, "uTexel"), 1.0f / static_cast<float>(frame_.width),
                            1.0f / static_cast<float>(frame_.height));
                glUniform2f(glGetUniformLocation(blurProgram_, "uDirection"), dirX, dirY);
                glDrawArrays(GL_TRIANGLES, 0, 3);
            };
            blur(aoBlurFbo_, aoTex_, 1.0f, 0.0f);
            blur(aoFbo_, aoBlurTex_, 0.0f, 1.0f);
        }
        glBindFramebuffer(GL_FRAMEBUFFER, static_cast<unsigned>(widgetFbo));
        glViewport(0, 0, frame_.width, frame_.height);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glUseProgram(compositeProgram_);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, geomColor_);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, geomDepth_);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, ao ? aoTex_ : geomColor_);
        glUniform1i(glGetUniformLocation(compositeProgram_, "uColor"), 0);
        glUniform1i(glGetUniformLocation(compositeProgram_, "uDepth"), 1);
        glUniform1i(glGetUniformLocation(compositeProgram_, "uAo"), 2);
        glUniform1f(glGetUniformLocation(compositeProgram_, "uAoStrength"), ao ? frame_.aoStrength : 0.0f);
        glBindVertexArray(backgroundVao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glDepthFunc(GL_LESS);
        glActiveTexture(GL_TEXTURE0);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<unsigned>(widgetFbo));
    glViewport(0, 0, frame_.width, frame_.height);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(meshProgram_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowTex_);
    for (const auto& [key, batch] : batches) {
        if (batchBlends(key)) {
            drawBatch(key, batch, true);
        }
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
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
