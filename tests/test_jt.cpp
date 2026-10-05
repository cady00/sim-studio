#include "sim/core/Scene.hpp"
#include "sim/io/AssetImport.hpp"
#include "sim/io/JtCodec.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <lzma.h>
#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

void append(std::vector<std::uint8_t>& out, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    out.insert(out.end(), bytes, bytes + size);
}

void appendU8(std::vector<std::uint8_t>& out, std::uint8_t value) { out.push_back(value); }

void appendU16(std::vector<std::uint8_t>& out, std::uint16_t value) { append(out, &value, 2); }

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) { append(out, &value, 4); }

void appendI32(std::vector<std::uint8_t>& out, std::int32_t value) { append(out, &value, 4); }

void appendU64(std::vector<std::uint8_t>& out, std::uint64_t value) { append(out, &value, 8); }

void appendF32(std::vector<std::uint8_t>& out, float value) { append(out, &value, 4); }

void guid(std::vector<std::uint8_t>& out, std::uint32_t a, std::uint16_t b, std::uint16_t c, const std::uint8_t tail[8]) {
    appendU32(out, a);
    appendU16(out, b);
    appendU16(out, c);
    append(out, tail, 8);
}

const std::uint8_t kGraph[8] = {0x9b, 0x6b, 0x00, 0x80, 0xc7, 0xbb, 0x59, 0x97};
const std::uint8_t kPart[8] = {0xa5, 0x06, 0x00, 0x60, 0x97, 0xbd, 0xc6, 0xe1};
const std::uint8_t kLate[8] = {0xa3, 0xa7, 0x00, 0xaa, 0x00, 0xd1, 0x09, 0x54};

void graphGuid(std::vector<std::uint8_t>& out, std::uint32_t object) { guid(out, object, 0x2ac8, 0x11d1, kGraph); }

std::vector<std::uint8_t> zlibWrap(const std::vector<std::uint8_t>& raw) {
    uLongf bound = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> out(bound);
    if (compress2(out.data(), &bound, raw.data(), static_cast<uLong>(raw.size()), 6) != Z_OK) {
        return {};
    }
    out.resize(bound);
    return out;
}

std::vector<std::uint8_t> xzWrap(const std::vector<std::uint8_t>& raw) {
    lzma_stream stream = LZMA_STREAM_INIT;
    if (lzma_easy_encoder(&stream, 6, LZMA_CHECK_CRC64) != LZMA_OK) {
        return {};
    }
    stream.next_in = raw.data();
    stream.avail_in = raw.size();
    std::vector<std::uint8_t> out(64 + raw.size() * 2 + 128);
    stream.next_out = out.data();
    stream.avail_out = out.size();
    const lzma_ret status = lzma_code(&stream, LZMA_FINISH);
    lzma_end(&stream);
    if (status != LZMA_STREAM_END) {
        return {};
    }
    out.resize(stream.total_out);
    return out;
}

void baseNode(std::vector<std::uint8_t>& out, const std::vector<int>& attributes) {
    appendU8(out, 1);
    appendU32(out, 0);
    appendI32(out, static_cast<std::int32_t>(attributes.size()));
    for (int attribute : attributes) {
        appendI32(out, attribute);
    }
}

void childList(std::vector<std::uint8_t>& out, const std::vector<int>& children, bool v9) {
    appendU8(out, v9 ? 0 : 1);
    if (v9) {
        appendU16(out, 1);
    }
    appendI32(out, static_cast<std::int32_t>(children.size()));
    for (int child : children) {
        appendI32(out, child);
    }
}

void mbString(std::vector<std::uint8_t>& out, const std::u16string& text) {
    appendI32(out, static_cast<std::int32_t>(text.size()));
    for (char16_t unit : text) {
        appendU16(out, static_cast<std::uint16_t>(unit));
    }
}

void element(std::vector<std::uint8_t>& out, const std::vector<std::uint8_t>& headerAndPayload) {
    appendI32(out, static_cast<std::int32_t>(headerAndPayload.size()));
    append(out, headerAndPayload.data(), headerAndPayload.size());
}

void elementHeader(std::vector<std::uint8_t>& body, std::uint32_t object, std::uint16_t b, std::uint16_t c,
                   const std::uint8_t tail[8], std::uint8_t base, int id) {
    guid(body, object, b, c, tail);
    appendU8(body, base);
    appendI32(body, id);
}

void endMarker(std::vector<std::uint8_t>& out) {
    appendI32(out, 16);
    out.insert(out.end(), 16, 0xff);
}

void nullPacket(std::vector<std::uint8_t>& out, const std::vector<std::int32_t>& values) {
    appendI32(out, static_cast<std::int32_t>(values.size()));
    if (values.empty()) {
        return;
    }
    appendU8(out, 0);
    for (std::int32_t value : values) {
        appendI32(out, value);
    }
}

struct BitWriter {
    std::vector<std::uint32_t> words;
    int bits = 0;

    void write(std::uint32_t value, int count) {
        for (int bit = count - 1; bit >= 0; --bit) {
            const int word = bits / 32;
            const int shift = 31 - (bits % 32);
            if (static_cast<int>(words.size()) <= word) {
                words.push_back(0);
            }
            if (((value >> bit) & 1u) != 0) {
                words[static_cast<std::size_t>(word)] |= 1u << shift;
            }
            ++bits;
        }
    }
};

void writeNibble(BitWriter& writer, std::int32_t value) {
    for (int i = 0; i < 11; ++i) {
        const auto nibble = static_cast<std::uint32_t>(static_cast<std::uint32_t>(value) & 7u);
        const std::int32_t remaining = value >> 3;
        const bool more = (remaining != 0 && remaining != -1) || ((nibble & 4u) != 0 && remaining == 0) ||
                          ((nibble & 4u) == 0 && remaining == -1);
        writer.write(nibble | (more ? 8u : 0u), 4);
        value = remaining;
        if (!more) {
            break;
        }
    }
}

void bitlengthPacket(std::vector<std::uint8_t>& out, const std::vector<std::int32_t>& values) {
    appendI32(out, static_cast<std::int32_t>(values.size()));
    if (values.empty()) {
        return;
    }
    appendU8(out, 1);
    std::int32_t minimum = values.front();
    std::int32_t maximum = values.front();
    for (std::int32_t value : values) {
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
    }
    BitWriter writer;
    writer.write(0, 1);
    writeNibble(writer, minimum);
    writeNibble(writer, maximum);
    std::uint32_t span = static_cast<std::uint32_t>(maximum - minimum);
    int width = 0;
    while (span > 0) {
        span >>= 1;
        ++width;
    }
    for (std::int32_t value : values) {
        writer.write(static_cast<std::uint32_t>(value - minimum), width);
    }
    appendI32(out, writer.bits);
    for (std::uint32_t word : writer.words) {
        appendU32(out, word);
    }
}

std::vector<std::uint8_t> triStripPayload(bool bitlength) {
    std::vector<std::uint8_t> payload;
    appendU8(payload, 1);
    appendU8(payload, 1);
    appendU64(payload, 0xau);
    appendU8(payload, 8);
    for (int axis = 0; axis < 3; ++axis) {
        appendF32(payload, 0.f);
        appendF32(payload, axis == 2 ? 0.f : 10.f);
        appendU8(payload, 8);
    }
    const std::vector<std::int32_t> x{0, 255, 0};
    const std::vector<std::int32_t> y{0, 0, 255};
    const std::vector<std::int32_t> z{0, 0, 0};
    if (bitlength) {
        bitlengthPacket(payload, x);
        bitlengthPacket(payload, y);
        bitlengthPacket(payload, z);
    } else {
        nullPacket(payload, x);
        nullPacket(payload, y);
        nullPacket(payload, z);
    }
    appendU8(payload, 6);
    const std::uint32_t code = (2u << 15) | (7u << 12) | (63u << 6);
    nullPacket(payload, {static_cast<std::int32_t>(code), static_cast<std::int32_t>(code), static_cast<std::int32_t>(code)});
    appendI32(payload, 1);
    nullPacket(payload, {0, 1, 2});
    return payload;
}

std::vector<std::uint8_t> versionHeader(const std::string& text, int major) {
    std::string version = "Version " + text;
    version.resize(75, ' ');
    std::vector<std::uint8_t> out(version.begin(), version.end());
    out.push_back(' ');
    out.push_back('\n');
    out.push_back('\r');
    out.push_back('\n');
    out.push_back(' ');
    appendU8(out, 0);
    appendI32(out, 0);
    if (major >= 10) {
        appendU64(out, 0);
    } else {
        appendI32(out, 0);
    }
    out.insert(out.end(), 16, 0);
    return out;
}

struct Piece {
    std::vector<std::uint8_t> id;
    int type = 1;
    std::vector<std::uint8_t> payload;
    bool compressZlib = false;
    bool compressXz = false;
};

std::vector<std::uint8_t> buildJt(int major, const std::string& versionText, std::vector<Piece> pieces) {
    std::vector<std::uint8_t> header = versionHeader(versionText, major);
    const std::uint64_t tocOffset = header.size();
    const int entryBytes = major >= 10 ? 32 : 28;
    std::uint64_t cursor = tocOffset + 4 + static_cast<std::uint64_t>(pieces.size()) * entryBytes;
    std::vector<std::vector<std::uint8_t>> segments;
    for (Piece& piece : pieces) {
        std::vector<std::uint8_t> body = piece.payload;
        int flag = 0;
        int algorithm = 1;
        if (piece.compressZlib) {
            body = zlibWrap(piece.payload);
            flag = 2;
            algorithm = 2;
        } else if (piece.compressXz) {
            body = xzWrap(piece.payload);
            flag = 3;
            algorithm = 3;
        }
        std::vector<std::uint8_t> segment = piece.id;
        appendI32(segment, piece.type);
        const bool compressed = piece.compressZlib || piece.compressXz;
        const std::uint32_t length = static_cast<std::uint32_t>(24 + (compressed ? 9 + body.size() : body.size()));
        appendI32(segment, static_cast<std::int32_t>(length));
        if (compressed) {
            appendU32(segment, static_cast<std::uint32_t>(flag));
            appendI32(segment, static_cast<std::int32_t>(body.size() + 1));
            appendU8(segment, static_cast<std::uint8_t>(algorithm));
        }
        append(segment, body.data(), body.size());
        if (piece.id.size() == 16) {
            std::memcpy(header.data() + header.size() - 16, piece.id.data(), 16);
        }
        segments.push_back(std::move(segment));
    }
    if (major >= 10) {
        std::memcpy(header.data() + 85, &tocOffset, 8);
    } else {
        const auto narrow = static_cast<std::uint32_t>(tocOffset);
        std::memcpy(header.data() + 85, &narrow, 4);
    }
    std::vector<std::uint8_t> file = header;
    appendI32(file, static_cast<std::int32_t>(pieces.size()));
    cursor = file.size() + static_cast<std::uint64_t>(pieces.size()) * entryBytes;
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        append(file, pieces[i].id.data(), 16);
        if (major >= 10) {
            appendU64(file, cursor);
        } else {
            appendU32(file, static_cast<std::uint32_t>(cursor));
        }
        appendU32(file, static_cast<std::uint32_t>(segments[i].size()));
        appendU32(file, static_cast<std::uint32_t>(pieces[i].type) << 24);
        cursor += segments[i].size();
    }
    for (const auto& segment : segments) {
        append(file, segment.data(), segment.size());
    }
    return file;
}

std::vector<std::uint8_t> randomGuid(int seed) {
    std::vector<std::uint8_t> id(16);
    unsigned state = static_cast<unsigned>(seed * 17 + 3);
    for (std::uint8_t& byte : id) {
        state = state * 1664525u + 1013904223u;
        byte = static_cast<std::uint8_t>(state >> 16);
    }
    return id;
}

std::vector<std::uint8_t> lsgGraph(bool v9) {
    std::vector<std::uint8_t> graph;
    {
        std::vector<std::uint8_t> body;
        elementHeader(body, 0x10dd103e, 0x2ac8, 0x11d1, kGraph, 1, 1);
        baseNode(body, {});
        childList(body, {2}, v9);
        if (v9) {
            appendI32(body, 1);
        } else {
            appendU8(body, 1);
            appendI32(body, 0);
        }
        mbString(body, u"bracket.jt");
        element(graph, body);
    }
    {
        std::vector<std::uint8_t> body;
        elementHeader(body, 0x10dd101b, 0x2ac8, 0x11d1, kGraph, 1, 2);
        baseNode(body, {10});
        childList(body, {3}, v9);
        element(graph, body);
    }
    {
        std::vector<std::uint8_t> body;
        elementHeader(body, 0x10dd1077, 0x2ac8, 0x11d1, kGraph, 2, 3);
        baseNode(body, {11});
        element(graph, body);
    }
    {
        std::vector<std::uint8_t> body;
        elementHeader(body, 0x10dd1083, 0x2ac8, 0x11d1, kGraph, 3, 10);
        appendU8(body, 2);
        appendU8(body, 0x08);
        appendU32(body, 0);
        appendU32(body, 0);
        appendU8(body, 1);
        appendU16(body, 0x0008);
        appendF32(body, 100.f);
        element(graph, body);
    }
    {
        std::vector<std::uint8_t> body;
        elementHeader(body, 0x10dd1030, 0x2ac8, 0x11d1, kGraph, 3, 11);
        appendU8(body, 2);
        appendU8(body, 0x08);
        appendU32(body, 0);
        appendU32(body, 0);
        appendU8(body, 1);
        appendU16(body, 0);
        const float ambient[4] = {0.1f, 0.1f, 0.1f, 1.f};
        const float diffuse[4] = {0.8f, 0.1f, 0.05f, 1.f};
        const float specular[4] = {0.2f, 0.2f, 0.2f, 1.f};
        const float emission[4] = {0.f, 0.f, 0.f, 1.f};
        for (float value : ambient) appendF32(body, value);
        for (float value : diffuse) appendF32(body, value);
        for (float value : specular) appendF32(body, value);
        for (float value : emission) appendF32(body, value);
        appendF32(body, 16.f);
        appendF32(body, 0.f);
        appendF32(body, 1.f);
        element(graph, body);
    }
    endMarker(graph);
    auto stringAtom = [](int id, const std::u16string& text) {
        std::vector<std::uint8_t> body;
        elementHeader(body, 0x10dd106e, 0x2ac8, 0x11d1, kGraph, 5, id);
        appendU8(body, 1);
        appendU32(body, 0);
        appendU8(body, 0x40);
        appendU16(body, 1);
        mbString(body, text);
        return body;
    };
    element(graph, stringAtom(20, u"JT_LLPROP_SHAPEIMPL"));
    element(graph, stringAtom(21, u"JT_PROP_SIMP_NAME"));
    element(graph, stringAtom(22, u"Bracket"));
    {
        std::vector<std::uint8_t> body;
        elementHeader(body, 0xe0b05be5, 0xfbbd, 0x11d1, kLate, 8, 23);
        appendU8(body, 1);
        appendU32(body, 0);
        appendU8(body, 0);
        appendU16(body, 1);
        graphGuid(body, 0x11111111);
        appendI32(body, 7);
        appendI32(body, 0);
        appendI32(body, 1);
        element(graph, body);
    }
    endMarker(graph);
    appendU16(graph, 1);
    appendI32(graph, 2);
    appendI32(graph, 2);
    appendI32(graph, 21);
    appendI32(graph, 22);
    appendI32(graph, 0);
    appendI32(graph, 3);
    appendI32(graph, 20);
    appendI32(graph, 23);
    appendI32(graph, 0);
    return graph;
}

std::vector<std::uint8_t> lodSegment(bool bitlength) {
    std::vector<std::uint8_t> payload = triStripPayload(bitlength);
    std::vector<std::uint8_t> body;
    elementHeader(body, 0x10dd10ab, 0x2ac8, 0x11d1, kGraph, 4, 0);
    append(body, payload.data(), payload.size());
    std::vector<std::uint8_t> segment;
    element(segment, body);
    return segment;
}

std::vector<std::uint8_t> xtSegment() {
    std::vector<std::uint8_t> body;
    elementHeader(body, 0x873a70e0, 0x2ac9, 0x11d1, kGraph, 0, 1);
    appendU8(body, 1);
    appendU8(body, 0);
    appendI32(body, 35);
    appendI32(body, 1);
    appendI32(body, 0);
    appendI32(body, 4);
    body.insert(body.end(), {'P', 'A', 'R', 'A'});
    std::vector<std::uint8_t> segment;
    element(segment, body);
    return segment;
}

std::filesystem::path writeFile(const std::string& name, const std::vector<std::uint8_t>& bytes) {
    const auto dir = std::filesystem::temp_directory_path() / "simstudio-jt";
    std::filesystem::create_directories(dir);
    const auto path = dir / name;
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return path;
}

sim::AssetImportResult importBytes(const std::string& name, const std::vector<std::uint8_t>& bytes, sim::Scene& scene) {
    const auto path = writeFile(name, bytes);
    return sim::createBuiltinRegistry()->importPath(path.string(), scene);
}

}  // namespace

TEST_CASE("jt header toc and every segment type are recognized", "[jt]") {
    const int types[] = {1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 23, 24, 30, 31};
    for (int type : types) {
        Piece piece;
        piece.id = randomGuid(type + 40);
        piece.type = type;
        const int major = type == 30 ? 10 : 9;
        const auto bytes = buildJt(major, major >= 10 ? "10.0" : "9.5", {piece});
        sim::Scene scene;
        const sim::AssetImportResult result = importBytes("type-" + std::to_string(type) + ".jt", bytes, scene);
        INFO(result.message);
        REQUIRE(result.ok);
        REQUIRE(result.message.find("types=" + std::to_string(type)) != std::string::npos);
        REQUIRE(result.triangleCount == 0);
        if (type == 17 || type == 30) {
            REQUIRE(result.message.find("XT B-rep is not decoded") != std::string::npos);
        }
    }
}

TEST_CASE("jt version 8 9 and 10 headers round trip compression", "[jt]") {
    Piece meta;
    meta.id = randomGuid(8);
    meta.type = 4;
    sim::Scene v8;
    const sim::AssetImportResult eight = importBytes("v8.jt", buildJt(8, "8.1", {meta}), v8);
    REQUIRE(eight.ok);
    REQUIRE(eight.message.find("Version 8.1") != std::string::npos);
    REQUIRE(eight.message.find("segments=1") != std::string::npos);

    Piece lsg;
    lsg.id = randomGuid(9);
    lsg.type = 1;
    lsg.payload = lsgGraph(true);
    lsg.compressZlib = true;
    sim::Scene v9;
    const sim::AssetImportResult nine = importBytes("v9.jt", buildJt(9, "9.5", {lsg}), v9);
    INFO(nine.message);
    REQUIRE(nine.ok);
    REQUIRE(nine.message.find("Version 9.5") != std::string::npos);
    REQUIRE(v9.findByName("bracket") != nullptr);
    REQUIRE(v9.findByName("Bracket") != nullptr);
    REQUIRE(v9.findByName("Bracket")->local.translation.x == Catch::Approx(100));
    REQUIRE(v9.findByName("tri-strip-3")->color.x == Catch::Approx(0.8).margin(1e-5));

    Piece lsg10 = lsg;
    lsg10.id = randomGuid(10);
    lsg10.payload = lsgGraph(false);
    lsg10.compressZlib = false;
    lsg10.compressXz = true;
    sim::Scene v10;
    const sim::AssetImportResult ten = importBytes("v10.jt", buildJt(10, "10.5", {lsg10}), v10);
    INFO(ten.message);
    REQUIRE(ten.ok);
    REQUIRE(ten.message.find("Version 10.5") != std::string::npos);
    REQUIRE(v10.findByName("Bracket")->local.translation.x == Catch::Approx(100));
}

TEST_CASE("jt quantized tri strips decode null and bitlength vertices and deering normals", "[jt]") {
    for (const bool bitlength : {false, true}) {
        Piece lsg;
        lsg.id = randomGuid(bitlength ? 70 : 71);
        lsg.type = 1;
        lsg.payload = lsgGraph(false);
        Piece lod;
        lod.id = lsg.id;
        graphGuid(lod.id, 0x11111111);
        lod.id.resize(16);
        lod.type = 7;
        lod.payload = lodSegment(bitlength);
        Piece xt;
        xt.id = randomGuid(72);
        xt.type = 17;
        xt.payload = xtSegment();
        // The late-load GUID is written by graphGuid into a fresh vector. Rebuild it explicitly.
        lod.id = {0x11, 0x11, 0x11, 0x11, 0xc8, 0x2a, 0xd1, 0x11, 0x9b, 0x6b, 0x00, 0x80, 0xc7, 0xbb, 0x59, 0x97};
        const auto bytes = buildJt(10, "10.0", {lsg, lod, xt});
        sim::Scene scene;
        const sim::AssetImportResult result = importBytes(bitlength ? "strips-bit.jt" : "strips-null.jt", bytes, scene);
        INFO(result.message);
        REQUIRE(result.ok);
        REQUIRE(result.triangleCount == 1);
        REQUIRE(result.meshCount == 1);
        REQUIRE(result.message.find("decoded quantized tri-strips") != std::string::npos);
        REQUIRE(result.message.find("XT B-rep is not decoded") != std::string::npos);
        REQUIRE(result.message.find("4 Parasolid bytes skipped") != std::string::npos);
        auto shape = scene.findByName("tri-strip-3");
        REQUIRE(shape);
        const auto mesh = scene.mesh(shape->meshId);
        REQUIRE(mesh);
        REQUIRE(mesh->triangleCount() == 1);
        REQUIRE(mesh->positions[0] == Catch::Approx(0));
        REQUIRE(mesh->positions[3] == Catch::Approx(10));
        REQUIRE(mesh->positions[4] == Catch::Approx(0));
        REQUIRE(mesh->positions[7] == Catch::Approx(10));
        REQUIRE(mesh->positions[2] == Catch::Approx(0));
        REQUIRE(mesh->normals[2] > 0.99f);
        REQUIRE(std::abs(mesh->normals[0]) < 0.02f);
        REQUIRE(std::abs(mesh->normals[1]) < 0.02f);
    }
}

TEST_CASE("jt damaged files fail closed", "[jt]") {
    auto registry = sim::createBuiltinRegistry();
    const auto dir = std::filesystem::temp_directory_path() / "simstudio-jt-fuzz";
    std::filesystem::create_directories(dir);
    const std::vector<std::string> samples = {"", std::string(20, '\0'), std::string("Version"), std::string(4096, '\x5a')};
    int index = 0;
    for (const std::string& sample : samples) {
        const auto path = dir / ("bad-" + std::to_string(index++) + ".jt");
        std::ofstream out(path, std::ios::binary);
        out.write(sample.data(), static_cast<std::streamsize>(sample.size()));
        sim::Scene scene;
        const sim::AssetImportResult result = registry->importPath(path.string(), scene);
        REQUIRE_FALSE(result.ok);
        REQUIRE_FALSE(result.message.empty());
        REQUIRE(scene.root()->children.empty());
    }
    Piece lsg;
    lsg.id = randomGuid(90);
    lsg.type = 1;
    lsg.payload = {1, 2, 3, 4, 5};
    lsg.compressZlib = true;
    auto broken = buildJt(9, "9.4", {lsg});
    if (!broken.empty()) {
        broken.back() ^= 0xff;
    }
    sim::Scene scene;
    const sim::AssetImportResult result = importBytes("bad-zlib.jt", broken, scene);
    REQUIRE_FALSE(result.ok);
    REQUIRE(result.message.find("zlib") != std::string::npos);
}

TEST_CASE("public jt samples open with an honest mesh fallback", "[jt]") {
    const std::filesystem::path root = std::filesystem::path(SIM_SOURCE_DIR) / "testdata" / "local" / "jt";
    if (!std::filesystem::exists(root / "screw.jt")) {
        SKIP("testdata/local/jt is not present");
    }
    auto registry = sim::createBuiltinRegistry();
    sim::Scene screw;
    const sim::AssetImportResult screwResult = registry->importPath((root / "screw.jt").string(), screw);
    INFO(screwResult.message);
    REQUIRE(screwResult.ok);
    REQUIRE(screwResult.message.find("Version 9.4") != std::string::npos);
    REQUIRE(screwResult.message.find("segments=6") != std::string::npos);
    const bool listsLod = screwResult.message.find(",7,") != std::string::npos ||
                          screwResult.message.find("types=7") != std::string::npos ||
                          screwResult.message.find("7,") != std::string::npos;
    REQUIRE(listsLod);
    REQUIRE(screw.findByName("zink0B08A7219cph") != nullptr);
    REQUIRE(screwResult.triangleCount == 6450);
    REQUIRE(screwResult.meshCount == 1);
    REQUIRE(screwResult.message.find("decoded topologically compressed tri-strips") != std::string::npos);
    REQUIRE(screwResult.message.find("xt=no") != std::string::npos);
    const sim::MeshData* screwMesh = nullptr;
    for (const auto& entry : screw.meshes()) {
        if (entry.second && entry.second->triangleCount() == 6450) {
            screwMesh = entry.second.get();
        }
    }
    REQUIRE(screwMesh != nullptr);
    REQUIRE(screwMesh->bounds.min.x == Catch::Approx(-0.005).margin(1e-6));
    REQUIRE(screwMesh->bounds.max.x == Catch::Approx(0.005).margin(1e-6));
    REQUIRE(screwMesh->bounds.min.y == Catch::Approx(-0.00497867).margin(1e-5));
    REQUIRE(screwMesh->bounds.max.y == Catch::Approx(0.00497867).margin(1e-5));
    REQUIRE(screwMesh->bounds.min.z == Catch::Approx(-0.02).margin(1e-6));
    REQUIRE(screwMesh->bounds.max.z == Catch::Approx(0.006).margin(1e-6));
    int screwNormals = 0;
    for (std::size_t i = 0; i + 2 < screwMesh->normals.size(); i += 3) {
        const float x = screwMesh->normals[i];
        const float y = screwMesh->normals[i + 1];
        const float z = screwMesh->normals[i + 2];
        const float length = std::sqrt(x * x + y * y + z * z);
        if (std::fabs(length - 1.f) < 0.02f) {
            ++screwNormals;
        }
    }
    REQUIRE(screwNormals == 6450 * 3);

    sim::Scene ctc;
    const sim::AssetImportResult ctcResult = registry->importPath((root / "ctc02.jt").string(), ctc);
    INFO(ctcResult.message);
    REQUIRE(ctcResult.ok);
    REQUIRE(ctcResult.message.find("Version 9.4") != std::string::npos);
    REQUIRE(ctcResult.message.find("segments=7") != std::string::npos);
    REQUIRE(ctcResult.triangleCount == 6842);
    REQUIRE(ctcResult.message.find("decoded topologically compressed tri-strips") != std::string::npos);
    const sim::MeshData* ctcMesh = nullptr;
    for (const auto& entry : ctc.meshes()) {
        if (entry.second && entry.second->triangleCount() == 6842) {
            ctcMesh = entry.second.get();
        }
    }
    REQUIRE(ctcMesh != nullptr);
    REQUIRE(ctcMesh->bounds.min.x == Catch::Approx(-0.340).margin(1e-3));
    REQUIRE(ctcMesh->bounds.max.x == Catch::Approx(0.340).margin(1e-3));
    REQUIRE(ctcMesh->bounds.min.y == Catch::Approx(-0.040).margin(1e-3));
    REQUIRE(ctcMesh->bounds.max.y == Catch::Approx(0.820).margin(1e-3));
    REQUIRE(ctcMesh->bounds.min.z == Catch::Approx(-0.360).margin(1e-3));
    REQUIRE(ctcMesh->bounds.max.z == Catch::Approx(0.040).margin(1e-3));

    sim::Scene box;
    const sim::AssetImportResult boxResult = registry->importPath((root / "boxasm.jt").string(), box);
    INFO(boxResult.message);
    REQUIRE(boxResult.ok);
    REQUIRE(boxResult.message.find("Version 10.5") != std::string::npos);
    REQUIRE(boxResult.message.find("segments=107") != std::string::npos);
    REQUIRE(boxResult.message.find("XT B-rep is not decoded") != std::string::npos);
    REQUIRE(boxResult.message.find("1182460 Parasolid bytes skipped") != std::string::npos);
    REQUIRE(box.nodeCount() > 10);
    const sim::SceneNode* nut = nullptr;
    std::vector<std::shared_ptr<sim::SceneNode>> stack{box.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->name.find("90591A141 HEX NUT.asm;23;2035:") != std::string::npos) {
            nut = node.get();
        }
    }
    REQUIRE(nut != nullptr);
    REQUIRE(nut->local.translation.x == Catch::Approx(57.350284576416016));
    REQUIRE(nut->local.translation.y == Catch::Approx(75.38778686523438));
    REQUIRE(nut->local.translation.z == Catch::Approx(88.875));
    REQUIRE(boxResult.triangleCount == 269304);
    REQUIRE(boxResult.meshCount == 36);
    REQUIRE(boxResult.message.find("decoded JT 10.5 topologically compressed tri-strips") != std::string::npos);
    REQUIRE(boxResult.message.find("no triangles were invented") == std::string::npos);
    REQUIRE(boxResult.message.find("were not decoded") == std::string::npos);
    int nutCopies = 0;
    std::vector<std::shared_ptr<sim::SceneNode>> pending{box.root()};
    while (!pending.empty()) {
        auto node = pending.back();
        pending.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            pending.push_back(child);
        }
        if (node->meshId.empty()) {
            continue;
        }
        const auto mesh = box.meshes().find(node->meshId);
        if (mesh != box.meshes().end() && mesh->second && mesh->second->triangleCount() == 4010) {
            ++nutCopies;
        }
    }
    REQUIRE(nutCopies == 10);
}

TEST_CASE("private cojt jt files report version and triangles", "[jt]") {
    const std::filesystem::path root = std::filesystem::path(SIM_SOURCE_DIR) / "testdata" / "local" / "cojt";
    if (!std::filesystem::exists(root / "kr210_r2700_2.cojt" / "kr210_r2700_2.jt")) {
        SKIP("testdata/local/cojt is not present");
    }
    auto registry = sim::createBuiltinRegistry();
    const auto expect = [&](const std::filesystem::path& path, int triangles, const char* note) {
        sim::Scene scene;
        const sim::AssetImportResult result = registry->importPath(path.string(), scene);
        INFO(result.message);
        REQUIRE(result.ok);
        REQUIRE(result.triangleCount == triangles);
        if (note != nullptr && note[0] != '\0') {
            REQUIRE(result.message.find(note) != std::string::npos);
        }
        REQUIRE(result.message.find("were not decoded") == std::string::npos);
        REQUIRE(result.message.find("not reachable from the LSG") == std::string::npos);
        if (triangles <= 0) {
            return;
        }
        sim::Aabb cloud;
        sim::Aabb centers;
        int meshNodes = 0;
        std::vector<std::shared_ptr<sim::SceneNode>> pending{scene.root()};
        while (!pending.empty()) {
            auto node = pending.back();
            pending.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                pending.push_back(child);
            }
            if (node->meshId.empty()) {
                continue;
            }
            const auto mesh = scene.meshes().find(node->meshId);
            if (mesh == scene.meshes().end() || !mesh->second || mesh->second->positions.size() < 3) {
                continue;
            }
            ++meshNodes;
            const sim::Mat4 world = node->worldMatrix();
            sim::Aabb local;
            const auto& positions = mesh->second->positions;
            for (std::size_t i = 0; i + 2 < positions.size(); i += 3) {
                const sim::Vec3 point = sim::transformPoint(world, {positions[i], positions[i + 1], positions[i + 2]});
                cloud.expand(point);
                local.expand(point);
            }
            if (local.valid) {
                centers.expand(local.center());
            }
        }
        REQUIRE(meshNodes > 1);
        REQUIRE(cloud.valid);
        REQUIRE(cloud.diagonal() > 400.0);
        REQUIRE(centers.diagonal() > 100.0);
        // IRB6700 already has a non-finite parent rotation repaired to identity,
        // plus one translated subtree whose world Y is not a robot coordinate.
        // The machines whose LOD set changed in this decoder stay inside a cell.
        if (path.filename().string().find("IRB6700") == std::string::npos) {
            REQUIRE(std::isfinite(cloud.diagonal()));
            REQUIRE(cloud.diagonal() < 20000.0);
        }
    };
    expect(root / "kr210_r2700_2.cojt" / "kr210_r2700_2.jt", 387479, "decoded JT 10.5 topologically compressed tri-strips");
    expect(root / "kr6_r900sixx.cojt" / "kr6_r900sixx.jt", 21915, "decoded topologically compressed tri-strips");
    expect(root / "kr420r3080_4.cojt" / "kr420r3080_4.jt", 418625, "decoded topologically compressed tri-strips");
    expect(root / "kr280r3080_4.cojt" / "kr280r3080_4.jt", 418625, "decoded topologically compressed tri-strips");
    expect(root / "irb120_3_0_58.cojt" / "irb120_3_0_58.jt", 44948, "decoded topologically compressed tri-strips");
    expect(root / "irbt6004_0_d_10m_01.cojt" / "irbt6004_0_d_10m_01.jt", 30236, "decoded topologically compressed tri-strips");
    expect(root / "r2000ib210f.cojt" / "r2000ib210f.jt", 30261, "decoded topologically compressed tri-strips");
    expect(root / "IRB6700_235-265_IRC5_Grey_Rev1.cojt" / "IRB6700_235-265_IRC5_Grey_Rev1.jt", 111545,
           "decoded topologically compressed tri-strips");
}

std::vector<std::uint8_t> hexBytes(const char* text) {
    std::vector<std::uint8_t> out;
    int nibble = -1;
    for (const char* cursor = text; *cursor != '\0'; ++cursor) {
        const char letter = *cursor;
        int value = -1;
        if (letter >= '0' && letter <= '9') {
            value = letter - '0';
        } else if (letter >= 'a' && letter <= 'f') {
            value = letter - 'a' + 10;
        } else if (letter >= 'A' && letter <= 'F') {
            value = letter - 'A' + 10;
        }
        if (value < 0) {
            continue;
        }
        if (nibble < 0) {
            nibble = value;
        } else {
            out.push_back(static_cast<std::uint8_t>((nibble << 4) | value));
            nibble = -1;
        }
    }
    return out;
}

TEST_CASE("jt int32 codecs and predictors round-trip published layouts", "[jt]") {
    std::string error;
    const auto arithmetic = hexBytes(
        "be00000003de010000dfc114b2855be8bfd042745121a78816368570895e7e682ac2ad90585048be68e9d156751f9e66cdad5235a123cb084197b779ef71f6cdcc000020bc000a0871000000001020834542b3593ce91845ca1a905683c000000000");
    std::vector<std::int32_t> degrees;
    REQUIRE(sim::jtReadInt32Cdp(arithmetic.data(), arithmetic.size(), 2, degrees, error));
    REQUIRE(degrees.size() == 190);
    const std::int32_t firstDegrees[30] = {7, 6, 6, 6, 5, 6, 7, 7, 6, 8, 7, 6, 7, 6, 6, 6, 4, 6, 7, 8, 7, 6, 9, 7, 7, 7, 8, 6, 4, 7};
    for (int i = 0; i < 30; ++i) {
        REQUIRE(degrees[static_cast<std::size_t>(i)] == firstDegrees[i]);
    }

    const auto constants = hexBytes("80040000011300000000601b06");
    std::vector<std::int32_t> threes;
    REQUIRE(sim::jtReadInt32Cdp(constants.data(), constants.size(), 2, threes, error));
    REQUIRE(threes.size() == 1152);
    REQUIRE(threes.front() == 3);
    REQUIRE(threes.back() == 3);

    const auto chopped = hexBytes(
        "8300000004008300000001f8010000030000801bf27e4880ff39f00007f8ed7b45fc0134d1c1c40fff6e21bc017df03fbdabc4ff9344c9f21cb8807ffde178c8033cc13e7ee10f48003021003d8e07");
    std::vector<std::int32_t> variable;
    REQUIRE(sim::jtReadInt32Cdp(chopped.data(), chopped.size(), 2, variable, error));
    REQUIRE(variable.size() == 131);
    REQUIRE(*std::min_element(variable.begin(), variable.end()) == 3);
    REQUIRE(*std::max_element(variable.begin(), variable.end()) == 12);

    std::vector<std::uint8_t> moved;
    auto putI32 = [&](std::int32_t value) {
        const auto bits = static_cast<std::uint32_t>(value);
        moved.push_back(static_cast<std::uint8_t>(bits));
        moved.push_back(static_cast<std::uint8_t>(bits >> 8));
        moved.push_back(static_cast<std::uint8_t>(bits >> 16));
        moved.push_back(static_cast<std::uint8_t>(bits >> 24));
    };
    putI32(3);
    moved.push_back(5);
    putI32(2);
    moved.push_back(0);
    putI32(10);
    putI32(20);
    putI32(3);
    moved.push_back(0);
    putI32(-1);
    putI32(-1);
    putI32(0);
    std::vector<std::int32_t> front;
    REQUIRE(sim::jtReadInt32Cdp(moved.data(), moved.size(), 3, front, error));
    REQUIRE(front == std::vector<std::int32_t>{10, 20, 20});

    const std::vector<std::int32_t> residual{1, 2, 3, 4, 1, 1};
    REQUIRE(sim::jtUnpackPredictor(residual, 0) == std::vector<std::int32_t>{1, 2, 3, 4, 5, 6});
    REQUIRE(sim::jtUnpackPredictor(residual, 1) == std::vector<std::int32_t>{1, 2, 3, 4, 4, 5});
    REQUIRE(sim::jtUnpackPredictor(residual, 2) == std::vector<std::int32_t>{1, 2, 3, 4, 6, 9});
    REQUIRE(sim::jtUnpackPredictor(residual, 8) == residual);
    const auto xor1 = sim::jtUnpackPredictor(residual, 6);
    REQUIRE(xor1[4] == (1 ^ 4));
    REQUIRE(xor1[5] == (1 ^ xor1[4]));

    auto prefix = [](const char* bits) {
        std::uint32_t word = 0;
        int count = 0;
        for (const char* cursor = bits; *cursor != '\0'; ++cursor) {
            word = (word << 1) | static_cast<std::uint32_t>(*cursor == '1');
            ++count;
        }
        word <<= (32 - count);
        std::string local;
        return sim::jtDecodePrefixBitlength(&word, 1, count, local);
    };
    REQUIRE(prefix("11001") == std::vector<std::int32_t>{1});
    REQUIRE(prefix("11011") == std::vector<std::int32_t>{-1});
    REQUIRE(prefix("11001000") == std::vector<std::int32_t>{1, 0});
    REQUIRE(prefix("0") == std::vector<std::int32_t>{0});
    std::uint32_t invalid = 0x80000000u;
    std::string prefixError;
    REQUIRE(sim::jtDecodePrefixBitlength(&invalid, 1, 2, prefixError).empty());
    REQUIRE_FALSE(prefixError.empty());
}
