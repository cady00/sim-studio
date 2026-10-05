#include "sim/core/Scene.hpp"
#include "sim/io/AssetImport.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <lzma.h>
#include <zlib.h>

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
    REQUIRE(screwResult.triangleCount == 0);
    REQUIRE(screwResult.message.find("arithmetic") != std::string::npos);
    REQUIRE(screwResult.message.find("xt=no") != std::string::npos);

    sim::Scene ctc;
    const sim::AssetImportResult ctcResult = registry->importPath((root / "ctc02.jt").string(), ctc);
    INFO(ctcResult.message);
    REQUIRE(ctcResult.ok);
    REQUIRE(ctcResult.message.find("Version 9.4") != std::string::npos);
    REQUIRE(ctcResult.message.find("segments=7") != std::string::npos);
    REQUIRE(ctcResult.triangleCount == 0);

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
}
