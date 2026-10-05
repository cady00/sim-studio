#include "JtImporter.hpp"

#include "sim/io/Cojt.hpp"

#include "sim/core/Mesh.hpp"
#include "sim/io/JtCodec.hpp"

#include <lzma.h>
#include <zlib.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace sim {
namespace {

constexpr int kHeaderBytes = 80;
constexpr std::size_t kMaxInflate = 32u * 1024u * 1024u;
constexpr int kMaxSegments = 100000;
constexpr int kMaxElements = 2000000;

struct Guid {
    std::uint8_t b[16]{};

    bool operator==(const Guid& other) const { return std::memcmp(b, other.b, 16) == 0; }
    bool all(std::uint8_t value) const {
        for (std::uint8_t byte : b) {
            if (byte != value) {
                return false;
            }
        }
        return true;
    }
};

struct Reader {
    const std::uint8_t* cur = nullptr;
    const std::uint8_t* end = nullptr;
    bool big = false;
    bool ok = true;

    std::size_t left() const { return static_cast<std::size_t>(end - cur); }

    void need(std::size_t n) {
        if (left() < n) {
            ok = false;
        }
    }

    std::uint8_t u8() {
        need(1);
        if (!ok) {
            return 0;
        }
        return *cur++;
    }

    std::uint16_t u16() {
        need(2);
        if (!ok) {
            return 0;
        }
        std::uint16_t value;
        std::memcpy(&value, cur, 2);
        cur += 2;
        if (big) {
            value = static_cast<std::uint16_t>((value >> 8) | (value << 8));
        }
        return value;
    }

    std::uint32_t u32() {
        need(4);
        if (!ok) {
            return 0;
        }
        std::uint32_t value;
        std::memcpy(&value, cur, 4);
        cur += 4;
        if (big) {
            value = (value >> 24) | ((value >> 8) & 0xff00) | ((value << 8) & 0xff0000) | (value << 24);
        }
        return value;
    }

    std::uint64_t u64() {
        need(8);
        if (!ok) {
            return 0;
        }
        std::uint64_t value;
        std::memcpy(&value, cur, 8);
        cur += 8;
        if (big) {
            std::uint64_t out = 0;
            for (int i = 0; i < 8; ++i) {
                out = (out << 8) | ((value >> (8 * i)) & 0xff);
            }
            return out;
        }
        return value;
    }

    std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
    std::int16_t i16() { return static_cast<std::int16_t>(u16()); }
    float f32() {
        const std::uint32_t bits = u32();
        float value;
        std::memcpy(&value, &bits, 4);
        return value;
    }
    double f64() {
        const std::uint64_t bits = u64();
        double value;
        std::memcpy(&value, &bits, 8);
        return value;
    }

    Guid guid() {
        Guid value;
        need(16);
        if (!ok) {
            return value;
        }
        if (!big) {
            std::memcpy(value.b, cur, 16);
        } else {
            value.b[0] = cur[3];
            value.b[1] = cur[2];
            value.b[2] = cur[1];
            value.b[3] = cur[0];
            value.b[4] = cur[5];
            value.b[5] = cur[4];
            value.b[6] = cur[7];
            value.b[7] = cur[6];
            std::memcpy(value.b + 8, cur + 8, 8);
        }
        cur += 16;
        return value;
    }

    std::string utf16(int chars) {
        if (chars < 0 || static_cast<std::size_t>(chars) > left() / 2) {
            ok = false;
            return {};
        }
        std::string text;
        text.reserve(static_cast<std::size_t>(chars));
        for (int i = 0; i < chars; ++i) {
            const std::uint16_t unit = u16();
            if (unit < 0x80) {
                text.push_back(static_cast<char>(unit));
            } else if (unit < 0x800) {
                text.push_back(static_cast<char>(0xc0 | (unit >> 6)));
                text.push_back(static_cast<char>(0x80 | (unit & 0x3f)));
            } else {
                text.push_back(static_cast<char>(0xe0 | (unit >> 12)));
                text.push_back(static_cast<char>(0x80 | ((unit >> 6) & 0x3f)));
                text.push_back(static_cast<char>(0x80 | (unit & 0x3f)));
            }
        }
        return text;
    }
};

Guid makeGuid(std::uint32_t a, std::uint16_t b, std::uint16_t c, const std::uint8_t tail[8]) {
    Guid guid;
    std::memcpy(guid.b, &a, 4);
    std::memcpy(guid.b + 4, &b, 2);
    std::memcpy(guid.b + 6, &c, 2);
    std::memcpy(guid.b + 8, tail, 8);
    return guid;
}

const std::uint8_t kGraphTail[8] = {0x9b, 0x6b, 0x00, 0x80, 0xc7, 0xbb, 0x59, 0x97};
const std::uint8_t kPartTail[8] = {0xa5, 0x06, 0x00, 0x60, 0x97, 0xbd, 0xc6, 0xe1};
const std::uint8_t kLateTail[8] = {0xa3, 0xa7, 0x00, 0xaa, 0x00, 0xd1, 0x09, 0x54};

const Guid kPartition = makeGuid(0x10dd103e, 0x2ac8, 0x11d1, kGraphTail);
const Guid kGroup = makeGuid(0x10dd101b, 0x2ac8, 0x11d1, kGraphTail);
const Guid kInstance = makeGuid(0x10dd102a, 0x2ac8, 0x11d1, kGraphTail);
const Guid kPart = makeGuid(0xce357244, 0x38fb, 0x11d1, kPartTail);
const Guid kMeta = makeGuid(0xce357245, 0x38fb, 0x11d1, kPartTail);
const Guid kLod = makeGuid(0x10dd102c, 0x2ac8, 0x11d1, kGraphTail);
const Guid kRangeLod = makeGuid(0x10dd104c, 0x2ac8, 0x11d1, kGraphTail);
const Guid kSwitch = makeGuid(0x10dd10f3, 0x2ac8, 0x11d1, kGraphTail);
const Guid kTriStripNode = makeGuid(0x10dd1077, 0x2ac8, 0x11d1, kGraphTail);
const Guid kMaterial = makeGuid(0x10dd1030, 0x2ac8, 0x11d1, kGraphTail);
const Guid kTransform = makeGuid(0x10dd1083, 0x2ac8, 0x11d1, kGraphTail);
const Guid kStringProp = makeGuid(0x10dd106e, 0x2ac8, 0x11d1, kGraphTail);
const Guid kIntProp = makeGuid(0x10dd102b, 0x2ac8, 0x11d1, kGraphTail);
const Guid kFloatProp = makeGuid(0x10dd1019, 0x2ac8, 0x11d1, kGraphTail);
const Guid kLateLoad = makeGuid(0xe0b05be5, 0xfbbd, 0x11d1, kLateTail);
const Guid kTriStripLod = makeGuid(0x10dd10ab, 0x2ac8, 0x11d1, kGraphTail);
const Guid kXtBrep = makeGuid(0x873a70e0, 0x2ac9, 0x11d1, kGraphTail);

bool compressible(int type) {
    return type == 1 || type == 2 || type == 3 || type == 4 || type == 17 || type == 18 || type == 20 || type == 23 ||
           type == 24 || type == 30;
}

const char* segmentName(int type) {
    switch (type) {
    case 1:
        return "lsg";
    case 2:
        return "jt-brep";
    case 3:
        return "pmi";
    case 4:
        return "meta";
    case 6:
        return "shape";
    case 17:
        return "xt-brep";
    case 18:
        return "wireframe";
    case 20:
        return "ulp";
    case 23:
        return "segment-23";
    case 24:
        return "lwpa";
    case 30:
        return "xt-brep-30";
    case 31:
        return "segment-31";
    default:
        if (type >= 7 && type <= 16) {
            return "shape-lod";
        }
        return "segment";
    }
}

bool inflateZlib(const std::uint8_t* data, std::size_t size, std::vector<std::uint8_t>& out, std::string& error) {
    z_stream stream{};
    if (inflateInit(&stream) != Z_OK) {
        error = "zlib init failed";
        return false;
    }
    stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(data));
    stream.avail_in = static_cast<uInt>(size);
    out.clear();
    std::uint8_t block[1 << 15];
    int status = Z_OK;
    while (status != Z_STREAM_END) {
        stream.next_out = block;
        stream.avail_out = sizeof(block);
        status = inflate(&stream, Z_NO_FLUSH);
        if (status != Z_OK && status != Z_STREAM_END) {
            inflateEnd(&stream);
            error = "zlib segment did not inflate";
            return false;
        }
        out.insert(out.end(), block, block + (sizeof(block) - stream.avail_out));
        if (out.size() > kMaxInflate) {
            inflateEnd(&stream);
            error = "zlib segment is larger than 32 MB";
            return false;
        }
    }
    inflateEnd(&stream);
    return true;
}

bool inflateLzma(const std::uint8_t* data, std::size_t size, std::vector<std::uint8_t>& out, std::string& error) {
    lzma_stream stream = LZMA_STREAM_INIT;
    if (lzma_auto_decoder(&stream, UINT64_MAX, 0) != LZMA_OK) {
        error = "lzma init failed";
        return false;
    }
    stream.next_in = data;
    stream.avail_in = size;
    out.clear();
    std::uint8_t block[1 << 15];
    lzma_ret status = LZMA_OK;
    while (status != LZMA_STREAM_END) {
        stream.next_out = block;
        stream.avail_out = sizeof(block);
        status = lzma_code(&stream, LZMA_FINISH);
        if (status != LZMA_OK && status != LZMA_STREAM_END) {
            lzma_end(&stream);
            error = "LZMA/XZ segment did not inflate";
            return false;
        }
        out.insert(out.end(), block, block + (sizeof(block) - stream.avail_out));
        if (out.size() > kMaxInflate) {
            lzma_end(&stream);
            error = "LZMA segment is larger than 32 MB";
            return false;
        }
    }
    lzma_end(&stream);
    return true;
}

struct Segment {
    Guid id;
    int type = 0;
    std::uint64_t offset = 0;
    std::uint32_t length = 0;
    std::vector<std::uint8_t> payload;
};

struct LateRef {
    Guid segment;
    int type = 0;
    int payloadId = 0;
};

enum class NodeRole { Group, Instance, Part, Meta, Lod, RangeLod, Switch, Shape, Other };

struct GraphObject {
    Guid type;
    int id = 0;
    NodeRole role = NodeRole::Other;
    std::vector<int> children;
    std::vector<int> attributes;
    int selectedChild = -1;
    std::string name;
    bool hasTransform = false;
    double matrix[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    bool hasColor = false;
    float diffuse[3] = {0.55f, 0.58f, 0.66f};
    std::shared_ptr<MeshData> mesh;
};

struct PropAtom {
    Guid type;
    std::string text;
    int integer = 0;
    float number = 0;
    LateRef late;
    bool isLate = false;
};

struct BitCursor {
    const std::uint32_t* words = nullptr;
    int wordCount = 0;
    int totalBits = 0;
    int bitPos = 0;
    bool ok = true;

    std::uint32_t readU(int bits) {
        if (bits <= 0) {
            return 0;
        }
        if (bits > 32 || bitPos + bits > totalBits) {
            ok = false;
            return 0;
        }
        std::uint32_t value = 0;
        for (int i = 0; i < bits; ++i) {
            const int word = bitPos / 32;
            const int shift = 31 - (bitPos % 32);
            if (word < 0 || word >= wordCount) {
                ok = false;
                return 0;
            }
            value = (value << 1) | ((words[word] >> shift) & 1u);
            ++bitPos;
        }
        return value;
    }

    std::int32_t readS(int bits) {
        const std::uint32_t raw = readU(bits);
        if (!ok || bits <= 0 || bits >= 32) {
            return static_cast<std::int32_t>(raw);
        }
        const std::uint32_t sign = 1u << (bits - 1);
        if ((raw & sign) == 0) {
            return static_cast<std::int32_t>(raw);
        }
        const std::uint32_t extended = raw | ~((1u << bits) - 1u);
        return static_cast<std::int32_t>(extended);
    }
};

int bitsizeUnsigned(std::uint32_t value) {
    if (value == 0) {
        return 0;
    }
    int bits = 0;
    while (value > 0) {
        value >>= 1;
        ++bits;
    }
    return bits;
}

std::int32_t readNibbleInt(BitCursor& bits) {
    std::int32_t value = 0;
    int shift = 0;
    for (int i = 0; i < 11; ++i) {
        const std::uint32_t nibble = bits.readU(4);
        if (!bits.ok) {
            return 0;
        }
        value |= static_cast<std::int32_t>(nibble & 7u) << shift;
        shift += 3;
        if ((nibble & 8u) == 0) {
            if (shift < 32 && (value & (1 << (shift - 1))) != 0) {
                value |= ~((1 << shift) - 1);
            }
            return value;
        }
    }
    bits.ok = false;
    return 0;
}

// Fixed-width and variable-width bitlength packets from ISO 14306 clause 13.2.2.
// The bitstream is MSB-first inside each stored U32. Signed headers use 4-bit
// nibbles: 3 value bits and a continuation bit.
std::vector<std::int32_t> decodeBitlength(const std::uint32_t* words, int wordCount, int totalBits, int count,
                                          std::string& error) {
    BitCursor cursor;
    cursor.words = words;
    cursor.wordCount = wordCount;
    cursor.totalBits = totalBits;
    std::vector<std::int32_t> values(static_cast<std::size_t>(std::max(count, 0)));
    const std::uint32_t tag = cursor.readU(1);
    if (!cursor.ok) {
        error = "bitlength packet is truncated";
        return {};
    }
    if (tag == 0) {
        const std::int32_t minimum = readNibbleInt(cursor);
        const std::int32_t maximum = readNibbleInt(cursor);
        const int width = bitsizeUnsigned(static_cast<std::uint32_t>(maximum - minimum));
        for (int i = 0; i < count; ++i) {
            values[static_cast<std::size_t>(i)] = static_cast<std::int32_t>(cursor.readU(width)) + minimum;
        }
    } else {
        const std::int32_t mean = readNibbleInt(cursor);
        int width = 0;
        int produced = 0;
        int guard = 0;
        while (produced < count && cursor.ok && guard++ < count * 16 + 8) {
            int delta = 0;
            do {
                delta = cursor.readS(4);
                width += delta;
            } while (cursor.ok && (delta == 7 || delta == -8));
            if (width < 0) {
                width = 0;
            }
            if (width > 32) {
                error = "bitlength field width is invalid";
                return {};
            }
            const std::uint32_t run = cursor.readU(4);
            for (std::uint32_t k = 0; k < run && produced < count; ++k) {
                values[static_cast<std::size_t>(produced++)] = cursor.readS(width) + mean;
            }
        }
        if (produced != count) {
            error = "bitlength packet did not yield Value Count symbols";
            return {};
        }
    }
    if (!cursor.ok) {
        error = "bitlength packet is truncated";
        return {};
    }
    return values;
}

const char* codecName(int codec) {
    switch (codec) {
    case 0:
        return "null";
    case 1:
        return "bitlength";
    case 3:
        return "arithmetic";
    case 4:
        return "chopper";
    case 5:
        return "move-to-front";
    default:
        return "unknown";
    }
}

bool readInt32Packet(Reader& reader, std::vector<std::int32_t>& values, std::string& error) {
    const std::int32_t count = reader.i32();
    if (!reader.ok || count < 0 || count > 5000000) {
        error = "Int32 packet count is invalid";
        return false;
    }
    values.assign(static_cast<std::size_t>(count), 0);
    if (count == 0) {
        return true;
    }
    const int codec = reader.u8();
    if (!reader.ok) {
        error = "Int32 packet is truncated";
        return false;
    }
    if (codec == 0) {
        for (int i = 0; i < count; ++i) {
            values[static_cast<std::size_t>(i)] = reader.i32();
        }
        return reader.ok;
    }
    if (codec == 1) {
        const std::int32_t bitLength = reader.i32();
        if (!reader.ok || bitLength < 0 || bitLength > 1 << 26) {
            error = "bitlength CodeText length is invalid";
            return false;
        }
        const int words = (bitLength + 31) / 32;
        if (reader.left() < static_cast<std::size_t>(words) * 4) {
            error = "bitlength CodeText is truncated";
            return false;
        }
        std::vector<std::uint32_t> text(static_cast<std::size_t>(words));
        for (int i = 0; i < words; ++i) {
            text[static_cast<std::size_t>(i)] = reader.u32();
        }
        values = decodeBitlength(text.data(), words, bitLength, count, error);
        return error.empty() && static_cast<int>(values.size()) == count;
    }
    error = std::string("Int32 codec ") + codecName(codec);
    return false;
}

float dequant(float minimum, float maximum, int bits, std::int32_t code) {
    if (bits <= 0) {
        return minimum;
    }
    const double steps = static_cast<double>((1u << bits) - 1u);
    if (steps <= 0) {
        return minimum;
    }
    return static_cast<float>(minimum + static_cast<double>(code) * (static_cast<double>(maximum) - minimum) / steps);
}

void deeringNormal(std::uint32_t code, int bits, float& x, float& y, float& z) {
    if (bits < 1) {
        bits = 1;
    }
    if (bits > 13) {
        bits = 13;
    }
    const std::uint32_t mask = (1u << bits) - 1u;
    const std::uint32_t sextant = (code >> (bits + bits + 3)) & 7u;
    const std::uint32_t octant = (code >> (bits + bits)) & 7u;
    std::uint32_t theta = (code >> bits) & mask;
    const std::uint32_t psi = code & mask;
    theta += (sextant & 1u);
    const float psiMax = 0.615479709f;
    const float bitRange = static_cast<float>(1 << bits);
    const float thetaArg = psiMax * (bitRange - static_cast<float>(theta)) / bitRange;
    const float fTheta = std::asin(std::tan(thetaArg));
    const float fPsi = psiMax * (static_cast<float>(psi) / bitRange);
    const float cosTheta = std::cos(fTheta);
    const float sinTheta = std::sin(fTheta);
    const float cosPsi = std::cos(fPsi);
    const float sinPsi = std::sin(fPsi);
    const float xx = cosTheta * cosPsi;
    const float yy = sinPsi;
    const float zz = sinTheta * cosPsi;
    x = xx;
    y = yy;
    z = zz;
    switch (sextant) {
    case 1:
        z = xx;
        x = zz;
        break;
    case 2:
        z = xx;
        x = yy;
        y = zz;
        break;
    case 3:
        y = xx;
        x = yy;
        break;
    case 4:
        y = xx;
        z = yy;
        x = zz;
        break;
    case 5:
        z = yy;
        y = zz;
        break;
    default:
        break;
    }
    if ((octant & 4u) == 0) {
        x = -x;
    }
    if ((octant & 2u) == 0) {
        y = -y;
    }
    if ((octant & 1u) == 0) {
        z = -z;
    }
}

// Quantized tri-strip packet. This is the uniform-code path (Null and Bitlength
// Int32CDP), not the DualVFMesh topology used by NX arithmetic files.
bool decodeQuantizedStrips(const std::uint8_t* data, std::size_t size, bool big, std::shared_ptr<MeshData>& mesh,
                           std::string& error) {
    Reader reader{data, data + size, big, true};
    // Version 10.5 writes I8, I8, U64 bindings, then a U32 byte count equal to
    // the payload size minus that 15-byte prefix and a trailing marker. That
    // count is not a quantization bit width.
    if (size >= 15 && data[0] == 1 && data[1] == 1) {
        std::uint32_t framed = 0;
        std::memcpy(&framed, data + 10, 4);
        if (big) {
            framed = (framed >> 24) | ((framed >> 8) & 0xff00u) | ((framed << 8) & 0xff0000u) | (framed << 24);
        }
        if (framed == static_cast<std::uint32_t>(size - 15)) {
            error = "JT 10.5 tri-strip LOD uses a local layout that is not in the published reference";
            return false;
        }
    }
    const int baseVersion = reader.u8();
    const int vertexVersion = reader.u8();
    const std::uint64_t bindings = reader.u64();
    if (!reader.ok || baseVersion != 1 || vertexVersion != 1) {
        error = "tri-strip LOD is not the quantized-strip packet";
        return false;
    }
    const bool hasPosition = (bindings & 0x2ull) != 0;
    const bool hasNormal = (bindings & 0x8ull) != 0;
    if (!hasPosition) {
        error = "tri-strip LOD has no vertex coordinates";
        return false;
    }
    const int positionBits = reader.u8();
    if (!reader.ok || positionBits < 0 || positionBits > 24) {
        error = "tri-strip position bits are out of range";
        return false;
    }
    float minimum[3] = {};
    float maximum[3] = {};
    int componentBits[3] = {positionBits, positionBits, positionBits};
    if (positionBits > 0) {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = reader.f32();
            maximum[axis] = reader.f32();
            componentBits[axis] = reader.u8();
        }
        if (componentBits[0] != positionBits || componentBits[1] != positionBits || componentBits[2] != positionBits) {
            error = "point quantizer component bit counts differ";
            return false;
        }
    }
    std::vector<std::int32_t> codes[3];
    for (int axis = 0; axis < 3; ++axis) {
        if (!readInt32Packet(reader, codes[axis], error)) {
            return false;
        }
        if (axis > 0 && codes[axis].size() != codes[0].size()) {
            error = "tri-strip component counts differ";
            return false;
        }
    }
    const std::size_t vertexCount = codes[0].size();
    std::vector<float> normals;
    if (hasNormal) {
        const int normalBits = reader.u8();
        if (!reader.ok || normalBits < 0 || normalBits > 13) {
            error = "normal quantization bits are out of range";
            return false;
        }
        normals.resize(vertexCount * 3);
        if (normalBits == 0) {
            for (int axis = 0; axis < 3; ++axis) {
                std::vector<std::int32_t> raw;
                if (!readInt32Packet(reader, raw, error) || raw.size() != vertexCount) {
                    error = error.empty() ? "normal component count differs" : error;
                    return false;
                }
                for (std::size_t i = 0; i < vertexCount; ++i) {
                    const std::uint32_t bits = static_cast<std::uint32_t>(raw[i]);
                    float value;
                    std::memcpy(&value, &bits, 4);
                    normals[i * 3 + static_cast<std::size_t>(axis)] = value;
                }
            }
        } else {
            std::vector<std::int32_t> packed;
            if (!readInt32Packet(reader, packed, error) || packed.size() != vertexCount) {
                error = error.empty() ? "Deering normal count differs" : error;
                return false;
            }
            for (std::size_t i = 0; i < vertexCount; ++i) {
                float x = 0;
                float y = 0;
                float z = 0;
                deeringNormal(static_cast<std::uint32_t>(packed[i]), normalBits, x, y, z);
                normals[i * 3] = x;
                normals[i * 3 + 1] = y;
                normals[i * 3 + 2] = z;
            }
        }
    }
    const std::int32_t stripCount = reader.i32();
    if (!reader.ok || stripCount < 0 || stripCount > 1000000) {
        error = "tri-strip count is invalid";
        return false;
    }
    mesh = std::make_shared<MeshData>();
    mesh->positions.resize(vertexCount * 3);
    for (std::size_t i = 0; i < vertexCount; ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            float value = 0;
            if (positionBits == 0) {
                const std::uint32_t bits = static_cast<std::uint32_t>(codes[axis][i]);
                std::memcpy(&value, &bits, 4);
            } else {
                value = dequant(minimum[axis], maximum[axis], positionBits, codes[axis][i]);
            }
            mesh->positions[i * 3 + static_cast<std::size_t>(axis)] = value;
        }
    }
    if (normals.size() == mesh->positions.size()) {
        mesh->normals = normals;
    } else {
        mesh->normals.assign(mesh->positions.size(), 0.f);
    }
    for (int strip = 0; strip < stripCount; ++strip) {
        std::vector<std::int32_t> indices;
        if (!readInt32Packet(reader, indices, error)) {
            return false;
        }
        for (std::size_t i = 2; i < indices.size(); ++i) {
            const int order[3] = {(i % 2) == 0 ? 0 : 1, (i % 2) == 0 ? 1 : 0, 2};
            const std::int32_t tri[3] = {indices[i - 2 + order[0]], indices[i - 2 + order[1]], indices[i - 2 + 2]};
            if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0 || static_cast<std::size_t>(tri[0]) >= vertexCount ||
                static_cast<std::size_t>(tri[1]) >= vertexCount || static_cast<std::size_t>(tri[2]) >= vertexCount) {
                error = "tri-strip index is out of range";
                return false;
            }
            if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) {
                continue;
            }
            mesh->indices.push_back(static_cast<std::uint32_t>(tri[0]));
            mesh->indices.push_back(static_cast<std::uint32_t>(tri[1]));
            mesh->indices.push_back(static_cast<std::uint32_t>(tri[2]));
            if (normals.empty()) {
                const float* a = &mesh->positions[static_cast<std::size_t>(tri[0]) * 3];
                const float* b = &mesh->positions[static_cast<std::size_t>(tri[1]) * 3];
                const float* c = &mesh->positions[static_cast<std::size_t>(tri[2]) * 3];
                const float nx = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]);
                const float ny = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]);
                const float nz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
                const float span = std::sqrt(nx * nx + ny * ny + nz * nz);
                const float inv = span < 1e-12f ? 0.f : 1.f / span;
                for (int k : {tri[0], tri[1], tri[2]}) {
                    mesh->normals[static_cast<std::size_t>(k) * 3] = nx * inv;
                    mesh->normals[static_cast<std::size_t>(k) * 3 + 1] = ny * inv;
                    mesh->normals[static_cast<std::size_t>(k) * 3 + 2] = nz * inv;
                }
            }
        }
    }
    if (reader.left() != 0 || !reader.ok) {
        error = "quantized tri-strip packet did not consume the element";
        mesh.reset();
        return false;
    }
    if (mesh->indices.size() < 3) {
        error = "quantized tri-strip packet has no triangles";
        mesh.reset();
        return false;
    }
    mesh->recomputeBounds();
    error.clear();
    return true;
}

std::string firstCodec(const std::uint8_t* data, std::size_t size, bool big) {
    Reader reader{data, data + size, big, true};
    std::string fallback = "topological";
    while (reader.ok && reader.left() >= 5) {
        const auto* mark = reader.cur;
        const std::int32_t count = reader.i32();
        const int codec = reader.u8();
        if (count > 16 && count < 5000000 && (codec == 3 || codec == 4 || codec == 5)) {
            if (codec == 3) {
                return "arithmetic";
            }
            if (fallback == "topological") {
                fallback = codecName(codec);
            }
        }
        reader.cur = mark + 1;
        reader.ok = true;
    }
    return fallback;
}

bool readCountedIds(Reader& reader, std::vector<int>& ids) {
    const std::int32_t count = reader.i32();
    if (!reader.ok || count < 0 || count > 100000 || reader.left() < static_cast<std::size_t>(count) * 4) {
        return false;
    }
    ids.resize(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        ids[static_cast<std::size_t>(i)] = reader.i32();
    }
    return reader.ok;
}

bool readBaseNode(Reader& reader, std::vector<int>& attributes) {
    reader.u8();
    reader.u32();
    return readCountedIds(reader, attributes);
}

// Process Simulate JT 9.1 writes a zero byte before the attribute count and again
// before the child count. The 9.4/10 layout has no such byte.
bool readBaseNode91(Reader& reader, std::vector<int>& attributes) {
    reader.u8();
    reader.u32();
    if (reader.u8() != 0) {
        return false;
    }
    return readCountedIds(reader, attributes);
}

bool readChildList(Reader& reader, int major, std::vector<int>& children) {
    const auto* start = reader.cur;
    const auto tryOnce = [&](bool extra) {
        reader.cur = start;
        reader.ok = true;
        reader.u8();
        if (extra) {
            reader.u16();
        }
        return readCountedIds(reader, children);
    };
    if (major >= 10) {
        return tryOnce(false) || tryOnce(true);
    }
    return tryOnce(true) || tryOnce(false);
}

bool readChildList91(Reader& reader, std::vector<int>& children) {
    reader.u8();
    if (reader.u8() != 0) {
        return false;
    }
    return readCountedIds(reader, children);
}

std::string readMbString(Reader& reader) {
    const std::int32_t count = reader.i32();
    if (!reader.ok || count < 0 || count > 100000) {
        reader.ok = false;
        return {};
    }
    return reader.utf16(count);
}

std::string fileStem(const std::string& path) {
    std::string name = path;
    const auto slash = name.find_last_of("/\\");
    if (slash != std::string::npos) {
        name = name.substr(slash + 1);
    }
    const auto dot = name.find_last_of('.');
    if (dot != std::string::npos) {
        name = name.substr(0, dot);
    }
    return name.empty() ? path : name;
}

void readTransform(Reader& reader, double matrix[16]) {
    for (int i = 0; i < 16; ++i) {
        matrix[i] = (i % 5 == 0) ? 1.0 : 0.0;
    }
    reader.u8();
    reader.u8();
    reader.u32();
    reader.u32();
    reader.u8();
    const std::uint16_t mask = reader.u16();
    if (!reader.ok) {
        return;
    }
    int stored = 0;
    for (int bit = 0; bit < 16; ++bit) {
        if ((mask & (1u << (15 - bit))) != 0) {
            ++stored;
        }
    }
    const std::size_t remain = reader.left();
    const bool wide = stored > 0 && remain >= static_cast<std::size_t>(stored) * 8 &&
                      remain - static_cast<std::size_t>(stored) * 8 <= 8;
    for (int element = 0; element < 16; ++element) {
        if ((mask & (1u << (15 - element))) == 0) {
            continue;
        }
        matrix[element] = wide ? reader.f64() : static_cast<double>(reader.f32());
    }
}

void readMaterial(Reader& reader, float diffuse[3], bool& hasColor) {
    reader.u8();
    reader.u8();
    reader.u32();
    reader.u32();
    reader.u8();
    reader.u16();
    float rgba[4][4];
    for (int color = 0; color < 4; ++color) {
        for (int channel = 0; channel < 4; ++channel) {
            rgba[color][channel] = reader.f32();
        }
    }
    if (!reader.ok) {
        return;
    }
    // Byte order that matches Version 10.5 samples: ambient, diffuse, specular, emission.
    diffuse[0] = rgba[1][0];
    diffuse[1] = rgba[1][1];
    diffuse[2] = rgba[1][2];
    hasColor = true;
}

bool readStringAtom(const std::uint8_t* data, std::size_t size, bool big, std::string& text) {
    const int offsets[] = {8, 6, 5};
    for (int offset : offsets) {
        if (static_cast<std::size_t>(offset) + 4 > size) {
            continue;
        }
        Reader reader{data + offset, data + size, big, true};
        const std::int32_t count = reader.i32();
        if (!reader.ok || count < 0 || count > 100000) {
            continue;
        }
        if (reader.left() < static_cast<std::size_t>(count) * 2) {
            continue;
        }
        text = reader.utf16(count);
        if (reader.ok && reader.left() <= 4) {
            return true;
        }
    }
    return false;
}

bool readLateAtom(const std::uint8_t* data, std::size_t size, bool big, LateRef& late) {
    const int offsets[] = {8, 6, 5};
    for (int offset : offsets) {
        if (static_cast<std::size_t>(offset) + 24 > size) {
            continue;
        }
        Reader reader{data + offset, data + size, big, true};
        late.segment = reader.guid();
        late.type = reader.i32();
        late.payloadId = reader.i32();
        // Version 9 writes one more I32. Version 10.5 atoms end after the payload id.
        if (reader.left() >= 4) {
            reader.i32();
        }
        if (reader.ok && reader.left() == 0 && late.type >= 0 && late.type < 64) {
            return true;
        }
    }
    return false;
}

Quat quatFromRows(const double* m) {
    const double r00 = m[0];
    const double r01 = m[1];
    const double r02 = m[2];
    const double r10 = m[4];
    const double r11 = m[5];
    const double r12 = m[6];
    const double r20 = m[8];
    const double r21 = m[9];
    const double r22 = m[10];
    const double trace = r00 + r11 + r22;
    Quat rotation;
    if (trace > 0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        rotation.w = 0.25 * s;
        rotation.x = (r21 - r12) / s;
        rotation.y = (r02 - r20) / s;
        rotation.z = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
        rotation.w = (r21 - r12) / s;
        rotation.x = 0.25 * s;
        rotation.y = (r01 + r10) / s;
        rotation.z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
        rotation.w = (r02 - r20) / s;
        rotation.x = (r01 + r10) / s;
        rotation.y = 0.25 * s;
        rotation.z = (r12 + r21) / s;
    } else {
        const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
        rotation.w = (r10 - r01) / s;
        rotation.x = (r02 + r20) / s;
        rotation.y = (r12 + r21) / s;
        rotation.z = 0.25 * s;
    }
    rotation = rotation.normalized();
    if (!std::isfinite(rotation.w) || !std::isfinite(rotation.x) || !std::isfinite(rotation.y) || !std::isfinite(rotation.z)) {
        return {};
    }
    return rotation;
}

struct FileModel {
    std::string version;
    int major = 0;
    int minor = 0;
    std::vector<int> types;
    std::unordered_map<int, GraphObject> nodes;
    std::unordered_map<int, PropAtom> atoms;
    std::unordered_map<int, std::vector<std::pair<int, int>>> properties;
    std::vector<Segment> segments;
    bool xt = false;
    int xtBytes = 0;
    std::string meshNote;
    double unitScale = 1;
    int rootId = -1;
};

bool loadFile(const std::string& path, FileModel& model, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    const std::string bytes = buffer.str();
    if (bytes.size() < 85 || bytes.compare(0, 8, "Version ") != 0) {
        error = "JT header is missing the Version string";
        return false;
    }
    model.version.assign(bytes.data(), kHeaderBytes);
    while (!model.version.empty() && (model.version.back() == ' ' || model.version.back() == '\n' ||
                                      model.version.back() == '\r' || model.version.back() == '\0')) {
        model.version.pop_back();
    }
    const auto number = model.version.find(' ');
    const std::string token = number == std::string::npos ? std::string() : model.version.substr(number + 1);
    const auto dot = token.find('.');
    try {
        model.major = std::stoi(token);
        model.minor = dot == std::string::npos ? 0 : std::stoi(token.substr(dot + 1));
    } catch (const std::exception&) {
        error = "JT version string is not numeric";
        return false;
    }
    if (model.major < 8 || model.major > 10) {
        error = "JT version " + std::to_string(model.major) + " is outside 8.x/9.x/10.x";
        return false;
    }
    Reader header{reinterpret_cast<const std::uint8_t*>(bytes.data()),
                  reinterpret_cast<const std::uint8_t*>(bytes.data()) + bytes.size(), false, true};
    header.cur += kHeaderBytes;
    const int order = header.u8();
    if (order != 0 && order != 1) {
        error = "JT byte order is invalid";
        return false;
    }
    header.big = order == 1;
    const std::int32_t empty = header.i32();
    const std::uint64_t tocOffset = model.major >= 10 ? header.u64() : header.u32();
    if (empty != 0) {
        header.guid();
    }
    const Guid lsgId = header.guid();
    if (!header.ok || tocOffset >= bytes.size()) {
        error = "JT TOC offset is outside the file";
        return false;
    }
    Reader toc{header.cur - (header.cur - (reinterpret_cast<const std::uint8_t*>(bytes.data()))),
               reinterpret_cast<const std::uint8_t*>(bytes.data()) + bytes.size(), header.big, true};
    toc.cur = reinterpret_cast<const std::uint8_t*>(bytes.data()) + tocOffset;
    const std::int32_t entryCount = toc.i32();
    const int entryBytes = model.major >= 10 ? 32 : 28;
    if (!toc.ok || entryCount < 1 || entryCount > kMaxSegments ||
        toc.left() < static_cast<std::size_t>(entryCount) * static_cast<std::size_t>(entryBytes)) {
        error = "JT TOC entry count does not fit the file";
        return false;
    }
    model.segments.resize(static_cast<std::size_t>(entryCount));
    model.types.reserve(static_cast<std::size_t>(entryCount));
    for (int i = 0; i < entryCount; ++i) {
        Segment& segment = model.segments[static_cast<std::size_t>(i)];
        segment.id = toc.guid();
        segment.offset = model.major >= 10 ? toc.u64() : toc.u32();
        segment.length = toc.u32();
        const std::uint32_t attributes = toc.u32();
        segment.type = static_cast<int>(attributes >> 24);
        model.types.push_back(segment.type);
        if (segment.offset > bytes.size() || segment.length < 24 ||
            segment.offset + segment.length > bytes.size()) {
            error = "JT segment extent is outside the file";
            return false;
        }
        const auto* raw = reinterpret_cast<const std::uint8_t*>(bytes.data()) + segment.offset;
        Reader seg{raw, raw + segment.length, header.big, true};
        const Guid id = seg.guid();
        const std::int32_t type = seg.i32();
        const std::int32_t length = seg.i32();
        if (!seg.ok || !(id == segment.id) || type != segment.type || static_cast<std::uint32_t>(length) != segment.length) {
            error = "JT segment header does not match the TOC";
            return false;
        }
        if (compressible(segment.type) && seg.left() >= 9) {
            std::uint32_t flag = 0;
            std::memcpy(&flag, seg.cur, 4);
            if (seg.big) {
                flag = (flag >> 24) | ((flag >> 8) & 0xff00) | ((flag << 8) & 0xff0000) | (flag << 24);
            }
            // Flag 2 is the zlib preface used by 9.x files. Flag 3 is the LZMA/XZ preface in 10.x.
            // Other values mean the element list starts immediately.
            if (flag == 2 || flag == 3) {
                seg.u32();
                const std::int32_t compressedLength = seg.i32();
                const int algorithm = seg.u8();
                if (seg.ok && compressedLength >= 1 && seg.left() >= static_cast<std::size_t>(compressedLength - 1)) {
                    const std::uint8_t* packed = seg.cur;
                    const std::size_t packedSize = static_cast<std::size_t>(compressedLength - 1);
                    if (algorithm == 2 || (flag == 2 && algorithm != 3)) {
                        if (!inflateZlib(packed, packedSize, segment.payload, error)) {
                            return false;
                        }
                    } else if (algorithm == 3 || flag == 3) {
                        if (!inflateLzma(packed, packedSize, segment.payload, error)) {
                            return false;
                        }
                    } else {
                        segment.payload.assign(packed, packed + packedSize);
                    }
                }
            }
        }
        if (segment.payload.empty()) {
            segment.payload.assign(seg.cur, raw + segment.length);
        }
        if (segment.type == 17 || segment.type == 30) {
            model.xt = true;
        }
    }
    const auto lsg = std::find_if(model.segments.begin(), model.segments.end(), [&](const Segment& segment) {
        return segment.id == lsgId || segment.type == 1;
    });
    if (lsg != model.segments.end()) {
        Reader graph{lsg->payload.data(), lsg->payload.data() + lsg->payload.size(), header.big, true};
        int ends = 0;
        int elements = 0;
        while (graph.ok && graph.left() >= 4 && ends < 2 && elements < kMaxElements) {
            const std::int32_t elementLength = graph.i32();
            if (elementLength == 16 && graph.left() >= 16) {
                const Guid marker = graph.guid();
                if (marker.all(0xff)) {
                    ++ends;
                    continue;
                }
                graph.cur -= 16;
            }
            if (elementLength < 21 || graph.left() < static_cast<std::size_t>(elementLength)) {
                break;
            }
            const std::uint8_t* element = graph.cur;
            graph.cur += elementLength;
            ++elements;
            Reader body{element, element + elementLength, header.big, true};
            const Guid type = body.guid();
            body.u8();
            const int objectId = body.i32();
            if (!body.ok) {
                continue;
            }
            if (ends == 0) {
                GraphObject object;
                object.type = type;
                object.id = objectId;
                if (type == kPartition || type == kGroup || type == kPart || type == kMeta || type == kLod ||
                    type == kRangeLod || type == kSwitch) {
                    object.role = type == kPart            ? NodeRole::Part
                                  : type == kMeta          ? NodeRole::Meta
                                  : type == kLod           ? NodeRole::Lod
                                  : type == kRangeLod      ? NodeRole::RangeLod
                                  : type == kSwitch        ? NodeRole::Switch
                                                           : NodeRole::Group;
                    const std::uint8_t* payloadStart = body.cur;
                    if (!readBaseNode(body, object.attributes) || !readChildList(body, model.major, object.children)) {
                        body.cur = payloadStart;
                        body.ok = true;
                        object.attributes.clear();
                        object.children.clear();
                        if (!readBaseNode91(body, object.attributes) || !readChildList91(body, object.children)) {
                            continue;
                        }
                    }
                    if (type == kSwitch && body.left() >= 4) {
                        body.u8();
                        object.selectedChild = body.i32();
                    }
                    if (type == kPartition && body.left() >= 8) {
                        const auto* nameAt = body.cur;
                        if (model.major >= 10) {
                            body.u8();
                            body.i32();
                        } else {
                            body.i32();
                        }
                        std::string filename = readMbString(body);
                        if (!body.ok || filename.empty()) {
                            body.cur = nameAt;
                            body.ok = true;
                        } else {
                            object.name = fileStem(filename);
                        }
                    }
                } else if (type == kInstance) {
                    object.role = NodeRole::Instance;
                    if (!readBaseNode(body, object.attributes)) {
                        continue;
                    }
                    body.u8();
                    if (body.left() >= 2 && model.major < 10) {
                        const auto* mark = body.cur;
                        body.u16();
                        const std::int32_t child = body.i32();
                        if (!body.ok) {
                            body.cur = mark;
                            body.ok = true;
                            object.children.push_back(body.i32());
                        } else {
                            object.children.push_back(child);
                        }
                    } else {
                        object.children.push_back(body.i32());
                    }
                } else if (type == kTriStripNode || type == makeGuid(0x10dd1059, 0x2ac8, 0x11d1, kGraphTail)) {
                    object.role = NodeRole::Shape;
                    readBaseNode(body, object.attributes);
                } else if (type == kTransform) {
                    GraphObject transform;
                    transform.type = type;
                    transform.id = objectId;
                    transform.role = NodeRole::Other;
                    readTransform(body, transform.matrix);
                    transform.hasTransform = body.ok;
                    model.nodes[objectId] = transform;
                    continue;
                } else if (type == kMaterial) {
                    GraphObject material;
                    material.id = objectId;
                    material.type = type;
                    readMaterial(body, material.diffuse, material.hasColor);
                    model.nodes[objectId] = material;
                    continue;
                } else {
                    object.role = NodeRole::Other;
                }
                if (model.rootId < 0 && type == kPartition) {
                    model.rootId = objectId;
                }
                model.nodes[objectId] = object;
            } else {
                PropAtom atom;
                atom.type = type;
                const std::uint8_t* payload = body.cur;
                const std::size_t payloadSize = static_cast<std::size_t>(body.end - body.cur);
                if (type == kStringProp) {
                    readStringAtom(payload, payloadSize, header.big, atom.text);
                } else if (type == kIntProp && payloadSize >= 10) {
                    Reader value{payload, payload + payloadSize, header.big, true};
                    value.cur += std::min<std::size_t>(payloadSize, 6);
                    atom.integer = value.i32();
                } else if (type == kFloatProp && payloadSize >= 10) {
                    Reader value{payload, payload + payloadSize, header.big, true};
                    value.cur += std::min<std::size_t>(payloadSize, 6);
                    atom.number = value.f32();
                } else if (type == kLateLoad && readLateAtom(payload, payloadSize, header.big, atom.late)) {
                    atom.isLate = true;
                }
                model.atoms[objectId] = atom;
            }
        }
        if (ends >= 2 && graph.left() >= 6) {
            const std::int16_t version = graph.i16();
            const std::int32_t tables = graph.i32();
            if (graph.ok && version >= 0 && version < 16 && tables >= 0 && tables < 1000000) {
                for (int i = 0; i < tables && graph.ok; ++i) {
                    const int elementId = graph.i32();
                    std::vector<std::pair<int, int>> pairs;
                    while (graph.ok) {
                        const int key = graph.i32();
                        if (key == 0) {
                            break;
                        }
                        const int value = graph.i32();
                        pairs.emplace_back(key, value);
                    }
                    model.properties[elementId] = pairs;
                }
            }
        }
    }
    for (const auto& entry : model.properties) {
        auto node = model.nodes.find(entry.first);
        if (node == model.nodes.end()) {
            continue;
        }
        for (const auto& pair : entry.second) {
            const auto key = model.atoms.find(pair.first);
            const auto value = model.atoms.find(pair.second);
            if (key == model.atoms.end() || value == model.atoms.end()) {
                continue;
            }
            if (key->second.text == "JT_PROP_SIMP_NAME" || key->second.text == "JT_PROP_NAME" ||
                key->second.text == "Name") {
                if (!value->second.text.empty()) {
                    node->second.name = value->second.text;
                }
            }
            if (key->second.text.find("MEASUREMENT_UNITS") != std::string::npos) {
                std::string units = value->second.text;
                for (char& letter : units) {
                    letter = static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
                }
                // "millimeters" contains "meter", so the milli check has to win.
                if (units.find("milli") != std::string::npos || units == "mm") {
                    model.unitScale = 1;
                } else if (units.find("meter") != std::string::npos || units.find("metre") != std::string::npos ||
                           units == "m") {
                    model.unitScale = 1000;
                }
            }
        }
    }
    std::unordered_map<std::string, const Segment*> byGuid;
    auto guidKey = [](const Guid& guid) {
        std::ostringstream stream;
        for (std::uint8_t byte : guid.b) {
            stream << std::hex << static_cast<int>(byte);
        }
        return stream.str();
    };
    for (const Segment& segment : model.segments) {
        byGuid[guidKey(segment.id)] = &segment;
    }
    bool arithmetic = false;
    bool decodedMesh = false;
    bool decodedTopo = false;
    bool decodedQuant = false;
    bool decoded105 = false;
    int failedLods = 0;
    std::string failNote;
    for (auto& entry : model.nodes) {
        GraphObject& node = entry.second;
        if (node.role != NodeRole::Shape) {
            continue;
        }
        const Segment* chosen = nullptr;
        int chosenType = 99;
        auto consider = [&](int atomId) {
            const auto atom = model.atoms.find(atomId);
            if (atom == model.atoms.end() || !atom->second.isLate) {
                return;
            }
            // Type 6 is the shape segment. The tessellation lives in LOD segments 7..16.
            if (atom->second.late.type < 7 || atom->second.late.type > 16) {
                return;
            }
            const auto found = byGuid.find(guidKey(atom->second.late.segment));
            if (found == byGuid.end()) {
                return;
            }
            if (atom->second.late.type < chosenType) {
                chosenType = atom->second.late.type;
                chosen = found->second;
            }
        };
        const auto props = model.properties.find(node.id);
        if (props != model.properties.end()) {
            for (const auto& pair : props->second) {
                consider(pair.second);
            }
        }
        if (chosen == nullptr) {
            continue;
        }
        Reader shape{chosen->payload.data(), chosen->payload.data() + chosen->payload.size(), header.big, true};
        while (shape.ok && shape.left() >= 25) {
            const std::int32_t elementLength = shape.i32();
            if (elementLength < 21 || shape.left() < static_cast<std::size_t>(elementLength)) {
                break;
            }
            const std::uint8_t* element = shape.cur;
            shape.cur += elementLength;
            Reader body{element, element + elementLength, header.big, true};
            const Guid type = body.guid();
            body.u8();
            body.i32();
            if (type == kTriStripLod) {
                std::string meshError;
                const std::size_t bodySize = static_cast<std::size_t>(body.end - body.cur);
                const bool local105 = bodySize >= 15 && body.cur[0] == 1 && body.cur[1] == 1 && [&]() {
                    std::uint32_t framed = 0;
                    std::memcpy(&framed, body.cur + 10, 4);
                    if (header.big) {
                        framed = (framed >> 24) | ((framed >> 8) & 0xff00u) | ((framed << 8) & 0xff0000u) | (framed << 24);
                    }
                    return framed == static_cast<std::uint32_t>(bodySize - 15);
                }();
                std::shared_ptr<MeshData> decoded;
                if (local105) {
                    if (decodeJt105TriStrip(body.cur, bodySize, header.big, decoded, meshError)) {
                        node.mesh = std::move(decoded);
                        decodedMesh = true;
                        decodedTopo = true;
                        decoded105 = true;
                    } else {
                        ++failedLods;
                        failNote = meshError;
                        model.meshNote = meshError + "; no triangles were invented";
                    }
                } else if (decodeQuantizedStrips(body.cur, bodySize, header.big, decoded, meshError)) {
                    node.mesh = std::move(decoded);
                    decodedMesh = true;
                    decodedQuant = true;
                } else if (decodeTopologicalTriStrip(body.cur, bodySize, model.major, decoded, meshError)) {
                    node.mesh = std::move(decoded);
                    decodedMesh = true;
                    decodedTopo = true;
                } else {
                    ++failedLods;
                    const std::string codec = meshError.find("codec") != std::string::npos
                                                  ? meshError
                                                  : firstCodec(body.cur, bodySize, header.big);
                    const std::string label = codec.find("arithmetic") != std::string::npos   ? "arithmetic"
                                              : codec.find("chopper") != std::string::npos     ? "chopper"
                                              : codec.find("move-to-front") != std::string::npos ? "move-to-front"
                                                                                                : "topological";
                    arithmetic = label == "arithmetic";
                    if (failNote.empty()) {
                        failNote = meshError.empty() ? label : meshError;
                    }
                    if (model.meshNote.empty()) {
                        model.meshNote = meshError.empty()
                                             ? "tri-strip LOD uses " + label +
                                                   " Int32 codec and topological compression; no triangles were invented"
                                             : meshError + "; no triangles were invented";
                    }
                }
            }
        }
    }
    if (decodedMesh) {
        if (decodedTopo && decodedQuant) {
            model.meshNote = "decoded quantized and topologically compressed tri-strips";
        } else if (decoded105) {
            model.meshNote = "decoded JT 10.5 topologically compressed tri-strips";
        } else if (decodedTopo) {
            model.meshNote = "decoded topologically compressed tri-strips";
        } else {
            model.meshNote = "decoded quantized tri-strips";
        }
        if (failedLods > 0) {
            model.meshNote += "; " + std::to_string(failedLods) + " tri-strip LOD(s) were not decoded (" + failNote + ")";
        }
    } else if (model.meshNote.empty() && arithmetic) {
        model.meshNote = "tri-strip LOD uses arithmetic Int32 codec and topological compression; no triangles were invented";
    }
    for (const Segment& segment : model.segments) {
        if (segment.type != 17 && segment.type != 30) {
            continue;
        }
        Reader shape{segment.payload.data(), segment.payload.data() + segment.payload.size(), header.big, true};
        if (shape.left() < 25) {
            continue;
        }
        const std::int32_t elementLength = shape.i32();
        if (elementLength < 21 || shape.left() < static_cast<std::size_t>(elementLength)) {
            continue;
        }
        Reader body{shape.cur, shape.cur + elementLength, header.big, true};
        const Guid type = body.guid();
        if (!(type == kXtBrep)) {
            continue;
        }
        body.u8();
        body.i32();
        body.u8();
        body.u8();
        body.i32();
        body.i32();
        body.i32();
        const auto* lengthAt = body.cur;
        const std::int32_t dataLength = body.i32();
        if (body.ok && dataLength >= 0 && static_cast<std::size_t>(dataLength) <= body.left()) {
            model.xtBytes += dataLength;
        } else {
            // Version 10 files insert one byte between the Parasolid schema version and the byte count.
            body.cur = lengthAt;
            body.ok = true;
            body.u8();
            const std::int32_t wrapped = body.i32();
            if (body.ok && wrapped >= 0 && static_cast<std::size_t>(wrapped) <= body.left()) {
                model.xtBytes += wrapped;
            }
        }
        model.xt = true;
    }
    return true;
}

void scaleMesh(MeshData& mesh, double scale) {
    if (scale == 1) {
        return;
    }
    for (float& value : mesh.positions) {
        value = static_cast<float>(value * scale);
    }
    mesh.recomputeBounds();
}

struct BuiltNode {
    std::shared_ptr<SceneNode> node;
};

int subtreeTriangles(const FileModel& model, int id, int depth) {
    if (depth > 128) {
        return 0;
    }
    const auto found = model.nodes.find(id);
    if (found == model.nodes.end()) {
        return 0;
    }
    int total = 0;
    if (found->second.mesh && !found->second.mesh->empty()) {
        total += static_cast<int>(found->second.mesh->triangleCount());
    }
    for (int child : found->second.children) {
        total += subtreeTriangles(model, child, depth + 1);
    }
    return total;
}

void buildGraph(FileModel& model, Scene& scene, int objectId, const std::shared_ptr<SceneNode>& parent,
                std::vector<int>& stack, std::unordered_map<int, std::string>& placedMeshes, int depth, int& nodes,
                int& meshes, int& triangles) {
    if (depth > 256 || parent == nullptr) {
        return;
    }
    if (std::find(stack.begin(), stack.end(), objectId) != stack.end()) {
        return;
    }
    auto found = model.nodes.find(objectId);
    if (found == model.nodes.end() || found->second.role == NodeRole::Other) {
        return;
    }
    GraphObject& object = found->second;
    const NodeKind kind = object.role == NodeRole::Shape ? NodeKind::Part : NodeKind::Assembly;
    std::string name = object.name.empty() ? segmentName(1) : object.name;
    if (object.name.empty()) {
        name = object.role == NodeRole::Shape ? "tri-strip" : "assembly";
        name += "-" + std::to_string(object.id);
    }
    auto node = scene.makeNode(kind, name);
    for (int attribute : object.attributes) {
        const auto attr = model.nodes.find(attribute);
        if (attr == model.nodes.end()) {
            continue;
        }
        if (attr->second.hasTransform) {
            node->local.translation = {attr->second.matrix[12] * model.unitScale, attr->second.matrix[13] * model.unitScale,
                                       attr->second.matrix[14] * model.unitScale};
            node->local.rotation = quatFromRows(attr->second.matrix);
        }
        if (attr->second.hasColor) {
            node->color = {attr->second.diffuse[0], attr->second.diffuse[1], attr->second.diffuse[2]};
        }
    }
    if (object.mesh && !object.mesh->empty()) {
        const auto placed = placedMeshes.find(object.id);
        if (placed == placedMeshes.end()) {
            node->meshId = scene.addMesh(object.mesh);
            placedMeshes.emplace(object.id, node->meshId);
        } else {
            node->meshId = placed->second;
        }
        ++meshes;
        triangles += static_cast<int>(object.mesh->triangleCount());
    }
    scene.addChild(parent, node);
    ++nodes;
    std::vector<int> children = object.children;
    if (object.role == NodeRole::Lod || object.role == NodeRole::RangeLod) {
        int picked = -1;
        int pickedTriangles = -1;
        for (std::size_t i = 0; i < children.size(); ++i) {
            const int count = subtreeTriangles(model, children[i], 0);
            if (count > pickedTriangles) {
                pickedTriangles = count;
                picked = static_cast<int>(i);
            }
        }
        if (picked >= 0 && pickedTriangles > 0) {
            children = {children[static_cast<std::size_t>(picked)]};
        } else if (!children.empty()) {
            children.resize(1);
        }
    } else if (object.role == NodeRole::Switch && !children.empty()) {
        int index = 0;
        if (object.selectedChild >= 0 && object.selectedChild < static_cast<int>(children.size())) {
            index = object.selectedChild;
        }
        // JT 9.1 often selects an empty branch and keeps the mesh on the other one.
        if (subtreeTriangles(model, children[static_cast<std::size_t>(index)], 0) <= 0) {
            int bestCount = 0;
            for (std::size_t i = 0; i < children.size(); ++i) {
                const int count = subtreeTriangles(model, children[i], 0);
                if (count > bestCount) {
                    bestCount = count;
                    index = static_cast<int>(i);
                }
            }
        }
        children = {children[static_cast<std::size_t>(index)]};
    }
    stack.push_back(objectId);
    for (int child : children) {
        buildGraph(model, scene, child, node, stack, placedMeshes, depth + 1, nodes, meshes, triangles);
    }
    stack.pop_back();
}

}  // namespace

bool JtImporter::probeContent(const std::string& head) const {
    return head.size() >= 8 && head.compare(0, 8, "Version ") == 0 && head.find("JT") != std::string::npos;
}

AssetImportResult JtImporter::importFile(const std::string& path, Scene& scene) const {
    AssetImportResult result;
    result.format = format();
    FileModel model;
    std::string error;
    if (!loadFile(path, model, error)) {
        result.message = error.empty() ? "JT file was not read" : error;
        return result;
    }
    int decodedTriangles = 0;
    for (auto& entry : model.nodes) {
        if (entry.second.mesh && !entry.second.mesh->empty()) {
            decodedTriangles += static_cast<int>(entry.second.mesh->triangleCount());
            scaleMesh(*entry.second.mesh, model.unitScale);
        }
    }
    scene.beginEdit();
    int nodes = 0;
    int meshes = 0;
    int triangles = 0;
    std::vector<int> stack;
    std::unordered_map<int, std::string> placedMeshes;
    if (model.rootId >= 0 && scene.root()) {
        buildGraph(model, scene, model.rootId, scene.root(), stack, placedMeshes, 0, nodes, meshes, triangles);
    }
    scene.endEdit();
    std::vector<int> unique = model.types;
    std::sort(unique.begin(), unique.end());
    unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
    std::ostringstream types;
    for (std::size_t i = 0; i < unique.size(); ++i) {
        if (i != 0) {
            types << ',';
        }
        types << unique[i];
    }
    std::ostringstream message;
    message << "JT " << model.version << " segments=" << model.segments.size() << " types=" << types.str()
            << " nodes=" << nodes << " triangles=" << triangles << " xt=" << (model.xt ? "yes" : "no");
    if (model.xt) {
        message << ". XT B-rep is not decoded; mesh LOD is used";
        if (model.xtBytes > 0) {
            message << " (" << model.xtBytes << " Parasolid bytes skipped)";
        }
    }
    if (!model.meshNote.empty()) {
        message << ". " << model.meshNote;
    }
    if (triangles == 0 && decodedTriangles > 0) {
        message << ". " << decodedTriangles
                << " decoded triangles are not reachable from the LSG; no triangles were invented";
    }
    std::string cojtNote;
    attachCojtFromJt(path, scene, cojtNote);
    if (!cojtNote.empty()) {
        message << ". " << cojtNote;
        nodes = scene.nodeCount();
    }
    result.ok = true;
    result.nodeCount = nodes;
    result.meshCount = meshes;
    result.triangleCount = triangles;
    result.message = message.str();
    return result;
}

}  // namespace sim
