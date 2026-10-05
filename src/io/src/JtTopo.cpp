#include "sim/io/JtCodec.hpp"

#include "sim/core/Mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>

namespace sim {
namespace {

struct ByteReader {
    const std::uint8_t* cur = nullptr;
    const std::uint8_t* end = nullptr;
    bool ok = true;

    std::size_t left() const { return static_cast<std::size_t>(end - cur); }

    std::uint8_t u8() {
        if (cur >= end) {
            ok = false;
            return 0;
        }
        return *cur++;
    }

    std::uint32_t u32() {
        if (left() < 4) {
            ok = false;
            return 0;
        }
        std::uint32_t value;
        std::memcpy(&value, cur, 4);
        cur += 4;
        return value;
    }

    std::int32_t i32() { return static_cast<std::int32_t>(u32()); }

    std::uint16_t u16() {
        if (left() < 2) {
            ok = false;
            return 0;
        }
        std::uint16_t value;
        std::memcpy(&value, cur, 2);
        cur += 2;
        return value;
    }

    std::int16_t i16() { return static_cast<std::int16_t>(u16()); }

    std::uint64_t u64() {
        if (left() < 8) {
            ok = false;
            return 0;
        }
        std::uint64_t value;
        std::memcpy(&value, cur, 8);
        cur += 8;
        return value;
    }

    float f32() {
        const std::uint32_t bits = u32();
        float value;
        std::memcpy(&value, &bits, 4);
        return value;
    }
};

struct BitReader {
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::size_t pos = 0;
    int bit = 0;
    bool ok = true;

    std::uint32_t read(int n) {
        if (n <= 0) {
            return 0;
        }
        std::uint32_t value = 0;
        for (int i = 0; i < n; ++i) {
            if (pos >= size) {
                ok = false;
                return 0;
            }
            value = (value << 1) | ((data[pos] >> (7 - bit)) & 1u);
            if (++bit == 8) {
                bit = 0;
                ++pos;
            }
        }
        return value;
    }

    std::int32_t readSigned(int n) {
        const std::uint32_t raw = read(n);
        if (n <= 0) {
            return 0;
        }
        if (n >= 32) {
            return static_cast<std::int32_t>(raw);
        }
        if ((raw & (1u << (n - 1))) != 0) {
            return static_cast<std::int32_t>(raw) - (1 << n);
        }
        return static_cast<std::int32_t>(raw);
    }

    void align() {
        if (bit != 0) {
            bit = 0;
            ++pos;
        }
    }
};

struct WordBits {
    const std::uint32_t* words = nullptr;
    int total = 0;
    int pos = 0;
    bool ok = true;

    std::uint32_t readU(int n) {
        if (n <= 0) {
            return 0;
        }
        if (n > 32 || pos + n > total) {
            ok = false;
            return 0;
        }
        std::uint32_t value = 0;
        for (int i = 0; i < n; ++i) {
            const int p = pos++;
            const std::uint32_t bit = (words[p / 32] >> (31 - (p % 32))) & 1u;
            value = (value << 1) | bit;
        }
        return value;
    }

    std::int32_t readS(int n) {
        const std::uint32_t raw = readU(n);
        if (!ok || n <= 0) {
            return 0;
        }
        if (n < 32 && (raw & (1u << (n - 1))) != 0) {
            return static_cast<std::int32_t>(raw) - (1 << n);
        }
        return static_cast<std::int32_t>(raw);
    }
};

int nlz32(std::uint32_t value) {
    if (value == 0) {
        return 32;
    }
    // __builtin_clz is defined for every non-zero value, including 0x80000000.
    // A shift loop that stops on the sign flag does not, and it does not terminate.
    return __builtin_clz(value);
}

int unsignedBits(std::uint32_t value) { return 32 - nlz32(value); }

std::int32_t readNibble(WordBits& bits) {
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
                value -= 1 << shift;
            }
            return value;
        }
    }
    bits.ok = false;
    return 0;
}

// JT 10 bitlength nibbles are four magnitude bits followed by a separate
// continuation bit. The version 9 packet packs three magnitude bits and the
// continuation into the same nibble.
std::int32_t readV10Nibble(WordBits& bits) {
    std::int32_t result = 0;
    int groups = 0;
    while (groups < 8) {
        const std::uint32_t chunk = bits.readU(4);
        const std::uint32_t more = bits.readU(1);
        if (!bits.ok) {
            return 0;
        }
        result |= static_cast<std::int32_t>(chunk) << (groups * 4);
        ++groups;
        if (more == 0) {
            break;
        }
    }
    const int width = groups * 4;
    if (width > 0 && width < 32 && (result & (1 << (width - 1))) != 0) {
        result -= 1 << width;
    }
    return result;
}

bool decodeV10Bitlength(const std::uint32_t* words, int totalBits, int count, std::vector<std::int32_t>& values) {
    WordBits bits;
    bits.words = words;
    bits.total = totalBits;
    const std::uint32_t tag = bits.readU(1);
    values.assign(static_cast<std::size_t>(std::max(count, 0)), 0);
    if (!bits.ok) {
        return false;
    }
    if (tag == 0) {
        const std::int32_t minimum = readV10Nibble(bits);
        const std::int32_t maximum = readV10Nibble(bits);
        if (!bits.ok || maximum < minimum) {
            return false;
        }
        const std::uint32_t span =
            static_cast<std::uint32_t>(static_cast<std::int64_t>(maximum) - static_cast<std::int64_t>(minimum));
        const int width = unsignedBits(span);
        for (int i = 0; i < count; ++i) {
            values[static_cast<std::size_t>(i)] = static_cast<std::int32_t>(bits.readU(width)) + minimum;
        }
    } else {
        const std::int32_t mean = readV10Nibble(bits);
        int width = 0;
        int produced = 0;
        constexpr int kBlock = 4;
        const int maxIncr = (1 << (kBlock - 1)) - 1;
        const int maxDecr = -(1 << (kBlock - 1));
        while (produced < count && bits.ok) {
            int delta = 0;
            do {
                delta = bits.readS(kBlock);
                width += delta;
            } while (bits.ok && (delta == maxIncr || delta == maxDecr));
            if (width < 0 || width > 32) {
                return false;
            }
            const std::uint32_t run = bits.readU(kBlock);
            if (!bits.ok || run == 0) {
                return false;
            }
            for (std::uint32_t k = 0; k < run; ++k) {
                if (produced >= count) {
                    return false;
                }
                values[static_cast<std::size_t>(produced++)] = bits.readS(width) + mean;
            }
        }
        if (produced != count) {
            return false;
        }
    }
    return bits.ok && bits.pos == totalBits;
}

bool decodeNibbleBitlength(const std::uint32_t* words, int totalBits, int count, std::vector<std::int32_t>& values) {
    WordBits bits;
    bits.words = words;
    bits.total = totalBits;
    const std::uint32_t tag = bits.readU(1);
    values.assign(static_cast<std::size_t>(std::max(count, 0)), 0);
    if (!bits.ok) {
        return false;
    }
    if (tag == 0) {
        const std::int32_t minimum = readNibble(bits);
        const std::int32_t maximum = readNibble(bits);
        if (!bits.ok || maximum < minimum) {
            return false;
        }
        const std::uint32_t span =
            static_cast<std::uint32_t>(static_cast<std::int64_t>(maximum) - static_cast<std::int64_t>(minimum));
        const int width = unsignedBits(span);
        for (int i = 0; i < count; ++i) {
            values[static_cast<std::size_t>(i)] = static_cast<std::int32_t>(bits.readU(width)) + minimum;
        }
    } else {
        const std::int32_t mean = readNibble(bits);
        int width = 0;
        int produced = 0;
        constexpr int kBlock = 4;
        const int maxIncr = (1 << (kBlock - 1)) - 1;
        const int maxDecr = -(1 << (kBlock - 1));
        while (produced < count && bits.ok) {
            int delta = 0;
            do {
                delta = bits.readS(kBlock);
                width += delta;
            } while (bits.ok && (delta == maxIncr || delta == maxDecr));
            if (width < 0 || width > 32) {
                return false;
            }
            const std::uint32_t run = bits.readU(kBlock);
            if (!bits.ok || run == 0) {
                return false;
            }
            for (std::uint32_t k = 0; k < run; ++k) {
                if (produced >= count) {
                    return false;
                }
                values[static_cast<std::size_t>(produced++)] = bits.readS(width) + mean;
            }
        }
        if (produced != count) {
            return false;
        }
    }
    return bits.ok && bits.pos == totalBits;
}

bool decodeMk2Bitlength(const std::uint32_t* words, int totalBits, int count, std::vector<std::int32_t>& values) {
    WordBits bits;
    bits.words = words;
    bits.total = totalBits;
    const std::uint32_t tag = bits.readU(1);
    values.clear();
    if (!bits.ok) {
        return false;
    }
    if (tag == 0) {
        const int minBits = static_cast<int>(bits.readU(6));
        const int maxBits = static_cast<int>(bits.readU(6));
        const std::int32_t minimum = bits.readS(minBits);
        const std::int32_t maximum = bits.readS(maxBits);
        if (!bits.ok || maximum < minimum) {
            return false;
        }
        const std::uint32_t span =
            static_cast<std::uint32_t>(static_cast<std::int64_t>(maximum) - static_cast<std::int64_t>(minimum));
        const int width = unsignedBits(span);
        values.resize(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            values[static_cast<std::size_t>(i)] = static_cast<std::int32_t>(bits.readU(width)) + minimum;
        }
    } else {
        const std::int32_t mean = bits.readS(32);
        const int valueBits = static_cast<int>(bits.readU(3));
        const int lengthBits = static_cast<int>(bits.readU(3));
        if (!bits.ok || valueBits < 1 || valueBits > 6 || lengthBits < 1 || lengthBits > 6) {
            return false;
        }
        const int maxIncr = (1 << (valueBits - 1)) - 1;
        const int maxDecr = -(1 << (valueBits - 1));
        int width = 0;
        int produced = 0;
        values.resize(static_cast<std::size_t>(count));
        while (produced < count && bits.ok) {
            int delta = 0;
            do {
                delta = bits.readS(valueBits);
                width += delta;
            } while (bits.ok && (delta == maxIncr || delta == maxDecr));
            if (width < 0) {
                width = 0;
            }
            if (width > 32) {
                return false;
            }
            const std::uint32_t run = bits.readU(lengthBits);
            if (!bits.ok || run == 0) {
                return false;
            }
            for (std::uint32_t k = 0; k < run; ++k) {
                if (produced >= count) {
                    return false;
                }
                values[static_cast<std::size_t>(produced++)] = bits.readS(width) + mean;
            }
        }
        if (produced != count) {
            return false;
        }
    }
    return bits.ok && bits.pos == totalBits;
}

struct ProbEntry {
    bool escape = false;
    int occurrence = 0;
    std::int32_t value = 0;
    int cumulative = 0;
};

bool readProbability(BitReader& bits, int generation, std::vector<ProbEntry>& entries) {
    const int count = static_cast<int>(bits.read(16));
    if (!bits.ok || count < 0 || count > 4096) {
        return false;
    }
    int symbolBits = 0;
    int occurrenceBits = 0;
    int valueBits = 0;
    std::int32_t minimum = 0;
    if (generation >= 3) {
        occurrenceBits = static_cast<int>(bits.read(6));
        valueBits = static_cast<int>(bits.read(7));
        minimum = bits.readSigned(32);
    } else {
        symbolBits = static_cast<int>(bits.read(6));
        occurrenceBits = static_cast<int>(bits.read(6));
        valueBits = static_cast<int>(bits.read(6));
        minimum = bits.readSigned(32);
    }
    if (!bits.ok) {
        return false;
    }
    entries.resize(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        ProbEntry& entry = entries[static_cast<std::size_t>(i)];
        if (generation >= 3) {
            entry.escape = bits.read(1) != 0;
        } else {
            const int symbol = static_cast<int>(bits.read(symbolBits)) - 2;
            entry.escape = symbol == -2;
        }
        entry.occurrence = static_cast<int>(bits.read(occurrenceBits));
        const std::uint32_t stored = bits.read(valueBits);
        entry.value = minimum + static_cast<std::int32_t>(stored);
        if (!bits.ok) {
            return false;
        }
    }
    bits.align();
    return bits.ok;
}

std::uint32_t nextCodeBit(const std::uint32_t* words, int totalBits, int& pos) {
    if (pos >= totalBits) {
        return 0;
    }
    const int p = pos++;
    return (words[p / 32] >> (31 - (p % 32))) & 1u;
}

std::uint32_t takeCodeBits(const std::uint32_t* words, int totalBits, int& pos, int n) {
    std::uint32_t value = 0;
    for (int i = 0; i < n; ++i) {
        value = (value << 1) | nextCodeBit(words, totalBits, pos);
    }
    return value;
}

bool decodeArithmetic(const std::vector<ProbEntry>& table, const std::uint32_t* words, int codeBits, int count,
                      const std::vector<std::int32_t>& outOfBand, std::vector<std::int32_t>& values) {
    int total = 0;
    for (const ProbEntry& entry : table) {
        total += entry.occurrence;
    }
    if (total <= 0) {
        return false;
    }
    std::vector<ProbEntry> entries = table;
    int cumulative = 0;
    for (ProbEntry& entry : entries) {
        entry.cumulative = cumulative;
        cumulative += entry.occurrence;
    }
    int bitPos = 0;
    const int primed = std::min(32, codeBits);
    std::uint32_t buffer = takeCodeBits(words, codeBits, bitPos, primed);
    int buffered = primed;
    if (buffered < 32) {
        buffer <<= (32 - buffered);
    }
    std::uint32_t code = (buffer >> 16) & 0xffffu;
    buffer = (buffer << 16) & 0xffffffffu;
    buffered -= 16;
    std::uint32_t low = 0;
    std::uint32_t high = 0xffff;
    int oob = 0;
    values.resize(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const std::uint32_t span = (high - low + 1) & 0xffffffffu;
        const std::uint32_t rescaled =
            static_cast<std::uint32_t>(((static_cast<std::uint64_t>(code - low + 1) * static_cast<std::uint64_t>(total)) - 1) /
                                       span);
        const ProbEntry* found = nullptr;
        for (const ProbEntry& entry : entries) {
            if (rescaled >= static_cast<std::uint32_t>(entry.cumulative) &&
                rescaled < static_cast<std::uint32_t>(entry.cumulative + entry.occurrence)) {
                found = &entry;
                break;
            }
        }
        if (found == nullptr) {
            return false;
        }
        if (found->escape) {
            if (oob >= static_cast<int>(outOfBand.size())) {
                return false;
            }
            values[static_cast<std::size_t>(i)] = outOfBand[static_cast<std::size_t>(oob++)];
        } else {
            values[static_cast<std::size_t>(i)] = found->value;
        }
        const std::uint32_t range = high - low + 1;
        high = low + static_cast<std::uint32_t>((static_cast<std::uint64_t>(range) *
                                                 static_cast<std::uint64_t>(found->cumulative + found->occurrence)) /
                                                static_cast<std::uint64_t>(total)) -
               1;
        low = low + static_cast<std::uint32_t>((static_cast<std::uint64_t>(range) * static_cast<std::uint64_t>(found->cumulative)) /
                                               static_cast<std::uint64_t>(total));
        high &= 0xffffu;
        low &= 0xffffu;
        while (true) {
            if (((~(high ^ low)) & 0x8000u) != 0) {
            } else if (((low >> 14) & 3u) == 1u && ((high >> 14) & 3u) == 2u) {
                code ^= 0x4000u;
                low &= 0x3fffu;
                high |= 0x4000u;
            } else {
                break;
            }
            low = (low << 1) & 0xffffu;
            high = ((high << 1) | 1u) & 0xffffu;
            if (buffered == 0) {
                const int left = codeBits - bitPos;
                const int got = std::max(0, std::min(32, left));
                buffer = got ? takeCodeBits(words, codeBits, bitPos, got) : 0;
                if (got < 32) {
                    buffer <<= (32 - got);
                }
                buffered = got;
            }
            const std::uint32_t bit = (buffer >> 31) & 1u;
            buffer = (buffer << 1) & 0xffffffffu;
            --buffered;
            code = ((code << 1) | bit) & 0xffffu;
        }
    }
    return true;
}

std::int32_t wrapAdd(std::int32_t value, std::int32_t bias) {
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(value) + static_cast<std::uint32_t>(bias));
}

bool readCdp(ByteReader& reader, int generation, int depth, std::vector<std::int32_t>& values, std::string& error) {
    if (depth > 8) {
        error = "Int32 packet recursion is too deep";
        return false;
    }
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
        if (!reader.ok) {
            error = "null Int32 packet is truncated";
            return false;
        }
        return true;
    }
    if (codec == 1) {
        const std::int32_t bitLength = reader.i32();
        if (!reader.ok || bitLength < 0 || bitLength > (1 << 26)) {
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
        const bool decoded = generation >= 4
                                 ? decodeV10Bitlength(text.data(), bitLength, count, values)
                                 : generation >= 3 ? decodeNibbleBitlength(text.data(), bitLength, count, values)
                                                   : decodeMk2Bitlength(text.data(), bitLength, count, values);
        if (!decoded) {
            error = "bitlength packet did not consume its code text";
            return false;
        }
        return true;
    }
    if (codec == 4) {
        const int chop = reader.u8();
        if (!reader.ok) {
            error = "chopper packet is truncated";
            return false;
        }
        if (chop == 0) {
            if (generation >= 3) {
                error = "JT 10 chopper width is zero";
                return false;
            }
            std::vector<std::int32_t> nested;
            if (!readCdp(reader, generation, depth + 1, nested, error) || static_cast<int>(nested.size()) != count) {
                error = error.empty() ? "unchopped packet size differs" : error;
                return false;
            }
            values = std::move(nested);
            return true;
        }
        const std::int32_t bias = reader.i32();
        const int span = reader.u8();
        if (!reader.ok || chop >= 32 || span < chop || span > 32) {
            error = "chopper width is invalid";
            return false;
        }
        std::vector<std::int32_t> msb;
        std::vector<std::int32_t> lsb;
        if (!readCdp(reader, generation, depth + 1, msb, error) || !readCdp(reader, generation, depth + 1, lsb, error) ||
            static_cast<int>(msb.size()) != count || static_cast<int>(lsb.size()) != count) {
            error = error.empty() ? "chopped packet size differs" : error;
            return false;
        }
        const int shift = span - chop;
        const std::uint32_t mask = shift >= 32 ? 0xffffffffu : (1u << shift) - 1u;
        for (int i = 0; i < count; ++i) {
            const std::uint32_t low = static_cast<std::uint32_t>(lsb[static_cast<std::size_t>(i)]) & mask;
            const std::uint32_t high = static_cast<std::uint32_t>(msb[static_cast<std::size_t>(i)]) << shift;
            values[static_cast<std::size_t>(i)] = wrapAdd(static_cast<std::int32_t>(low | high), bias);
        }
        return true;
    }
    if (codec == 3) {
        std::vector<ProbEntry> entries;
        std::int32_t codeBits = 0;
        std::vector<std::uint32_t> text;
        auto readText = [&]() {
            codeBits = reader.i32();
            if (!reader.ok || codeBits < 0 || codeBits > (1 << 26)) {
                error = "arithmetic CodeText length is invalid";
                return false;
            }
            const int words = (codeBits + 31) / 32;
            if (reader.left() < static_cast<std::size_t>(words) * 4) {
                error = "arithmetic CodeText is truncated";
                return false;
            }
            text.resize(static_cast<std::size_t>(words));
            for (int i = 0; i < words; ++i) {
                text[static_cast<std::size_t>(i)] = reader.u32();
            }
            if (!reader.ok) {
                error = "arithmetic CodeText is truncated";
            }
            return reader.ok;
        };
        if (generation >= 3 && generation < 4) {
            BitReader bits{reader.cur, static_cast<std::size_t>(reader.end - reader.cur), 0, 0, true};
            if (!readProbability(bits, generation, entries)) {
                error = "arithmetic probability context is invalid";
                return false;
            }
            reader.cur += bits.pos;
            if (!readText()) {
                return false;
            }
        } else {
            if (!readText()) {
                return false;
            }
            BitReader bits{reader.cur, static_cast<std::size_t>(reader.end - reader.cur), 0, 0, true};
            if (!readProbability(bits, generation, entries)) {
                error = "arithmetic probability context is invalid";
                return false;
            }
            reader.cur += bits.pos;
        }
        const bool escaped = std::any_of(entries.begin(), entries.end(), [](const ProbEntry& entry) { return entry.escape; });
        std::vector<std::int32_t> outOfBand;
        if (generation < 3 || escaped || codeBits == 0) {
            if (!readCdp(reader, generation, depth + 1, outOfBand, error)) {
                return false;
            }
        }
        if (codeBits == 0) {
            if (static_cast<int>(outOfBand.size()) != count) {
                error = "out-of-band copy size differs";
                return false;
            }
            values = std::move(outOfBand);
            return true;
        }
        if (!decodeArithmetic(entries, text.data(), codeBits, count, outOfBand, values)) {
            error = "arithmetic code text did not decode";
            return false;
        }
        return true;
    }
    if (codec == 5 && generation >= 3) {
        std::vector<std::int32_t> windowValues;
        std::vector<std::int32_t> offsets;
        if (!readCdp(reader, generation, depth + 1, windowValues, error) ||
            !readCdp(reader, generation, depth + 1, offsets, error) || static_cast<int>(offsets.size()) != count) {
            error = error.empty() ? "move-to-front packet is invalid" : error;
            return false;
        }
        std::vector<std::int32_t> window;
        int next = 0;
        for (int i = 0; i < count; ++i) {
            const std::int32_t offset = offsets[static_cast<std::size_t>(i)];
            std::int32_t value = 0;
            if (offset < 0 || offset >= static_cast<std::int32_t>(window.size())) {
                if (next >= static_cast<int>(windowValues.size())) {
                    error = "move-to-front window is exhausted";
                    return false;
                }
                value = windowValues[static_cast<std::size_t>(next++)];
            } else {
                value = window[static_cast<std::size_t>(offset)];
                window.erase(window.begin() + offset);
            }
            window.insert(window.begin(), value);
            if (window.size() > 16) {
                window.pop_back();
            }
            values[static_cast<std::size_t>(i)] = value;
        }
        return true;
    }
    error = "Int32 codec " + std::to_string(codec) + " is not decoded";
    return false;
}

void mixHash(std::uint32_t& a, std::uint32_t& b, std::uint32_t& c) {
    a = a - b - c;
    a ^= c >> 13;
    b = b - c - a;
    b ^= a << 8;
    c = c - a - b;
    c ^= b >> 13;
    a = a - b - c;
    a ^= c >> 12;
    b = b - c - a;
    b ^= a << 16;
    c = c - a - b;
    c ^= b >> 5;
    a = a - b - c;
    a ^= c >> 3;
    b = b - c - a;
    b ^= a << 10;
    c = c - a - b;
    c ^= b >> 15;
}

std::uint32_t hash32(const std::int32_t* words, int count, std::uint32_t seed) {
    std::uint32_t a = 0x9e3779b9u;
    std::uint32_t b = 0x9e3779b9u;
    std::uint32_t c = seed;
    int i = 0;
    int left = count;
    while (left >= 3) {
        a += static_cast<std::uint32_t>(words[i]);
        b += static_cast<std::uint32_t>(words[i + 1]);
        c += static_cast<std::uint32_t>(words[i + 2]);
        mixHash(a, b, c);
        i += 3;
        left -= 3;
    }
    c += static_cast<std::uint32_t>(count);
    if (left >= 2) {
        b += static_cast<std::uint32_t>(words[i + 1]);
    }
    if (left >= 1) {
        a += static_cast<std::uint32_t>(words[i]);
    }
    mixHash(a, b, c);
    return c;
}

std::uint32_t hash32u(const std::uint32_t* words, int count, std::uint32_t seed) {
    return hash32(reinterpret_cast<const std::int32_t*>(words), count, seed);
}

std::uint32_t hash16(const std::uint16_t* words, int count, std::uint32_t seed) {
    std::uint32_t a = 0x9e3779b9u;
    std::uint32_t b = 0x9e3779b9u;
    std::uint32_t c = seed;
    int i = 0;
    while (count - i >= 6) {
        a += words[i] + (static_cast<std::uint32_t>(words[i + 1]) << 16);
        b += words[i + 2] + (static_cast<std::uint32_t>(words[i + 3]) << 16);
        c += words[i + 4] + (static_cast<std::uint32_t>(words[i + 5]) << 16);
        mixHash(a, b, c);
        i += 6;
    }
    c += static_cast<std::uint32_t>(count);
    const int rest = count - i;
    if (rest >= 5) {
        c += static_cast<std::uint32_t>(words[i + 4]) << 16;
    }
    if (rest >= 4) {
        b += static_cast<std::uint32_t>(words[i + 3]) << 16;
    }
    if (rest >= 3) {
        b += words[i + 2];
    }
    if (rest >= 2) {
        a += static_cast<std::uint32_t>(words[i + 1]) << 16;
    }
    if (rest >= 1) {
        a += words[i];
    }
    mixHash(a, b, c);
    return c;
}

struct Face {
    std::vector<int> verts;
    std::uint64_t mask = 0;
};

struct DualMesh {
    std::vector<std::vector<int>> verts;
    std::vector<int> flags;
    std::vector<Face> faces;

    int newVertex(int valence, int flag) {
        verts.emplace_back(static_cast<std::size_t>(valence), -1);
        flags.push_back(flag);
        return static_cast<int>(verts.size()) - 1;
    }

    int newFace(int degree, std::uint64_t mask) {
        Face face;
        face.verts.assign(static_cast<std::size_t>(degree), -1);
        face.mask = mask;
        faces.push_back(std::move(face));
        return static_cast<int>(faces.size()) - 1;
    }

    int valence(int vertex) const { return static_cast<int>(verts[static_cast<std::size_t>(vertex)].size()); }
    int degree(int face) const { return static_cast<int>(faces[static_cast<std::size_t>(face)].verts.size()); }
    int faceAt(int vertex, int slot) const { return verts[static_cast<std::size_t>(vertex)][static_cast<std::size_t>(slot)]; }
    int vertAt(int face, int slot) const { return faces[static_cast<std::size_t>(face)].verts[static_cast<std::size_t>(slot)]; }

    bool setVertexFace(int vertex, int slot, int face) {
        if (vertex < 0 || slot < 0 || vertex >= static_cast<int>(verts.size()) || slot >= valence(vertex)) {
            return false;
        }
        verts[static_cast<std::size_t>(vertex)][static_cast<std::size_t>(slot)] = face;
        return true;
    }

    bool setFaceVertex(int face, int slot, int vertex) {
        if (face < 0 || slot < 0 || face >= static_cast<int>(faces.size()) || slot >= degree(face)) {
            return false;
        }
        faces[static_cast<std::size_t>(face)].verts[static_cast<std::size_t>(slot)] = vertex;
        return true;
    }

    int findVertexSlot(int face, int vertex) const {
        const auto& ring = faces[static_cast<std::size_t>(face)].verts;
        for (int i = 0; i < static_cast<int>(ring.size()); ++i) {
            if (ring[static_cast<std::size_t>(i)] == vertex) {
                return i;
            }
        }
        return -1;
    }

    int findFaceSlot(int vertex, int face) const {
        const auto& ring = verts[static_cast<std::size_t>(vertex)];
        for (int i = 0; i < static_cast<int>(ring.size()); ++i) {
            if (ring[static_cast<std::size_t>(i)] == face) {
                return i;
            }
        }
        return -1;
    }

    int emptySlots(int face) const {
        int empty = 0;
        for (int vertex : faces[static_cast<std::size_t>(face)].verts) {
            if (vertex < 0) {
                ++empty;
            }
        }
        return empty;
    }
};

int incMod(int value, int size) { return (value + 1) % size; }
int decMod(int value, int size) { return (value + size - 1) % size; }

struct SymbolCursor {
    std::vector<std::vector<std::int32_t>> degrees;
    std::vector<std::int32_t> valences;
    std::vector<std::int32_t> groups;
    std::vector<std::int32_t> flags;
    std::vector<std::uint64_t> masks[8];
    std::vector<std::int32_t> splits;
    std::vector<std::int32_t> positions;
    int degreeAt[8]{};
    int maskAt[8]{};
    int valenceAt = 0;
    int groupAt = 0;
    int flagAt = 0;
    int splitAt = 0;
    int positionAt = 0;
};

bool readSymbols(ByteReader& reader, int generation, SymbolCursor& symbols, std::string& error) {
    symbols.degrees.resize(8);
    for (int i = 0; i < 8; ++i) {
        if (!readCdp(reader, generation, 0, symbols.degrees[static_cast<std::size_t>(i)], error)) {
            return false;
        }
    }
    if (!readCdp(reader, generation, 0, symbols.valences, error) || !readCdp(reader, generation, 0, symbols.groups, error) ||
        !readCdp(reader, generation, 0, symbols.flags, error)) {
        return false;
    }
    symbols.flags = jtUnpackPredictor(symbols.flags, 0);
    std::vector<std::vector<std::int32_t>> low(8);
    for (int i = 0; i < 8; ++i) {
        if (!readCdp(reader, generation, 0, low[static_cast<std::size_t>(i)], error)) {
            return false;
        }
    }
    std::vector<std::int32_t> mid;
    std::vector<std::int32_t> high;
    if (generation >= 3) {
        if (!readCdp(reader, generation, 0, mid, error)) {
            return false;
        }
    } else {
        if (!readCdp(reader, generation, 0, mid, error) || !readCdp(reader, generation, 0, high, error)) {
            return false;
        }
    }
    if (mid.size() != low[7].size() || (generation < 3 && high.size() != low[7].size())) {
        error = "face-attribute mask widths differ";
        return false;
    }
    for (int context = 0; context < 7; ++context) {
        symbols.masks[context].reserve(low[static_cast<std::size_t>(context)].size());
        for (std::int32_t value : low[static_cast<std::size_t>(context)]) {
            symbols.masks[context].push_back(static_cast<std::uint32_t>(value) & 0x3fffffffu);
        }
    }
    symbols.masks[7].resize(low[7].size());
    for (std::size_t i = 0; i < low[7].size(); ++i) {
        if (generation >= 3) {
            const std::uint64_t lo = static_cast<std::uint32_t>(low[7][i]);
            const std::uint64_t hi = static_cast<std::uint32_t>(mid[i]);
            symbols.masks[7][i] = lo | (hi << 32);
        } else {
            const std::uint64_t lo = static_cast<std::uint32_t>(low[7][i]) & 0x3fffffffu;
            const std::uint64_t md = static_cast<std::uint32_t>(mid[i]) & 0x3fffffffu;
            const std::uint64_t hi = static_cast<std::uint32_t>(high[i]) & 0xfu;
            symbols.masks[7][i] = lo | (md << 30) | (hi << 60);
        }
    }
    const std::int32_t largeCount = reader.i32();
    if (!reader.ok || largeCount < 0 || reader.left() < static_cast<std::size_t>(largeCount) * 4) {
        error = "high-degree face mask count is invalid";
        return false;
    }
    std::vector<std::uint32_t> large(static_cast<std::size_t>(largeCount));
    for (int i = 0; i < largeCount; ++i) {
        large[static_cast<std::size_t>(i)] = reader.u32();
    }
    if (!large.empty()) {
        error = "high-degree face masks are not used by this mesh";
        return false;
    }
    if (!readCdp(reader, generation, 0, symbols.splits, error) || !readCdp(reader, generation, 0, symbols.positions, error)) {
        return false;
    }
    symbols.splits = jtUnpackPredictor(symbols.splits, 0);
    const std::uint32_t stored = reader.u32();
    if (!reader.ok) {
        error = "topology hash is truncated";
        return false;
    }
    std::uint32_t hash = 0;
    for (const auto& degree : symbols.degrees) {
        hash = hash32(degree.data(), static_cast<int>(degree.size()), hash);
    }
    hash = hash32(symbols.valences.data(), static_cast<int>(symbols.valences.size()), hash);
    hash = hash32(symbols.groups.data(), static_cast<int>(symbols.groups.size()), hash);
    std::vector<std::uint16_t> flag16(symbols.flags.size());
    for (std::size_t i = 0; i < symbols.flags.size(); ++i) {
        flag16[i] = static_cast<std::uint16_t>(symbols.flags[i]);
    }
    hash = hash16(flag16.data(), static_cast<int>(flag16.size()), hash);
    for (int context = 0; context < 7; ++context) {
        std::vector<std::uint32_t> words(symbols.masks[context].size());
        for (std::size_t i = 0; i < words.size(); ++i) {
            words[i] = static_cast<std::uint32_t>(symbols.masks[context][i]);
        }
        hash = hash32u(words.data(), static_cast<int>(words.size()), hash);
    }
    std::vector<std::uint32_t> lowWords(symbols.masks[7].size());
    std::vector<std::uint32_t> midWords(symbols.masks[7].size());
    std::vector<std::uint32_t> highWords;
    for (std::size_t i = 0; i < symbols.masks[7].size(); ++i) {
        if (generation >= 3) {
            lowWords[i] = static_cast<std::uint32_t>(symbols.masks[7][i]);
            midWords[i] = static_cast<std::uint32_t>(symbols.masks[7][i] >> 32);
        } else {
            lowWords[i] = static_cast<std::uint32_t>(symbols.masks[7][i] & 0x3fffffffu);
            midWords[i] = static_cast<std::uint32_t>((symbols.masks[7][i] >> 30) & 0x3fffffffu);
        }
    }
    hash = hash32u(lowWords.data(), static_cast<int>(lowWords.size()), hash);
    hash = hash32u(midWords.data(), static_cast<int>(midWords.size()), hash);
    if (generation < 3) {
        highWords.resize(symbols.masks[7].size());
        for (std::size_t i = 0; i < highWords.size(); ++i) {
            highWords[i] = static_cast<std::uint32_t>((symbols.masks[7][i] >> 60) & 0xfu);
        }
        hash = hash32u(highWords.data(), static_cast<int>(highWords.size()), hash);
    }
    hash = hash32u(large.data(), static_cast<int>(large.size()), hash);
    hash = hash32(symbols.splits.data(), static_cast<int>(symbols.splits.size()), hash);
    hash = hash32(symbols.positions.data(), static_cast<int>(symbols.positions.size()), hash);
    if (hash != stored) {
        error = "topology composite hash does not match";
        return false;
    }
    return true;
}

bool walkDual(SymbolCursor& symbols, DualMesh& dual, std::string& error) {
    std::vector<int> active;
    std::vector<char> removed;
    auto nextDegree = [&](int context) -> int {
        if (symbols.degreeAt[context] >= static_cast<int>(symbols.degrees[static_cast<std::size_t>(context)].size())) {
            return std::numeric_limits<int>::min();
        }
        return symbols.degrees[static_cast<std::size_t>(context)][static_cast<std::size_t>(symbols.degreeAt[context]++)];
    };
    auto nextMask = [&](int context) -> std::uint64_t {
        if (symbols.maskAt[context] >= static_cast<int>(symbols.masks[context].size())) {
            return std::numeric_limits<std::uint64_t>::max();
        }
        return symbols.masks[context][static_cast<std::size_t>(symbols.maskAt[context]++)];
    };
    auto faceContext = [&](int vertex) {
        const int valence = dual.valence(vertex);
        int known = 0;
        int total = 0;
        for (int slot = 0; slot < valence; ++slot) {
            const int face = dual.faceAt(vertex, slot);
            if (face < 0 || face >= static_cast<int>(dual.faces.size())) {
                continue;
            }
            ++known;
            total += dual.degree(face);
        }
        if (valence == 3) {
            if (total < known * 6) {
                return 0;
            }
            return total == known * 6 ? 1 : 2;
        }
        if (valence == 4) {
            if (total < known * 4) {
                return 3;
            }
            return total == known * 4 ? 4 : 5;
        }
        return valence == 5 ? 6 : 7;
    };
    auto ioVertex = [&]() {
        if (symbols.valenceAt >= static_cast<int>(symbols.valences.size())) {
            return -1;
        }
        const int valence = symbols.valences[static_cast<std::size_t>(symbols.valenceAt++)];
        const int group = symbols.groupAt < static_cast<int>(symbols.groups.size())
                              ? symbols.groups[static_cast<std::size_t>(symbols.groupAt++)]
                              : 0;
        (void)group;
        const int flag = symbols.flagAt < static_cast<int>(symbols.flags.size())
                             ? symbols.flags[static_cast<std::size_t>(symbols.flagAt++)]
                             : 0;
        if (valence <= 0 || valence > 256) {
            return -2;
        }
        return dual.newVertex(valence, flag);
    };
    auto ioFace = [&](int vertex) -> int {
        const int context = faceContext(vertex);
        const int degree = nextDegree(context);
        if (degree == std::numeric_limits<int>::min()) {
            error = "face-degree context is exhausted";
            return -2;
        }
        if (degree == 0) {
            return -1;
        }
        if (degree < 0 || degree > 64) {
            error = "face degree is outside the decoded mask";
            return -2;
        }
        const int maskContext = std::min(7, std::max(0, degree - 2));
        const std::uint64_t mask = nextMask(maskContext);
        if (mask == std::numeric_limits<std::uint64_t>::max()) {
            error = "face-attribute mask is exhausted";
            return -2;
        }
        return dual.newFace(degree, mask);
    };
    std::function<bool(int, int, int, int)> addVertexToFace = [&](int vertex, int vertexSlot, int face, int faceSlot) {
        const int cw = decMod(faceSlot, dual.degree(face));
        const int ccw = incMod(faceSlot, dual.degree(face));
        if (!dual.setFaceVertex(face, faceSlot, vertex)) {
            return false;
        }
        const int previous = dual.vertAt(face, cw);
        if (previous >= 0) {
            int previousSlot = dual.findFaceSlot(previous, face);
            const int slot = incMod(vertexSlot, dual.valence(vertex));
            if (previousSlot >= 0 && dual.faceAt(vertex, slot) < 0) {
                previousSlot = decMod(previousSlot, dual.valence(previous));
                dual.setVertexFace(vertex, slot, dual.faceAt(previous, previousSlot));
            }
        }
        const int next = dual.vertAt(face, ccw);
        if (next >= 0) {
            int nextSlot = dual.findFaceSlot(next, face);
            const int slot = decMod(vertexSlot, dual.valence(vertex));
            if (nextSlot >= 0 && dual.faceAt(vertex, slot) < 0) {
                nextSlot = incMod(nextSlot, dual.valence(next));
                dual.setVertexFace(vertex, slot, dual.faceAt(next, nextSlot));
            }
        }
        return true;
    };
    std::function<int(int, int)> activateFace = [&](int vertex, int slot) -> int {
        const int face = ioFace(vertex);
        if (face >= 0) {
            if (!dual.setVertexFace(vertex, slot, face) || !dual.setFaceVertex(face, 0, vertex)) {
                return -2;
            }
            active.push_back(face);
            return face;
        }
        if (face == -1) {
            if (symbols.splitAt >= static_cast<int>(symbols.splits.size()) ||
                symbols.positionAt >= static_cast<int>(symbols.positions.size())) {
                error = "split-face list is exhausted";
                return -2;
            }
            const int offset = symbols.splits[static_cast<std::size_t>(symbols.splitAt++)];
            const int splitSlot = symbols.positions[static_cast<std::size_t>(symbols.positionAt++)];
            if (offset <= 0 || offset > static_cast<int>(active.size())) {
                error = "split-face offset is outside the active list";
                return -2;
            }
            const int real = active[active.size() - static_cast<std::size_t>(offset)];
            dual.setVertexFace(vertex, slot, real);
            if (!addVertexToFace(vertex, slot, real, splitSlot)) {
                return -2;
            }
            return real;
        }
        return face;
    };
    std::function<void(int, int)> completeVertex = [&](int vertex, int slot) {
        const int valence = dual.valence(vertex);
        int previousFace = dual.faceAt(vertex, 0);
        int previousSlot = slot;
        int i = 1;
        while (i < valence && dual.faceAt(vertex, i) >= 0) {
            const int nextFace = dual.faceAt(vertex, i);
            previousSlot = decMod(previousSlot, dual.degree(previousFace));
            const int across = dual.vertAt(previousFace, previousSlot);
            if (across < 0) {
                break;
            }
            int acrossSlot = dual.findVertexSlot(nextFace, across);
            if (acrossSlot < 0) {
                break;
            }
            acrossSlot = decMod(acrossSlot, dual.degree(nextFace));
            addVertexToFace(vertex, i, nextFace, acrossSlot);
            previousFace = nextFace;
            previousSlot = acrossSlot;
            ++i;
            if (i >= valence) {
                return;
            }
        }
        const int last = i;
        previousFace = dual.faceAt(vertex, 0);
        previousSlot = slot;
        i = valence - 1;
        while (i >= last && dual.faceAt(vertex, i) >= 0) {
            const int nextFace = dual.faceAt(vertex, i);
            previousSlot = incMod(previousSlot, dual.degree(previousFace));
            const int across = dual.vertAt(previousFace, previousSlot);
            if (across < 0) {
                break;
            }
            int acrossSlot = dual.findVertexSlot(nextFace, across);
            if (acrossSlot < 0) {
                break;
            }
            acrossSlot = incMod(acrossSlot, dual.degree(nextFace));
            addVertexToFace(vertex, i, nextFace, acrossSlot);
            previousFace = nextFace;
            previousSlot = acrossSlot;
            --i;
            if (i < last) {
                return;
            }
        }
        for (int unresolved = last; unresolved <= i; ++unresolved) {
            if (activateFace(vertex, unresolved) < -1) {
                error = error.empty() ? "topology walker could not activate a face" : error;
                return;
            }
        }
    };
    auto completeFace = [&](int face) {
        while (error.empty()) {
            int slot = -1;
            const auto& ring = dual.faces[static_cast<std::size_t>(face)].verts;
            for (int i = 0; i < static_cast<int>(ring.size()); ++i) {
                if (ring[static_cast<std::size_t>(i)] < 0) {
                    slot = i;
                    break;
                }
            }
            if (slot < 0) {
                break;
            }
            const int vertex = ioVertex();
            if (vertex < 0) {
                error = "topology walker ran out of vertices";
                return;
            }
            dual.setVertexFace(vertex, 0, face);
            addVertexToFace(vertex, 0, face, slot);
            if (dual.vertAt(face, slot) != vertex || dual.faceAt(vertex, 0) != face) {
                error = "face-vertex link is inconsistent";
                return;
            }
            completeVertex(vertex, slot);
        }
    };
    auto nextActive = [&]() {
        while (!active.empty() && active.back() < static_cast<int>(removed.size()) && removed[static_cast<std::size_t>(active.back())]) {
            active.pop_back();
        }
        int best = -1;
        int bestEmpty = 1000000000;
        int index = static_cast<int>(active.size()) - 1;
        // The 16-face window is recomputed after each deletion. A fixed stop
        // drops faces the published walker still considers.
        while (index >= 0 && index >= static_cast<int>(active.size()) - 16) {
            const int face = active[static_cast<std::size_t>(index)];
            if (face < static_cast<int>(removed.size()) && removed[static_cast<std::size_t>(face)]) {
                active.erase(active.begin() + index);
                --index;
                continue;
            }
            const int empty = dual.emptySlots(face);
            if (empty < bestEmpty) {
                bestEmpty = empty;
                best = face;
            }
            --index;
        }
        return best;
    };
    while (error.empty()) {
        const int seed = ioVertex();
        if (seed == -1) {
            break;
        }
        if (seed < 0) {
            error = "vertex valence is invalid";
            break;
        }
        for (int slot = 0; slot < dual.valence(seed); ++slot) {
            if (activateFace(seed, slot) < -1) {
                if (error.empty()) {
                    error = "seed face was not created";
                }
                break;
            }
        }
        while (error.empty()) {
            const int face = nextActive();
            if (face < 0) {
                break;
            }
            completeFace(face);
            if (face >= static_cast<int>(removed.size())) {
                removed.resize(static_cast<std::size_t>(face) + 1);
            }
            removed[static_cast<std::size_t>(face)] = 1;
        }
    }
    if (!error.empty()) {
        return false;
    }
    for (int i = 0; i < 8; ++i) {
        if (symbols.degreeAt[i] != static_cast<int>(symbols.degrees[static_cast<std::size_t>(i)].size()) ||
            symbols.maskAt[i] != static_cast<int>(symbols.masks[i].size())) {
            error = "topology symbols were not consumed";
            return false;
        }
    }
    if (symbols.valenceAt != static_cast<int>(symbols.valences.size()) ||
        symbols.splitAt != static_cast<int>(symbols.splits.size()) ||
        symbols.positionAt != static_cast<int>(symbols.positions.size())) {
        error = "topology symbols were not consumed";
        return false;
    }
    return true;
}

float dequant(float minimum, float maximum, int bits, std::int32_t code) {
    if (bits <= 0) {
        return minimum;
    }
    const double steps = static_cast<double>((1u << bits) - 1u);
    return static_cast<float>(minimum + static_cast<double>(code) * (static_cast<double>(maximum) - minimum) / steps);
}

float losslessFloat(std::int32_t exponent, std::int32_t mantissa) {
    const std::uint32_t sign = (static_cast<std::uint32_t>(exponent) >> 8) & 1u;
    const std::uint32_t biased = static_cast<std::uint32_t>(exponent) & 0xffu;
    const std::uint32_t fraction = static_cast<std::uint32_t>(mantissa) & 0x7fffffu;
    const std::uint32_t bits = (sign << 31) | (biased << 23) | fraction;
    float value;
    std::memcpy(&value, &bits, 4);
    return value;
}

void deering(int sextant, int octant, int theta, int psi, int bits, float& x, float& y, float& z) {
    theta += (sextant & 1);
    const float psiMax = 0.615479709f;
    const float bitRange = static_cast<float>(1 << bits);
    const float thetaArg = psiMax * (bitRange - static_cast<float>(theta)) / bitRange;
    const float clamped = std::max(-1.f, std::min(1.f, std::tan(thetaArg)));
    const float fTheta = std::asin(clamped);
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
    if ((octant & 4) == 0) {
        x = -x;
    }
    if ((octant & 2) == 0) {
        y = -y;
    }
    if ((octant & 1) == 0) {
        z = -z;
    }
}

bool readCoordinateArray(ByteReader& reader, int generation, int expected, std::vector<float>& coords, std::string& error) {
    const std::int32_t unique = reader.i32();
    const int components = reader.u8();
    if (!reader.ok || unique < 0 || unique > 5000000 || components != 3) {
        error = "vertex coordinate header is invalid";
        return false;
    }
    if (expected >= 0 && unique != expected) {
        error = "unique vertex count differs from the topology";
        return false;
    }
    float minimum[3];
    float maximum[3];
    int bits[3];
    for (int axis = 0; axis < 3; ++axis) {
        minimum[axis] = reader.f32();
        maximum[axis] = reader.f32();
        bits[axis] = reader.u8();
    }
    if (!reader.ok || bits[0] != bits[1] || bits[1] != bits[2] || bits[0] < 0 || bits[0] > 24) {
        error = "point quantizer is invalid";
        return false;
    }
    coords.assign(static_cast<std::size_t>(unique) * 3, 0.f);
    std::uint32_t hash = 0;
    if (bits[0] == 0 && generation >= 4) {
        std::vector<std::int32_t> codes[3];
        for (int axis = 0; axis < 3; ++axis) {
            if (!readCdp(reader, generation, 0, codes[axis], error) ||
                static_cast<int>(codes[axis].size()) != unique) {
                error = "JT 10.5 lossless coordinate packet is invalid axis=" + std::to_string(axis) + " " + error;
                return false;
            }
        }
        const std::uint32_t stored = reader.u32();
        std::uint32_t rawHash = 0;
        std::uint32_t lagHash = 0;
        for (int axis = 0; axis < 3; ++axis) {
            rawHash = hash32(codes[axis].data(), unique, rawHash);
            const std::vector<std::int32_t> lag = jtUnpackPredictor(codes[axis], 0);
            lagHash = hash32(lag.data(), unique, lagHash);
        }
        if (!reader.ok || (rawHash != stored && lagHash != stored)) {
            error = "JT 10.5 lossless coordinate hash does not match";
            return false;
        }
        for (int axis = 0; axis < 3; ++axis) {
            const std::vector<std::int32_t> values =
                rawHash == stored ? codes[axis] : jtUnpackPredictor(codes[axis], 0);
            for (int i = 0; i < unique; ++i) {
                const std::uint32_t bitsValue = static_cast<std::uint32_t>(values[static_cast<std::size_t>(i)]);
                float value = 0;
                std::memcpy(&value, &bitsValue, 4);
                if (value < minimum[axis] - 1e-3f || value > maximum[axis] + 1e-3f) {
                    error = "JT 10.5 lossless coordinate is outside the quantizer range";
                    return false;
                }
                coords[static_cast<std::size_t>(i) * 3 + static_cast<std::size_t>(axis)] = value;
            }
        }
        return true;
    }
    if (bits[0] == 0) {
        std::vector<std::int32_t> decoded[3][2];
        for (int axis = 0; axis < 3; ++axis) {
            if (!readCdp(reader, generation, 0, decoded[axis][0], error) ||
                !readCdp(reader, generation, 0, decoded[axis][1], error)) {
                return false;
            }
            decoded[axis][0] = jtUnpackPredictor(decoded[axis][0], 0);
            decoded[axis][1] = jtUnpackPredictor(decoded[axis][1], 0);
            if (static_cast<int>(decoded[axis][0].size()) != unique || static_cast<int>(decoded[axis][1].size()) != unique) {
                error = "lossless coordinate count differs";
                return false;
            }
        }
        for (int i = 0; i < unique; ++i) {
            for (int axis = 0; axis < 3; ++axis) {
                const float value = losslessFloat(decoded[axis][0][static_cast<std::size_t>(i)],
                                                  decoded[axis][1][static_cast<std::size_t>(i)]);
                if (value < minimum[axis] - 1e-5f || value > maximum[axis] + 1e-5f) {
                    error = "lossless coordinate is outside the quantizer range";
                    return false;
                }
                coords[static_cast<std::size_t>(i) * 3 + static_cast<std::size_t>(axis)] = value;
            }
        }
    } else {
        std::vector<std::int32_t> codes[3];
        for (int axis = 0; axis < 3; ++axis) {
            if (!readCdp(reader, generation, 0, codes[axis], error)) {
                return false;
            }
            codes[axis] = jtUnpackPredictor(codes[axis], 0);
            if (static_cast<int>(codes[axis].size()) != unique) {
                error = "quantized coordinate count differs";
                return false;
            }
            hash = hash32(codes[axis].data(), unique, hash);
            for (int i = 0; i < unique; ++i) {
                coords[static_cast<std::size_t>(i) * 3 + static_cast<std::size_t>(axis)] =
                    dequant(minimum[axis], maximum[axis], bits[axis], codes[axis][static_cast<std::size_t>(i)]);
            }
        }
    }
    const std::uint32_t stored = reader.u32();
    if (!reader.ok) {
        error = "coordinate hash is truncated";
        return false;
    }
    if (bits[0] > 0 && hash != stored) {
        error = "quantized coordinate hash does not match";
        return false;
    }
    return true;
}

bool readNormalArray(ByteReader& reader, int generation, int expected, std::vector<float>& normals, std::string& error) {
    const std::int32_t count = reader.i32();
    const int components = reader.u8();
    const int quantBits = reader.u8();
    if (!reader.ok || count != expected || components != 3 || quantBits < 0 || quantBits > 32) {
        error = "normal array header is invalid";
        return false;
    }
    normals.assign(static_cast<std::size_t>(count) * 3, 0.f);
    std::uint32_t hash = 0;
    if (quantBits == 0) {
        std::vector<std::int32_t> decoded[3][2];
        for (int axis = 0; axis < 3; ++axis) {
            if (!readCdp(reader, generation, 0, decoded[axis][0], error) ||
                !readCdp(reader, generation, 0, decoded[axis][1], error)) {
                return false;
            }
            if (static_cast<int>(decoded[axis][0].size()) != count || static_cast<int>(decoded[axis][1].size()) != count) {
                error = "lossless normal count differs";
                return false;
            }
        }
        for (int i = 0; i < count; ++i) {
            float value[3];
            for (int axis = 0; axis < 3; ++axis) {
                value[axis] = losslessFloat(decoded[axis][0][static_cast<std::size_t>(i)],
                                            decoded[axis][1][static_cast<std::size_t>(i)]);
                normals[static_cast<std::size_t>(i) * 3 + static_cast<std::size_t>(axis)] = value[axis];
            }
            const float length = std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
            if (std::fabs(length - 1.f) > 0.02f) {
                error = "lossless normal is not a unit vector";
                return false;
            }
        }
    } else {
        std::vector<std::int32_t> sextant;
        std::vector<std::int32_t> octant;
        std::vector<std::int32_t> theta;
        std::vector<std::int32_t> psi;
        if (!readCdp(reader, generation, 0, sextant, error) || !readCdp(reader, generation, 0, octant, error) ||
            !readCdp(reader, generation, 0, theta, error) || !readCdp(reader, generation, 0, psi, error)) {
            return false;
        }
        if (static_cast<int>(sextant.size()) != count || static_cast<int>(octant.size()) != count ||
            static_cast<int>(theta.size()) != count || static_cast<int>(psi.size()) != count) {
            error = "Deering normal count differs";
            return false;
        }
        hash = hash32(sextant.data(), count, hash);
        hash = hash32(octant.data(), count, hash);
        hash = hash32(theta.data(), count, hash);
        hash = hash32(psi.data(), count, hash);
        for (int i = 0; i < count; ++i) {
            float x = 0;
            float y = 0;
            float z = 0;
            deering(sextant[static_cast<std::size_t>(i)], octant[static_cast<std::size_t>(i)],
                    theta[static_cast<std::size_t>(i)], psi[static_cast<std::size_t>(i)], quantBits, x, y, z);
            normals[static_cast<std::size_t>(i) * 3] = x;
            normals[static_cast<std::size_t>(i) * 3 + 1] = y;
            normals[static_cast<std::size_t>(i) * 3 + 2] = z;
        }
    }
    const std::uint32_t stored = reader.u32();
    if (!reader.ok) {
        error = "normal hash is truncated";
        return false;
    }
    if (quantBits > 0 && hash != stored) {
        error = "Deering normal hash does not match";
        return false;
    }
    return true;
}

bool buildMesh(const DualMesh& dual, const std::vector<float>& coords, const std::vector<float>& normals,
               std::shared_ptr<MeshData>& mesh, std::string& error) {
    std::vector<std::vector<int>> attribute(dual.faces.size());
    int cursor = 0;
    for (std::size_t face = 0; face < dual.faces.size(); ++face) {
        const int degree = dual.degree(static_cast<int>(face));
        attribute[face].assign(static_cast<std::size_t>(degree), -1);
        const std::uint64_t mask = dual.faces[face].mask;
        for (int slot = 0; slot < degree; ++slot) {
            if (((mask >> slot) & 1ull) != 0) {
                attribute[face][static_cast<std::size_t>(slot)] = cursor++;
            }
        }
        for (int pass = 0; pass < degree; ++pass) {
            for (int slot = 0; slot < degree; ++slot) {
                if (attribute[face][static_cast<std::size_t>(slot)] < 0) {
                    attribute[face][static_cast<std::size_t>(slot)] =
                        attribute[face][static_cast<std::size_t>((slot + degree - 1) % degree)];
                }
            }
        }
    }
    if (!normals.empty() && cursor != static_cast<int>(normals.size() / 3)) {
        error = "attribute-mask bits do not match the normal count";
        return false;
    }
    mesh = std::make_shared<MeshData>();
    const int coordCount = static_cast<int>(coords.size() / 3);
    for (int vertex = 0; vertex < static_cast<int>(dual.verts.size()); ++vertex) {
        if (dual.flags[static_cast<std::size_t>(vertex)] != 0) {
            continue;
        }
        const auto& ring = dual.verts[static_cast<std::size_t>(vertex)];
        if (ring.size() < 3) {
            continue;
        }
        bool complete = true;
        for (int face : ring) {
            if (face < 0 || face >= coordCount) {
                complete = false;
                break;
            }
        }
        if (!complete) {
            continue;
        }
        for (int i = 1; i + 1 < static_cast<int>(ring.size()); ++i) {
            const int corners[3] = {ring[0], ring[static_cast<std::size_t>(i)], ring[static_cast<std::size_t>(i + 1)]};
            if (corners[0] == corners[1] || corners[1] == corners[2] || corners[0] == corners[2]) {
                continue;
            }
            std::uint32_t indices[3];
            for (int k = 0; k < 3; ++k) {
                const int corner = corners[k];
                const int slot = dual.findVertexSlot(corner, vertex);
                int normalIndex = -1;
                if (slot >= 0) {
                    normalIndex = attribute[static_cast<std::size_t>(corner)][static_cast<std::size_t>(slot)];
                }
                indices[k] = static_cast<std::uint32_t>(mesh->positions.size() / 3);
                mesh->positions.push_back(coords[static_cast<std::size_t>(corner) * 3]);
                mesh->positions.push_back(coords[static_cast<std::size_t>(corner) * 3 + 1]);
                mesh->positions.push_back(coords[static_cast<std::size_t>(corner) * 3 + 2]);
                if (normalIndex >= 0 && static_cast<std::size_t>(normalIndex) * 3 + 2 < normals.size()) {
                    mesh->normals.push_back(normals[static_cast<std::size_t>(normalIndex) * 3]);
                    mesh->normals.push_back(normals[static_cast<std::size_t>(normalIndex) * 3 + 1]);
                    mesh->normals.push_back(normals[static_cast<std::size_t>(normalIndex) * 3 + 2]);
                } else {
                    mesh->normals.push_back(0);
                    mesh->normals.push_back(0);
                    mesh->normals.push_back(1);
                }
            }
            mesh->indices.push_back(indices[0]);
            mesh->indices.push_back(indices[1]);
            mesh->indices.push_back(indices[2]);
        }
    }
    if (mesh->indices.size() < 3) {
        error = "topological mesh produced no triangles";
        mesh.reset();
        return false;
    }
    mesh->recomputeBounds();
    return true;
}

bool readPackedDeering(ByteReader& reader, int generation, int expected, std::vector<float>& normals, std::string& error) {
    const std::int32_t count = reader.i32();
    const int components = reader.u8();
    const int quantBits = reader.u8();
    if (!reader.ok || count != expected || components != 3 || quantBits < 0 || quantBits > 13) {
        error = "JT 10.5 packed normal header is invalid";
        return false;
    }
    if (quantBits == 0) {
        std::vector<std::int32_t> codes[3];
        for (int axis = 0; axis < 3; ++axis) {
            if (!readCdp(reader, generation, 0, codes[axis], error) || static_cast<int>(codes[axis].size()) != count) {
                error = error.empty() ? "JT 10.5 lossless normal count differs" : error;
                return false;
            }
        }
        const std::uint32_t stored = reader.u32();
        std::uint32_t rawHash = 0;
        std::uint32_t lagHash = 0;
        for (int axis = 0; axis < 3; ++axis) {
            rawHash = hash32(codes[axis].data(), count, rawHash);
            const std::vector<std::int32_t> lag = jtUnpackPredictor(codes[axis], 0);
            lagHash = hash32(lag.data(), count, lagHash);
        }
        if (!reader.ok || (rawHash != stored && lagHash != stored)) {
            error = "JT 10.5 lossless normal hash does not match";
            return false;
        }
        const bool lag = lagHash == stored;
        std::vector<std::int32_t> restored[3];
        for (int axis = 0; axis < 3; ++axis) {
            restored[axis] = lag ? jtUnpackPredictor(codes[axis], 0) : codes[axis];
        }
        normals.assign(static_cast<std::size_t>(count) * 3, 0.f);
        for (int i = 0; i < count; ++i) {
            float value[3];
            for (int axis = 0; axis < 3; ++axis) {
                const std::uint32_t bitsValue = static_cast<std::uint32_t>(restored[axis][static_cast<std::size_t>(i)]);
                std::memcpy(&value[axis], &bitsValue, 4);
                normals[static_cast<std::size_t>(i) * 3 + static_cast<std::size_t>(axis)] = value[axis];
            }
            const float length = std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
            if (std::fabs(length - 1.f) > 0.02f) {
                error = "JT 10.5 lossless normal is not a unit vector";
                return false;
            }
        }
        return true;
    }
    std::vector<std::int32_t> codes;
    if (!readCdp(reader, generation, 0, codes, error)) {
        return false;
    }
    if (static_cast<int>(codes.size()) != count) {
        error = "JT 10.5 packed normal count differs";
        return false;
    }
    const std::uint32_t hash = hash32(codes.data(), count, 0);
    const std::uint32_t stored = reader.u32();
    if (!reader.ok) {
        error = "JT 10.5 packed normal hash is truncated";
        return false;
    }
    if (hash != stored) {
        error = "JT 10.5 packed normal hash does not match";
        return false;
    }
    normals.assign(static_cast<std::size_t>(count) * 3, 0.f);
    const std::uint32_t mask = (1u << quantBits) - 1u;
    for (int i = 0; i < count; ++i) {
        const std::uint32_t code = static_cast<std::uint32_t>(codes[static_cast<std::size_t>(i)]);
        const int sextant = static_cast<int>((code >> (quantBits + quantBits + 3)) & 7u);
        const int octant = static_cast<int>((code >> (quantBits + quantBits)) & 7u);
        const int theta = static_cast<int>((code >> quantBits) & mask);
        const int psi = static_cast<int>(code & mask);
        float x = 0;
        float y = 0;
        float z = 0;
        deering(sextant, octant, theta, psi, quantBits, x, y, z);
        const float length = std::sqrt(x * x + y * y + z * z);
        if (std::fabs(length - 1.f) > 0.02f) {
            error = "JT 10.5 Deering normal is not a unit vector";
            return false;
        }
        normals[static_cast<std::size_t>(i) * 3] = x;
        normals[static_cast<std::size_t>(i) * 3 + 1] = y;
        normals[static_cast<std::size_t>(i) * 3 + 2] = z;
    }
    return true;
}

}  // namespace

bool jtReadInt32Cdp(const std::uint8_t* data, std::size_t size, int generation, std::vector<std::int32_t>& values,
                    std::string& error) {
    ByteReader reader{data, data + size, true};
    if (!readCdp(reader, generation, 0, values, error)) {
        return false;
    }
    if (reader.left() != 0) {
        error = "Int32 packet did not consume its bytes";
        return false;
    }
    return true;
}

std::vector<std::int32_t> jtUnpackPredictor(const std::vector<std::int32_t>& residual, int predictor) {
    std::vector<std::int32_t> values = residual;
    if (predictor == 8) {
        return values;
    }
    for (std::size_t i = 4; i < values.size(); ++i) {
        const std::int32_t v1 = values[i - 1];
        const std::int32_t v2 = values[i - 2];
        const std::int32_t v4 = values[i - 4];
        std::int32_t predicted = v1;
        switch (predictor) {
        case 1:
        case 7:
            predicted = v2;
            break;
        case 2:
            predicted = v1 + (v1 - v2);
            break;
        case 3:
            predicted = v2 + (v2 - v4);
            break;
        case 4:
            predicted = (v2 - v4 > -8 && v2 - v4 < 8) ? v2 + (v2 - v4) : v2 + 2;
            break;
        case 5:
            predicted = static_cast<std::int32_t>(i);
            break;
        case 0:
        case 6:
        default:
            predicted = v1;
            break;
        }
        if (predictor == 6 || predictor == 7) {
            values[i] = residual[i] ^ predicted;
        } else {
            values[i] = residual[i] + predicted;
        }
    }
    return values;
}

std::vector<std::int32_t> jtDecodePrefixBitlength(const std::uint32_t* words, int wordCount, int totalBits, std::string& error) {
    WordBits bits;
    bits.words = words;
    bits.total = totalBits;
    (void)wordCount;
    int state = 0;
    int width = 0;
    int last = 0;
    std::uint32_t accumulated = 0;
    int accumulatedBits = 0;
    std::vector<std::int32_t> values;
    auto emit = [&](std::int32_t symbol) { values.push_back(symbol); };
    while (bits.pos < totalBits && bits.ok) {
        if (state == 2) {
            const int need = width - accumulatedBits;
            const int available = totalBits - bits.pos;
            const int take = std::min(need, available);
            for (int i = 0; i < take; ++i) {
                accumulated = (accumulated << 1) | bits.readU(1);
                ++accumulatedBits;
            }
            if (!bits.ok) {
                break;
            }
            if (accumulatedBits >= width) {
                std::int32_t symbol = 0;
                if (width > 0 && width < 32 && (accumulated & (1u << (width - 1))) != 0) {
                    symbol = static_cast<std::int32_t>(accumulated) - (1 << width);
                } else if (width >= 32) {
                    symbol = static_cast<std::int32_t>(accumulated);
                } else if (width > 0) {
                    symbol = static_cast<std::int32_t>(accumulated);
                }
                emit(symbol);
                state = 0;
                accumulated = 0;
                accumulatedBits = 0;
            }
        } else {
            const std::uint32_t bit = bits.readU(1);
            if (!bits.ok) {
                break;
            }
            if (state == 0) {
                if (bit == 0) {
                    state = 2;
                } else {
                    state = 1;
                    last = 2;
                }
            } else if (last != 2 && (bit ^ static_cast<std::uint32_t>(last)) != 0) {
                state = 2;
                last = 2;
            } else {
                width += bit == 1 ? 2 : -2;
                last = static_cast<int>(bit);
                if (width < 0 || width > 32) {
                    error = "prefix bitlength width is invalid";
                    return {};
                }
            }
        }
    }
    if (!bits.ok || bits.pos != totalBits) {
        error = "prefix bitlength did not consume its code text";
        return {};
    }
    if (state == 2 && width == 0) {
        emit(0);
    }
    error.clear();
    return values;
}

bool decodeTopologicalTriStrip(const std::uint8_t* data, std::size_t size, int major, std::shared_ptr<MeshData>& mesh,
                               std::string& error) {
    error.clear();
    if (major >= 10) {
        error = "JT 10 topological tri-strip layout is not decoded";
        return false;
    }
    ByteReader reader{data, data + size, true};
    const int baseVersion = reader.i16();
    const int vertexVersion = reader.i16();
    const std::uint64_t bindings = reader.u64();
    const int collectionVersion = reader.i16();
    reader.i32();
    const int repVersion = reader.i16();
    const bool classic = collectionVersion == 2 && repVersion == 2;
    const bool early = collectionVersion == 1 && repVersion == 1;
    if (!reader.ok || baseVersion != 1 || vertexVersion != 1 || (!classic && !early)) {
        error = "tri-strip LOD is not a version 9 topological mesh (" + std::to_string(baseVersion) + "," +
                std::to_string(vertexVersion) + "," + std::to_string(collectionVersion) + "," +
                std::to_string(repVersion) + ")";
        return false;
    }
    const bool hasPosition = (bindings & 0x2ull) != 0;
    const bool hasNormal = (bindings & 0x8ull) != 0;
    const bool hasFlags = (bindings & 0x40ull) != 0;
    if (!hasPosition) {
        error = "topological mesh has no coordinates";
        return false;
    }
    SymbolCursor symbols;
    if (!readSymbols(reader, 2, symbols, error)) {
        return false;
    }
    DualMesh dual;
    error.clear();
    if (!walkDual(symbols, dual, error)) {
        return false;
    }
    const std::uint64_t recordBindings = reader.u64();
    reader.u8();
    reader.u8();
    reader.u8();
    reader.u8();
    const std::int32_t topologicalVertices = reader.i32();
    const std::int32_t attributeCount = reader.i32();
    if (!reader.ok || topologicalVertices != static_cast<std::int32_t>(dual.faces.size())) {
        error = "topological vertex count does not match the dual mesh";
        return false;
    }
    (void)recordBindings;
    std::vector<float> coords;
    if (!readCoordinateArray(reader, 2, topologicalVertices, coords, error)) {
        return false;
    }
    std::vector<float> normals;
    if (hasNormal) {
        if (!readNormalArray(reader, 2, attributeCount, normals, error)) {
            return false;
        }
    }
    if (hasFlags && reader.left() >= 4) {
        std::vector<std::int32_t> flags;
        const auto* mark = reader.cur;
        const std::int32_t flagCount = reader.i32();
        reader.cur = mark;
        if (flagCount == attributeCount) {
            reader.i32();
            if (!readCdp(reader, 2, 0, flags, error)) {
                return false;
            }
        }
    }
    return buildMesh(dual, coords, normals, mesh, error);
}

bool decodeJt105TriStrip(const std::uint8_t* data, std::size_t size, bool big, std::shared_ptr<MeshData>& mesh,
                         std::string& error) {
    error.clear();
    mesh.reset();
    if (big) {
        error = "JT 10.5 big-endian tri-strip is not decoded";
        return false;
    }
    if (data == nullptr || size < 16) {
        error = "JT 10.5 tri-strip LOD is truncated";
        return false;
    }
    ByteReader reader{data, data + size, true};
    const int baseVersion = reader.u8();
    const int vertexVersion = reader.u8();
    const std::uint64_t bindings = reader.u64();
    const std::uint32_t framed = reader.u32();
    if (!reader.ok || baseVersion != 1 || vertexVersion != 1 || framed != static_cast<std::uint32_t>(size - 15)) {
        error = "JT 10.5 tri-strip LOD is not the local-header layout";
        return false;
    }
    if (data[size - 1] != 0x01) {
        error = "JT 10.5 tri-strip trailing marker is missing";
        return false;
    }
    reader.end = data + 14 + framed;
    static const std::uint8_t kLocalGuid[16] = {0xad, 0xa5, 0x30, 0xf8, 0x4c, 0xbe, 0xbc, 0x4f,
                                                0x9b, 0x5f, 0xb9, 0x26, 0x92, 0x78, 0xd2, 0xe1};
    for (unsigned char byte : kLocalGuid) {
        if (reader.u8() != byte) {
            error = "JT 10.5 local header GUID is not the tri-strip payload";
            return false;
        }
    }
    const int preamble = reader.u8();
    const std::int32_t preambleCount = reader.i32();
    const int preambleFlag = reader.u8();
    reader.u32();
    const int preambleTail = reader.u8();
    if (!reader.ok || preamble != 0x09 || preambleCount != 1 || preambleFlag != 0x01 || preambleTail != 0x01) {
        error = "JT 10.5 local preamble is not the tri-strip payload";
        return false;
    }
    constexpr std::uint64_t kKnownBindings = 0x2ull | 0x8ull | 0x40ull;
    if ((bindings & 0x2ull) == 0) {
        error = "JT 10.5 topological mesh has no coordinates";
        return false;
    }
    if ((bindings & ~kKnownBindings) != 0) {
        error = "JT 10.5 vertex binding includes a field that is not decoded";
        return false;
    }
    SymbolCursor symbols;
    if (!readSymbols(reader, 4, symbols, error)) {
        return false;
    }
    DualMesh dual;
    if (!walkDual(symbols, dual, error)) {
        return false;
    }
    const std::uint64_t recordBindings = reader.u64();
    reader.u8();
    reader.u8();
    reader.u8();
    reader.u8();
    const std::int32_t topologicalVertices = reader.i32();
    const std::int32_t attributeCount = reader.i32();
    if (!reader.ok || recordBindings != bindings || topologicalVertices != static_cast<std::int32_t>(dual.faces.size())) {
        error = "JT 10.5 topological vertex count does not match the dual mesh";
        return false;
    }
    std::vector<float> coords;
    if (!readCoordinateArray(reader, 4, topologicalVertices, coords, error)) {
        return false;
    }
    std::vector<float> normals;
    if ((bindings & 0x8ull) != 0) {
        if (!readPackedDeering(reader, 4, attributeCount, normals, error)) {
            return false;
        }
    }
    if ((bindings & 0x40ull) != 0) {
        const std::int32_t flagCount = reader.i32();
        std::vector<std::int32_t> flags;
        if (!reader.ok || flagCount != attributeCount || !readCdp(reader, 4, 0, flags, error) ||
            static_cast<int>(flags.size()) != flagCount) {
            if (error.empty()) {
                error = "JT 10.5 vertex flag packet is invalid";
            }
            return false;
        }
    }
    if (!reader.ok || reader.left() != 0) {
        error = "JT 10.5 tri-strip LOD has unread bytes";
        return false;
    }
    return buildMesh(dual, coords, normals, mesh, error);
}

}  // namespace sim
