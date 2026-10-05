#pragma once

#include "sim/core/Mesh.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sim {

// Int32 compressed-data packet.
// generation 2: JT 9 Mk.2 (code text, then the probability context, 6-bit bitlength).
// generation 3: context first, 3+1 nibble bitlength, out-of-band only on escape.
// generation 4: JT 10.5 (4+1 nibble bitlength, code text then the generation-3
//               probability context, out-of-band only on escape or empty code text).
bool jtReadInt32Cdp(const std::uint8_t* data, std::size_t size, int generation, std::vector<std::int32_t>& values,
                    std::string& error);

// Predictor enum from the published JT 9 tables. The first four residuals
// are primers. There is no geometric "moving frame" predictor in that enum.
std::vector<std::int32_t> jtUnpackPredictor(const std::vector<std::int32_t>& residual, int predictor);

// First-generation prefix bitlength walker (step of 2). The samples use the
// later fixed/variable bitlength codecs; this walker is still decoded and tested.
std::vector<std::int32_t> jtDecodePrefixBitlength(const std::uint32_t* words, int wordCount, int totalBits,
                                                  std::string& error);

// Version 9 topological tri-strip LOD. Version 10 without the local header returns false.
bool decodeTopologicalTriStrip(const std::uint8_t* data, std::size_t size, int major, std::shared_ptr<MeshData>& mesh,
                               std::string& error);

// JT 10.5 tri-strip whose element body starts with the two version bytes, the
// vertex bindings, a U32 length, a local GUID, and an 11-byte preamble that
// the published reference does not describe. Triangles come only from a
// generation-4 topological mesh. `big` is refused rather than byte-swapped
// into a guess.
bool decodeJt105TriStrip(const std::uint8_t* data, std::size_t size, bool big, std::shared_ptr<MeshData>& mesh,
                         std::string& error);

}  // namespace sim
