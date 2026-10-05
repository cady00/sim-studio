#pragma once

#include "sim/core/Mesh.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sim {

// Int32 compressed-data packet. generation 2 is the JT 9 Mk.2 layout
// (code text, then the probability context, 6-bit bitlength). generation 3
// is the JT 10 layout (context first, nibble bitlength, out-of-band only
// when the context has an escape).
bool jtReadInt32Cdp(const std::uint8_t* data, std::size_t size, int generation, std::vector<std::int32_t>& values,
                    std::string& error);

// Predictor enum from the published JT 9 tables. The first four residuals
// are primers. There is no geometric "moving frame" predictor in that enum.
std::vector<std::int32_t> jtUnpackPredictor(const std::vector<std::int32_t>& residual, int predictor);

// First-generation prefix bitlength walker (step of 2). The samples use the
// later fixed/variable bitlength codecs; this walker is still decoded and tested.
std::vector<std::int32_t> jtDecodePrefixBitlength(const std::uint32_t* words, int wordCount, int totalBits,
                                                  std::string& error);

// Version 9 topological tri-strip LOD. Version 10 returns false without inventing triangles.
bool decodeTopologicalTriStrip(const std::uint8_t* data, std::size_t size, int major, std::shared_ptr<MeshData>& mesh,
                               std::string& error);

}  // namespace sim
