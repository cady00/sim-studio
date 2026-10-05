#pragma once

#include "sim/io/AssetImport.hpp"

namespace sim {

// Independent reader of the JT layout published in ISO 14306.
// Not Siemens JT Open Toolkit, and not derived from that code.
class JtImporter final : public IAssetImporter {
public:
    const char* format() const override { return "jt"; }
    std::vector<std::string> extensions() const override { return {"jt"}; }
    bool probeContent(const std::string& head) const override;
    AssetImportResult importFile(const std::string& path, Scene& scene) const override;
};

}  // namespace sim
