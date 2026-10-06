#pragma once

#include "sim/core/Scene.hpp"

#include <atomic>
#include <functional>
#include <string>

namespace sim {

// complete: geometry is present and nothing in this import was dropped.
// partial: some geometry arrived, and the report says what was dropped.
// structure_only: labels exist and there are no triangles. Not a simulation model.
// failed / cancelled: the caller's project must be left unchanged.
enum class ImportStatus { Complete, Partial, StructureOnly, Failed, Cancelled };

inline const char* importStatusName(ImportStatus status) {
    switch (status) {
    case ImportStatus::Complete:
        return "complete";
    case ImportStatus::Partial:
        return "partial";
    case ImportStatus::StructureOnly:
        return "structure_only";
    case ImportStatus::Cancelled:
        return "cancelled";
    case ImportStatus::Failed:
        return "failed";
    }
    return "failed";
}

struct ImportResult {
    // True only when the result may be simulated (complete or partial, with triangles).
    bool ok = false;
    ImportStatus status = ImportStatus::Failed;
    std::string message;
    // JSON object. Present for both success and failure. Not a claim that every
    // section-5 field is filled; missing fields are omitted rather than invented.
    std::string report;
    int nodeCount = 0;
    int meshCount = 0;
    int triangleCount = 0;
    double elapsedMs = 0;

    bool simulatable() const { return ok && triangleCount > 0 && (status == ImportStatus::Complete || status == ImportStatus::Partial); }
    // Structure is shown, but it is not a successful simulation import.
    bool keepsStructure() const {
        return status == ImportStatus::Complete || status == ImportStatus::Partial || status == ImportStatus::StructureOnly;
    }
};

inline void classifyImportResult(ImportResult& result) {
    if (result.status == ImportStatus::Cancelled) {
        result.ok = false;
        return;
    }
    if (!result.ok) {
        if (result.status == ImportStatus::Complete) {
            result.status = ImportStatus::Failed;
        }
        return;
    }
    if (result.triangleCount <= 0) {
        result.status = ImportStatus::StructureOnly;
        result.ok = false;
        if (result.message.empty() || result.message == "imported") {
            result.message = "structure_only";
        }
        return;
    }
    result.status = ImportStatus::Complete;
}

struct ImportProgress {
    // Negative percent means the stage length is unknown (busy indicator).
    int percent = -1;
    std::string stageKey;
    int current = 0;
    int total = 0;
};

using ImportProgressFn = std::function<void(const ImportProgress&)>;

class IGeometryImporter {
public:
    virtual ~IGeometryImporter() = default;
    virtual ImportResult importFile(const std::string& path, Scene& scene) = 0;
};

}  // namespace sim
