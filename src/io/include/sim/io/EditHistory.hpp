#pragma once

#include "sim/core/Scene.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace sim {

// Snapshots the project document on Structure and Transform notifications.
// Selection and Simulation are ignored. Depth is capped so a long session stays bounded.
class EditHistory {
public:
    static constexpr int kDepth = 16;

    EditHistory() = default;
    ~EditHistory();

    EditHistory(const EditHistory&) = delete;
    EditHistory& operator=(const EditHistory&) = delete;

    void watch(Scene& scene);
    // Playback moves the scene every step. Those notices are not edits.
    void setRecording(bool enabled) { recording_ = enabled; }
    bool recording() const { return recording_; }
    bool undo();
    bool redo();
    int undoDepth() const { return static_cast<int>(undo_.size()); }
    int redoDepth() const { return static_cast<int>(redo_.size()); }

private:
    void record(ChangeKind kind);
    std::vector<std::uint8_t> capture() const;

    Scene* scene_ = nullptr;
    std::function<void(ChangeKind)> previous_;
    std::vector<std::uint8_t> sealed_;
    std::vector<std::vector<std::uint8_t>> undo_;
    std::vector<std::vector<std::uint8_t>> redo_;
    bool replaying_ = false;
    bool recording_ = true;
};

}  // namespace sim
