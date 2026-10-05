#include "sim/io/EditHistory.hpp"

#include "sim/io/ProjectStore.hpp"

namespace sim {

EditHistory::~EditHistory() {
    if (scene_ != nullptr) {
        scene_->onChanged = std::move(previous_);
        scene_ = nullptr;
    }
}

void EditHistory::watch(Scene& scene) {
    if (scene_ == &scene) {
        sealed_ = capture();
        undo_.clear();
        redo_.clear();
        return;
    }
    if (scene_ != nullptr) {
        scene_->onChanged = std::move(previous_);
    }
    scene_ = &scene;
    previous_ = std::move(scene.onChanged);
    sealed_ = capture();
    undo_.clear();
    redo_.clear();
    scene.onChanged = [this](ChangeKind kind) { record(kind); };
}

bool EditHistory::undo() {
    if (scene_ == nullptr || replaying_ || undo_.empty() || sealed_.empty()) {
        return false;
    }
    std::vector<std::uint8_t> prior = std::move(undo_.back());
    undo_.pop_back();
    redo_.push_back(std::move(sealed_));
    if (static_cast<int>(redo_.size()) > kDepth) {
        redo_.erase(redo_.begin());
    }
    sealed_ = std::move(prior);
    replaying_ = true;
    const ProjectResult loaded = ProjectStore::loadBytes(sealed_, *scene_);
    replaying_ = false;
    return loaded.ok;
}

bool EditHistory::redo() {
    if (scene_ == nullptr || replaying_ || redo_.empty() || sealed_.empty()) {
        return false;
    }
    std::vector<std::uint8_t> next = std::move(redo_.back());
    redo_.pop_back();
    undo_.push_back(std::move(sealed_));
    if (static_cast<int>(undo_.size()) > kDepth) {
        undo_.erase(undo_.begin());
    }
    sealed_ = std::move(next);
    replaying_ = true;
    const ProjectResult loaded = ProjectStore::loadBytes(sealed_, *scene_);
    replaying_ = false;
    return loaded.ok;
}

void EditHistory::record(ChangeKind kind) {
    if (recording_ && !replaying_ && (kind == ChangeKind::Structure || kind == ChangeKind::Transform)) {
        std::vector<std::uint8_t> next = capture();
        if (!sealed_.empty() && !next.empty()) {
            undo_.push_back(std::move(sealed_));
            if (static_cast<int>(undo_.size()) > kDepth) {
                undo_.erase(undo_.begin());
            }
            redo_.clear();
            sealed_ = std::move(next);
        }
    }
    if (previous_) {
        previous_(kind);
    }
}

std::vector<std::uint8_t> EditHistory::capture() const {
    std::vector<std::uint8_t> bytes;
    if (scene_ == nullptr) {
        return bytes;
    }
    const ProjectResult saved = ProjectStore::saveBytes(*scene_, bytes);
    if (!saved.ok) {
        bytes.clear();
    }
    return bytes;
}

}  // namespace sim
