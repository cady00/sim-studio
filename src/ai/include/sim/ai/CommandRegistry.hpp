#pragma once

#include "sim/core/Scene.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace sim {

// One catalog for menus, the command palette, Python, and the AI plan.
// Scene edits still go through CommandExecutor. UI-only ids start with "ui.".
struct CommandEntry {
    std::string id;
    std::string titleZh;
    std::string titleEn;
    std::string group;
    std::string shortcut;
    std::string risk;
    bool aiAllowed = false;
    bool executable = true;
};

struct CommandHit {
    CommandEntry entry;
    std::string detail;
};

struct PlanStep {
    std::string op;
    nlohmann::json args = nlohmann::json::object();
    std::string risk = "R0";
    std::string title;
};

struct CommandPlan {
    std::string intent;
    std::vector<PlanStep> steps;
    std::string note;
    bool needsClarify = false;
};

class CommandRegistry {
public:
    static std::vector<CommandEntry> builtin();
    static std::string pinyinInitials(std::string_view utf8);
    static std::vector<CommandHit> search(const std::string& query, const Scene* scene);
    // Local rules only. Does not call a model and does not invent unreachable ops.
    static CommandPlan planIntent(const std::string& text);
};

}  // namespace sim
