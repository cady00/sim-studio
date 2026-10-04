#include "sim/ai/CommandRegistry.hpp"

#include "sim/ai/Commands.hpp"

#include <algorithm>
#include <cctype>

namespace sim {
namespace {

struct Glyph {
    char32_t code;
    char initial;
};

// Initials for the characters used by the shell and command titles.
// This is not a general pinyin dictionary.
constexpr Glyph kGlyphs[] = {
    {U'示', 's'}, {U'教', 'j'}, {U'点', 'd'}, {U'建', 'j'}, {U'模', 'm'}, {U'运', 'y'}, {U'动', 'd'},
    {U'学', 'x'}, {U'逻', 'l'}, {U'辑', 'j'}, {U'机', 'j'}, {U'器', 'q'}, {U'人', 'r'}, {U'调', 't'},
    {U'试', 's'}, {U'虚', 'x'}, {U'拟', 'n'}, {U'播', 'b'}, {U'放', 'f'}, {U'暂', 'z'}, {U'停', 't'},
    {U'止', 'z'}, {U'复', 'f'}, {U'位', 'w'}, {U'聚', 'j'}, {U'焦', 'j'}, {U'全', 'q'}, {U'部', 'b'},
    {U'可', 'k'}, {U'见', 'j'}, {U'移', 'y'}, {U'旋', 'x'}, {U'转', 'z'}, {U'测', 'c'}, {U'量', 'l'},
    {U'保', 'b'}, {U'存', 'c'}, {U'打', 'd'}, {U'开', 'k'}, {U'项', 'x'}, {U'目', 'm'}, {U'导', 'd'},
    {U'入', 'r'}, {U'专', 'z'}, {U'注', 'z'}, {U'深', 's'}, {U'色', 's'}, {U'浅', 'q'}, {U'主', 'z'},
    {U'题', 't'}, {U'信', 'x'}, {U'号', 'h'}, {U'命', 'm'}, {U'令', 'l'}, {U'面', 'm'}, {U'板', 'b'},
    {U'撤', 'c'}, {U'销', 'x'}, {U'应', 'y'}, {U'用', 'y'}, {U'新', 'x'}, {U'关', 'g'}, {U'闭', 'b'},
    {U'路', 'l'}, {U'径', 'j'}, {U'碰', 'p'}, {U'撞', 'z'}, {U'近', 'j'}, {U'视', 's'}, {U'图', 't'},
    {U'前', 'q'}, {U'后', 'h'}, {U'左', 'z'}, {U'右', 'y'}, {U'上', 's'}, {U'下', 'x'}, {U'等', 'd'},
    {U'轴', 'z'},
};

char initialOf(char32_t code) {
    static const std::vector<Glyph> glyphs = [] {
        std::vector<Glyph> copy(std::begin(kGlyphs), std::end(kGlyphs));
        std::sort(copy.begin(), copy.end(), [](const Glyph& a, const Glyph& b) { return a.code < b.code; });
        return copy;
    }();
    const auto found = std::lower_bound(glyphs.begin(), glyphs.end(), code,
                                        [](const Glyph& glyph, char32_t value) { return glyph.code < value; });
    if (found != glyphs.end() && found->code == code) {
        return found->initial;
    }
    return 0;
}

std::string lowerCopy(std::string text) {
    for (char& ch : text) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

bool contains(const std::string& hay, const std::string& needle) {
    return !needle.empty() && hay.find(needle) != std::string::npos;
}

CommandEntry ui(const char* id, const char* zh, const char* en, const char* shortcut, const char* risk, bool ai) {
    CommandEntry entry;
    entry.id = id;
    entry.titleZh = zh;
    entry.titleEn = en;
    entry.group = "command";
    entry.shortcut = shortcut;
    entry.risk = risk;
    entry.aiAllowed = ai;
    entry.executable = true;
    return entry;
}

}  // namespace

std::string CommandRegistry::pinyinInitials(std::string_view utf8) {
    std::string out;
    for (std::size_t i = 0; i < utf8.size();) {
        const auto lead = static_cast<unsigned char>(utf8[i]);
        char32_t code = 0;
        std::size_t width = 1;
        if (lead < 0x80) {
            code = lead;
        } else if ((lead & 0xE0) == 0xC0 && i + 1 < utf8.size()) {
            code = (lead & 0x1F) << 6 | (static_cast<unsigned char>(utf8[i + 1]) & 0x3F);
            width = 2;
        } else if ((lead & 0xF0) == 0xE0 && i + 2 < utf8.size()) {
            code = (lead & 0x0F) << 12 | (static_cast<unsigned char>(utf8[i + 1]) & 0x3F) << 6 |
                   (static_cast<unsigned char>(utf8[i + 2]) & 0x3F);
            width = 3;
        } else if ((lead & 0xF8) == 0xF0 && i + 3 < utf8.size()) {
            code = (lead & 0x07) << 18 | (static_cast<unsigned char>(utf8[i + 1]) & 0x3F) << 12 |
                   (static_cast<unsigned char>(utf8[i + 2]) & 0x3F) << 6 |
                   (static_cast<unsigned char>(utf8[i + 3]) & 0x3F);
            width = 4;
        }
        if (code >= 'A' && code <= 'Z') {
            out.push_back(static_cast<char>(code - 'A' + 'a'));
        } else if (code >= 'a' && code <= 'z') {
            out.push_back(static_cast<char>(code));
        } else if (const char initial = initialOf(code)) {
            out.push_back(initial);
        }
        i += width;
    }
    return out;
}

std::vector<CommandEntry> CommandRegistry::builtin() {
    std::vector<CommandEntry> entries = {
        ui("ui.new", "新建项目", "New Project", "Ctrl+N", "R1", false),
        ui("ui.open", "打开项目", "Open Project", "Ctrl+O", "R1", false),
        ui("ui.save", "保存项目", "Save Project", "Ctrl+S", "R1", false),
        ui("ui.import_step", "导入 STEP", "Import STEP", "Ctrl+Shift+I", "R2", false),
        ui("ui.import_obj", "导入 OBJ", "Import OBJ", "", "R2", false),
        ui("ui.play", "播放", "Play", "Space", "R0", true),
        ui("ui.pause", "暂停", "Pause", "", "R0", true),
        ui("ui.stop", "停止", "Stop", "", "R1", true),
        ui("ui.reset", "复位", "Reset", "", "R1", true),
        ui("ui.fit", "聚焦所选", "Frame Selection", "F", "R0", true),
        ui("ui.fit_all", "全部可见", "Frame All", "Home", "R0", true),
        ui("ui.move", "移动", "Move", "W", "R0", false),
        ui("ui.rotate", "旋转", "Rotate", "E", "R0", false),
        ui("ui.measure", "测量", "Measure", "M", "R0", false),
        ui("ui.focus", "专注模式", "Focus Mode", "Ctrl+\\", "R0", false),
        ui("ui.theme", "切换深浅色", "Toggle Theme", "", "R0", false),
        ui("ui.teach", "示教点", "Teach Target", "Ctrl+T", "R1", true),
        ui("ui.mode.model", "建模", "Model", "Ctrl+1", "R0", false),
        ui("ui.mode.kinematics", "运动学", "Kinematics", "Ctrl+2", "R0", false),
        ui("ui.mode.logic", "逻辑", "Logic", "Ctrl+3", "R0", false),
        ui("ui.mode.robot", "机器人", "Robot", "Ctrl+4", "R0", false),
        ui("ui.mode.debug", "调试", "Debug", "Ctrl+5", "R0", false),
        ui("ui.mode.commission", "虚拟调试", "Virtual Commissioning", "Ctrl+6", "R0", false),
        ui("ui.palette", "命令面板", "Command Palette", "Ctrl+K", "R0", false),
        ui("ui.intent", "AI 意图栏", "AI Intent", "Ctrl+I", "R0", false),
    };
    for (const auto& item : CommandExecutor::catalog()) {
        CommandEntry entry;
        entry.id = item.value("op", "");
        entry.titleEn = item.value("summary", entry.id);
        entry.titleZh = entry.titleEn;
        entry.group = "command";
        entry.risk = item.value("executable", false) ? "R1" : "R3";
        entry.aiAllowed = item.value("executable", false);
        entry.executable = item.value("executable", false);
        if (entry.id == "sim_play") {
            entry.titleZh = "播放仿真";
            entry.shortcut = "Space";
            entry.risk = "R0";
        } else if (entry.id == "sim_pause") {
            entry.titleZh = "暂停仿真";
        } else if (entry.id == "sim_stop") {
            entry.titleZh = "停止仿真";
        } else if (entry.id == "set_bool") {
            entry.titleZh = "写布尔信号";
        } else if (entry.id == "collision_query") {
            entry.titleZh = "碰撞与近碰查询";
            entry.risk = "R0";
        }
        entries.push_back(std::move(entry));
    }
    return entries;
}

std::vector<CommandHit> CommandRegistry::search(const std::string& query, const Scene* scene) {
    std::string raw = query;
    const auto trim = raw.find_first_not_of(" \t");
    if (trim == std::string::npos) {
        raw.clear();
    } else {
        raw = raw.substr(trim);
    }
    std::string kind = "all";
    if (!raw.empty() && raw[0] == '@') {
        kind = "object";
        raw = raw.substr(1);
    } else if (!raw.empty() && raw[0] == '#') {
        kind = "signal";
        raw = raw.substr(1);
    }
    const std::string needle = lowerCopy(raw);
    std::vector<CommandHit> hits;
    if (kind != "object" && kind != "signal") {
        for (const CommandEntry& entry : builtin()) {
            const std::string initials = pinyinInitials(entry.titleZh);
            const std::string blob = lowerCopy(entry.id + " " + entry.titleZh + " " + entry.titleEn + " " + initials);
            if (needle.empty() || contains(blob, needle) || contains(initials, needle)) {
                CommandHit hit;
                hit.entry = entry;
                hit.detail = entry.shortcut;
                hits.push_back(std::move(hit));
            }
        }
    }
    if (scene != nullptr && kind != "signal") {
        std::vector<std::shared_ptr<SceneNode>> stack{scene->root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            const std::string initials = pinyinInitials(node->name);
            const std::string blob = lowerCopy(node->name + " " + node->id + " " + initials);
            if (kind == "object" || contains(blob, needle) || (!needle.empty() && contains(initials, needle))) {
                if (needle.empty() && kind != "object") {
                    continue;
                }
                CommandHit hit;
                hit.entry.id = "object:" + node->id;
                hit.entry.titleZh = node->name;
                hit.entry.titleEn = node->name;
                hit.entry.group = "object";
                hit.entry.risk = "R0";
                hit.detail = "@" + node->name;
                hits.push_back(std::move(hit));
            }
        }
    }
    if (scene != nullptr && kind != "object") {
        for (const Signal& signal : scene->signals) {
            const std::string initials = pinyinInitials(signal.name);
            const std::string blob = lowerCopy(signal.name + " " + initials);
            if (kind == "signal" || contains(blob, needle) || (!needle.empty() && contains(initials, needle))) {
                if (needle.empty() && kind != "signal") {
                    continue;
                }
                CommandHit hit;
                hit.entry.id = "signal:" + signal.name;
                hit.entry.titleZh = signal.name;
                hit.entry.titleEn = signal.name;
                hit.entry.group = "signal";
                hit.entry.risk = "R0";
                hit.detail = "#" + signal.name;
                hits.push_back(std::move(hit));
            }
        }
    }
    if (hits.size() > 12) {
        hits.resize(12);
    }
    return hits;
}

CommandPlan CommandRegistry::planIntent(const std::string& text) {
    CommandPlan plan;
    plan.intent = text;
    const std::string folded = lowerCopy(text);
    auto add = [&](const char* op, const char* title, const char* risk) {
        PlanStep step;
        step.op = op;
        step.title = title;
        step.risk = risk;
        plan.steps.push_back(std::move(step));
    };
    if (text.find("播放") != std::string::npos || folded.find("play") != std::string::npos) {
        add("sim_play", "播放仿真", "R0");
    }
    if (text.find("暂停") != std::string::npos || folded.find("pause") != std::string::npos) {
        add("sim_pause", "暂停仿真", "R0");
    }
    if (text.find("停止") != std::string::npos || folded.find("stop") != std::string::npos) {
        add("sim_stop", "停止仿真", "R1");
    }
    if (text.find("近碰") != std::string::npos || folded.find("collision") != std::string::npos) {
        add("collision_query", "碰撞与近碰查询", "R0");
    }
    const auto hash = text.find('#');
    if (hash != std::string::npos && (text.find("信号") != std::string::npos || folded.find("signal") != std::string::npos)) {
        std::string name;
        for (std::size_t i = hash + 1; i < text.size(); ++i) {
            const unsigned char ch = static_cast<unsigned char>(text[i]);
            if (ch == ' ' || ch < 0x20) {
                break;
            }
            if (ch >= 0x80) {
                break;
            }
            name.push_back(static_cast<char>(ch));
        }
        if (!name.empty()) {
            PlanStep step;
            step.op = "set_bool";
            step.args = {{"name", name}, {"value", true}};
            step.title = "写布尔信号 " + name;
            step.risk = "R1";
            plan.steps.push_back(std::move(step));
        }
    }
    if (plan.steps.empty()) {
        plan.needsClarify = true;
        plan.note = "需要澄清：这句话没有对应到已注册的命令。不会猜测，也不会改场景。";
    } else {
        plan.note = "本地规则生成的计划。模型提供者关闭，没有联网。";
    }
    return plan;
}

}  // namespace sim
