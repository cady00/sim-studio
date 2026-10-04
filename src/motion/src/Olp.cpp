#include "sim/motion/Olp.hpp"

#include "sim/motion/Motion.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <unordered_map>

namespace sim {
namespace {

struct AxisSample {
    double arm[6]{};
    double ext[6]{};
    bool extUsed[6]{};
    bool cartesian = false;
    Vec3 position{0, 0, 0};
    double a = 0;
    double b = 0;
    double c = 0;
    double speed = 100;
    double zone = 0;
    PathMove move = PathMove::Ptp;
};

std::string lowerCopy(std::string text) {
    for (char& ch : text) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

std::string formatNum(double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.3f", value);
    return buffer;
}

std::string ident(std::string name, const std::string& fallback) {
    if (name.empty()) {
        name = fallback;
    }
    for (char& ch : name) {
        const unsigned char u = static_cast<unsigned char>(ch);
        if (!std::isalnum(u) && ch != '_') {
            ch = '_';
        }
    }
    if (!name.empty() && std::isdigit(static_cast<unsigned char>(name.front()))) {
        name = "p_" + name;
    }
    if (name.size() > 24) {
        name.resize(24);
    }
    return name;
}

int externalSlotOf(const std::string& name) {
    if (name.size() == 2 && (name[0] == 'E' || name[0] == 'e') && name[1] >= '1' && name[1] <= '6') {
        return name[1] - '1';
    }
    return -1;
}

AxisSample samplePoint(const KinematicResource& resource, const PathPoint& point) {
    AxisSample sample;
    sample.cartesian = point.cartesian || point.move != PathMove::Ptp;
    sample.move = point.move;
    sample.position = point.position;
    sample.speed = point.speed;
    sample.zone = point.zone;
    double rx = 0;
    double ry = 0;
    double rz = 0;
    point.orientation.toEulerXYZ(rx, ry, rz);
    sample.c = rad2deg(rx);
    sample.b = rad2deg(ry);
    sample.a = rad2deg(rz);
    int arm = 0;
    for (std::size_t i = 0; i < resource.joints.size(); ++i) {
        const double value = i < point.joints.size() ? point.joints[i] : 0;
        if (resource.joints[i].externalName.empty()) {
            if (arm < 6) {
                sample.arm[arm++] = value;
            }
        }
    }
    if (arm == 0) {
        for (int i = 0; i < 6 && i < static_cast<int>(point.joints.size()); ++i) {
            sample.arm[i] = point.joints[static_cast<std::size_t>(i)];
        }
    }
    const std::vector<OlpAxisName> map = externalExportMap(resource);
    const std::vector<int> indices = externalJointIndices(resource);
    for (std::size_t k = 0; k < map.size() && k < indices.size(); ++k) {
        const int slot = externalSlotOf(map[k].e);
        if (slot < 0 || slot > 5) {
            continue;
        }
        double value = 0;
        bool has = false;
        const int jointIndex = indices[k];
        if (jointIndex >= 0 && jointIndex < static_cast<int>(point.joints.size())) {
            value = point.joints[static_cast<std::size_t>(jointIndex)];
            has = true;
        }
        if (k < point.externalValues.size()) {
            value = point.externalValues[k];
            has = true;
        }
        sample.ext[slot] = value;
        sample.extUsed[slot] = has;
    }
    return sample;
}

PathPoint pointFromSample(const KinematicResource* resource, const AxisSample& sample) {
    PathPoint point;
    point.move = sample.move;
    point.cartesian = sample.cartesian;
    point.position = sample.position;
    point.orientation = Quat::fromEulerXYZ(deg2rad(sample.c), deg2rad(sample.b), deg2rad(sample.a));
    point.speed = sample.speed;
    point.zone = sample.zone;
    if (resource == nullptr) {
        point.joints.assign(sample.arm, sample.arm + 6);
        point.externalValues.assign(sample.ext, sample.ext + 6);
        return point;
    }
    point.joints.assign(resource->joints.size(), 0);
    int arm = 0;
    for (std::size_t i = 0; i < resource->joints.size(); ++i) {
        if (resource->joints[i].externalName.empty()) {
            if (arm < 6) {
                point.joints[i] = sample.arm[arm++];
            }
            continue;
        }
        const int slot = externalSlotOf(resource->joints[i].externalName);
        if (slot >= 0) {
            point.joints[i] = sample.ext[slot];
            point.externalValues.push_back(sample.ext[slot]);
        }
    }
    return point;
}

class Scanner {
public:
    explicit Scanner(std::string text) : text_(std::move(text)) {}

    bool done() const { return i_ >= text_.size(); }

    void skip() {
        while (i_ < text_.size()) {
            const unsigned char ch = static_cast<unsigned char>(text_[i_]);
            if (std::isspace(ch)) {
                ++i_;
                continue;
            }
            if (ch == ';') {
                while (i_ < text_.size() && text_[i_] != '\n') {
                    ++i_;
                }
                continue;
            }
            break;
        }
    }

    bool peekWord(const char* word) {
        skip();
        const std::size_t n = std::char_traits<char>::length(word);
        if (i_ + n > text_.size()) {
            return false;
        }
        for (std::size_t k = 0; k < n; ++k) {
            const char a = static_cast<char>(std::tolower(static_cast<unsigned char>(text_[i_ + k])));
            const char b = static_cast<char>(std::tolower(static_cast<unsigned char>(word[k])));
            if (a != b) {
                return false;
            }
        }
        if (i_ + n < text_.size()) {
            const unsigned char next = static_cast<unsigned char>(text_[i_ + n]);
            if (std::isalnum(next) || next == '_') {
                return false;
            }
        }
        return true;
    }

    bool take(const char* word) {
        if (!peekWord(word)) {
            return false;
        }
        i_ += std::char_traits<char>::length(word);
        return true;
    }

    bool takeChar(char ch) {
        skip();
        if (i_ < text_.size() && text_[i_] == ch) {
            ++i_;
            return true;
        }
        return false;
    }

    std::string takeIdent() {
        skip();
        const std::size_t begin = i_;
        while (i_ < text_.size()) {
            const unsigned char ch = static_cast<unsigned char>(text_[i_]);
            if (!std::isalnum(ch) && text_[i_] != '_') {
                break;
            }
            ++i_;
        }
        return text_.substr(begin, i_ - begin);
    }

    bool takeNumber(double& out) {
        skip();
        const std::size_t begin = i_;
        if (i_ < text_.size() && (text_[i_] == '+' || text_[i_] == '-')) {
            ++i_;
        }
        bool digit = false;
        while (i_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[i_]))) {
            digit = true;
            ++i_;
        }
        if (i_ < text_.size() && text_[i_] == '.') {
            ++i_;
            while (i_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[i_]))) {
                digit = true;
                ++i_;
            }
        }
        if (i_ < text_.size() && (text_[i_] == 'e' || text_[i_] == 'E')) {
            const std::size_t mark = i_++;
            if (i_ < text_.size() && (text_[i_] == '+' || text_[i_] == '-')) {
                ++i_;
            }
            bool exp = false;
            while (i_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[i_]))) {
                exp = true;
                ++i_;
            }
            if (!exp) {
                i_ = mark;
            }
        }
        if (!digit) {
            i_ = begin;
            return false;
        }
        try {
            out = std::stod(text_.substr(begin, i_ - begin));
        } catch (...) {
            i_ = begin;
            return false;
        }
        return true;
    }

    std::string takeBrace() {
        skip();
        if (i_ >= text_.size() || text_[i_] != '{') {
            return {};
        }
        const std::size_t begin = i_;
        int depth = 0;
        do {
            if (text_[i_] == '{') {
                ++depth;
            } else if (text_[i_] == '}') {
                --depth;
            }
            ++i_;
        } while (i_ < text_.size() && depth > 0);
        return text_.substr(begin, i_ - begin);
    }

    std::string takeBrackets() {
        skip();
        if (i_ >= text_.size() || text_[i_] != '[') {
            return {};
        }
        const std::size_t begin = i_;
        int depth = 0;
        do {
            if (text_[i_] == '[') {
                ++depth;
            } else if (text_[i_] == ']') {
                --depth;
            }
            ++i_;
        } while (i_ < text_.size() && depth > 0);
        return text_.substr(begin, i_ - begin);
    }

    std::size_t index() const { return i_; }
    std::size_t& indexRef() { return i_; }
    void setIndex(std::size_t index) { i_ = index; }
    void bump() {
        if (i_ < text_.size()) {
            ++i_;
        }
    }
    const std::string& text() const { return text_; }

private:
    std::string text_;
    std::size_t i_ = 0;
};

void readAggregate(const std::string& body, AxisSample& sample) {
    Scanner scan(body);
    scan.takeChar('{');
    while (!scan.done()) {
        if (scan.takeChar('}')) {
            break;
        }
        const std::string key = scan.takeIdent();
        if (key.empty()) {
            break;
        }
        scan.takeChar('=');
        double value = 0;
        if (!scan.takeNumber(value)) {
            break;
        }
        const std::string name = lowerCopy(key);
        if (name.size() == 2 && name[0] == 'a' && name[1] >= '1' && name[1] <= '6') {
            sample.arm[name[1] - '1'] = value;
        } else if (name.size() == 2 && name[0] == 'e' && name[1] >= '1' && name[1] <= '6') {
            sample.ext[name[1] - '1'] = value;
            sample.extUsed[name[1] - '1'] = true;
        } else if (name == "x") {
            sample.position.x = value;
            sample.cartesian = true;
        } else if (name == "y") {
            sample.position.y = value;
        } else if (name == "z") {
            sample.position.z = value;
        } else if (name == "a") {
            sample.a = value;
        } else if (name == "b") {
            sample.b = value;
        } else if (name == "c") {
            sample.c = value;
        }
        scan.takeChar(',');
    }
}

std::string kukaAggregate(const AxisSample& sample, bool cartesian) {
    std::ostringstream out;
    out << '{';
    if (cartesian) {
        out << "X " << formatNum(sample.position.x) << ",Y " << formatNum(sample.position.y) << ",Z "
            << formatNum(sample.position.z) << ",A " << formatNum(sample.a) << ",B " << formatNum(sample.b) << ",C "
            << formatNum(sample.c);
    } else {
        for (int i = 0; i < 6; ++i) {
            if (i > 0) {
                out << ',';
            }
            out << 'A' << (i + 1) << ' ' << formatNum(sample.arm[i]);
        }
    }
    for (int i = 0; i < 6; ++i) {
        out << ",E" << (i + 1) << ' ' << formatNum(sample.ext[i]);
    }
    out << '}';
    return out.str();
}

class KukaProgram final : public IOlpProgram {
public:
    const char* vendor() const override { return "kuka"; }

    bool exportProgram(const KinematicResource& resource, const RobotPath& path, std::string& src, std::string& dat,
                       std::string& error) const override {
        if (path.points.empty()) {
            error = "path has no points";
            return false;
        }
        const std::string name = ident(path.name, "weld");
        std::ostringstream srcOut;
        std::ostringstream datOut;
        srcOut << "DEF " << name << "()\n";
        datOut << "DEFDAT " << name << "\n";
        int index = 1;
        for (const PathPoint& point : path.points) {
            const AxisSample sample = samplePoint(resource, point);
            const bool linear = point.move != PathMove::Ptp;
            const std::string pointName = (linear ? "XL" : "XP") + std::to_string(index);
            srcOut << "$VEL_AXIS[1]=" << formatNum(point.speed) << "\n";
            srcOut << (linear ? "LIN " : "PTP ") << pointName << "\n";
            datOut << "DECL " << (linear ? "E6POS " : "E6AXIS ") << pointName << "=" << kukaAggregate(sample, linear)
                   << "\n";
            datOut << "DECL REAL SP" << index << "=" << formatNum(point.speed) << "\n";
            ++index;
        }
        srcOut << "END\n";
        datOut << "ENDDAT\n";
        src = srcOut.str();
        dat = datOut.str();
        error.clear();
        return true;
    }

    bool importProgram(const KinematicResource* resource, const std::string& src, const std::string& dat, RobotPath& path,
                       std::string& error) const override {
        std::unordered_map<std::string, AxisSample> named;
        std::unordered_map<std::string, double> speeds;
        if (!collectKuka(dat, named, speeds, error) || !collectKuka(src, named, speeds, error)) {
            return false;
        }
        Scanner scan(src);
        std::string program = "imported";
        if (scan.take("DEF")) {
            program = scan.takeIdent();
            scan.take("()");
        }
        scan.setIndex(0);
        path = {};
        path.name = program.empty() ? "imported" : program;
        int index = 1;
        while (!scan.done()) {
            const bool ptp = scan.peekWord("PTP");
            const bool lin = scan.peekWord("LIN");
            if (!ptp && !lin) {
                if (scan.takeIdent().empty()) {
                    scan.bump();
                }
                continue;
            }
            scan.take(ptp ? "PTP" : "LIN");
            AxisSample sample;
            sample.move = ptp ? PathMove::Ptp : PathMove::Lin;
            sample.cartesian = lin;
            std::string inlineBody = scan.takeBrace();
            if (!inlineBody.empty()) {
                readAggregate(inlineBody, sample);
            } else {
                const std::string pointName = scan.takeIdent();
                const auto found = named.find(lowerCopy(pointName));
                if (found == named.end()) {
                    error = "missing KUKA point " + pointName;
                    return false;
                }
                sample = found->second;
                sample.move = ptp ? PathMove::Ptp : PathMove::Lin;
                sample.cartesian = lin || found->second.cartesian;
            }
            const auto speed = speeds.find("sp" + std::to_string(index));
            if (speed != speeds.end()) {
                sample.speed = speed->second;
            }
            path.points.push_back(pointFromSample(resource, sample));
            ++index;
        }
        if (path.points.empty()) {
            error = "no KUKA motion statements";
            return false;
        }
        error.clear();
        return true;
    }

private:
    bool collectKuka(const std::string& text, std::unordered_map<std::string, AxisSample>& named,
                     std::unordered_map<std::string, double>& speeds, std::string& error) const {
        Scanner scan(text);
        while (!scan.done()) {
            if (!scan.take("DECL")) {
                if (scan.takeIdent().empty()) {
                    scan.bump();
                }
                continue;
            }
            if (scan.take("E6AXIS") || scan.take("E6POS")) {
                const std::string pointName = scan.takeIdent();
                scan.takeChar('=');
                const std::string body = scan.takeBrace();
                if (pointName.empty() || body.empty()) {
                    error = "broken KUKA declaration";
                    return false;
                }
                AxisSample sample;
                readAggregate(body, sample);
                named[lowerCopy(pointName)] = sample;
                continue;
            }
            if (scan.take("REAL")) {
                const std::string speedName = scan.takeIdent();
                scan.takeChar('=');
                double value = 0;
                if (!scan.takeNumber(value)) {
                    error = "broken KUKA speed";
                    return false;
                }
                speeds[lowerCopy(speedName)] = value;
            }
        }
        return true;
    }
};

std::vector<double> numbersIn(const std::string& body) {
    std::vector<double> values;
    Scanner scan(body);
    while (!scan.done()) {
        double value = 0;
        if (scan.takeNumber(value)) {
            if (std::abs(value) > 1e8) {
                values.push_back(0);
            } else {
                values.push_back(value);
            }
            continue;
        }
        scan.bump();
    }
    return values;
}

std::string abbExt(const AxisSample& sample) {
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < 6; ++i) {
        if (i > 0) {
            out << ',';
        }
        if (sample.extUsed[i]) {
            out << formatNum(sample.ext[i]);
        } else {
            out << "9E9";
        }
    }
    out << ']';
    return out.str();
}

class AbbProgram final : public IOlpProgram {
public:
    const char* vendor() const override { return "abb"; }

    bool exportProgram(const KinematicResource& resource, const RobotPath& path, std::string& src, std::string& dat,
                       std::string& error) const override {
        if (path.points.empty()) {
            error = "path has no points";
            return false;
        }
        const std::string name = ident(path.name, "weld");
        std::ostringstream srcOut;
        std::ostringstream datOut;
        srcOut << "MODULE " << name << "\n  PROC main()\n";
        datOut << "MODULE " << name << "_data\n";
        int index = 1;
        for (const PathPoint& point : path.points) {
            const AxisSample sample = samplePoint(resource, point);
            const bool linear = point.move != PathMove::Ptp;
            const std::string pointName = (linear ? "p" : "j") + std::to_string(index);
            const int speed = std::max(1, static_cast<int>(std::lround(point.speed)));
            if (linear) {
                datOut << "  CONST robtarget " << pointName << ":=[[" << formatNum(sample.position.x) << ","
                       << formatNum(sample.position.y) << "," << formatNum(sample.position.z) << "],["
                       << formatNum(point.orientation.w) << "," << formatNum(point.orientation.x) << ","
                       << formatNum(point.orientation.y) << "," << formatNum(point.orientation.z) << "],[0,0,0,0],"
                       << abbExt(sample) << "];\n";
                srcOut << "    MoveL " << pointName << ",v" << speed << ",z10,tool0;\n";
            } else {
                datOut << "  CONST jointtarget " << pointName << ":=[[";
                for (int i = 0; i < 6; ++i) {
                    if (i > 0) {
                        datOut << ',';
                    }
                    datOut << formatNum(sample.arm[i]);
                }
                datOut << "]," << abbExt(sample) << "];\n";
                srcOut << "    MoveJ " << pointName << ",v" << speed << ",z10,tool0;\n";
            }
            ++index;
        }
        srcOut << "  ENDPROC\nENDMODULE\n";
        datOut << "ENDMODULE\n";
        src = srcOut.str();
        dat = datOut.str();
        error.clear();
        return true;
    }

    bool importProgram(const KinematicResource* resource, const std::string& src, const std::string& dat, RobotPath& path,
                       std::string& error) const override {
        std::unordered_map<std::string, AxisSample> named;
        if (!collectAbb(dat, named, error) || !collectAbb(src, named, error)) {
            return false;
        }
        Scanner scan(src);
        path = {};
        path.name = "imported";
        if (scan.take("MODULE")) {
            path.name = scan.takeIdent();
        }
        scan.setIndex(0);
        while (!scan.done()) {
            const bool moveJ = scan.peekWord("MoveJ");
            const bool moveL = scan.peekWord("MoveL");
            if (!moveJ && !moveL) {
                if (scan.takeIdent().empty()) {
                    scan.bump();
                }
                continue;
            }
            scan.take(moveJ ? "MoveJ" : "MoveL");
            AxisSample sample;
            sample.move = moveJ ? PathMove::Ptp : PathMove::Lin;
            sample.cartesian = moveL;
            const std::string inlineBody = scan.takeBrackets();
            if (!inlineBody.empty()) {
                if (!fillAbb(inlineBody, moveL, sample)) {
                    error = "broken RAPID target";
                    return false;
                }
            } else {
                const std::string pointName = scan.takeIdent();
                const auto found = named.find(lowerCopy(pointName));
                if (found == named.end()) {
                    error = "missing RAPID target " + pointName;
                    return false;
                }
                sample = found->second;
                sample.move = moveJ ? PathMove::Ptp : PathMove::Lin;
                sample.cartesian = moveL;
            }
            scan.takeChar(',');
            if (scan.takeChar('v')) {
                double speed = 100;
                if (scan.takeNumber(speed)) {
                    sample.speed = speed;
                }
            } else {
                scan.takeIdent();
            }
            path.points.push_back(pointFromSample(resource, sample));
        }
        if (path.points.empty()) {
            error = "no RAPID motion statements";
            return false;
        }
        error.clear();
        return true;
    }

private:
    bool fillAbb(const std::string& body, bool linear, AxisSample& sample) const {
        Scanner scan(body);
        if (!scan.takeChar('[')) {
            return false;
        }
        const std::string first = scan.takeBrackets();
        scan.takeChar(',');
        const std::string second = scan.takeBrackets();
        const std::vector<double> a = numbersIn(first);
        const std::vector<double> b = numbersIn(second);
        if (linear) {
            if (a.size() < 3 || b.size() < 4) {
                return false;
            }
            sample.position = {a[0], a[1], a[2]};
            sample.cartesian = true;
            const Quat rotation{b[0], b[1], b[2], b[3]};
            double rx = 0;
            double ry = 0;
            double rz = 0;
            rotation.toEulerXYZ(rx, ry, rz);
            sample.c = rad2deg(rx);
            sample.b = rad2deg(ry);
            sample.a = rad2deg(rz);
            scan.takeChar(',');
            scan.takeBrackets();
            scan.takeChar(',');
            const std::string ext = scan.takeBrackets();
            const std::vector<double> axes = numbersIn(ext);
            for (int i = 0; i < 6 && i < static_cast<int>(axes.size()); ++i) {
                sample.ext[i] = axes[static_cast<std::size_t>(i)];
                sample.extUsed[i] = true;
            }
            return true;
        }
        if (a.size() < 6) {
            return false;
        }
        for (int i = 0; i < 6; ++i) {
            sample.arm[i] = a[static_cast<std::size_t>(i)];
        }
        for (int i = 0; i < 6 && i < static_cast<int>(b.size()); ++i) {
            sample.ext[i] = b[static_cast<std::size_t>(i)];
            sample.extUsed[i] = true;
        }
        return true;
    }

    bool collectAbb(const std::string& text, std::unordered_map<std::string, AxisSample>& named, std::string& error) const {
        Scanner scan(text);
        while (!scan.done()) {
            const bool joint = scan.peekWord("jointtarget");
            const bool cartesian = scan.peekWord("robtarget");
            if (!joint && !cartesian) {
                if (scan.takeIdent().empty()) {
                    scan.bump();
                }
                continue;
            }
            scan.take(joint ? "jointtarget" : "robtarget");
            const std::string pointName = scan.takeIdent();
            scan.take(":=");
            const std::string body = scan.takeBrackets();
            AxisSample sample;
            if (!fillAbb(body, cartesian, sample)) {
                error = "broken RAPID declaration " + pointName;
                return false;
            }
            named[lowerCopy(pointName)] = sample;
        }
        return true;
    }
};

class FanucProgram final : public IOlpProgram {
public:
    const char* vendor() const override { return "fanuc"; }

    bool exportProgram(const KinematicResource& resource, const RobotPath& path, std::string& src, std::string& dat,
                       std::string& error) const override {
        if (path.points.empty()) {
            error = "path has no points";
            return false;
        }
        const std::string name = ident(path.name, "weld");
        std::ostringstream mn;
        std::ostringstream pos;
        mn << "/PROG " << name << "\n/ATTR\nCOMMENT = \"ASCII LS subset\";\n/MN\n";
        pos << "/POS\n";
        int index = 1;
        for (const PathPoint& point : path.points) {
            const AxisSample sample = samplePoint(resource, point);
            const bool linear = point.move != PathMove::Ptp;
            const int speed = std::max(1, static_cast<int>(std::lround(point.speed)));
            if (linear) {
                mn << "   " << index << ":L P[" << index << "] " << speed << "mm/sec FINE ;\n";
                pos << "P[" << index << "]{\n   GP1:\n";
                pos << "\tX = " << formatNum(sample.position.x) << " mm,\tY = " << formatNum(sample.position.y)
                    << " mm,\tZ = " << formatNum(sample.position.z) << " mm,\n";
                pos << "\tW = " << formatNum(sample.c) << " deg,\tP = " << formatNum(sample.b) << " deg,\tR = "
                    << formatNum(sample.a) << " deg\n";
            } else {
                mn << "   " << index << ":J P[" << index << "] " << speed << "% FINE ;\n";
                pos << "P[" << index << "]{\n   GP1:\n";
                pos << "\tJ1=" << formatNum(sample.arm[0]) << " deg,\tJ2=" << formatNum(sample.arm[1]) << " deg,\tJ3="
                    << formatNum(sample.arm[2]) << " deg,\n";
                pos << "\tJ4=" << formatNum(sample.arm[3]) << " deg,\tJ5=" << formatNum(sample.arm[4]) << " deg,\tJ6="
                    << formatNum(sample.arm[5]) << " deg\n";
            }
            pos << "   GP2:\n";
            pos << "\tJ1=" << formatNum(sample.ext[0]) << " deg,\tJ2=" << formatNum(sample.ext[1]) << " deg,\tJ3="
                << formatNum(sample.ext[2]) << " deg,\n";
            pos << "\tJ4=" << formatNum(sample.ext[3]) << " deg,\tJ5=" << formatNum(sample.ext[4]) << " deg,\tJ6="
                << formatNum(sample.ext[5]) << " deg\n};\n";
            ++index;
        }
        mn << "/END\n";
        src = mn.str() + pos.str();
        dat = pos.str();
        error.clear();
        return true;
    }

    bool importProgram(const KinematicResource* resource, const std::string& src, const std::string& dat, RobotPath& path,
                       std::string& error) const override {
        const std::string joined = src + "\n" + dat;
        std::unordered_map<int, AxisSample> points;
        Scanner scan(joined);
        path = {};
        path.name = "imported";
        if (scan.take("/PROG") || scan.peekWord("/PROG")) {
            scan.take("/PROG");
            path.name = scan.takeIdent();
        }
        scan.setIndex(0);
        while (!scan.done()) {
            if (!(scan.take("P") && scan.takeChar('['))) {
                if (scan.takeIdent().empty()) {
                    scan.bump();
                }
                continue;
            }
            double id = 0;
            if (!scan.takeNumber(id) || !scan.takeChar(']') || !scan.takeChar('{')) {
                continue;
            }
            const std::size_t begin = scan.index();
            int depth = 1;
            while (scan.index() < joined.size() && depth > 0) {
                const char ch = joined[scan.index()];
                if (ch == '{') {
                    ++depth;
                } else if (ch == '}') {
                    --depth;
                }
                scan.bump();
            }
            const std::string body = joined.substr(begin, scan.index() - begin);
            AxisSample sample;
            readFanucPoint(body, sample);
            points[static_cast<int>(id)] = sample;
        }
        Scanner motion(src);
        while (!motion.done()) {
            double line = 0;
            if (!motion.takeNumber(line) || !motion.takeChar(':')) {
                if (motion.takeIdent().empty()) {
                    motion.bump();
                }
                continue;
            }
            const bool joint = motion.take("J");
            const bool linear = motion.take("L");
            if (!joint && !linear) {
                continue;
            }
            if (!motion.take("P") || !motion.takeChar('[')) {
                continue;
            }
            double id = 0;
            if (!motion.takeNumber(id)) {
                continue;
            }
            motion.takeChar(']');
            double speed = 100;
            motion.takeNumber(speed);
            const auto found = points.find(static_cast<int>(id));
            if (found == points.end()) {
                error = "missing FANUC P[" + std::to_string(static_cast<int>(id)) + "]";
                return false;
            }
            AxisSample sample = found->second;
            sample.move = joint ? PathMove::Ptp : PathMove::Lin;
            sample.cartesian = linear;
            sample.speed = speed;
            path.points.push_back(pointFromSample(resource, sample));
        }
        if (path.points.empty()) {
            error = "no FANUC motion statements";
            return false;
        }
        error.clear();
        return true;
    }

private:
    void readGroup(const std::string& body, bool second, AxisSample& sample) const {
        const std::string mark = second ? "GP2" : "GP1";
        const auto start = body.find(mark);
        if (start == std::string::npos) {
            return;
        }
        const auto next = body.find("GP", start + mark.size());
        const std::string slice = body.substr(start, next == std::string::npos ? std::string::npos : next - start);
        Scanner scan(slice);
        while (!scan.done()) {
            const std::string key = scan.takeIdent();
            if (key.empty()) {
                scan.bump();
                continue;
            }
            scan.takeChar('=');
            double value = 0;
            if (!scan.takeNumber(value)) {
                continue;
            }
            const std::string name = lowerCopy(key);
            if (!second && name.size() == 2 && name[0] == 'j' && name[1] >= '1' && name[1] <= '6') {
                sample.arm[name[1] - '1'] = value;
                sample.cartesian = false;
            } else if (second && name.size() == 2 && name[0] == 'j' && name[1] >= '1' && name[1] <= '6') {
                sample.ext[name[1] - '1'] = value;
                sample.extUsed[name[1] - '1'] = true;
            } else if (!second && name == "x") {
                sample.position.x = value;
                sample.cartesian = true;
            } else if (!second && name == "y") {
                sample.position.y = value;
            } else if (!second && name == "z") {
                sample.position.z = value;
            } else if (!second && name == "w") {
                sample.c = value;
            } else if (!second && name == "p") {
                sample.b = value;
            } else if (!second && name == "r") {
                sample.a = value;
            }
        }
    }

    void readFanucPoint(const std::string& body, AxisSample& sample) const {
        readGroup(body, false, sample);
        readGroup(body, true, sample);
    }
};

}  // namespace

std::unique_ptr<IOlpProgram> createOlpProgram(const std::string& vendor) {
    const std::string name = lowerCopy(vendor);
    if (name == "kuka") {
        return std::make_unique<KukaProgram>();
    }
    if (name == "abb") {
        return std::make_unique<AbbProgram>();
    }
    if (name == "fanuc") {
        return std::make_unique<FanucProgram>();
    }
    return nullptr;
}

}  // namespace sim
