#include "sim/app/TaskPanels.hpp"

#include "sim/core/Scene.hpp"
#include "sim/io/OpcUa.hpp"
#include "sim/motion/Motion.hpp"

#include <QCheckBox>
#include <QEnterEvent>
#include <QFile>
#include <QFileDialog>
#include <QListWidget>
#include <QMouseEvent>

#include <functional>
#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QInputDialog>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>

// Qt defines `signals` as a macro. The scene stores named I/O signals under that member.
#ifdef signals
#undef signals
#endif

namespace sim {
namespace {

class LogicNodeItem : public QGraphicsRectItem {
public:
    LogicNodeItem(LogicCanvas* canvas, std::string key, const QString& title, const QString& subtitle, const QColor& fill,
                  bool dark)
        : canvas_(canvas), key_(std::move(key)) {
        setRect(0, 0, 168, 64);
        setBrush(fill);
        setPen(QPen(QColor(44, 110, 206), 1.5));
        setFlags(ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges);
        auto* heading = new QGraphicsSimpleTextItem(title, this);
        heading->setBrush(dark ? QColor(231, 233, 247) : QColor(23, 29, 56));
        heading->setPos(10, 8);
        auto* body = new QGraphicsSimpleTextItem(subtitle, this);
        body->setBrush(dark ? QColor(166, 172, 203) : QColor(76, 83, 115));
        body->setPos(10, 32);
    }

    const std::string& key() const { return key_; }
    void setHot(bool hot) { setPen(QPen(hot ? QColor(214, 148, 48) : QColor(44, 110, 206), hot ? 3.0 : 1.5)); }

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override {
        if (event->button() == Qt::LeftButton && event->pos().x() > rect().width() - 18) {
            wiring_ = true;
            event->accept();
            return;
        }
        QGraphicsRectItem::mousePressEvent(event);
    }

    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override {
        if (wiring_) {
            wiring_ = false;
            if (canvas_ != nullptr) {
                canvas_->finishWire(key_, event->scenePos());
            }
            event->accept();
            return;
        }
        QGraphicsRectItem::mouseReleaseEvent(event);
    }

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override {
        if (canvas_ != nullptr) {
            canvas_->editKey(key_);
        }
        QGraphicsRectItem::mouseDoubleClickEvent(event);
    }

    QVariant itemChange(GraphicsItemChange change, const QVariant& value) override {
        const QVariant result = QGraphicsRectItem::itemChange(change, value);
        if (change == ItemPositionHasChanged && canvas_ != nullptr && !canvas_->arranging()) {
            canvas_->remember(key_, pos());
            canvas_->refreshEdges();
        }
        return result;
    }

private:
    LogicCanvas* canvas_ = nullptr;
    std::string key_;
    bool wiring_ = false;
};

QString stepTitle(const LogicStep& step) {
    if (step.actions.empty()) {
        return "空步骤";
    }
    const LogicAction& action = step.actions.front();
    if (action.kind == "move_pose") {
        return QString::fromStdString("姿态 " + action.pose);
    }
    if (action.kind == "jog") {
        return QString::fromStdString("点动 " + action.joint);
    }
    if (action.kind == "set_output" || action.kind == "reset_output") {
        return QString::fromStdString(action.kind + " " + action.signal);
    }
    return QString::fromStdString(action.kind);
}

}  // namespace

LogicCanvas::LogicCanvas(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    setMinimumSize(420, 280);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    auto* tools = new QHBoxLayout;
    tools->setContentsMargins(8, 6, 8, 6);
    auto add = [&](const QString& label, auto slot) {
        auto* button = new QPushButton(label);
        connect(button, &QPushButton::clicked, this, slot);
        tools->addWidget(button);
    };
    add("添加块", [this] { addBlock(); });
    add("添加输入", [this] { addInput(); });
    add("添加步骤", [this] { addStep(); });
    add("添加输出", [this] { addOutput(); });
    add("重命名", [this] { renameSelected(); });
    add("应用文本", [this] { applyText(); });
    tools->addStretch(1);
    layout->addLayout(tools);
    source_ = new QPlainTextEdit(this);
    source_->setPlaceholderText("name block\nin go\nout done\npython:\ndef on_step(dt):\n    pass\n---\nwait 0.1\nstep");
    source_->setMaximumHeight(120);
    layout->addWidget(source_);
    graph_ = new QGraphicsScene(this);
    view_ = new QGraphicsView(graph_, this);
    view_->setRenderHint(QPainter::Antialiasing);
    view_->setDragMode(QGraphicsView::RubberBandDrag);
    view_->setBackgroundBrush(QColor(15, 21, 41));
    layout->addWidget(view_, 1);
    connect(graph_, &QGraphicsScene::selectionChanged, this, [this] {
        const auto items = graph_->selectedItems();
        if (items.isEmpty()) {
            return;
        }
        if (auto* node = dynamic_cast<LogicNodeItem*>(items.front())) {
            const std::string& key = node->key();
            const auto slash = key.find('/');
            if (slash != std::string::npos) {
                selected_ = std::atoi(key.substr(0, slash).c_str());
                showText();
            }
        }
    });
    reload();
}

void LogicCanvas::setDark(bool dark) {
    dark_ = dark;
    if (view_ != nullptr) {
        view_->setBackgroundBrush(dark ? QColor(15, 21, 41) : QColor(246, 247, 251));
    }
    reload();
}

void LogicCanvas::remember(const std::string& key, const QPointF& position) {
    pins_[key] = position;
    const auto slash = key.find('/');
    if (slash == std::string::npos) {
        return;
    }
    const int index = std::atoi(key.substr(0, slash).c_str());
    if (index < 0 || index >= static_cast<int>(scene_.logicBlocks.size())) {
        return;
    }
    LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(index)];
    const std::string suffix = key.substr(slash + 1);
    for (auto& pin : block.pins) {
        if (pin.first == suffix) {
            pin.second = {position.x(), position.y()};
            return;
        }
    }
    block.pins.emplace_back(suffix, std::make_pair(position.x(), position.y()));
}

void LogicCanvas::refreshEdges() {
    const auto existing = graph_->items();
    for (QGraphicsItem* item : existing) {
        if (item->data(0).toString() == "edge") {
            graph_->removeItem(item);
            delete item;
        }
    }
    std::map<std::string, LogicNodeItem*> nodes;
    for (QGraphicsItem* item : graph_->items()) {
        if (auto* node = dynamic_cast<LogicNodeItem*>(item)) {
            nodes[node->key()] = node;
        }
    }
    auto link = [&](const std::string& fromKey, const std::string& toKey, const QColor& color, Qt::PenStyle style) {
        const auto from = nodes.find(fromKey);
        const auto to = nodes.find(toKey);
        if (from == nodes.end() || to == nodes.end()) {
            return;
        }
        const QPointF a = from->second->scenePos() + QPointF(from->second->rect().width(), from->second->rect().height() / 2);
        const QPointF b = to->second->scenePos() + QPointF(0, to->second->rect().height() / 2);
        QPainterPath path(a);
        path.cubicTo(a + QPointF(36, 0), b - QPointF(36, 0), b);
        auto* edge = graph_->addPath(path, QPen(color, 1.6, style));
        edge->setData(0, "edge");
        edge->setZValue(-1);
    };
    for (int index = 0; index < static_cast<int>(scene_.logicBlocks.size()); ++index) {
        const LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(index)];
        std::vector<std::string> keys;
        keys.push_back(std::to_string(index) + "/in");
        for (int step = 0; step < static_cast<int>(block.steps.size()); ++step) {
            keys.push_back(std::to_string(index) + "/s" + std::to_string(step));
        }
        keys.push_back(std::to_string(index) + "/out");
        for (std::size_t i = 1; i < keys.size(); ++i) {
            link(keys[i - 1], keys[i], QColor(61, 134, 232), Qt::SolidLine);
        }
    }
    for (int from = 0; from < static_cast<int>(scene_.logicBlocks.size()); ++from) {
        for (int to = 0; to < static_cast<int>(scene_.logicBlocks.size()); ++to) {
            if (from == to) {
                continue;
            }
            for (const std::string& output : scene_.logicBlocks[static_cast<std::size_t>(from)].outputs) {
                const auto& inputs = scene_.logicBlocks[static_cast<std::size_t>(to)].inputs;
                if (std::find(inputs.begin(), inputs.end(), output) != inputs.end()) {
                    link(std::to_string(from) + "/out", std::to_string(to) + "/in", QColor(192, 132, 252), Qt::DashLine);
                    break;
                }
            }
        }
    }
}

void LogicCanvas::reload() {
    arranging_ = true;
    graph_->clear();
    if (scene_.logicBlocks.empty()) {
        auto* note = graph_->addText("还没有逻辑块。块与块之间只通过信号交换。");
        note->setDefaultTextColor(dark_ ? QColor(166, 172, 203) : QColor(76, 83, 115));
        note->setPos(16, 16);
        arranging_ = false;
        return;
    }
    for (int index = 0; index < static_cast<int>(scene_.logicBlocks.size()); ++index) {
        const LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(index)];
        std::vector<LogicNodeItem*> row;
        const QColor nodeFill = dark_ ? QColor(26, 33, 64) : QColor(255, 255, 255);
        const QColor stepFill = dark_ ? QColor(22, 36, 72) : QColor(236, 242, 252);
        const QColor outFill = dark_ ? QColor(18, 40, 64) : QColor(232, 244, 242);
        auto place = [&](const std::string& suffix, const QString& title, const QString& subtitle, const QColor& fill) {
            const std::string key = std::to_string(index) + "/" + suffix;
            auto* node = new LogicNodeItem(this, key, title, subtitle, fill, dark_);
            const auto found = pins_.find(key);
            bool placed = false;
            if (found != pins_.end()) {
                node->setPos(found->second);
                placed = true;
            } else {
                for (const auto& pin : block.pins) {
                    if (pin.first == suffix) {
                        node->setPos(pin.second.first, pin.second.second);
                        pins_[key] = node->pos();
                        placed = true;
                        break;
                    }
                }
            }
            if (!placed) {
                node->setPos(24 + static_cast<int>(row.size()) * 210, 36 + index * 140);
                pins_[key] = node->pos();
            }
            graph_->addItem(node);
            row.push_back(node);
        };
        if (block.inputs.empty()) {
            place("in", QString::fromStdString(block.name), "无输入", nodeFill);
        } else {
            QString ports;
            for (const std::string& name : block.inputs) {
                ports += QString::fromStdString(name) + " ";
            }
            place("in", QString::fromStdString(block.name), "输入 " + ports.trimmed(), nodeFill);
        }
        for (int step = 0; step < static_cast<int>(block.steps.size()); ++step) {
            const LogicStep& item = block.steps[static_cast<std::size_t>(step)];
            QString when;
            for (const LogicCondition& condition : item.when) {
                when += QString::fromStdString(condition.signal) + " ";
            }
            place("s" + std::to_string(step), stepTitle(item), when.isEmpty() ? "无条件" : "当 " + when.trimmed(), stepFill);
        }
        QString outs;
        for (const std::string& name : block.outputs) {
            outs += QString::fromStdString(name) + " ";
        }
        place("out", "输出", outs.isEmpty() ? "无" : outs.trimmed(), outFill);
        (void)row;
    }
    arranging_ = false;
    refreshEdges();
    restyle();
    showText();
}

void LogicCanvas::addBlock() {
    LogicBlock block;
    block.name = "block" + std::to_string(scene_.logicBlocks.size() + 1);
    block.inputs = {"in"};
    block.outputs = {"out"};
    LogicStep step;
    LogicAction wait;
    wait.kind = "wait";
    wait.seconds = 0.1;
    step.actions.push_back(wait);
    block.steps.push_back(step);
    scene_.logicBlocks.push_back(std::move(block));
    scene_.notify(ChangeKind::Structure);
    selected_ = static_cast<int>(scene_.logicBlocks.size()) - 1;
    reload();
}

void LogicCanvas::addInput() {
    if (scene_.logicBlocks.empty()) {
        addBlock();
        return;
    }
    selected_ = std::clamp(selected_, 0, static_cast<int>(scene_.logicBlocks.size()) - 1);
    LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(selected_)];
    block.inputs.push_back("in" + std::to_string(block.inputs.size() + 1));
    scene_.notify(ChangeKind::Structure);
    reload();
}

void LogicCanvas::addStep() {
    if (scene_.logicBlocks.empty()) {
        addBlock();
        return;
    }
    selected_ = std::clamp(selected_, 0, static_cast<int>(scene_.logicBlocks.size()) - 1);
    LogicStep step;
    LogicAction wait;
    wait.kind = "wait";
    wait.seconds = 0.1;
    step.actions.push_back(wait);
    scene_.logicBlocks[static_cast<std::size_t>(selected_)].steps.push_back(step);
    scene_.notify(ChangeKind::Structure);
    reload();
}

void LogicCanvas::addOutput() {
    if (scene_.logicBlocks.empty()) {
        addBlock();
        return;
    }
    selected_ = std::clamp(selected_, 0, static_cast<int>(scene_.logicBlocks.size()) - 1);
    LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(selected_)];
    block.outputs.push_back("out" + std::to_string(block.outputs.size() + 1));
    scene_.notify(ChangeKind::Structure);
    reload();
}

void LogicCanvas::renameSelected() {
    if (scene_.logicBlocks.empty()) {
        return;
    }
    selected_ = std::clamp(selected_, 0, static_cast<int>(scene_.logicBlocks.size()) - 1);
    LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(selected_)];
    bool ok = false;
    const QString name = QInputDialog::getText(this, "逻辑块", "名称", QLineEdit::Normal, QString::fromStdString(block.name), &ok);
    if (!ok || name.trimmed().isEmpty()) {
        return;
    }
    block.name = name.trimmed().toStdString();
    scene_.notify(ChangeKind::Structure);
    reload();
}

void LogicCanvas::showText() {
    if (source_ == nullptr || source_->hasFocus()) {
        return;
    }
    if (scene_.logicBlocks.empty()) {
        source_->clear();
        return;
    }
    selected_ = std::clamp(selected_, 0, static_cast<int>(scene_.logicBlocks.size()) - 1);
    source_->setPlainText(QString::fromStdString(logicToText(scene_.logicBlocks[static_cast<std::size_t>(selected_)])));
}

void LogicCanvas::applyText() {
    if (source_ == nullptr || scene_.logicBlocks.empty()) {
        return;
    }
    selected_ = std::clamp(selected_, 0, static_cast<int>(scene_.logicBlocks.size()) - 1);
    LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(selected_)];
    LogicBlock parsed = block;
    std::string error;
    if (!logicFromText(source_->toPlainText().toStdString(), parsed, error)) {
        source_->setPlaceholderText(QString::fromStdString(error));
        return;
    }
    parsed.pins = block.pins;
    parsed.resource = block.resource;
    parsed.repeat = block.repeat;
    block = std::move(parsed);
    scene_.notify(ChangeKind::Structure);
    reload();
}

void LogicCanvas::finishWire(const std::string& fromKey, const QPointF& scenePos) {
    if (graph_ == nullptr) {
        return;
    }
    const auto slash = fromKey.find('/');
    if (slash == std::string::npos || fromKey.substr(slash + 1) != "out") {
        return;
    }
    const int fromIndex = std::atoi(fromKey.substr(0, slash).c_str());
    for (QGraphicsItem* item : graph_->items(scenePos)) {
        auto* node = dynamic_cast<LogicNodeItem*>(item);
        if (node == nullptr) {
            continue;
        }
        const auto toSlash = node->key().find('/');
        if (toSlash == std::string::npos || node->key().substr(toSlash + 1) != "in") {
            continue;
        }
        const int toIndex = std::atoi(node->key().substr(0, toSlash).c_str());
        if (toIndex == fromIndex || toIndex < 0 || fromIndex < 0 ||
            toIndex >= static_cast<int>(scene_.logicBlocks.size()) ||
            fromIndex >= static_cast<int>(scene_.logicBlocks.size())) {
            return;
        }
        std::string signal;
        if (!wireLogicBlocks(scene_.logicBlocks[static_cast<std::size_t>(fromIndex)],
                             scene_.logicBlocks[static_cast<std::size_t>(toIndex)], signal)) {
            return;
        }
        scene_.notify(ChangeKind::Structure);
        reload();
        return;
    }
}

void LogicCanvas::editKey(const std::string& key) {
    const auto slash = key.find('/');
    if (slash == std::string::npos) {
        return;
    }
    const int index = std::atoi(key.substr(0, slash).c_str());
    const std::string suffix = key.substr(slash + 1);
    if (index < 0 || index >= static_cast<int>(scene_.logicBlocks.size()) || suffix.size() < 2 || suffix[0] != 's') {
        return;
    }
    const int stepIndex = std::atoi(suffix.substr(1).c_str());
    LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(index)];
    if (stepIndex < 0 || stepIndex >= static_cast<int>(block.steps.size())) {
        return;
    }
    QString current = "wait 0.1";
    const LogicStep& step = block.steps[static_cast<std::size_t>(stepIndex)];
    if (!step.actions.empty()) {
        const LogicAction& action = step.actions.front();
        if (action.kind == "wait" || action.kind == "jog") {
            current = QString::fromStdString(action.kind + " " + std::to_string(action.seconds));
        } else if (action.kind == "move_pose") {
            current = QString::fromStdString("move_pose " + action.pose);
        } else if (!action.signal.empty()) {
            current = QString::fromStdString(action.kind + " " + action.signal);
        } else {
            current = QString::fromStdString(action.kind);
        }
    }
    bool ok = false;
    const QString line = QInputDialog::getText(this, "步骤", "kind signal 或 wait 秒", QLineEdit::Normal, current, &ok);
    if (!ok || line.trimmed().isEmpty()) {
        return;
    }
    const std::string text = "name holder\npython:\n---\n" + line.trimmed().toStdString() + "\nstep\n";
    LogicBlock parsed;
    std::string error;
    if (!logicFromText(text, parsed, error) || parsed.steps.empty()) {
        return;
    }
    block.steps[static_cast<std::size_t>(stepIndex)] = parsed.steps.front();
    scene_.notify(ChangeKind::Structure);
    reload();
}

void LogicCanvas::setLive(const std::vector<std::string>& stepKeys, const std::vector<std::string>& hotSignals) {
    liveKeys_ = stepKeys;
    hotSignals_ = hotSignals;
    restyle();
}

void LogicCanvas::restyle() {
    if (graph_ == nullptr) {
        return;
    }
    for (QGraphicsItem* item : graph_->items()) {
        auto* node = dynamic_cast<LogicNodeItem*>(item);
        if (node == nullptr) {
            continue;
        }
        bool hot = std::find(liveKeys_.begin(), liveKeys_.end(), node->key()) != liveKeys_.end();
        if (!hot) {
            const auto slash = node->key().find('/');
            if (slash != std::string::npos) {
                const int index = std::atoi(node->key().substr(0, slash).c_str());
                if (index >= 0 && index < static_cast<int>(scene_.logicBlocks.size())) {
                    const LogicBlock& block = scene_.logicBlocks[static_cast<std::size_t>(index)];
                    for (const std::string& name : block.inputs) {
                        if (std::find(hotSignals_.begin(), hotSignals_.end(), name) != hotSignals_.end()) {
                            hot = node->key().substr(slash + 1) == "in";
                        }
                    }
                    for (const std::string& name : block.outputs) {
                        const Signal* signal = scene_.findSignal(name);
                        if (signal != nullptr && signal->flag &&
                            std::find(hotSignals_.begin(), hotSignals_.end(), name) != hotSignals_.end() &&
                            node->key().substr(slash + 1) == "out") {
                            hot = true;
                        }
                    }
                }
            }
        }
        node->setHot(hot);
    }
}

PathTimeline::PathTimeline(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    setMinimumHeight(220);
}

void PathTimeline::setDark(bool dark) {
    dark_ = dark;
    update();
}

void PathTimeline::reload() {
    signature_.clear();
    planned_ = false;
    ok_ = false;
    seconds_ = 0;
    segmentSeconds_.clear();
    samples_.clear();
    note_.clear();
    if (scene_.robotPaths.empty() || scene_.robotPaths.front().points.empty()) {
        note_ = "还没有机器人路径。";
        update();
        return;
    }
    const RobotPath& path = scene_.robotPaths.front();
    KinematicResource* resource = findResource(scene_.resources, path.resource);
    if (resource == nullptr && !scene_.resources.empty()) {
        resource = &scene_.resources.front();
    }
    if (resource == nullptr) {
        note_ = "找不到运动学资源，没有调用规划器。";
        update();
        return;
    }
    std::string next = path.name + "|" + path.resource + "|" + std::to_string(path.points.size());
    for (const PathPoint& point : path.points) {
        next += "|" + point.name + "|" + std::to_string(point.speed) + "|" + std::to_string(point.zone) + "|" +
                std::to_string(static_cast<int>(point.move));
    }
    if (planned_ && signature_ == next) {
        update();
        return;
    }
    const MotionPlan plan = planRobotPath(scene_, *resource, path);
    planned_ = true;
    ok_ = plan.ok;
    seconds_ = plan.seconds;
    dt_ = plan.dt;
    segmentSeconds_ = plan.segmentSeconds;
    samples_ = plan.samples;
    note_.clear();
    for (const std::string& warning : plan.warnings) {
        if (!note_.empty()) {
            note_ += "；";
        }
        note_ += warning;
    }
    if (note_.empty()) {
        note_ = plan.ok ? "规划完成" : "规划带回了限制或不可达";
    }
    signature_ = next;
    update();
}

void PathTimeline::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    const QColor paper = dark_ ? QColor(19, 26, 49) : QColor(255, 255, 255);
    const QColor ink = dark_ ? QColor(231, 233, 247) : QColor(23, 29, 56);
    const QColor muted = dark_ ? QColor(166, 172, 203) : QColor(76, 83, 115);
    painter.fillRect(rect(), paper);
    painter.setPen(muted);
    if (!planned_) {
        painter.drawText(rect().adjusted(12, 8, -12, -8), Qt::AlignLeft | Qt::AlignTop,
                         note_.empty() ? "打开机器人模式后计算规划节拍。" : QString::fromStdString(note_));
        return;
    }
    if (scene_.robotPaths.empty()) {
        painter.drawText(12, 22, QString::fromStdString(note_));
        return;
    }
    const RobotPath& path = scene_.robotPaths.front();
    painter.setPen(ink);
    painter.drawText(12, 20, QString::fromStdString(path.name + "  ·  " + path.resource));
    painter.setPen(muted);
    double segmentSum = 0;
    for (double span : segmentSeconds_) {
        segmentSum += span;
    }
    painter.drawText(12, 40, QString("分段合计 %1 s · 过渡后 %2 s").arg(segmentSum, 0, 'f', 3).arg(seconds_, 0, 'f', 3));

    const int count = static_cast<int>(path.points.size());
    const int span = std::max(48, (width() - 32) / std::max(count, 1));
    for (int i = 0; i < count; ++i) {
        const PathPoint& point = path.points[static_cast<std::size_t>(i)];
        const double seconds = i < static_cast<int>(segmentSeconds_.size()) ? segmentSeconds_[static_cast<std::size_t>(i)] : 0;
        const QRect block(16 + i * span, 50, std::max(28, span - 8), 26);
        const QColor color = point.move == PathMove::Lin ? QColor(45, 212, 191) : QColor(44, 110, 206);
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawRoundedRect(block, 5, 5);
        painter.setPen(point.move == PathMove::Lin ? QColor(12, 16, 48) : QColor(255, 255, 255));
        painter.drawText(block, Qt::AlignCenter, QString::fromStdString(point.name.empty() ? pathMoveName(point.move) : point.name));
        painter.setPen(muted);
        painter.drawText(block.left(), block.bottom() + 16, QString::number(seconds, 'f', 2) + " s");
    }

    const QRect plot(16, 100, std::max(40, width() - 32), std::max(40, height() - 128));
    painter.setPen(dark_ ? QColor(37, 46, 82) : QColor(205, 209, 224));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(plot);
    if (samples_.size() >= 2) {
        const int joints = static_cast<int>(samples_.front().size());
        const int shown = std::min(joints, 6);
        const QColor inks[] = {QColor(61, 134, 232), QColor(45, 212, 191), QColor(192, 132, 252),
                               QColor(255, 178, 36), QColor(231, 233, 247), QColor(95, 211, 141)};
        for (int joint = 0; joint < shown; ++joint) {
            double lo = samples_.front()[static_cast<std::size_t>(joint)];
            double hi = lo;
            for (const auto& sample : samples_) {
                if (static_cast<int>(sample.size()) <= joint) {
                    continue;
                }
                lo = std::min(lo, sample[static_cast<std::size_t>(joint)]);
                hi = std::max(hi, sample[static_cast<std::size_t>(joint)]);
            }
            const double spanValue = std::max(hi - lo, 1e-6);
            QPainterPath curve;
            bool started = false;
            for (std::size_t i = 0; i < samples_.size(); ++i) {
                if (static_cast<int>(samples_[i].size()) <= joint) {
                    continue;
                }
                const double u = static_cast<double>(i) / static_cast<double>(samples_.size() - 1);
                const double v = (samples_[i][static_cast<std::size_t>(joint)] - lo) / spanValue;
                const QPointF at(plot.left() + u * plot.width(), plot.bottom() - v * (plot.height() - 4) - 2);
                if (!started) {
                    curve.moveTo(at);
                    started = true;
                } else {
                    curve.lineTo(at);
                }
            }
            painter.setPen(QPen(inks[joint], 1.4));
            painter.drawPath(curve);
        }
    }
    painter.setPen(ink);
    const QString footer = QString("色块是各点未混合时长，合计节拍 %1 s 已计入过渡。关节曲线来自规划采样。%2")
                               .arg(seconds_, 0, 'f', 3)
                               .arg(QString::fromStdString(note_));
    painter.drawText(QRect(12, height() - 22, width() - 24, 18), Qt::AlignLeft | Qt::AlignVCenter, footer);
}

PlcCards::PlcCards(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    auto* layout = new QVBoxLayout(this);
    auto* note = new QLabel("OPC UA 支持 SecurityPolicy None，以及 Basic256Sha256 的签名和签名加密、自签证书、用户名口令、断线重连。信号表可导入 CSV 或 TIA 变量表。PLCSIM Advanced 运行时未安装，只保留地址。");
    note->setWordWrap(true);
    layout->addWidget(note);

    auto addCard = [&](const QString& title, const QString& endpoint, QLabel*& state) {
        auto* frame = new QWidget;
        frame->setObjectName("commandPalette");
        auto* column = new QVBoxLayout(frame);
        column->addWidget(new QLabel(title));
        column->addWidget(new QLabel(endpoint));
        state = new QLabel("未连接");
        state->setWordWrap(true);
        auto* button = new QPushButton("连接");
        column->addWidget(state);
        column->addWidget(button);
        layout->addWidget(frame);
        return button;
    };
    QLabel* plcsimState = nullptr;
    QPushButton* plcsimButton = addCard("PLCSIM Advanced", "127.0.0.1", plcsimState);
    plcsimState->setText("未连接 · 本机没有安装 PLCSIM Advanced 运行时");
    connect(plcsimButton, &QPushButton::clicked, this, [plcsimState] {
        plcsimState->setText("未连接 · 本机没有安装 PLCSIM Advanced 运行时");
    });
    QPushButton* opcButton = addCard("OPC UA", "opc.tcp://127.0.0.1:4840", opcState_);
    connect(opcButton, &QPushButton::clicked, this, [this] { connectOpcUa(); });
    auto* import = new QPushButton("导入信号表");
    connect(import, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, "导入信号表", QString(), "信号表 (*.csv *.txt);;所有文件 (*)");
        if (path.isEmpty()) {
            return;
        }
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly)) {
            opcState_->setText("未导入 · 打不开文件");
            return;
        }
        importText(QString::fromUtf8(file.readAll()));
    });
    layout->addWidget(import);
    layout->addStretch(1);
}

bool PlcCards::importText(const QString& text) {
    std::vector<ImportedSignal> rows;
    std::string error;
    if (!importSignalTable(text.toStdString(), rows, error)) {
        opcState_->setText(QString::fromUtf8("未导入 · ") + QString::fromStdString(error));
        return false;
    }
    applyImportedSignals(scene_, rows);
    opcState_->setText(QString("已导入 %1 个信号").arg(static_cast<int>(rows.size())));
    emit signalsImported();
    return true;
}

bool PlcCards::demonstrateSecure() {
    importText("name,type,value\nextend,bool,true\ncount,int,4\ngap,real,12.5\n");
    OpcUaCertificate serverCert;
    OpcUaCertificate clientCert;
    std::string error;
    if (!serverCert.createSelfSigned("simstudio-server", error) || !clientCert.createSelfSigned("simstudio-client", error)) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        return false;
    }
    delete secureServer_;
    secureServer_ = new OpcUaServer;
    secureServer_->useSecurity(OpcUaSecurity::SignAndEncrypt, serverCert);
    secureServer_->setUser("operator", "secret");
    if (!secureServer_->listen(0, error)) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        return false;
    }
    const int port = secureServer_->port();
    OpcUaServer* server = secureServer_;
    std::thread([server] {
        std::string serveError;
        server->serve(serveError);
    }).detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    delete client_;
    client_ = new OpcUaClient;
    client_->useSecurity(OpcUaSecurity::SignAndEncrypt, clientCert, serverCert);
    client_->setUser("operator", "secret");
    const std::string endpoint = "opc.tcp://127.0.0.1:" + std::to_string(port);
    const bool opened = client_->connect("127.0.0.1", port, error) && client_->hello(endpoint, error) &&
                        client_->openSecureChannel(error) && client_->createSession(error) && client_->activateSession(error);
    if (!opened) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        return false;
    }
    std::vector<OpcUaReference> refs;
    if (!client_->browse(85, refs, error)) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        return false;
    }
    bool value = false;
    if (!client_->readBool(10001, value, error)) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        return false;
    }
    publishPlcBool(scene_, "part_seen", value);
    const QString name = refs.empty() ? QStringLiteral("i=10001") : QString::fromStdString(refs.front().browseName);
    opcState_->setText(QString("已连接 · Basic256Sha256 SignAndEncrypt · operator · %1 → part_seen · %2")
                           .arg(name)
                           .arg(value ? "true" : "false"));
    emit signalsImported();
    return true;
}

class ScopePlot : public QWidget {
public:
    explicit ScopePlot(QWidget* parent = nullptr) : QWidget(parent) {
        setObjectName("scope.plot");
        setMouseTracking(true);
        setMinimumHeight(160);
    }

    ScopeTrace* trace = nullptr;
    std::vector<std::string> visible;
    bool velocity = true;
    bool acceleration = true;
    bool dark = true;
    double cursor = 0;
    bool holdCursor = false;
    QString emptyText;
    std::function<void()> cursorChanged;

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        const QColor paper = dark ? QColor(18, 24, 42) : QColor(248, 249, 252);
        const QColor ink = dark ? QColor(230, 232, 242) : QColor(28, 32, 44);
        const QColor grid = dark ? QColor(42, 51, 80) : QColor(210, 214, 224);
        painter.fillRect(rect(), paper);
        if (trace == nullptr || trace->time.size() < 2) {
            painter.setPen(ink);
            painter.drawText(rect(), Qt::AlignCenter, emptyText);
            return;
        }
        struct Lane {
            QString name;
            QColor color;
            Qt::PenStyle style;
            std::vector<double> values;
        };
        const QColor palette[] = {QColor(91, 141, 239), QColor(227, 155, 58), QColor(61, 190, 122),
                                  QColor(212, 106, 106), QColor(176, 123, 224), QColor(78, 196, 212)};
        std::vector<Lane> lanes;
        int colorIndex = 0;
        for (const std::string& id : visible) {
            const ScopeTrace::Series* series = trace->findSeries(id);
            if (series == nullptr) {
                continue;
            }
            const QColor color = palette[colorIndex % 6];
            ++colorIndex;
            lanes.push_back(Lane{QString::fromStdString(id), color, Qt::SolidLine, series->value});
            if (id.rfind("jnt:", 0) == 0 && velocity) {
                lanes.push_back(Lane{QString::fromStdString(id + ".v"), color, Qt::DashLine, trace->velocity(id)});
            }
            if (id.rfind("jnt:", 0) == 0 && acceleration) {
                lanes.push_back(Lane{QString::fromStdString(id + ".a"), color, Qt::DotLine, trace->acceleration(id)});
            }
        }
        if (lanes.empty()) {
            painter.setPen(ink);
            painter.drawText(rect(), Qt::AlignCenter, emptyText);
            return;
        }
        const int left = 132;
        const int right = 12;
        const int top = 8;
        const int bottom = 18;
        const QRect plot(left, top, std::max(1, width() - left - right), std::max(1, height() - top - bottom));
        const double t0 = trace->time.front();
        const double t1 = std::max(trace->time.back(), t0 + 1e-6);
        const int laneHeight = std::max(22, plot.height() / static_cast<int>(lanes.size()));
        painter.setPen(grid);
        painter.drawRect(plot);
        for (std::size_t lane = 0; lane < lanes.size(); ++lane) {
            const int y0 = plot.top() + static_cast<int>(lane) * laneHeight;
            painter.drawLine(plot.left(), y0, plot.right(), y0);
        }
        auto xOf = [&](double time) {
            return plot.left() + static_cast<int>((time - t0) / (t1 - t0) * plot.width());
        };
        for (std::size_t lane = 0; lane < lanes.size(); ++lane) {
            const Lane& item = lanes[lane];
            const int y0 = plot.top() + static_cast<int>(lane) * laneHeight;
            const int y1 = std::min(plot.bottom(), y0 + laneHeight);
            double lo = 0;
            double hi = 0;
            bool any = false;
            const std::size_t count = std::min(trace->time.size(), item.values.size());
            for (std::size_t i = 0; i < count; ++i) {
                lo = any ? std::min(lo, item.values[i]) : item.values[i];
                hi = any ? std::max(hi, item.values[i]) : item.values[i];
                any = true;
            }
            if (!any) {
                continue;
            }
            if (std::abs(hi - lo) < 1e-9) {
                hi = lo + 1;
            }
            QPainterPath path;
            for (std::size_t i = 0; i < count; ++i) {
                const double fraction = (item.values[i] - lo) / (hi - lo);
                const QPointF point(xOf(trace->time[i]), y1 - fraction * (y1 - y0 - 4) - 2);
                if (i == 0) {
                    path.moveTo(point);
                } else {
                    path.lineTo(point);
                }
            }
            QPen pen(item.color, item.style == Qt::SolidLine ? 1.6 : 1.2, item.style);
            painter.setPen(pen);
            painter.drawPath(path);
            painter.setPen(ink);
            painter.drawText(QRect(4, y0, left - 8, laneHeight), Qt::AlignVCenter | Qt::AlignLeft,
                             painter.fontMetrics().elidedText(item.name, Qt::ElideMiddle, left - 8));
        }
        const int cursorX = std::clamp(xOf(cursor), plot.left(), plot.right());
        painter.setPen(QPen(QColor(240, 192, 64), 1.2));
        painter.drawLine(cursorX, plot.top(), cursorX, plot.bottom());
        painter.setPen(ink);
        painter.drawText(QRect(plot.left(), height() - bottom, plot.width(), bottom), Qt::AlignLeft | Qt::AlignVCenter,
                         QString::number(t0, 'f', 2) + QStringLiteral(" s"));
        painter.drawText(QRect(plot.left(), height() - bottom, plot.width(), bottom), Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(t1, 'f', 2) + QStringLiteral(" s"));
    }

    void mousePressEvent(QMouseEvent* event) override { moveCursor(event->position().x()); }
    void mouseMoveEvent(QMouseEvent* event) override {
        if (event->buttons() & Qt::LeftButton || holdCursor) {
            moveCursor(event->position().x());
        }
    }
    void enterEvent(QEnterEvent*) override { holdCursor = true; }
    void leaveEvent(QEvent*) override {
        holdCursor = false;
        if (trace != nullptr && !trace->time.empty()) {
            cursor = trace->time.back();
            update();
            if (cursorChanged) {
                cursorChanged();
            }
        }
    }

private:
    void moveCursor(int x) {
        if (trace == nullptr || trace->time.size() < 2) {
            return;
        }
        const int left = 132;
        const int right = 12;
        const int plotWidth = std::max(1, width() - left - right);
        const double t0 = trace->time.front();
        const double t1 = std::max(trace->time.back(), t0 + 1e-6);
        const double fraction = std::clamp((x - left) / static_cast<double>(plotWidth), 0.0, 1.0);
        cursor = t0 + fraction * (t1 - t0);
        holdCursor = true;
        update();
        if (cursorChanged) {
            cursorChanged();
        }
    }
};

SignalScope::SignalScope(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    auto* row = new QHBoxLayout;
    velocityBox_ = new QCheckBox;
    velocityBox_->setObjectName("scope.velocity");
    velocityBox_->setChecked(true);
    accelerationBox_ = new QCheckBox;
    accelerationBox_->setObjectName("scope.acceleration");
    accelerationBox_->setChecked(true);
    exportButton_ = new QPushButton;
    exportButton_->setObjectName("scope.export");
    clearButton_ = new QPushButton;
    clearButton_->setObjectName("scope.clear");
    row->addWidget(velocityBox_);
    row->addWidget(accelerationBox_);
    row->addStretch(1);
    row->addWidget(exportButton_);
    row->addWidget(clearButton_);
    layout->addLayout(row);
    auto* body = new QHBoxLayout;
    list_ = new QListWidget;
    list_->setObjectName("scope.list");
    list_->setMaximumWidth(220);
    plot_ = new ScopePlot;
    plot_->trace = &trace_;
    plot_->cursorChanged = [this] { refreshReadout(); };
    body->addWidget(list_);
    body->addWidget(plot_, 1);
    layout->addLayout(body, 1);
    readout_ = new QLabel;
    readout_->setObjectName("scope.readout");
    readout_->setWordWrap(true);
    layout->addWidget(readout_);
    connect(list_, &QListWidget::itemChanged, this, [this] {
        if (filling_) {
            return;
        }
        userPicked_ = true;
        syncWatch();
    });
    connect(velocityBox_, &QCheckBox::toggled, this, [this](bool checked) {
        if (plot_ != nullptr) {
            plot_->velocity = checked;
            plot_->update();
            refreshReadout();
        }
    });
    connect(accelerationBox_, &QCheckBox::toggled, this, [this](bool checked) {
        if (plot_ != nullptr) {
            plot_->acceleration = checked;
            plot_->update();
            refreshReadout();
        }
    });
    connect(exportButton_, &QPushButton::clicked, this, [this] { exportCsv(); });
    connect(clearButton_, &QPushButton::clicked, this, [this] { clearTrace(); });
    setTexts(QStringLiteral("速度"), QStringLiteral("加速度"), QStringLiteral("导出 CSV"), QStringLiteral("清空"),
             QStringLiteral("等待仿真"));
    reload();
}

void SignalScope::setTexts(const QString& velocity, const QString& acceleration, const QString& exportCsv, const QString& clearText,
                           const QString& waiting) {
    waiting_ = waiting;
    if (velocityBox_ != nullptr) {
        velocityBox_->setText(velocity);
    }
    if (accelerationBox_ != nullptr) {
        accelerationBox_->setText(acceleration);
    }
    if (exportButton_ != nullptr) {
        exportButton_->setText(exportCsv);
    }
    if (clearButton_ != nullptr) {
        clearButton_->setText(clearText);
    }
    if (plot_ != nullptr) {
        plot_->emptyText = waiting;
        plot_->update();
    }
}

void SignalScope::setDark(bool dark) {
    if (plot_ != nullptr) {
        plot_->dark = dark;
        plot_->update();
    }
}

void SignalScope::sample(double timeSeconds) {
    trace_.sample(scene_, timeSeconds);
    if (plot_ != nullptr && !plot_->holdCursor && !trace_.time.empty()) {
        plot_->cursor = trace_.time.back();
    }
    refreshReadout();
    if (plot_ != nullptr) {
        plot_->update();
    }
}

void SignalScope::clearTrace() {
    trace_.clear();
    if (plot_ != nullptr) {
        plot_->cursor = 0;
        plot_->update();
    }
    refreshReadout();
}

void SignalScope::reload() {
    if (list_ == nullptr) {
        return;
    }
    std::vector<std::string> previous;
    for (int i = 0; i < list_->count(); ++i) {
        if (list_->item(i)->checkState() == Qt::Checked) {
            previous.push_back(list_->item(i)->data(Qt::UserRole).toString().toStdString());
        }
    }
    const bool first = list_->count() == 0 && !userPicked_;
    filling_ = true;
    list_->clear();
    auto add = [this](const std::string& id) {
        auto* item = new QListWidgetItem(QString::fromStdString(id));
        item->setData(Qt::UserRole, QString::fromStdString(id));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
        list_->addItem(item);
    };
    for (const Signal& signal : scene_.signals) {
        add("sig:" + signal.name);
    }
    for (const KinematicResource& resource : scene_.resources) {
        for (const KinJoint& joint : resource.joints) {
            add("jnt:" + resource.name + "/" + joint.name);
        }
    }
    int checked = 0;
    for (int i = 0; i < list_->count(); ++i) {
        const std::string id = list_->item(i)->data(Qt::UserRole).toString().toStdString();
        const bool was = std::find(previous.begin(), previous.end(), id) != previous.end();
        const bool pick = first && checked < 6;
        if (was || pick) {
            list_->item(i)->setCheckState(Qt::Checked);
            ++checked;
        }
    }
    filling_ = false;
    syncWatch();
}

void SignalScope::syncWatch() {
    std::vector<std::string> ids;
    std::vector<std::string> visible;
    if (list_ != nullptr) {
        for (int i = 0; i < list_->count(); ++i) {
            const std::string id = list_->item(i)->data(Qt::UserRole).toString().toStdString();
            if (list_->item(i)->checkState() == Qt::Checked) {
                ids.push_back(id);
                visible.push_back(id);
            }
        }
    }
    trace_.watch(ids);
    if (plot_ != nullptr) {
        plot_->visible = std::move(visible);
        plot_->velocity = velocityBox_ != nullptr && velocityBox_->isChecked();
        plot_->acceleration = accelerationBox_ != nullptr && accelerationBox_->isChecked();
        plot_->update();
    }
    refreshReadout();
}

void SignalScope::refreshReadout() {
    if (readout_ == nullptr) {
        return;
    }
    const double cursor = plot_ == nullptr ? 0 : plot_->cursor;
    QString text = QStringLiteral("t=%1 s").arg(cursor, 0, 'f', 3);
    const bool velocity = velocityBox_ != nullptr && velocityBox_->isChecked();
    const bool acceleration = accelerationBox_ != nullptr && accelerationBox_->isChecked();
    if (plot_ != nullptr) {
        for (const std::string& id : plot_->visible) {
            text += QStringLiteral("  ") + QString::fromStdString(id) + QStringLiteral("=") +
                    QString::number(trace_.valueAt(id, cursor), 'f', 2);
            if (velocity && id.rfind("jnt:", 0) == 0) {
                const std::vector<double> speed = trace_.velocity(id);
                double shown = 0;
                const std::size_t count = std::min(trace_.time.size(), speed.size());
                for (std::size_t i = 0; i < count; ++i) {
                    if (trace_.time[i] <= cursor + 1e-12) {
                        shown = speed[i];
                    }
                }
                text += QStringLiteral(" v=") + QString::number(shown, 'f', 1);
            }
            if (acceleration && id.rfind("jnt:", 0) == 0) {
                const std::vector<double> accel = trace_.acceleration(id);
                double shown = 0;
                const std::size_t count = std::min(trace_.time.size(), accel.size());
                for (std::size_t i = 0; i < count; ++i) {
                    if (trace_.time[i] <= cursor + 1e-12) {
                        shown = accel[i];
                    }
                }
                text += QStringLiteral(" a=") + QString::number(shown, 'f', 1);
            }
        }
    }
    readout_->setText(text);
}

void SignalScope::exportCsv() {
    const QString path = QFileDialog::getSaveFileName(this, exportButton_ == nullptr ? QStringLiteral("CSV") : exportButton_->text(),
                                                      QString(), QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) {
        return;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return;
    }
    const bool velocity = velocityBox_ != nullptr && velocityBox_->isChecked();
    const bool acceleration = accelerationBox_ != nullptr && accelerationBox_->isChecked();
    const std::string csv = trace_.toCsv(velocity, acceleration);
    file.write(csv.data(), static_cast<qint64>(csv.size()));
}

PlcCards::~PlcCards() { delete client_; }

bool PlcCards::connectOpcUa() {
    delete client_;
    client_ = new OpcUaClient;
    std::string error;
    const bool opened = client_->connect("127.0.0.1", 4840, error) && client_->hello("opc.tcp://127.0.0.1:4840", error) &&
                        client_->openSecureChannel(error) && client_->createSession(error) && client_->activateSession(error);
    if (!opened) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        delete client_;
        client_ = nullptr;
        return false;
    }
    std::vector<OpcUaReference> refs;
    if (!client_->browse(85, refs, error)) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        return false;
    }
    bool value = false;
    if (!client_->readBool(10001, value, error)) {
        opcState_->setText(QString::fromUtf8("未连接 · ") + QString::fromStdString(error));
        return false;
    }
    publishPlcBool(scene_, "part_seen", value);
    const QString name = refs.empty() ? QStringLiteral("i=10001") : QString::fromStdString(refs.front().browseName);
    opcState_->setText(QString("已连接 · %1 → part_seen · %2").arg(name).arg(value ? "true" : "false"));
    return true;
}

}  // namespace sim
