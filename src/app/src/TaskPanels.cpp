#include "sim/app/TaskPanels.hpp"

#include "sim/core/Scene.hpp"
#include "sim/motion/Motion.hpp"

#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

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

protected:
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
    tools->addStretch(1);
    layout->addLayout(tools);
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

void LogicCanvas::remember(const std::string& key, const QPointF& position) { pins_[key] = position; }

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
            if (found != pins_.end()) {
                node->setPos(found->second);
            } else {
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

PlcCards::PlcCards(QWidget* parent) : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    auto* note = new QLabel("虚拟调试连接只是界面。Phase 5 之前不会打开套接字，地址固定在本机回环。");
    note->setWordWrap(true);
    layout->addWidget(note);
    const struct {
        const char* title;
        const char* endpoint;
    } cards[] = {
        {"PLCSIM Advanced", "127.0.0.1"},
        {"OPC UA", "opc.tcp://127.0.0.1:4840"},
    };
    for (const auto& card : cards) {
        auto* frame = new QWidget;
        frame->setObjectName("commandPalette");
        auto* column = new QVBoxLayout(frame);
        auto* title = new QLabel(QString::fromUtf8(card.title));
        auto* endpoint = new QLabel(QString::fromUtf8(card.endpoint));
        auto* state = new QLabel("未连接");
        auto* button = new QPushButton("连接");
        column->addWidget(title);
        column->addWidget(endpoint);
        column->addWidget(state);
        column->addWidget(button);
        layout->addWidget(frame);
        connect(button, &QPushButton::clicked, this, [state] {
            state->setText("未连接 · Phase 5 之前不会打开套接字");
        });
    }
    layout->addStretch(1);
}

}  // namespace sim
