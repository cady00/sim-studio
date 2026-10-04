#include "sim/app/TaskPanels.hpp"

#include "sim/core/Scene.hpp"

#include <QPaintEvent>
#include <QPainter>

#include <algorithm>

namespace sim {

LogicCanvas::LogicCanvas(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    setMinimumHeight(220);
}

void LogicCanvas::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(15, 21, 41));
    painter.setPen(QColor(37, 46, 82));
    for (int x = 0; x < width(); x += 22) {
        painter.drawLine(x, 0, x, height());
    }
    for (int y = 0; y < height(); y += 22) {
        painter.drawLine(0, y, width(), y);
    }
    if (scene_.logicBlocks.empty()) {
        painter.setPen(QColor(166, 172, 203));
        painter.drawText(rect().adjusted(16, 16, -16, -16), Qt::AlignLeft | Qt::AlignTop,
                         "还没有逻辑块。块与块之间只通过信号交换。");
        return;
    }
    int index = 0;
    for (const LogicBlock& block : scene_.logicBlocks) {
        const QRect card(24 + (index % 3) * 250, 24 + (index / 3) * 120, 220, 96);
        painter.setPen(QColor(142, 146, 236));
        painter.setBrush(QColor(26, 33, 64));
        painter.drawRoundedRect(card, 8, 8);
        painter.setPen(QColor(231, 233, 247));
        painter.drawText(card.adjusted(10, 8, -10, -8), Qt::AlignLeft | Qt::AlignTop,
                         QString::fromStdString(block.name));
        painter.setPen(QColor(166, 172, 203));
        QString ports = "输入 ";
        for (const std::string& name : block.inputs) {
            ports += QString::fromStdString(name) + " ";
        }
        ports += "\n输出 ";
        for (const std::string& name : block.outputs) {
            ports += QString::fromStdString(name) + " ";
        }
        painter.drawText(card.adjusted(10, 32, -10, -8), ports);
        if (index > 0) {
            painter.setPen(QPen(QColor(231, 233, 247), 2));
            const QPoint from(24 + ((index - 1) % 3) * 250 + 220, 24 + ((index - 1) / 3) * 120 + 48);
            painter.drawLine(from, QPoint(card.left(), card.center().y()));
        }
        ++index;
    }
}

PathTimeline::PathTimeline(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    setMinimumHeight(160);
}

void PathTimeline::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(19, 26, 49));
    painter.setPen(QColor(166, 172, 203));
    if (scene_.robotPaths.empty() || scene_.robotPaths.front().points.empty()) {
        painter.drawText(rect().adjusted(12, 8, -12, -8), Qt::AlignLeft | Qt::AlignTop, "还没有机器人路径。");
        return;
    }
    const RobotPath& path = scene_.robotPaths.front();
    painter.drawText(12, 22, QString::fromStdString(path.name + "  ·  " + path.resource));
    const int left = 16;
    const int top = 40;
    const int count = static_cast<int>(path.points.size());
    const int span = std::max(40, (width() - 32) / std::max(count, 1));
    double time = 0;
    for (int i = 0; i < count; ++i) {
        const PathPoint& point = path.points[static_cast<std::size_t>(i)];
        const double seconds = point.speed > 1 ? 80.0 / point.speed : 0.4;
        time += seconds;
        const QRect block(left + i * span, top, std::max(24, span - 8), 28);
        const QColor color = point.move == PathMove::Lin ? QColor(98, 210, 242) : QColor(142, 146, 236);
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawRoundedRect(block, 5, 5);
        painter.setPen(QColor(12, 16, 48));
        painter.drawText(block, Qt::AlignCenter, QString::fromStdString(point.name.empty() ? pathMoveName(point.move) : point.name));
        painter.setPen(QColor(110, 118, 156));
        painter.drawText(block.left(), block.bottom() + 18, QString::number(time, 'f', 2) + " s");
    }
    painter.setPen(QColor(231, 233, 247));
    painter.drawText(12, height() - 12, QString("节拍约 %1 s  ·  采样按速度粗估，不是规划器的正式节拍").arg(time, 0, 'f', 2));
}

}  // namespace sim
