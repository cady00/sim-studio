#include "sim/app/LinePanels.hpp"

#include "sim/core/LineModel.hpp"
#include "sim/core/Scene.hpp"
#include "sim/motion/Motion.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <string>

namespace sim {
namespace {

QColor brand() { return QColor(44, 110, 206); }
QColor brandHot() { return QColor(61, 134, 232); }
QColor amber() { return QColor(214, 148, 48); }

struct PlanView {
    double originX = 0;
    double originY = 0;
    double scale = 0.05;
    double width = 1;
    double height = 1;
};

PlanView fitPlan(const Scene& scene, double width, double height) {
    PlanView view;
    view.width = std::max(1.0, width);
    view.height = std::max(1.0, height);
    double minX = 0;
    double minY = 0;
    double maxX = 1000;
    double maxY = 1000;
    bool any = false;
    auto grow = [&](double x, double y) {
        if (!any) {
            minX = maxX = x;
            minY = maxY = y;
            any = true;
            return;
        }
        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
    };
    for (const Station& station : scene.layout.stations) {
        grow(station.x - station.width * 0.5, station.y - station.depth * 0.5);
        grow(station.x + station.width * 0.5, station.y + station.depth * 0.5);
    }
    for (const auto& line : scene.layout.underlay.lines) {
        grow(line.first.x, line.first.y);
        grow(line.second.x, line.second.y);
    }
    const double spanX = std::max(100.0, maxX - minX);
    const double spanY = std::max(100.0, maxY - minY);
    const double pad = 36;
    view.scale = std::min((view.width - pad * 2) / spanX, (view.height - pad * 2) / spanY);
    if (!(view.scale > 1e-9)) {
        view.scale = 0.05;
    }
    view.originX = pad - minX * view.scale + (view.width - pad * 2 - spanX * view.scale) * 0.5;
    view.originY = view.height - pad + minY * view.scale - (view.height - pad * 2 - spanY * view.scale) * 0.5;
    return view;
}

QPointF toScreen(const PlanView& view, double x, double y) {
    return {view.originX + x * view.scale, view.originY - y * view.scale};
}

class FloorCanvas : public QWidget {
public:
    FloorCanvas(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) { setMinimumHeight(220); setMouseTracking(true); }

    void setDark(bool dark) {
        dark_ = dark;
        update();
    }
    void setZh(bool zh) {
        zh_ = zh;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), dark_ ? QColor(18, 24, 42) : QColor(244, 246, 251));
        const PlanView view = fitPlan(scene_, width(), height());
        const ScheduleReport report = scheduleSequence(scene_.operations, scene_.sequence, scene_.layout);
        const double grid = scene_.layout.gridMm > 1 ? scene_.layout.gridMm : 500;
        painter.setPen(QPen(dark_ ? QColor(42, 53, 80) : QColor(210, 216, 230), 1));
        const double minX = -view.originX / view.scale;
        const double maxX = (width() - view.originX) / view.scale;
        const double maxY = view.originY / view.scale;
        const double minY = (view.originY - height()) / view.scale;
        for (double x = std::floor(minX / grid) * grid; x <= maxX; x += grid) {
            const QPointF a = toScreen(view, x, minY);
            const QPointF b = toScreen(view, x, maxY);
            painter.drawLine(a, b);
        }
        for (double y = std::floor(minY / grid) * grid; y <= maxY; y += grid) {
            painter.drawLine(toScreen(view, minX, y), toScreen(view, maxX, y));
        }
        painter.setPen(QPen(dark_ ? QColor(120, 140, 170) : QColor(90, 104, 130), 1));
        for (const auto& line : scene_.layout.underlay.lines) {
            painter.drawLine(toScreen(view, line.first.x, line.first.y), toScreen(view, line.second.x, line.second.y));
        }
        if (scene_.layout.underlay.kind == "image" && !scene_.layout.underlay.path.empty()) {
            QImage image(QString::fromStdString(scene_.layout.underlay.path));
            if (!image.isNull()) {
                painter.setOpacity(std::clamp(scene_.layout.underlay.opacity, 0.05, 1.0));
                painter.drawImage(QRectF(24, 24, width() * 0.45, height() * 0.45), image);
                painter.setOpacity(1);
            }
        }
        for (const Station& station : scene_.layout.stations) {
            const QPointF center = toScreen(view, station.x, station.y);
            const QRectF box(center.x() - station.width * view.scale * 0.5, center.y() - station.depth * view.scale * 0.5,
                             station.width * view.scale, station.depth * view.scale);
            const bool bottleneck = station.id == report.bottleneck;
            painter.setBrush(bottleneck ? QColor(214, 148, 48, 70) : QColor(44, 110, 206, 70));
            painter.setPen(QPen(bottleneck ? amber() : brand(), bottleneck ? 3 : 1.5));
            painter.drawRoundedRect(box, 6, 6);
            painter.setPen(dark_ ? QColor(236, 240, 248) : QColor(23, 29, 56));
            const QString name = QString::fromStdString(station.name.empty() ? station.id : station.name);
            painter.drawText(box.adjusted(8, 6, -8, -6), Qt::AlignLeft | Qt::AlignTop, name);
            if (!station.resource.empty()) {
                painter.setPen(brandHot());
                painter.drawText(box.adjusted(8, 24, -8, -6), Qt::AlignLeft | Qt::AlignTop, QString::fromStdString(station.resource));
            }
        }
        if (scene_.layout.example) {
            painter.setPen(Qt::NoPen);
            painter.setBrush(brand());
            const QString badge = zh_ ? QStringLiteral("示例数据") : QStringLiteral("Example data");
            const QRectF pill(12, 10, zh_ ? 78 : 108, 22);
            painter.drawRoundedRect(pill, 4, 4);
            painter.setPen(Qt::white);
            painter.drawText(pill, Qt::AlignCenter, badge);
        }
    }

    void mousePressEvent(QMouseEvent* event) override {
        drag_ = hit(event->position());
        if (drag_ >= 0) {
            const Station& station = scene_.layout.stations[static_cast<std::size_t>(drag_)];
            grabX_ = station.x;
            grabY_ = station.y;
            grabPos_ = event->position();
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (drag_ < 0 || !(event->buttons() & Qt::LeftButton)) {
            return;
        }
        const PlanView view = fitPlan(scene_, width(), height());
        Station& station = scene_.layout.stations[static_cast<std::size_t>(drag_)];
        const QPointF delta = event->position() - grabPos_;
        station.x = snapToGrid(grabX_ + delta.x() / view.scale, scene_.layout.gridMm);
        station.y = snapToGrid(grabY_ - delta.y() / view.scale, scene_.layout.gridMm);
        update();
    }

    void mouseReleaseEvent(QMouseEvent*) override {
        if (drag_ >= 0) {
            scene_.notify(ChangeKind::Structure);
        }
        drag_ = -1;
    }

private:
    int hit(QPointF point) const {
        const PlanView view = fitPlan(scene_, width(), height());
        for (int i = static_cast<int>(scene_.layout.stations.size()) - 1; i >= 0; --i) {
            const Station& station = scene_.layout.stations[static_cast<std::size_t>(i)];
            const QPointF center = toScreen(view, station.x, station.y);
            const QRectF box(center.x() - station.width * view.scale * 0.5, center.y() - station.depth * view.scale * 0.5,
                             station.width * view.scale, station.depth * view.scale);
            if (box.contains(point)) {
                return i;
            }
        }
        return -1;
    }

    Scene& scene_;
    bool dark_ = true;
    bool zh_ = true;
    int drag_ = -1;
    double grabX_ = 0;
    double grabY_ = 0;
    QPointF grabPos_;
};

class GanttCanvas : public QWidget {
public:
    GanttCanvas(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) { setMinimumHeight(180); setMouseTracking(true); }

    void setDark(bool dark) {
        dark_ = dark;
        update();
    }
    void setZh(bool zh) {
        zh_ = zh;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), dark_ ? QColor(18, 24, 42) : QColor(244, 246, 251));
        const QColor ink = dark_ ? QColor(230, 232, 242) : QColor(23, 29, 56);
        painter.setPen(ink);
        const ScheduleReport report = scheduleSequence(scene_.operations, scene_.sequence, scene_.layout);
        if (!report.error.empty()) {
            painter.drawText(rect().adjusted(12, 12, -12, -12), QString::fromStdString(report.error));
            return;
        }
        const QString caption =
            (scene_.layout.example ? (zh_ ? QStringLiteral("示例数据 · ") : QStringLiteral("Example data · ")) : QString()) +
            (zh_ ? QStringLiteral("节拍 %1 s · 瓶颈 %2") : QStringLiteral("Cycle %1 s · Bottleneck %2"))
                .arg(report.makespan, 0, 'f', 1)
                .arg(QString::fromStdString(report.bottleneckName.empty() ? report.bottleneck : report.bottleneckName));
        painter.drawText(QRect(12, 6, width() - 24, 22), Qt::AlignLeft | Qt::AlignVCenter, caption);

        std::vector<std::string> rows;
        for (const Station& station : scene_.layout.stations) {
            rows.push_back(station.id);
        }
        for (const ScheduleBar& bar : report.bars) {
            if (std::find(rows.begin(), rows.end(), bar.station) == rows.end()) {
                rows.push_back(bar.station);
            }
        }
        if (rows.empty()) {
            painter.drawText(rect(), Qt::AlignCenter, zh_ ? QStringLiteral("没有工位") : QStringLiteral("No stations"));
            return;
        }
        const int top = 34;
        const int label = 88;
        const double makespan = std::max(report.makespan, 0.001);
        const double rowH = std::max(22.0, (height() - top - 16) / static_cast<double>(rows.size()));
        auto rowOf = [&](const std::string& id) {
            return static_cast<int>(std::find(rows.begin(), rows.end(), id) - rows.begin());
        };
        auto nameOf = [&](const std::string& id) {
            if (const Station* station = findStation(scene_.layout, id)) {
                return QString::fromStdString(station->name.empty() ? station->id : station->name);
            }
            return QString::fromStdString(id);
        };
        for (int row = 0; row < static_cast<int>(rows.size()); ++row) {
            const QRectF band(0, top + row * rowH, width(), rowH);
            if (rows[static_cast<std::size_t>(row)] == report.bottleneck) {
                painter.fillRect(band, QColor(214, 148, 48, 120));
                painter.fillRect(QRectF(0, band.top(), 5, band.height()), amber());
            } else if (row % 2 == 0) {
                painter.fillRect(band, dark_ ? QColor(255, 255, 255, 8) : QColor(0, 0, 0, 8));
            }
            painter.setPen(ink);
            painter.drawText(QRectF(8, band.top(), label - 12, band.height()), Qt::AlignVCenter | Qt::AlignLeft,
                             nameOf(rows[static_cast<std::size_t>(row)]));
        }
        hits_.clear();
        plotMakespan_ = makespan;
        plotLabel_ = label;
        auto xOf = [&](double t) { return label + (width() - label - 16) * (t / makespan); };
        auto barOf = [&](const std::string& id) -> const ScheduleBar* {
            for (const ScheduleBar& bar : report.bars) {
                if (bar.id == id) {
                    return &bar;
                }
            }
            return nullptr;
        };
        for (const ScheduleBar& bar : report.bars) {
            const int row = rowOf(bar.station);
            if (row < 0 || row >= static_cast<int>(rows.size())) {
                continue;
            }
            const double y = top + row * rowH + 6;
            const QRectF block(xOf(bar.start), y, std::max(4.0, xOf(bar.finish) - xOf(bar.start)), rowH - 12);
            double wait = 0;
            bool blocked = false;
            for (const Operation& operation : scene_.operations) {
                if (operation.id == bar.id) {
                    wait = std::max(0.0, operation.wait);
                    blocked = operation.timeNote == "干涉阻塞";
                }
            }
            if (wait > 1e-6) {
                const QRectF gap(xOf(bar.start - wait), y, std::max(2.0, xOf(bar.start) - xOf(bar.start - wait)), rowH - 12);
                painter.setPen(Qt::NoPen);
                painter.setBrush(QBrush(QColor(44, 110, 206, 90), Qt::BDiagPattern));
                painter.drawRect(gap);
            }
            Hit hit;
            hit.id = bar.id;
            hit.rect = block;
            hit.start = bar.start;
            hit.finish = bar.finish;
            hits_.push_back(hit);
            if (blocked) {
                painter.setBrush(QColor(196, 92, 42));
                painter.setPen(QPen(QColor(120, 48, 24), 2));
            } else {
                painter.setBrush(bar.critical ? brand() : QColor(96, 110, 138));
                painter.setPen(QPen(bar.critical ? brandHot() : QColor(70, 80, 102), bar.critical ? 2 : 1));
            }
            painter.drawRoundedRect(block, 3, 3);
            painter.setPen(Qt::white);
            painter.drawText(block.adjusted(4, 0, -4, 0), Qt::AlignVCenter | Qt::AlignLeft, QString::fromStdString(bar.name));
        }
        painter.setPen(QPen(brandHot(), 1.2));
        for (const Operation& operation : scene_.operations) {
            if (operation.kind != OpKind::Task) {
                continue;
            }
            const ScheduleBar* self = barOf(operation.id);
            if (self == nullptr) {
                continue;
            }
            std::vector<std::string> preds = operation.after;
            if (!operation.parent.empty()) {
                const Operation* parent = nullptr;
                for (const Operation& candidate : scene_.operations) {
                    if (candidate.id == operation.parent) {
                        parent = &candidate;
                    }
                }
                if (parent != nullptr && parent->kind == OpKind::Sequence) {
                    const Operation* previous = nullptr;
                    for (const Operation& candidate : scene_.operations) {
                        if (candidate.parent == parent->id && candidate.index < operation.index &&
                            (previous == nullptr || candidate.index > previous->index)) {
                            previous = &candidate;
                        }
                    }
                    if (previous != nullptr) {
                        if (previous->kind == OpKind::Task) {
                            preds.push_back(previous->id);
                        } else {
                            const ScheduleBar* best = nullptr;
                            for (const ScheduleBar& bar : report.bars) {
                                const Operation* owner = nullptr;
                                for (const Operation& candidate : scene_.operations) {
                                    if (candidate.id == bar.id) {
                                        owner = &candidate;
                                    }
                                }
                                if (owner != nullptr && owner->parent == previous->id && (best == nullptr || bar.finish > best->finish)) {
                                    best = &bar;
                                }
                            }
                            if (best != nullptr) {
                                preds.push_back(best->id);
                            }
                        }
                    }
                }
            }
            for (const std::string& pred : preds) {
                const ScheduleBar* from = barOf(pred);
                if (from == nullptr) {
                    continue;
                }
                const QPointF a(xOf(from->finish), top + rowOf(from->station) * rowH + rowH * 0.5);
                const QPointF b(xOf(self->start), top + rowOf(self->station) * rowH + rowH * 0.5);
                painter.drawLine(a, b);
            }
        }
    }

    void mousePressEvent(QMouseEvent* event) override {
        dragId_.clear();
        dragEdge_ = false;
        for (const Hit& hit : hits_) {
            if (!hit.rect.adjusted(-2, -2, 2, 2).contains(event->position())) {
                continue;
            }
            dragId_ = hit.id;
            dragEdge_ = event->position().x() >= hit.rect.right() - 8;
            dragOrigin_ = event->position();
            dragStart_ = hit.start;
            dragFinish_ = hit.finish;
            break;
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (dragId_.empty() || !(event->buttons() & Qt::LeftButton)) {
            return;
        }
        const double plot = std::max(1.0, width() - plotLabel_ - 16.0);
        const double dt = (event->position().x() - dragOrigin_.x()) / plot * std::max(plotMakespan_, 0.001);
        if (dragEdge_) {
            adjustOperationFinish(scene_.operations, scene_.layout, dragId_, dragFinish_ + dt);
        } else {
            adjustOperationStart(scene_.operations, scene_.layout, dragId_, dragStart_ + dt);
        }
        update();
    }

    void mouseReleaseEvent(QMouseEvent*) override {
        if (!dragId_.empty()) {
            scene_.notify(ChangeKind::Structure);
        }
        dragId_.clear();
    }

private:
    struct Hit {
        std::string id;
        QRectF rect;
        double start = 0;
        double finish = 0;
    };
    Scene& scene_;
    bool dark_ = true;
    bool zh_ = true;
    std::vector<Hit> hits_;
    std::string dragId_;
    bool dragEdge_ = false;
    QPointF dragOrigin_;
    double dragStart_ = 0;
    double dragFinish_ = 0;
    double plotMakespan_ = 1;
    int plotLabel_ = 88;
};

void fillOperationTree(QTreeWidgetItem* parent, QTreeWidget* tree, const Scene& scene, const std::string& parentId) {
    std::vector<const Operation*> children;
    for (const Operation& operation : scene.operations) {
        if (operation.parent == parentId) {
            children.push_back(&operation);
        }
    }
    std::sort(children.begin(), children.end(), [](const Operation* a, const Operation* b) {
        if (a->index != b->index) {
            return a->index < b->index;
        }
        return a->id < b->id;
    });
    for (const Operation* operation : children) {
        QStringList columns;
        columns << QString::fromStdString(operation->name.empty() ? operation->id : operation->name);
        columns << QString::fromUtf8(opKindName(operation->kind));
        columns << QString::fromStdString(operation->station);
        columns << (operation->kind == OpKind::Task ? QString::number(operation->seconds, 'f', 1) : QString());
        columns << QString::fromStdString(operation->timeSource);
        columns << QString::fromStdString(operation->path);
        columns << QString::fromStdString(operation->logic);
        QString after;
        for (const std::string& id : operation->after) {
            if (!after.isEmpty()) {
                after += QStringLiteral(", ");
            }
            after += QString::fromStdString(id);
        }
        columns << after;
        auto* item = parent == nullptr ? new QTreeWidgetItem(tree, columns) : new QTreeWidgetItem(parent, columns);
        fillOperationTree(item, tree, scene, operation->id);
        item->setExpanded(true);
    }
}

class SequenceCanvas : public QWidget {
public:
    SequenceCanvas(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) { setMinimumHeight(160); setObjectName("sequence.timeline"); }

    void setDark(bool dark) {
        dark_ = dark;
        update();
    }
    void setZh(bool zh) {
        zh_ = zh;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), dark_ ? QColor(18, 24, 42) : QColor(244, 246, 251));
        const QColor ink = dark_ ? QColor(230, 232, 242) : QColor(23, 29, 56);
        painter.setPen(ink);
        const ScheduleReport report = scheduleSequence(scene_.operations, scene_.sequence, scene_.layout);
        if (!report.error.empty()) {
            painter.drawText(rect().adjusted(12, 12, -12, -12), QString::fromStdString(report.error));
            return;
        }
        const double makespan = std::max(report.makespan, 0.001);
        const int label = 96;
        const int top = 28;
        painter.drawText(QRect(12, 4, width() - 24, 20),
                         (zh_ ? QStringLiteral("时间轴 · 节拍 %1 s") : QStringLiteral("Timeline · cycle %1 s")).arg(makespan, 0, 'f', 1));
        if (report.bars.empty()) {
            painter.drawText(rect(), Qt::AlignCenter, zh_ ? QStringLiteral("没有操作") : QStringLiteral("No operations"));
            return;
        }
        const double rowH = std::max(22.0, (height() - top - 28) / static_cast<double>(report.bars.size()));
        auto xOf = [&](double time) {
            return label + (width() - label - 16) * (time / makespan);
        };
        for (int i = 0; i < static_cast<int>(report.bars.size()); ++i) {
            const ScheduleBar& bar = report.bars[static_cast<std::size_t>(i)];
            const QRectF band(0, top + i * rowH, width(), rowH);
            painter.setPen(ink);
            painter.drawText(QRectF(8, band.top(), label - 12, band.height()), Qt::AlignVCenter | Qt::AlignLeft,
                             QString::fromStdString(bar.name.empty() ? bar.id : bar.name));
            const QRectF box(xOf(bar.start), band.top() + 4, std::max(4.0, xOf(bar.finish) - xOf(bar.start)), band.height() - 8);
            painter.setPen(Qt::NoPen);
            painter.setBrush(bar.critical ? brandHot() : brand());
            painter.drawRoundedRect(box, 3, 3);
        }
        painter.setBrush(amber());
        painter.setPen(Qt::NoPen);
        for (const SequenceEvent& event : scene_.sequence) {
            double time = event.offset;
            for (const ScheduleBar& bar : report.bars) {
                if (bar.id != event.operation) {
                    continue;
                }
                time = (event.edge == "start" ? bar.start : bar.finish) + (event.kind == "transition" ? 0 : event.offset);
                if (event.kind == "transition") {
                    time = bar.finish;
                }
            }
            const double x = xOf(std::clamp(time, 0.0, makespan));
            QPolygonF mark;
            mark << QPointF(x, height() - 18) << QPointF(x + 6, height() - 12) << QPointF(x, height() - 6) << QPointF(x - 6, height() - 12);
            painter.drawPolygon(mark);
        }
    }

private:
    Scene& scene_;
    bool dark_ = true;
    bool zh_ = true;
};

}  // namespace

FloorPlanView::FloorPlanView(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    auto* row = new QHBoxLayout;
    auto add = [&](const QString& name, const QString& text, auto handler) {
        auto* button = new QPushButton(text, this);
        button->setObjectName(name);
        QObject::connect(button, &QPushButton::clicked, this, handler);
        row->addWidget(button);
        return button;
    };
    add(QStringLiteral("line.snap"), QStringLiteral("栅格"), [this] {
        snapStations(scene_.layout);
        scene_.notify(ChangeKind::Structure);
    });
    add(QStringLiteral("line.align"), QStringLiteral("对齐"), [this] {
        alignStationsMinX(scene_.layout);
        scene_.notify(ChangeKind::Structure);
    });
    add(QStringLiteral("line.distribute"), QStringLiteral("分布"), [this] {
        distributeStationsX(scene_.layout);
        scene_.notify(ChangeKind::Structure);
    });
    add(QStringLiteral("line.dxf"), QStringLiteral("DXF"), [this] {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("DXF"), {}, QStringLiteral("DXF (*.dxf)"));
        if (path.isEmpty()) {
            return;
        }
        std::string error;
        loadDxfUnderlay(scene_.layout, path.toStdString(), scene_.layout.underlay.mmPerUnit, error);
        scene_.notify(ChangeKind::Structure);
    });
    add(QStringLiteral("line.image"), QStringLiteral("图片"), [this] {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Image"), {}, QStringLiteral("Image (*.png *.jpg *.jpeg *.bmp)"));
        if (path.isEmpty()) {
            return;
        }
        setImageUnderlay(scene_.layout, path.toStdString(), 0.35);
        scene_.notify(ChangeKind::Structure);
    });
    row->addStretch(1);
    layout->addLayout(row);
    canvas_ = new FloorCanvas(scene_, this);
    layout->addWidget(canvas_, 1);
}

void FloorPlanView::setDark(bool dark) {
    if (auto* canvas = static_cast<FloorCanvas*>(canvas_)) {
        canvas->setDark(dark);
    }
}

void FloorPlanView::setZh(bool zh) {
    if (auto* canvas = static_cast<FloorCanvas*>(canvas_)) {
        canvas->setZh(zh);
    }
    const auto text = [zh](const char* key) {
        if (std::string(key) == "line.snap") return zh ? QStringLiteral("栅格") : QStringLiteral("Grid");
        if (std::string(key) == "line.align") return zh ? QStringLiteral("对齐") : QStringLiteral("Align");
        if (std::string(key) == "line.distribute") return zh ? QStringLiteral("分布") : QStringLiteral("Spread");
        if (std::string(key) == "line.image") return zh ? QStringLiteral("图片") : QStringLiteral("Image");
        return QStringLiteral("DXF");
    };
    for (const char* key : {"line.snap", "line.align", "line.distribute", "line.dxf", "line.image"}) {
        if (auto* button = findChild<QPushButton*>(key)) {
            button->setText(text(key));
        }
    }
}

void FloorPlanView::reload() {
    if (canvas_ != nullptr) {
        canvas_->update();
    }
}

OperationTreeView::OperationTreeView(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    badge_ = new QLabel(this);
    tree_ = new QTreeWidget(this);
    tree_->setObjectName("line.operations");
    tree_->setColumnCount(8);
    tree_->setHeaderLabels({QStringLiteral("名称"), QStringLiteral("类型"), QStringLiteral("工位"), QStringLiteral("秒"),
                            QStringLiteral("来源"), QStringLiteral("路径"), QStringLiteral("逻辑"), QStringLiteral("前置")});
    tree_->setRootIsDecorated(true);
    layout->addWidget(badge_);
    layout->addWidget(tree_, 1);
    reload();
}

void OperationTreeView::setDark(bool) { reload(); }

void OperationTreeView::setZh(bool zh) {
    zh_ = zh;
    tree_->setHeaderLabels(zh ? QStringList{QStringLiteral("名称"), QStringLiteral("类型"), QStringLiteral("工位"), QStringLiteral("秒"),
                                             QStringLiteral("来源"), QStringLiteral("路径"), QStringLiteral("逻辑"), QStringLiteral("前置")}
                              : QStringList{QStringLiteral("Name"), QStringLiteral("Kind"), QStringLiteral("Station"), QStringLiteral("s"),
                                            QStringLiteral("Source"), QStringLiteral("Path"), QStringLiteral("Logic"), QStringLiteral("After")});
    reload();
}

void OperationTreeView::reload() {
    if (badge_ != nullptr) {
        const QString mark = scene_.layout.example ? (zh_ ? QStringLiteral("示例数据 · ") : QStringLiteral("Example data · ")) : QString();
        badge_->setText(mark + QString::fromStdString(scene_.layout.title));
    }
    if (tree_ == nullptr) {
        return;
    }
    tree_->clear();
    fillOperationTree(nullptr, tree_, scene_, "");
    tree_->resizeColumnToContents(0);
}

GanttView::GanttView(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    auto* row = new QHBoxLayout;
    auto* csv = new QPushButton(QStringLiteral("CSV"), this);
    auto* xlsx = new QPushButton(QStringLiteral("XLSX"), this);
    auto* measure = new QPushButton(QStringLiteral("测算节拍"), this);
    measure->setObjectName("line.measure");
    auto* produce = new QPushButton(QStringLiteral("连续生产"), this);
    produce->setObjectName("line.produce");
    auto* note = new QLabel(this);
    note->setObjectName("line.measureNote");
    note->setWordWrap(true);
    auto* productionNote = new QLabel(this);
    productionNote->setObjectName("line.productionNote");
    productionNote->setWordWrap(true);
    QObject::connect(measure, &QPushButton::clicked, this, [this, note] {
        const MeasureResult result = measureOperationTimes(scene_, false);
        note->setText(QString::fromStdString(result.note));
        scene_.notify(ChangeKind::Structure);
        reload();
    });
    QObject::connect(produce, &QPushButton::clicked, this, [this, productionNote] {
        LineProduction production;
        std::string error;
        if (!production.start(scene_, error)) {
            productionNote->setText(QString::fromStdString(error));
            return;
        }
        production.step(3600.0);
        const ProductionStats stats = production.stats();
        QString text = QString::fromStdString(production.note(zh_));
        for (const StationAccount& station : stats.stations) {
            const double util = stats.time > 1e-9 ? 100.0 * station.busySeconds / stats.time : 0;
            text += QStringLiteral(" · %1 利用率 %2% 阻塞 %3 s 饥饿 %4 s")
                        .arg(QString::fromStdString(station.id))
                        .arg(util, 0, 'f', 0)
                        .arg(station.blockedSeconds, 0, 'f', 0)
                        .arg(station.starvedSeconds, 0, 'f', 0);
        }
        productionNote->setText(text);
        reload();
    });
    QObject::connect(csv, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("CSV"), {}, QStringLiteral("CSV (*.csv)"));
        if (path.isEmpty()) {
            return;
        }
        std::string error;
        exportScheduleCsv(scheduleSequence(scene_.operations, scene_.sequence, scene_.layout), path.toStdString(), error);
    });
    QObject::connect(xlsx, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("XLSX"), {}, QStringLiteral("XLSX (*.xlsx)"));
        if (path.isEmpty()) {
            return;
        }
        std::string error;
        exportScheduleXlsx(scheduleSequence(scene_.operations, scene_.sequence, scene_.layout), path.toStdString(), error);
    });
    row->addWidget(measure);
    row->addWidget(produce);
    row->addWidget(csv);
    row->addWidget(xlsx);
    row->addWidget(note, 1);
    layout->addLayout(row);
    layout->addWidget(productionNote);
    canvas_ = new GanttCanvas(scene_, this);
    layout->addWidget(canvas_, 1);
}

void GanttView::setDark(bool dark) {
    if (auto* canvas = static_cast<GanttCanvas*>(canvas_)) {
        canvas->setDark(dark);
    }
}

void GanttView::setZh(bool zh) {
    zh_ = zh;
    if (auto* canvas = static_cast<GanttCanvas*>(canvas_)) {
        canvas->setZh(zh);
    }
    if (auto* button = findChild<QPushButton*>("line.measure")) {
        button->setText(zh ? QStringLiteral("测算节拍") : QStringLiteral("Measure cycle"));
    }
    if (auto* button = findChild<QPushButton*>("line.produce")) {
        button->setText(zh ? QStringLiteral("连续生产") : QStringLiteral("Run production"));
    }
}

void GanttView::reload() {
    if (canvas_ != nullptr) {
        canvas_->update();
    }
}

namespace {

const Operation* taskAt(const Scene& scene, int ordinal) {
    int seen = 0;
    for (const Operation& operation : scene.operations) {
        if (operation.kind != OpKind::Task) {
            continue;
        }
        if (seen == ordinal) {
            return &operation;
        }
        ++seen;
    }
    return nullptr;
}

SequenceEvent& addEvent(Scene& scene, const std::string& kind) {
    SequenceEvent event;
    event.id = "ev" + std::to_string(scene.sequence.size() + 1);
    event.kind = kind;
    const Operation* first = taskAt(scene, 0);
    const Operation* second = taskAt(scene, 1);
    if (first != nullptr) {
        event.operation = first->id;
        event.name = first->name;
    }
    if (kind == "transition" && second != nullptr) {
        event.target = second->id;
        event.delay = 0.2;
        event.name = (first != nullptr ? first->name : event.operation) + " → " + second->name;
    } else if (kind == "signal") {
        event.signal = "weld_done";
        event.action = "set";
        event.edge = "finish";
        event.name = event.signal;
    }
    scene.sequence.push_back(std::move(event));
    return scene.sequence.back();
}

}  // namespace

SequenceEditor::SequenceEditor(Scene& scene, QWidget* parent) : QWidget(parent), scene_(scene) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    auto* row = new QHBoxLayout;
    auto add = [this, row](const char* name, const QString& text, auto handler) {
        auto* button = new QPushButton(text, this);
        button->setObjectName(name);
        QObject::connect(button, &QPushButton::clicked, this, handler);
        row->addWidget(button);
    };
    add("sequence.transition", QStringLiteral("过渡"), [this] {
        addEvent(scene_, "transition");
        scene_.notify(ChangeKind::Structure);
    });
    add("sequence.signalSet", QStringLiteral("信号置位"), [this] {
        SequenceEvent& event = addEvent(scene_, "signal");
        event.action = "set";
        scene_.notify(ChangeKind::Structure);
    });
    add("sequence.signalWait", QStringLiteral("事件触发"), [this] {
        SequenceEvent& event = addEvent(scene_, "signal");
        event.action = "wait";
        event.edge = "start";
        const Operation* second = taskAt(scene_, 1);
        if (second != nullptr) {
            event.operation = second->id;
            event.name = second->name + " 等待 " + event.signal;
        }
        scene_.notify(ChangeKind::Structure);
    });
    row->addStretch(1);
    layout->addLayout(row);
    summary_ = new QLabel(this);
    summary_->setObjectName("sequence.summary");
    summary_->setWordWrap(true);
    layout->addWidget(summary_);
    canvas_ = new SequenceCanvas(scene_, this);
    layout->addWidget(canvas_, 1);
    reload();
}

void SequenceEditor::setDark(bool dark) {
    if (auto* canvas = static_cast<SequenceCanvas*>(canvas_)) {
        canvas->setDark(dark);
    }
}

void SequenceEditor::setZh(bool zh) {
    zh_ = zh;
    if (auto* canvas = static_cast<SequenceCanvas*>(canvas_)) {
        canvas->setZh(zh);
    }
    if (auto* button = findChild<QPushButton*>("sequence.transition")) {
        button->setText(zh ? QStringLiteral("过渡") : QStringLiteral("Transition"));
    }
    if (auto* button = findChild<QPushButton*>("sequence.signalSet")) {
        button->setText(zh ? QStringLiteral("信号置位") : QStringLiteral("Set signal"));
    }
    if (auto* button = findChild<QPushButton*>("sequence.signalWait")) {
        button->setText(zh ? QStringLiteral("事件触发") : QStringLiteral("Wait for event"));
    }
    reload();
}

void SequenceEditor::reload() {
    if (canvas_ != nullptr) {
        canvas_->update();
    }
    if (summary_ == nullptr) {
        return;
    }
    const ScheduleReport report = scheduleSequence(scene_.operations, scene_.sequence, scene_.layout);
    QString text = (zh_ ? QStringLiteral("操作 %1 · 事件 %2 · 节拍 %3 s") : QStringLiteral("Operations %1 · events %2 · cycle %3 s"))
                       .arg(static_cast<int>(scene_.operations.size()))
                       .arg(static_cast<int>(scene_.sequence.size()))
                       .arg(report.makespan, 0, 'f', 1);
    if (!report.error.empty()) {
        text += QStringLiteral(" · ") + QString::fromStdString(report.error);
    }
    for (const SequenceEvent& event : scene_.sequence) {
        text += QStringLiteral(" · ");
        if (event.kind == "transition") {
            text += (zh_ ? QStringLiteral("过渡 ") : QStringLiteral("transition ")) + QString::fromStdString(event.operation) +
                    QStringLiteral("→") + QString::fromStdString(event.target) + QStringLiteral(" ") +
                    QString::number(event.delay, 'f', 1) + QStringLiteral(" s");
        } else {
            text += QString::fromStdString(event.signal) + QStringLiteral(" ") +
                    QString::fromStdString(event.action == "wait" ? (zh_ ? "等待" : "wait") : (zh_ ? "置位" : "set"));
        }
    }
    summary_->setText(text);
}

}  // namespace sim
