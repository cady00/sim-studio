#include "sim/app/CommandPalette.hpp"

#include "sim/ai/CommandRegistry.hpp"
#include "sim/core/Scene.hpp"

#include <QKeyEvent>

#include <algorithm>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QVBoxLayout>

namespace sim {

CommandPalette::CommandPalette(QWidget* parent) : QFrame(parent) {
    setObjectName("commandPalette");
    setFixedSize(640, 420);
    hide();
    auto* layout = new QVBoxLayout(this);
    auto* title = new QLabel("命令");
    edit_ = new QLineEdit;
    edit_->setPlaceholderText("搜索中文、拼音首字母或 English，@对象  #信号");
    list_ = new QListWidget;
    layout->addWidget(title);
    layout->addWidget(edit_);
    layout->addWidget(list_, 1);
    connect(edit_, &QLineEdit::textChanged, this, [this] { refresh(); });
    connect(edit_, &QLineEdit::returnPressed, this, [this] { acceptCurrent(); });
    connect(list_, &QListWidget::itemActivated, this, [this] { acceptCurrent(); });
}

void CommandPalette::setHandler(std::function<void(const std::string&)> handler) { handler_ = std::move(handler); }

void CommandPalette::openFor(Scene* scene) {
    scene_ = scene;
    edit_->clear();
    refresh();
    show();
    raise();
    edit_->setFocus();
}

void CommandPalette::refresh() {
    list_->clear();
    const auto hits = CommandRegistry::search(edit_->text().toStdString(), scene_);
    for (const CommandHit& hit : hits) {
        const QString title = QString::fromStdString(hit.entry.titleZh);
        const QString extra = QString::fromStdString(hit.entry.titleEn);
        const QString shortcut = QString::fromStdString(hit.detail.empty() ? hit.entry.shortcut : hit.detail);
        const QString group = QString::fromStdString(hit.entry.group);
        auto* item = new QListWidgetItem(group + "   " + title + (title == extra ? "" : "   " + extra) + "    " + shortcut);
        item->setData(Qt::UserRole, QString::fromStdString(hit.entry.id));
        list_->addItem(item);
    }
    if (list_->count() > 0) {
        list_->setCurrentRow(0);
    }
}

void CommandPalette::acceptCurrent() {
    auto* item = list_->currentItem();
    if (item == nullptr || !handler_) {
        hide();
        return;
    }
    const std::string id = item->data(Qt::UserRole).toString().toStdString();
    hide();
    handler_(id);
}

void CommandPalette::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        hide();
        return;
    }
    if (event->key() == Qt::Key_Down) {
        list_->setCurrentRow(std::min(list_->currentRow() + 1, list_->count() - 1));
        return;
    }
    if (event->key() == Qt::Key_Up) {
        list_->setCurrentRow(std::max(list_->currentRow() - 1, 0));
        return;
    }
    QFrame::keyPressEvent(event);
}

}  // namespace sim
