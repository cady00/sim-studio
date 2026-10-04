#include "sim/app/IntentBar.hpp"

#include "sim/ai/CommandRegistry.hpp"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

namespace sim {

IntentBar::IntentBar(QWidget* parent) : QFrame(parent) {
    setObjectName("intentBar");
    setFixedSize(760, 360);
    hide();
    auto* layout = new QVBoxLayout(this);
    auto* title = new QLabel("AI 意图  ·  提供者关闭  ·  本地离线");
    edit_ = new QLineEdit;
    edit_->setPlaceholderText("例如：播放，或 写信号 #go。不会联网，也不会编造未注册的命令。");
    planLabel_ = new QLabel("计划会显示在这里。应用前场景不会被修改。");
    planLabel_->setWordWrap(true);
    auto* risk = new QCheckBox("允许 R2 结构修改（默认关）");
    connect(risk, &QCheckBox::toggled, this, [this](bool on) { allowR2_ = on; });
    auto* row = new QHBoxLayout;
    auto* previewButton = new QPushButton("预览计划");
    auto* apply = new QPushButton("应用");
    apply->setObjectName("accentButton");
    auto* cancel = new QPushButton("取消");
    row->addWidget(previewButton);
    row->addWidget(apply);
    row->addWidget(cancel);
    row->addStretch(1);
    log_ = new QListWidget;
    log_->setMaximumHeight(90);
    layout->addWidget(title);
    layout->addWidget(edit_);
    layout->addWidget(planLabel_);
    layout->addWidget(risk);
    layout->addLayout(row);
    layout->addWidget(new QLabel("活动记录"));
    layout->addWidget(log_);
    connect(previewButton, &QPushButton::clicked, this, [this] { preview(); });
    connect(apply, &QPushButton::clicked, this, [this] { this->apply(); });
    connect(cancel, &QPushButton::clicked, this, [this] { hide(); });
    connect(edit_, &QLineEdit::returnPressed, this, [this] { preview(); });
}

void IntentBar::setApplyHandler(std::function<bool(const std::string&)> handler) { applyHandler_ = std::move(handler); }

void IntentBar::openBar() {
    show();
    raise();
    edit_->setFocus();
    preview();
}

void IntentBar::preview() {
    const CommandPlan plan = CommandRegistry::planIntent(edit_->text().toStdString());
    QString text;
    if (plan.needsClarify) {
        text = QString::fromStdString(plan.note);
    } else {
        text = QString::fromStdString(plan.note) + "\n";
        for (const PlanStep& step : plan.steps) {
            text += QString::fromStdString(step.risk + "  " + step.op + "  " + step.title) + "\n";
            if (step.risk == "R2" && !allowR2_) {
                text += "这一步是结构修改，权限面板未打开，应用时会跳过。\n";
            }
            if (step.risk == "R3") {
                text += "R3 不会执行。\n";
            }
        }
        text += "预览模式 · 场景未修改";
    }
    planLabel_->setText(text);
}

void IntentBar::apply() {
    const std::string intent = edit_->text().toStdString();
    const CommandPlan plan = CommandRegistry::planIntent(intent);
    if (plan.needsClarify || !applyHandler_) {
        log_->addItem("未应用：" + QString::fromStdString(plan.note));
        return;
    }
    const bool ok = applyHandler_(intent);
    log_->addItem((ok ? "已应用 " : "失败 ") + QString::fromStdString(intent));
}

void IntentBar::note(const std::string& line) { log_->addItem(QString::fromStdString(line)); }

}  // namespace sim
