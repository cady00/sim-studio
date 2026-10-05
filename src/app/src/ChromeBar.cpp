#include "sim/app/ChromeBar.hpp"

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSize>

namespace sim {

ChromeBar::ChromeBar(QWidget* parent) : QWidget(parent) {
    setObjectName("chrome");
    setFixedHeight(44);
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(12, 6, 12, 6);
    layout->setSpacing(10);

    auto* mark = new QPushButton;
    mark->setIcon(QIcon(":/branding/logo_256.png"));
    mark->setIconSize(QSize(22, 22));
    mark->setFlat(true);
    mark->setFixedSize(28, 28);
    mark->setToolTip(QStringLiteral("关于"));
    mark->setCursor(Qt::PointingHandCursor);
    connect(mark, &QPushButton::clicked, this, &ChromeBar::aboutRequested);
    layout->addWidget(mark);
    brand_ = new QLabel(QString::fromUtf8("应点仿真"));
    brand_->setObjectName("chrome.brand");
    brand_->setStyleSheet("font-weight: 600;");
    layout->addWidget(brand_);

    layout->addStretch(1);
    modes_ = new QButtonGroup(this);
    modes_->setExclusive(true);
    const char* names[] = {"建模", "运动学", "逻辑", "机器人", "调试", "虚拟调试"};
    for (int i = 0; i < 6; ++i) {
        auto* button = new QPushButton(QString::fromUtf8(names[i]) + "  " + QString::number(i + 1));
        button->setObjectName("modeButton");
        button->setCheckable(true);
        button->setCursor(Qt::PointingHandCursor);
        modes_->addButton(button, i);
        layout->addWidget(button);
    }
    modes_->button(0)->setChecked(true);
    connect(modes_, &QButtonGroup::idClicked, this, &ChromeBar::modeChanged);
    layout->addStretch(1);

    auto* search = new QPushButton("搜索命令    Ctrl+K");
    search->setObjectName("commandHint");
    search->setMinimumWidth(220);
    connect(search, &QPushButton::clicked, this, &ChromeBar::commandPaletteRequested);
    layout->addWidget(search);

    aiButton_ = new QPushButton("AI 已关闭");
    connect(aiButton_, &QPushButton::clicked, this, &ChromeBar::intentRequested);
    layout->addWidget(aiButton_);

    langButton_ = new QPushButton("中 / EN");
    connect(langButton_, &QPushButton::clicked, this, &ChromeBar::localeToggled);
    layout->addWidget(langButton_);

    auto* theme = new QPushButton("主题");
    connect(theme, &QPushButton::clicked, this, &ChromeBar::themeToggled);
    layout->addWidget(theme);

    auto* focus = new QPushButton("专注");
    connect(focus, &QPushButton::clicked, this, &ChromeBar::focusRequested);
    layout->addWidget(focus);
}

void ChromeBar::setMode(int mode) {
    if (modes_ == nullptr) {
        return;
    }
    const QSignalBlocker blocker(modes_);
    if (auto* button = modes_->button(mode)) {
        button->setChecked(true);
    }
}

void ChromeBar::setAiEnabled(bool enabled) {
    aiButton_->setText(enabled ? "AI 本地" : "AI 已关闭");
}

void ChromeBar::setProductName(const QString& name) {
    if (brand_ != nullptr) {
        brand_->setText(name);
    }
}

void ChromeBar::setLocaleZh(bool zh) {
    zh_ = zh;
    langButton_->setText(zh ? "中 / EN" : "EN / 中");
    const char* zhNames[] = {"建模", "运动学", "逻辑", "机器人", "调试", "虚拟调试"};
    const char* enNames[] = {"Model", "Kinematics", "Logic", "Robot", "Debug", "Commission"};
    for (int i = 0; i < 6; ++i) {
        if (auto* button = modes_->button(i)) {
            button->setText(QString::fromUtf8(zh ? zhNames[i] : enNames[i]) + "  " + QString::number(i + 1));
        }
    }
}

}  // namespace sim
