#pragma once

#include <QWidget>

class QButtonGroup;
class QPushButton;

namespace sim {

class ChromeBar : public QWidget {
    Q_OBJECT

public:
    explicit ChromeBar(QWidget* parent = nullptr);
    void setMode(int mode);
    void setAiEnabled(bool enabled);
    void setLocaleZh(bool zh);

signals:
    void modeChanged(int mode);
    void commandPaletteRequested();
    void intentRequested();
    void themeToggled();
    void localeToggled();
    void focusRequested();

private:
    QButtonGroup* modes_ = nullptr;
    QPushButton* aiButton_ = nullptr;
    QPushButton* langButton_ = nullptr;
    bool zh_ = true;
};

}  // namespace sim
