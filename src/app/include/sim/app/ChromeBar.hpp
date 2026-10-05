#pragma once

#include <QWidget>

class QButtonGroup;
class QLabel;
class QPushButton;

namespace sim {

class ChromeBar : public QWidget {
    Q_OBJECT

public:
    explicit ChromeBar(QWidget* parent = nullptr);
    void setMode(int mode);
    void setAiEnabled(bool enabled);
    void setLocaleZh(bool zh);
    void setProductName(const QString& name);

signals:
    void modeChanged(int mode);
    void commandPaletteRequested();
    void intentRequested();
    void themeToggled();
    void localeToggled();
    void focusRequested();
    void aboutRequested();

private:
    QButtonGroup* modes_ = nullptr;
    QPushButton* aiButton_ = nullptr;
    QPushButton* langButton_ = nullptr;
    QLabel* brand_ = nullptr;
    bool zh_ = true;
};

}  // namespace sim
