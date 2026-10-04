#pragma once

#include <QFrame>

#include <functional>
#include <string>

class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;

namespace sim {

class IntentBar : public QFrame {
    Q_OBJECT

public:
    explicit IntentBar(QWidget* parent = nullptr);
    void openBar();
    void setApplyHandler(std::function<bool(const std::string& intent)> handler);
    void note(const std::string& line);
    bool allowsStructuralEdits() const { return allowR2_; }

private:
    void preview();
    void apply();

    QLineEdit* edit_ = nullptr;
    QLabel* planLabel_ = nullptr;
    QListWidget* log_ = nullptr;
    std::function<bool(const std::string&)> applyHandler_;
    bool allowR2_ = false;
};

}  // namespace sim
