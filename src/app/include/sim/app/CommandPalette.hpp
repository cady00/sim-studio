#pragma once

#include <QFrame>

#include <functional>

class QKeyEvent;
class QLineEdit;
class QListWidget;

namespace sim {

class Scene;

class CommandPalette : public QFrame {
    Q_OBJECT

public:
    explicit CommandPalette(QWidget* parent = nullptr);
    void openFor(Scene* scene);
    void setHandler(std::function<void(const std::string& id)> handler);

protected:
    void keyPressEvent(QKeyEvent* event) override;

private:
    void refresh();
    void acceptCurrent();

    Scene* scene_ = nullptr;
    QLineEdit* edit_ = nullptr;
    QListWidget* list_ = nullptr;
    std::function<void(const std::string&)> handler_;
};

}  // namespace sim
