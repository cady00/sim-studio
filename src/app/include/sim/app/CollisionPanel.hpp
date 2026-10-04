#pragma once

#include "sim/core/I18n.hpp"
#include "sim/core/Scene.hpp"

#include <QWidget>

#include <functional>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;

namespace sim {

class CollisionPanel : public QWidget {
    Q_OBJECT

public:
    CollisionPanel(Scene& scene, I18nCatalog& i18n, QWidget* parent = nullptr);

    void retranslate();
    void reload();
    void runQuery();
    void setViewport(QWidget* viewport) { viewport_ = viewport; }

private:
    void readFields();
    QString trk(const char* key) const;

    Scene& scene_;
    I18nCatalog& i18n_;
    QWidget* viewport_ = nullptr;
    bool loading_ = false;
    QDoubleSpinBox* clearance_ = nullptr;
    QLineEdit* setAName_ = nullptr;
    QLineEdit* setAObjects_ = nullptr;
    QLineEdit* setBName_ = nullptr;
    QLineEdit* setBObjects_ = nullptr;
    QListWidget* results_ = nullptr;
    QLabel* summary_ = nullptr;
};

}  // namespace sim
