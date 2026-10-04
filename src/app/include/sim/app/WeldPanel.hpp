#pragma once

#include "sim/core/I18n.hpp"
#include "sim/core/Scene.hpp"

#include <QWidget>

class QLabel;
class QListWidget;
class QPlainTextEdit;

namespace sim {

class WeldPanel : public QWidget {
    Q_OBJECT

public:
    WeldPanel(Scene& scene, I18nCatalog& i18n, QWidget* parent = nullptr);
    void retranslate();
    void reload();

private:
    QString trk(const char* key) const;
    void importText();
    void project();
    void align();
    void reach();
    void baseRegion();
    void planTour();
    void envelope();
    void interlock();
    KinematicResource* robot();

    Scene& scene_;
    I18nCatalog& i18n_;
    QPlainTextEdit* source_ = nullptr;
    QListWidget* list_ = nullptr;
    QLabel* summary_ = nullptr;
};

}  // namespace sim
