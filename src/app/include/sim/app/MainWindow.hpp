#pragma once

#include "sim/core/I18n.hpp"
#include "sim/core/Scene.hpp"
#include "sim/physics/PhysicsEngine.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <QMainWindow>

#include <atomic>
#include <memory>
#include <thread>

class QAction;
class QCloseEvent;
class QDockWidget;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QCheckBox;
class QProgressBar;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

namespace sim {

class ViewportWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QString locale = QString());
    ~MainWindow() override;

    void importStepAsync(const QString& path);
    void importObj(const QString& path);
    void openProject(const QString& path);
    void setScreenshotPath(const QString& path);
    void captureScreenshot();

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void newProject();
    void openProjectInteractive();
    void saveProjectInteractive();
    void saveProjectAsInteractive();
    void importStepInteractive();
    void importObjInteractive();
    void onImportProgress(int percent, const QString& stageKey, int current, int total);
    void onImportFinished(bool ok, const QString& error, int nodes, int meshes, int triangles, double elapsedMs);
    void onTreeCurrentChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void onViewportSelection(const QString& id);
    void onTransformEdited();
    void applyProperties();
    void cancelImport();
    void showAbout();
    void showPhysics();
    void showScripting();
    void setLocale(const QString& locale);

private:
    void buildUi();
    void retranslate();
    void bindScene();
    void rebuildTree();
    void refreshProperties();
    void refreshStatus();
    void selectNode(const QString& id, bool fromTree);
    bool confirmDiscard();
    bool saveProject(const QString& path);
    QString trk(const char* key) const;
    void finishScreenshot();

    Scene scene_;
    I18nCatalog i18n_;
    std::unique_ptr<IPhysicsEngine> physics_;
    std::unique_ptr<IScriptHost> scripting_;
    ViewportWidget* viewport_ = nullptr;
    QTreeWidget* tree_ = nullptr;
    QLineEdit* nameEdit_ = nullptr;
    QLabel* kindLabel_ = nullptr;
    QLabel* colorSwatch_ = nullptr;
    QCheckBox* visibleBox_ = nullptr;
    QDoubleSpinBox* translation_[3] = {};
    QDoubleSpinBox* rotation_[3] = {};
    QLabel* vertexLabel_ = nullptr;
    QLabel* triangleLabel_ = nullptr;
    QLabel* emptyLabel_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QLabel* unitLabel_ = nullptr;
    QLabel* nodeLabel_ = nullptr;
    QProgressBar* progress_ = nullptr;
    QPushButton* cancelButton_ = nullptr;
    QString projectPath_;
    QString screenshotPath_;
    bool refreshing_ = false;
    bool importing_ = false;
    std::thread importThread_;
    std::atomic<bool> cancelImport_{false};
    std::shared_ptr<std::atomic<bool>> alive_ = std::make_shared<std::atomic<bool>>(true);

    QAction* newAction_ = nullptr;
    QAction* openAction_ = nullptr;
    QAction* saveAction_ = nullptr;
    QAction* saveAsAction_ = nullptr;
    QAction* importStepAction_ = nullptr;
    QAction* importObjAction_ = nullptr;
    QAction* quitAction_ = nullptr;
    QAction* fitAction_ = nullptr;
    QAction* viewActions_[7] = {};
    QAction* moveAction_ = nullptr;
    QAction* rotateAction_ = nullptr;
    QAction* zhAction_ = nullptr;
    QAction* enAction_ = nullptr;
    QAction* physicsAction_ = nullptr;
    QAction* scriptAction_ = nullptr;
    QAction* aboutAction_ = nullptr;
    QDockWidget* treeDock_ = nullptr;
    QDockWidget* propertyDock_ = nullptr;
};

}  // namespace sim
