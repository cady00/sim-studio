#include "sim/app/MainWindow.hpp"

#include "sim/core/Version.hpp"
#include "sim/io/ObjImporter.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/io/StepImporter.hpp"
#include "sim/render/ViewportWidget.hpp"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QDebug>
#include <QDockWidget>
#include <QEventLoop>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMetaObject>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>
#include <functional>

namespace sim {
namespace {

QDoubleSpinBox* makeSpin(double minValue, double maxValue, int decimals) {
    auto* spin = new QDoubleSpinBox;
    spin->setRange(minValue, maxValue);
    spin->setDecimals(decimals);
    spin->setSingleStep(decimals >= 3 ? 1.0 : 5.0);
    spin->setKeyboardTracking(false);
    return spin;
}

}  // namespace

MainWindow::MainWindow(QString locale) {
    physics_ = createPhysicsEngine();
    scripting_ = createScriptHost();
    QSettings settings;
    if (locale.isEmpty()) {
        locale = settings.value("locale", QString::fromLatin1(I18nCatalog::kDefaultLocale)).toString();
    }
    auto loadCatalog = [this](const QString& name) {
        QFile file(":/i18n/" + name + ".json");
        if (!file.open(QIODevice::ReadOnly)) {
            qWarning("Sim Studio catalog %s was not found in resources", qPrintable(name));
            return;
        }
        std::string error;
        if (!i18n_.loadJson(name.toStdString(), file.readAll().toStdString(), error)) {
            qWarning("Sim Studio catalog %s failed: %s", qPrintable(name), error.c_str());
        }
    };
    loadCatalog("en");
    loadCatalog("zh_CN");
    i18n_.setLocale(locale.toStdString());
    buildUi();
    bindScene();
    retranslate();
    resize(1400, 900);
    refreshStatus();
}

MainWindow::~MainWindow() {
    alive_->store(false);
    cancelImport_.store(true);
    if (importThread_.joinable()) {
        importThread_.join();
    }
}

void MainWindow::buildUi() {
    viewport_ = new ViewportWidget(this);
    setCentralWidget(viewport_);
    viewport_->setScene(&scene_);

    tree_ = new QTreeWidget;
    tree_->setColumnCount(2);
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->setMinimumWidth(260);
    treeDock_ = new QDockWidget(this);
    treeDock_->setWidget(tree_);
    treeDock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock_);

    auto* formHost = new QWidget;
    auto* form = new QFormLayout(formHost);
    form->setContentsMargins(12, 12, 12, 12);
    nameEdit_ = new QLineEdit;
    kindLabel_ = new QLabel;
    visibleBox_ = new QCheckBox;
    colorSwatch_ = new QLabel;
    colorSwatch_->setFixedSize(48, 18);
    for (QDoubleSpinBox*& spin : translation_) {
        spin = makeSpin(-1.0e7, 1.0e7, 3);
    }
    for (QDoubleSpinBox*& spin : rotation_) {
        spin = makeSpin(-36000.0, 36000.0, 2);
    }
    vertexLabel_ = new QLabel("-");
    triangleLabel_ = new QLabel("-");
    emptyLabel_ = new QLabel;
    emptyLabel_->setWordWrap(true);
    form->addRow(emptyLabel_);
    form->addRow("name", nameEdit_);
    form->addRow("kind", kindLabel_);
    form->addRow("visible", visibleBox_);
    form->addRow("color", colorSwatch_);
    form->addRow("tx", translation_[0]);
    form->addRow("ty", translation_[1]);
    form->addRow("tz", translation_[2]);
    form->addRow("rx", rotation_[0]);
    form->addRow("ry", rotation_[1]);
    form->addRow("rz", rotation_[2]);
    form->addRow("verts", vertexLabel_);
    form->addRow("tris", triangleLabel_);
    propertyDock_ = new QDockWidget(this);
    propertyDock_->setWidget(formHost);
    propertyDock_->setMinimumWidth(280);
    addDockWidget(Qt::RightDockWidgetArea, propertyDock_);

    newAction_ = new QAction(this);
    openAction_ = new QAction(this);
    saveAction_ = new QAction(this);
    saveAsAction_ = new QAction(this);
    importStepAction_ = new QAction(this);
    importObjAction_ = new QAction(this);
    quitAction_ = new QAction(this);
    fitAction_ = new QAction(this);
    moveAction_ = new QAction(this);
    rotateAction_ = new QAction(this);
    zhAction_ = new QAction(this);
    enAction_ = new QAction(this);
    physicsAction_ = new QAction(this);
    scriptAction_ = new QAction(this);
    aboutAction_ = new QAction(this);
    moveAction_->setCheckable(true);
    rotateAction_->setCheckable(true);
    moveAction_->setChecked(true);
    zhAction_->setCheckable(true);
    enAction_->setCheckable(true);
    auto* gizmoGroup = new QActionGroup(this);
    gizmoGroup->addAction(moveAction_);
    gizmoGroup->addAction(rotateAction_);
    auto* langGroup = new QActionGroup(this);
    langGroup->addAction(zhAction_);
    langGroup->addAction(enAction_);

    newAction_->setShortcut(QKeySequence::New);
    openAction_->setShortcut(QKeySequence::Open);
    saveAction_->setShortcut(QKeySequence::Save);
    saveAsAction_->setShortcut(QKeySequence::SaveAs);
    importStepAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_I));
    quitAction_->setShortcut(QKeySequence::Quit);
    fitAction_->setShortcut(QKeySequence(Qt::Key_F));

    auto* fileMenu = menuBar()->addMenu("file");
    fileMenu->setObjectName("menu.file");
    fileMenu->addAction(newAction_);
    fileMenu->addAction(openAction_);
    fileMenu->addAction(saveAction_);
    fileMenu->addAction(saveAsAction_);
    fileMenu->addSeparator();
    fileMenu->addAction(importStepAction_);
    fileMenu->addAction(importObjAction_);
    fileMenu->addSeparator();
    fileMenu->addAction(quitAction_);
    auto* editMenu = menuBar()->addMenu("edit");
    editMenu->setObjectName("menu.edit");
    editMenu->addAction(fitAction_);
    auto* viewMenu = menuBar()->addMenu("view");
    viewMenu->setObjectName("menu.view");
    const StandardView views[7] = {StandardView::Front, StandardView::Back, StandardView::Left, StandardView::Right,
                                   StandardView::Top, StandardView::Bottom, StandardView::Iso};
    for (int i = 0; i < 7; ++i) {
        viewActions_[i] = new QAction(this);
        viewMenu->addAction(viewActions_[i]);
        connect(viewActions_[i], &QAction::triggered, this, [this, views, i] { viewport_->setStandardView(views[i]); });
    }
    viewMenu->addSeparator();
    viewMenu->addAction(moveAction_);
    viewMenu->addAction(rotateAction_);
    viewMenu->addSeparator();
    auto* langMenu = viewMenu->addMenu("language");
    langMenu->setObjectName("menu.language");
    langMenu->addAction(zhAction_);
    langMenu->addAction(enAction_);
    auto* toolsMenu = menuBar()->addMenu("tools");
    toolsMenu->setObjectName("menu.tools");
    toolsMenu->addAction(physicsAction_);
    toolsMenu->addAction(scriptAction_);
    auto* helpMenu = menuBar()->addMenu("help");
    helpMenu->setObjectName("menu.help");
    helpMenu->addAction(aboutAction_);

    auto* toolbar = addToolBar("main");
    toolbar->setObjectName("toolbar.main");
    toolbar->setMovable(false);
    toolbar->addAction(newAction_);
    toolbar->addAction(openAction_);
    toolbar->addAction(saveAction_);
    toolbar->addAction(importStepAction_);
    toolbar->addSeparator();
    toolbar->addAction(fitAction_);
    toolbar->addAction(viewActions_[0]);
    toolbar->addAction(viewActions_[4]);
    toolbar->addAction(viewActions_[6]);
    toolbar->addSeparator();
    toolbar->addAction(moveAction_);
    toolbar->addAction(rotateAction_);

    statusLabel_ = new QLabel;
    unitLabel_ = new QLabel;
    nodeLabel_ = new QLabel;
    progress_ = new QProgressBar;
    progress_->setFixedWidth(180);
    progress_->setVisible(false);
    cancelButton_ = new QPushButton;
    cancelButton_->setVisible(false);
    statusBar()->addWidget(statusLabel_, 1);
    statusBar()->addPermanentWidget(progress_);
    statusBar()->addPermanentWidget(cancelButton_);
    statusBar()->addPermanentWidget(nodeLabel_);
    statusBar()->addPermanentWidget(unitLabel_);

    connect(newAction_, &QAction::triggered, this, &MainWindow::newProject);
    connect(openAction_, &QAction::triggered, this, &MainWindow::openProjectInteractive);
    connect(saveAction_, &QAction::triggered, this, &MainWindow::saveProjectInteractive);
    connect(saveAsAction_, &QAction::triggered, this, &MainWindow::saveProjectAsInteractive);
    connect(importStepAction_, &QAction::triggered, this, &MainWindow::importStepInteractive);
    connect(importObjAction_, &QAction::triggered, this, &MainWindow::importObjInteractive);
    connect(quitAction_, &QAction::triggered, this, &QWidget::close);
    connect(fitAction_, &QAction::triggered, viewport_, &ViewportWidget::fitAll);
    connect(moveAction_, &QAction::triggered, this, [this] { viewport_->setGizmoMode(GizmoMode::Translate); });
    connect(rotateAction_, &QAction::triggered, this, [this] { viewport_->setGizmoMode(GizmoMode::Rotate); });
    connect(zhAction_, &QAction::triggered, this, [this] { setLocale("zh_CN"); });
    connect(enAction_, &QAction::triggered, this, [this] { setLocale("en"); });
    connect(physicsAction_, &QAction::triggered, this, &MainWindow::showPhysics);
    connect(scriptAction_, &QAction::triggered, this, &MainWindow::showScripting);
    connect(aboutAction_, &QAction::triggered, this, &MainWindow::showAbout);
    connect(cancelButton_, &QPushButton::clicked, this, &MainWindow::cancelImport);
    connect(tree_, &QTreeWidget::currentItemChanged, this, &MainWindow::onTreeCurrentChanged);
    connect(viewport_, &ViewportWidget::selectionChanged, this, &MainWindow::onViewportSelection);
    connect(viewport_, &ViewportWidget::transformEdited, this, &MainWindow::onTransformEdited);
    connect(viewport_, &ViewportWidget::glError, this, [this](const QString& message) { statusLabel_->setText(message); });
    connect(nameEdit_, &QLineEdit::editingFinished, this, &MainWindow::applyProperties);
    connect(visibleBox_, &QCheckBox::toggled, this, &MainWindow::applyProperties);
    for (QDoubleSpinBox* spin : translation_) {
        connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::applyProperties);
    }
    for (QDoubleSpinBox* spin : rotation_) {
        connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::applyProperties);
    }
}

void MainWindow::bindScene() {
    scene_.onChanged = [this](ChangeKind kind) {
        if (kind == ChangeKind::Structure) {
            rebuildTree();
            refreshProperties();
        } else if (kind == ChangeKind::Selection) {
            refreshProperties();
            if (tree_->currentItem() == nullptr ||
                tree_->currentItem()->data(0, Qt::UserRole).toString().toStdString() != scene_.selectedId()) {
                selectNode(QString::fromStdString(scene_.selectedId()), false);
            }
        } else {
            refreshProperties();
        }
        viewport_->update();
        refreshStatus();
    };
    rebuildTree();
    refreshProperties();
}

QString MainWindow::trk(const char* key) const { return QString::fromUtf8(i18n_.translate(key).c_str()); }

void MainWindow::retranslate() {
    setWindowTitle(trk("app.windowTitle"));
    auto setMenu = [this](const char* objectName, const char* key) {
        if (auto* menu = findChild<QMenu*>(objectName)) {
            menu->setTitle(trk(key));
        }
    };
    setMenu("menu.file", "menu.file");
    setMenu("menu.edit", "menu.edit");
    setMenu("menu.view", "menu.view");
    setMenu("menu.language", "menu.language");
    setMenu("menu.tools", "menu.tools");
    setMenu("menu.help", "menu.help");
    newAction_->setText(trk("action.new"));
    openAction_->setText(trk("action.open"));
    saveAction_->setText(trk("action.save"));
    saveAsAction_->setText(trk("action.saveAs"));
    importStepAction_->setText(trk("action.importStep"));
    importObjAction_->setText(trk("action.importObj"));
    quitAction_->setText(trk("action.quit"));
    fitAction_->setText(trk("action.fit"));
    const char* viewKeys[7] = {"action.view.front", "action.view.back", "action.view.left", "action.view.right",
                               "action.view.top", "action.view.bottom", "action.view.iso"};
    for (int i = 0; i < 7; ++i) {
        viewActions_[i]->setText(trk(viewKeys[i]));
    }
    moveAction_->setText(trk("action.gizmo.move"));
    rotateAction_->setText(trk("action.gizmo.rotate"));
    zhAction_->setText(trk("action.lang.zh"));
    enAction_->setText(trk("action.lang.en"));
    zhAction_->setChecked(i18n_.locale() == "zh_CN");
    enAction_->setChecked(i18n_.locale() == "en");
    physicsAction_->setText(trk("action.physics"));
    scriptAction_->setText(trk("action.script"));
    aboutAction_->setText(trk("action.about"));
    treeDock_->setWindowTitle(trk("dock.tree"));
    propertyDock_->setWindowTitle(trk("dock.properties"));
    tree_->setHeaderLabels({trk("tree.name"), trk("tree.kind")});
    cancelButton_->setText(trk("action.cancelImport"));
    if (auto* form = qobject_cast<QFormLayout*>(propertyDock_->widget()->layout())) {
        const char* labels[] = {"prop.empty", "prop.name", "prop.kind", "prop.visible", "prop.color", "prop.tx",
                                "prop.ty", "prop.tz", "prop.rx", "prop.ry", "prop.rz", "prop.vertices", "prop.triangles"};
        for (int i = 0; i < form->rowCount() && i < 13; ++i) {
            auto* item = form->itemAt(i, QFormLayout::FieldRole);
            if (item == nullptr || item->widget() == nullptr) {
                continue;
            }
            if (auto* label = qobject_cast<QLabel*>(form->labelForField(item->widget()))) {
                label->setText(trk(labels[i]));
            }
        }
    }
    unitLabel_->setText(trk("status.units"));
    if (!importing_) {
        statusLabel_->setText(trk("status.mouse"));
    }
    rebuildTree();
    refreshProperties();
    refreshStatus();
}

void MainWindow::setLocale(const QString& locale) {
    i18n_.setLocale(locale.toStdString());
    QSettings settings;
    settings.setValue("locale", locale);
    retranslate();
}

void MainWindow::rebuildTree() {
    const QString selected = QString::fromStdString(scene_.selectedId());
    tree_->blockSignals(true);
    tree_->clear();
    std::function<QTreeWidgetItem*(const std::shared_ptr<SceneNode>&, QTreeWidgetItem*)> add =
        [&](const std::shared_ptr<SceneNode>& node, QTreeWidgetItem* parent) -> QTreeWidgetItem* {
        if (!node) {
            return nullptr;
        }
        auto* item = parent == nullptr ? new QTreeWidgetItem(tree_) : new QTreeWidgetItem(parent);
        const QString name = (node->id == "root" && node->name == "World") ? trk("tree.world")
                                                                           : QString::fromUtf8(node->name.c_str());
        item->setText(0, name);
        item->setText(1, node->kind == NodeKind::Assembly ? trk("kind.assembly") : trk("kind.part"));
        item->setData(0, Qt::UserRole, QString::fromStdString(node->id));
        item->setExpanded(true);
        for (const auto& child : node->children) {
            add(child, item);
        }
        return item;
    };
    add(scene_.root(), nullptr);
    tree_->blockSignals(false);
    if (!selected.isEmpty()) {
        selectNode(selected, false);
    }
}

void MainWindow::selectNode(const QString& id, bool fromTree) {
    if (!fromTree) {
        std::function<QTreeWidgetItem*(QTreeWidgetItem*)> find = [&](QTreeWidgetItem* item) -> QTreeWidgetItem* {
            if (item == nullptr) {
                return nullptr;
            }
            if (item->data(0, Qt::UserRole).toString() == id) {
                return item;
            }
            for (int i = 0; i < item->childCount(); ++i) {
                if (auto* found = find(item->child(i))) {
                    return found;
                }
            }
            return nullptr;
        };
        tree_->blockSignals(true);
        QTreeWidgetItem* match = nullptr;
        for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
            match = find(tree_->topLevelItem(i));
            if (match != nullptr) {
                break;
            }
        }
        tree_->setCurrentItem(match);
        tree_->blockSignals(false);
    }
    if (scene_.selectedId() != id.toStdString()) {
        scene_.setSelected(id.toStdString());
    }
    refreshProperties();
    refreshStatus();
    viewport_->update();
}

void MainWindow::onTreeCurrentChanged(QTreeWidgetItem* current, QTreeWidgetItem*) {
    if (current == nullptr) {
        scene_.setSelected({});
        return;
    }
    selectNode(current->data(0, Qt::UserRole).toString(), true);
}

void MainWindow::onViewportSelection(const QString& id) { selectNode(id, false); }

void MainWindow::onTransformEdited() {
    refreshProperties();
    refreshStatus();
}

void MainWindow::refreshProperties() {
    refreshing_ = true;
    const auto node = scene_.find(scene_.selectedId());
    const bool hasNode = static_cast<bool>(node);
    emptyLabel_->setVisible(!hasNode);
    nameEdit_->setEnabled(hasNode);
    visibleBox_->setEnabled(hasNode);
    for (QDoubleSpinBox* spin : translation_) {
        spin->setEnabled(hasNode);
    }
    for (QDoubleSpinBox* spin : rotation_) {
        spin->setEnabled(hasNode);
    }
    if (!hasNode) {
        nameEdit_->clear();
        kindLabel_->clear();
        vertexLabel_->setText("-");
        triangleLabel_->setText("-");
        refreshing_ = false;
        return;
    }
    nameEdit_->setText(node->id == "root" && node->name == "World" ? trk("tree.world")
                                                                  : QString::fromUtf8(node->name.c_str()));
    kindLabel_->setText(node->kind == NodeKind::Assembly ? trk("kind.assembly") : trk("kind.part"));
    visibleBox_->setChecked(node->visible);
    const int red = std::clamp(static_cast<int>(node->color.x * 255.0), 0, 255);
    const int green = std::clamp(static_cast<int>(node->color.y * 255.0), 0, 255);
    const int blue = std::clamp(static_cast<int>(node->color.z * 255.0), 0, 255);
    colorSwatch_->setStyleSheet(QString("background:#%1%2%3; border:1px solid #555;")
                                    .arg(red, 2, 16, QChar('0'))
                                    .arg(green, 2, 16, QChar('0'))
                                    .arg(blue, 2, 16, QChar('0')));
    translation_[0]->setValue(node->local.translation.x);
    translation_[1]->setValue(node->local.translation.y);
    translation_[2]->setValue(node->local.translation.z);
    double rx = 0, ry = 0, rz = 0;
    node->local.rotation.toEulerXYZ(rx, ry, rz);
    rotation_[0]->setValue(rad2deg(rx));
    rotation_[1]->setValue(rad2deg(ry));
    rotation_[2]->setValue(rad2deg(rz));
    const auto mesh = scene_.mesh(node->meshId);
    vertexLabel_->setText(mesh ? QString::number(mesh->vertexCount()) : QString("-"));
    triangleLabel_->setText(mesh ? QString::number(mesh->triangleCount()) : QString("-"));
    refreshing_ = false;
}

void MainWindow::applyProperties() {
    if (refreshing_) {
        return;
    }
    auto node = scene_.find(scene_.selectedId());
    if (!node) {
        return;
    }
    QString editedName = nameEdit_->text().trimmed();
    if (!(node->id == "root" && node->name == "World" && editedName == trk("tree.world"))) {
        node->name = editedName.toUtf8().toStdString();
    }
    node->visible = visibleBox_->isChecked();
    node->local.translation = {translation_[0]->value(), translation_[1]->value(), translation_[2]->value()};
    node->local.rotation = Quat::fromEulerXYZ(deg2rad(rotation_[0]->value()), deg2rad(rotation_[1]->value()),
                                              deg2rad(rotation_[2]->value()));
    scene_.notify(ChangeKind::Transform);
    if (auto* item = tree_->currentItem()) {
        item->setText(0, node->id == "root" && node->name == "World" ? trk("tree.world")
                                                                    : QString::fromUtf8(node->name.c_str()));
    }
    viewport_->update();
}

void MainWindow::refreshStatus() {
    nodeLabel_->setText(trk("status.nodes").arg(scene_.nodeCount()));
    if (importing_) {
        return;
    }
    if (!scene_.selectedId().empty()) {
        if (const auto node = scene_.find(scene_.selectedId())) {
            const QString name = node->id == "root" && node->name == "World" ? trk("tree.world")
                                                                            : QString::fromUtf8(node->name.c_str());
            statusLabel_->setText(trk("status.selected").arg(name));
            return;
        }
    }
    statusLabel_->setText(trk("status.mouse"));
}

bool MainWindow::confirmDiscard() {
    if (!scene_.dirty()) {
        return true;
    }
    const auto answer = QMessageBox::question(this, trk("dialog.unsaved.title"), trk("dialog.unsaved.body"),
                                              QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
                                              QMessageBox::Save);
    if (answer == QMessageBox::Cancel) {
        return false;
    }
    if (answer == QMessageBox::Save) {
        saveProjectInteractive();
        return !scene_.dirty();
    }
    return true;
}

void MainWindow::newProject() {
    if (!confirmDiscard()) {
        return;
    }
    scene_.clear();
    projectPath_.clear();
    viewport_->fitAll();
}

void MainWindow::openProjectInteractive() {
    if (!confirmDiscard()) {
        return;
    }
    const QString path = QFileDialog::getOpenFileName(this, trk("action.open"), QString(), trk("file.filter.project"));
    if (!path.isEmpty()) {
        openProject(path);
    }
}

void MainWindow::openProject(const QString& path) {
    const ProjectResult result = ProjectStore::load(path.toStdString(), scene_);
    if (!result.ok) {
        QMessageBox::warning(this, trk("action.open"), trk("msg.openFail").arg(QString::fromStdString(result.message)));
        return;
    }
    projectPath_ = path;
    scene_.markClean();
    viewport_->fitAll();
    refreshStatus();
}

bool MainWindow::saveProject(const QString& path) {
    const ProjectResult result = ProjectStore::save(scene_, path.toStdString());
    if (!result.ok) {
        QMessageBox::warning(this, trk("action.save"), trk("msg.saveFail").arg(QString::fromStdString(result.message)));
        return false;
    }
    projectPath_ = path;
    scene_.markClean();
    statusLabel_->setText(trk("msg.saveOk").arg(path));
    return true;
}

void MainWindow::saveProjectInteractive() {
    if (projectPath_.isEmpty()) {
        saveProjectAsInteractive();
        return;
    }
    saveProject(projectPath_);
}

void MainWindow::saveProjectAsInteractive() {
    QString path = QFileDialog::getSaveFileName(this, trk("action.saveAs"), projectPath_, trk("file.filter.project"));
    if (path.isEmpty()) {
        return;
    }
    if (!path.endsWith(".simproj", Qt::CaseInsensitive)) {
        path += ".simproj";
    }
    saveProject(path);
}

void MainWindow::importStepInteractive() {
    if (importing_) {
        return;
    }
    if (!confirmDiscard()) {
        return;
    }
    const QString path = QFileDialog::getOpenFileName(this, trk("action.importStep"), QString(), trk("file.filter.step"));
    if (!path.isEmpty()) {
        importStepAsync(path);
    }
}

void MainWindow::importStepAsync(const QString& path) {
    if (importing_) {
        return;
    }
    if (importThread_.joinable()) {
        cancelImport_.store(true);
        importThread_.join();
    }
    importing_ = true;
    cancelImport_.store(false);
    importStepAction_->setEnabled(false);
    importObjAction_->setEnabled(false);
    progress_->setVisible(true);
    progress_->setRange(0, 0);
    cancelButton_->setVisible(true);
    statusLabel_->setText(trk("progress.reading"));
    const auto alive = alive_;
    importThread_ = std::thread([this, alive, path] {
        auto imported = std::make_shared<Scene>();
        StepImporter importer;
        const ImportResult result = importer.importFile(
            path.toStdString(), *imported,
            [this, alive](const ImportProgress& progress) {
                const int percent = progress.percent;
                const int current = progress.current;
                const int total = progress.total;
                const QString stage = QString::fromStdString(progress.stageKey);
                QMetaObject::invokeMethod(
                    this,
                    [this, alive, percent, stage, current, total] {
                        if (!alive->load()) {
                            return;
                        }
                        onImportProgress(percent, stage, current, total);
                    },
                    Qt::QueuedConnection);
            },
            &cancelImport_);
        auto scene = result.ok ? imported : std::shared_ptr<Scene>{};
        const bool ok = result.ok;
        const QString error = QString::fromStdString(result.message);
        const int nodes = result.nodeCount;
        const int meshes = result.meshCount;
        const int triangles = result.triangleCount;
        const double elapsed = result.elapsedMs;
        QMetaObject::invokeMethod(
            this,
            [this, alive, scene, ok, error, nodes, meshes, triangles, elapsed] {
                if (!alive->load()) {
                    return;
                }
                if (ok && scene) {
                    scene_.steal(*scene);
                }
                onImportFinished(ok, error, nodes, meshes, triangles, elapsed);
            },
            Qt::QueuedConnection);
    });
}

void MainWindow::onImportProgress(int percent, const QString& stageKey, int current, int total) {
    if (percent < 0) {
        progress_->setRange(0, 0);
    } else {
        progress_->setRange(0, 100);
        progress_->setValue(percent);
    }
    QString text = trk(stageKey.toUtf8().constData());
    if (stageKey == "progress.tessellate") {
        text = trk("progress.tessellate").arg(current).arg(total);
    }
    statusLabel_->setText(text);
}

void MainWindow::onImportFinished(bool ok, const QString& error, int nodes, int meshes, int triangles, double elapsedMs) {
    importing_ = false;
    importStepAction_->setEnabled(true);
    importObjAction_->setEnabled(true);
    progress_->setVisible(false);
    cancelButton_->setVisible(false);
    if (importThread_.joinable()) {
        // The worker has already queued this call, so it is about to exit.
    }
    if (!ok) {
        const QString message = error == "cancelled" ? trk("progress.cancel") : trk("msg.importFail").arg(error);
        statusLabel_->setText(message);
        if (!screenshotPath_.isEmpty()) {
            QTimer::singleShot(0, qApp, [] { QApplication::exit(2); });
        } else if (error != "cancelled") {
            QMessageBox::warning(this, trk("action.importStep"), message);
        }
        return;
    }
    projectPath_.clear();
    viewport_->setStandardView(StandardView::Iso);
    viewport_->fitAll();
    statusLabel_->setText(trk("msg.importOk").arg(nodes).arg(meshes).arg(triangles).arg(QString::number(elapsedMs, 'f', 0)));
    if (!screenshotPath_.isEmpty()) {
        resizeDocks({treeDock_, propertyDock_}, {460, 280}, Qt::Horizontal);
        QTimer::singleShot(1000, this, &MainWindow::captureScreenshot);
    }
}

void MainWindow::cancelImport() { cancelImport_.store(true); }

void MainWindow::importObj(const QString& path) {
    ObjImporter importer;
    const ImportResult result = importer.importFile(path.toStdString(), scene_);
    if (!result.ok) {
        QMessageBox::warning(this, trk("action.importObj"), trk("msg.objFail").arg(QString::fromStdString(result.message)));
        return;
    }
    viewport_->fitAll();
    statusLabel_->setText(trk("msg.importOk").arg(result.nodeCount).arg(result.meshCount).arg(result.triangleCount).arg(0));
}

void MainWindow::importObjInteractive() {
    if (!confirmDiscard()) {
        return;
    }
    const QString path = QFileDialog::getOpenFileName(this, trk("action.importObj"), QString(), trk("file.filter.obj"));
    if (!path.isEmpty()) {
        scene_.clear();
        importObj(path);
    }
}

void MainWindow::showAbout() {
    QMessageBox box(this);
    box.setWindowTitle(trk("dialog.about.title"));
    box.setIconPixmap(QPixmap(":/branding/logo.png").scaled(96, 96, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    box.setText(trk("dialog.about.body").arg(QString::fromLatin1(kVersion)));
    box.exec();
}

void MainWindow::showPhysics() {
    QMessageBox::information(this, trk("dialog.physics.title"),
                             trk("dialog.physics.body").arg(QString::fromLatin1(physics_->backendName())));
}

void MainWindow::showScripting() {
    QMessageBox::information(this, trk("dialog.script.title"),
                             trk("dialog.script.body").arg(QString::fromLatin1(scripting_->backendName())));
}

void MainWindow::setScreenshotPath(const QString& path) { screenshotPath_ = path; }

void MainWindow::captureScreenshot() {
    viewport_->update();
    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    QImage view = viewport_->grabFramebuffer();
    QImage window = grab().toImage();
    if (!view.isNull() && !window.isNull()) {
        const qreal dpr = devicePixelRatioF();
        const QPoint topLeft = viewport_->mapTo(this, QPoint(0, 0));
        QPainter painter(&window);
        painter.drawImage(QRectF(topLeft.x() * dpr, topLeft.y() * dpr, viewport_->width() * dpr, viewport_->height() * dpr),
                          view);
    }
    const QImage& output = window.isNull() ? view : window;
    const bool saved = !output.isNull() && output.save(screenshotPath_);
    QApplication::exit(saved ? 0 : 2);
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (!confirmDiscard()) {
        event->ignore();
        return;
    }
    QMainWindow::closeEvent(event);
}

}  // namespace sim
