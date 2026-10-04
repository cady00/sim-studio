#include "sim/app/MainWindow.hpp"
#include "sim/app/PhysicsShelf.hpp"

#include "sim/ai/CommandRegistry.hpp"
#include "sim/ai/Commands.hpp"
#include "sim/app/ChromeBar.hpp"
#include "sim/app/CollisionPanel.hpp"
#include "sim/app/CommandPalette.hpp"
#include "sim/app/IntentBar.hpp"
#include "sim/app/LinePanels.hpp"
#include "sim/app/SceneTree.hpp"
#include "sim/app/MotionPanel.hpp"
#include "sim/app/TaskPanels.hpp"
#include "sim/app/WeldPanel.hpp"
#include "sim/motion/Envelope.hpp"
#include "sim/motion/Olp.hpp"
#include "sim/motion/WeldPlan.hpp"
#include "sim/core/DemoScene.hpp"
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
#include <QComboBox>
#include <QDockWidget>
#include <QEventLoop>
#include <QPlainTextEdit>
#include <QResizeEvent>
#include <QScrollArea>
#include <QShortcut>
#include <QTableWidget>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QSurfaceFormat>
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
#include <QSystemTrayIcon>
#include <QTimer>
#include <QToolBar>
#include <QItemSelectionModel>
#include <QTreeView>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <cmath>
#include <functional>

// Qt defines `signals` as a macro. The scene stores named I/O signals under that member.
#ifdef signals
#undef signals
#endif

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
    simulation_ = std::make_unique<Simulation>(scene_, *physics_);
    robot_ = std::make_unique<BuiltinRobotSolver>();
    scripting_->bind(&scene_);
    scripting_->setPhysicsHooks([this](const std::string& id) { physics_->adoptSpawned(id); },
                                [this](const std::string& id) { physics_->releaseBody(id); });
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
    resize(1440, 900);
    applyTaskMode(0);
    setThemeName("dark");
    refreshStatus();
}

MainWindow::~MainWindow() {
    if (simulation_) {
        simulation_->stop();
    }
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

    treeModel_ = new SceneTreeModel(scene_, this);
    tree_ = new QTreeView;
    tree_->setModel(treeModel_);
    tree_->setUniformRowHeights(true);
    tree_->setAnimated(false);
    tree_->setHeaderHidden(false);
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->setMinimumWidth(260);
    treeSearch_ = new QLineEdit;
    treeSearch_->setClearButtonEnabled(true);
    treeSearch_->setPlaceholderText(QStringLiteral("搜索名称"));
    auto* treeHost = new QWidget;
    auto* treeLayout = new QVBoxLayout(treeHost);
    treeLayout->setContentsMargins(0, 0, 0, 0);
    treeLayout->addWidget(treeSearch_);
    treeLayout->addWidget(tree_, 1);
    treeDock_ = new QDockWidget(this);
    treeDock_->setWidget(treeHost);
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
    mechCombo_ = new QComboBox;
    massSpin_ = makeSpin(0, 1.0e6, 3);
    autoMassBox_ = new QCheckBox;
    kinematicBox_ = new QCheckBox;
    shapeCombo_ = new QComboBox;
    for (QDoubleSpinBox*& spin : extentSpin_) {
        spin = makeSpin(0.1, 1.0e6, 2);
    }
    radiusSpin_ = makeSpin(0.1, 1.0e6, 2);
    heightSpin_ = makeSpin(0.1, 1.0e6, 2);
    otherEdit_ = new QLineEdit;
    axisCombo_ = new QComboBox;
    limitedBox_ = new QCheckBox;
    lowerSpin_ = makeSpin(-1.0e6, 1.0e6, 2);
    upperSpin_ = makeSpin(-1.0e6, 1.0e6, 2);
    driveCombo_ = new QComboBox;
    targetSpin_ = makeSpin(-1.0e6, 1.0e6, 2);
    maxSpeedSpin_ = makeSpin(0, 1.0e6, 2);
    accelSpin_ = makeSpin(0, 1.0e7, 2);
    beltSpeedSpin_ = makeSpin(-1.0e6, 1.0e6, 2);
    rangeSpin_ = makeSpin(1, 1.0e6, 1);
    signalEdit_ = new QLineEdit;
    valueSignalEdit_ = new QLineEdit;
    intervalSpin_ = makeSpin(0.05, 3600, 2);
    prototypeEdit_ = new QLineEdit;
    form->addRow("mech", mechCombo_);
    form->addRow("mass", massSpin_);
    form->addRow("autoMass", autoMassBox_);
    form->addRow("kinematic", kinematicBox_);
    form->addRow("shape", shapeCombo_);
    form->addRow("hx", extentSpin_[0]);
    form->addRow("hy", extentSpin_[1]);
    form->addRow("hz", extentSpin_[2]);
    form->addRow("radius", radiusSpin_);
    form->addRow("height", heightSpin_);
    form->addRow("other", otherEdit_);
    form->addRow("axis", axisCombo_);
    form->addRow("limited", limitedBox_);
    form->addRow("lower", lowerSpin_);
    form->addRow("upper", upperSpin_);
    form->addRow("drive", driveCombo_);
    form->addRow("target", targetSpin_);
    form->addRow("maxSpeed", maxSpeedSpin_);
    form->addRow("accel", accelSpin_);
    form->addRow("belt", beltSpeedSpin_);
    form->addRow("range", rangeSpin_);
    form->addRow("signal", signalEdit_);
    form->addRow("valueSignal", valueSignalEdit_);
    form->addRow("interval", intervalSpin_);
    form->addRow("prototype", prototypeEdit_);
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setWidget(formHost);
    propertyDock_ = new QDockWidget(this);
    propertyDock_->setWidget(scroll);
    propertyDock_->setMinimumWidth(300);
    addDockWidget(Qt::RightDockWidgetArea, propertyDock_);

    signalTable_ = new QTableWidget(0, 3);
    signalTable_->horizontalHeader()->setStretchLastSection(true);
    signalDock_ = new QDockWidget(this);
    signalDock_->setObjectName("dock.signals");
    signalDock_->setWidget(signalTable_);
    addDockWidget(Qt::BottomDockWidgetArea, signalDock_);

    scriptCombo_ = new QComboBox;
    scriptEdit_ = new QPlainTextEdit;
    scriptEdit_->setPlaceholderText("def on_step(dt):\n    pass");
    scriptLog_ = new QPlainTextEdit;
    scriptLog_->setReadOnly(true);
    scriptLog_->setMaximumHeight(120);
    auto* scriptHost = new QWidget;
    auto* scriptLayout = new QVBoxLayout(scriptHost);
    scriptLayout->setContentsMargins(6, 6, 6, 6);
    auto* scriptRun = new QPushButton;
    scriptRun->setObjectName("script.run");
    scriptRun->setText(trk("script.run"));
    connect(scriptRun, &QPushButton::clicked, this, &MainWindow::runScriptSnippet);
    scriptLayout->addWidget(scriptCombo_);
    scriptLayout->addWidget(scriptEdit_, 1);
    scriptLayout->addWidget(scriptRun);
    scriptLayout->addWidget(scriptLog_);
    scriptDock_ = new QDockWidget(this);
    scriptDock_->setObjectName("dock.script");
    scriptDock_->setWidget(scriptHost);
    addDockWidget(Qt::BottomDockWidgetArea, scriptDock_);
    splitDockWidget(signalDock_, scriptDock_, Qt::Horizontal);

    collision_ = new CollisionPanel(scene_, i18n_, this);
    collision_->setViewport(viewport_);
    collisionDock_ = new QDockWidget(this);
    collisionDock_->setObjectName("dock.collision");
    collisionDock_->setWidget(collision_);
    addDockWidget(Qt::BottomDockWidgetArea, collisionDock_);

    motion_ = new MotionPanel(scene_, i18n_, logic_, *robot_, this);
    motion_->setViewport(viewport_);
    motion_->setPlayRequest([this](const std::string& path) {
        pendingPath_ = path;
        playSimulation();
    });
    connect(motion_, &MotionPanel::pythonRequested, this, [this](const QString& source) {
        if (scene_.scripts.empty()) {
            scene_.scripts.push_back(ProjectScript{"logic", source.toStdString()});
        } else if (scriptIndex_ >= 0 && scriptIndex_ < static_cast<int>(scene_.scripts.size())) {
            scene_.scripts[static_cast<std::size_t>(scriptIndex_)].source = source.toStdString();
        } else {
            scene_.scripts.front().source = source.toStdString();
        }
        loadScriptEditor(scriptIndex_);
    });
    motionDock_ = new QDockWidget(this);
    motionDock_->setObjectName("dock.motion");
    motionDock_->setWidget(motion_);
    addDockWidget(Qt::RightDockWidgetArea, motionDock_);
    splitDockWidget(propertyDock_, motionDock_, Qt::Vertical);

    logicCanvas_ = new LogicCanvas(scene_, this);
    logicDock_ = new QDockWidget(this);
    logicDock_->setObjectName("dock.logic");
    logicDock_->setWidget(logicCanvas_);
    addDockWidget(Qt::LeftDockWidgetArea, logicDock_);

    pathTimeline_ = new PathTimeline(scene_, this);
    pathDock_ = new QDockWidget(this);
    pathDock_->setObjectName("dock.path");
    pathDock_->setWidget(pathTimeline_);
    addDockWidget(Qt::BottomDockWidgetArea, pathDock_);

    weld_ = new WeldPanel(scene_, i18n_, this);
    weldDock_ = new QDockWidget(this);
    weldDock_->setObjectName("dock.weld");
    weldDock_->setWidget(weld_);
    addDockWidget(Qt::RightDockWidgetArea, weldDock_);

    plc_ = new PlcCards(scene_, this);
    connect(plc_, &PlcCards::signalsImported, this, [this] { refreshSignals(); });
    plcDock_ = new QDockWidget(this);
    plcDock_->setObjectName("dock.plc");
    plcDock_->setWidget(plc_);
    addDockWidget(Qt::RightDockWidgetArea, plcDock_);

    physicsShelf_ = new PhysicsShelf(scene_, [this](const char* key) { return trk(key); }, this);
    physicsShelf_->setEngineRequested([this](const std::string& engine) {
        usePhysicsBackend(engine == "builtin" ? PhysicsBackend::Builtin : PhysicsBackend::Jolt);
    });
    physicsDock_ = new QDockWidget(this);
    physicsDock_->setObjectName("dock.physics");
    physicsDock_->setWidget(physicsShelf_);
    addDockWidget(Qt::RightDockWidgetArea, physicsDock_);
    splitDockWidget(propertyDock_, physicsDock_, Qt::Vertical);

    floor_ = new FloorPlanView(scene_, this);
    floorDock_ = new QDockWidget(this);
    floorDock_->setObjectName("dock.floor");
    floorDock_->setWidget(floor_);
    addDockWidget(Qt::LeftDockWidgetArea, floorDock_);

    operations_ = new OperationTreeView(scene_, this);
    operationDock_ = new QDockWidget(this);
    operationDock_->setObjectName("dock.operations");
    operationDock_->setWidget(operations_);
    addDockWidget(Qt::RightDockWidgetArea, operationDock_);

    gantt_ = new GanttView(scene_, this);
    ganttDock_ = new QDockWidget(this);
    ganttDock_->setObjectName("dock.gantt");
    ganttDock_->setWidget(gantt_);
    addDockWidget(Qt::BottomDockWidgetArea, ganttDock_);

    tray_ = new QSystemTrayIcon(QIcon(":/branding/tray.ico"), this);
    tray_->setToolTip(QStringLiteral("Sim Studio"));
    if (tray_->icon().isNull()) {
        tray_->setIcon(QIcon(":/branding/logo_256.png"));
    }
    if (QSystemTrayIcon::isSystemTrayAvailable()) {
        tray_->show();
    }

    treeDock_->setObjectName("dock.tree");
    propertyDock_->setObjectName("dock.properties");

    chrome_ = new ChromeBar(this);
    setMenuWidget(chrome_);
    palette_ = new CommandPalette(this);
    intent_ = new IntentBar(this);
    connect(chrome_, &ChromeBar::modeChanged, this, [this](int mode) { applyTaskMode(mode); });
    connect(chrome_, &ChromeBar::commandPaletteRequested, this, [this] { showCommandPalette(); });
    connect(chrome_, &ChromeBar::intentRequested, this, [this] { showIntentBar(); });
    connect(chrome_, &ChromeBar::themeToggled, this, [this] { setThemeName(themeName_ == "dark" ? "light" : "dark"); });
    connect(chrome_, &ChromeBar::localeToggled, this, [this] { setLocale(i18n_.locale() == "zh_CN" ? "en" : "zh_CN"); });
    connect(chrome_, &ChromeBar::aboutRequested, this, &MainWindow::showAbout);
    connect(chrome_, &ChromeBar::focusRequested, this, [this] {
        focusMode_ = !focusMode_;
        applyTaskMode(taskMode_);
    });
    palette_->setHandler([this](const std::string& id) { runUiCommand(id); });
    intent_->setApplyHandler([this](const std::string& text) { return applyIntent(text); });
    connect(intent_, &IntentBar::undoRequested, this, [this] { undoIntent(); });
    connect(intent_, &IntentBar::closed, this, [this] {
        if (viewport_ != nullptr) {
            viewport_->setPreviewMode(false);
            viewport_->setGhost(false);
        }
    });
    connect(viewport_, &ViewportWidget::overlayCommand, this, [this](const QString& id) { runUiCommand(id.toStdString()); });

    auto* paletteShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_K), this);
    paletteShortcut->setContext(Qt::ApplicationShortcut);
    connect(paletteShortcut, &QShortcut::activated, this, [this] { showCommandPalette(); });
    auto* intentShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_I), this);
    intentShortcut->setContext(Qt::ApplicationShortcut);
    connect(intentShortcut, &QShortcut::activated, this, [this] { showIntentBar(); });
    auto* focusShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Backslash), this);
    focusShortcut->setContext(Qt::ApplicationShortcut);
    connect(focusShortcut, &QShortcut::activated, this, [this] {
        focusMode_ = !focusMode_;
        applyTaskMode(taskMode_);
    });
    for (int mode = 0; mode < 6; ++mode) {
        auto* shortcut = new QShortcut(QKeySequence(Qt::CTRL | (Qt::Key_1 + mode)), this);
        shortcut->setContext(Qt::ApplicationShortcut);
        connect(shortcut, &QShortcut::activated, this, [this, mode] { applyTaskMode(mode); });
    }
    auto* measureShortcut = new QShortcut(QKeySequence(Qt::Key_M), viewport_);
    connect(measureShortcut, &QShortcut::activated, this, [this] { runUiCommand("ui.measure"); });
    auto* teachShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_T), this);
    connect(teachShortcut, &QShortcut::activated, this, [this] { runUiCommand("ui.teach"); });
    auto* undoAction = new QAction(this);
    undoAction->setShortcut(QKeySequence::Undo);
    undoAction->setShortcutContext(Qt::WindowShortcut);
    addAction(undoAction);
    connect(undoAction, &QAction::triggered, this, [this] {
        QWidget* focus = QApplication::focusWidget();
        if (auto* line = qobject_cast<QLineEdit*>(focus)) {
            line->undo();
            return;
        }
        if (auto* plain = qobject_cast<QPlainTextEdit*>(focus)) {
            plain->undo();
            return;
        }
        undoIntent();
    });

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
    playAction_ = new QAction(this);
    pauseAction_ = new QAction(this);
    stopAction_ = new QAction(this);
    resetAction_ = new QAction(this);
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
    importStepAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_I));
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
    toolbar->addSeparator();
    toolbar->addAction(playAction_);
    toolbar->addAction(pauseAction_);
    toolbar->addAction(stopAction_);
    toolbar->addAction(resetAction_);
    speedSpin_ = makeSpin(0, 16, 2);
    speedSpin_->setValue(1);
    speedSpin_->setPrefix("x");
    toolbar->addWidget(speedSpin_);
    timeLabel_ = new QLabel("0.00 s");
    toolbar->addWidget(timeLabel_);

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
    offlineLabel_ = new QLabel("本地 · 离线");
    statusBar()->addPermanentWidget(offlineLabel_);

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
    connect(tree_->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onTreeCurrentChanged);
    connect(treeSearch_, &QLineEdit::textChanged, this, [this](const QString& text) { treeModel_->setFilter(text); });
    connect(viewport_, &ViewportWidget::selectionChanged, this, &MainWindow::onViewportSelection);
    connect(viewport_, &ViewportWidget::pointPicked, this,
            [this](double x, double y, double z, double nx, double ny, double nz, const QString& nodeId) {
                if (motion_ != nullptr) {
                    motion_->notePick({x, y, z}, {nx, ny, nz}, nodeId.toStdString());
                }
            });
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
    const auto connectMech = [this](auto* widget) {
        if (auto* spin = qobject_cast<QDoubleSpinBox*>(widget)) {
            connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::applyProperties);
        } else if (auto* box = qobject_cast<QCheckBox*>(widget)) {
            connect(box, &QCheckBox::toggled, this, &MainWindow::applyProperties);
        } else if (auto* combo = qobject_cast<QComboBox*>(widget)) {
            connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::applyProperties);
        } else if (auto* edit = qobject_cast<QLineEdit*>(widget)) {
            connect(edit, &QLineEdit::editingFinished, this, &MainWindow::applyProperties);
        }
    };
    connectMech(mechCombo_);
    connectMech(massSpin_);
    connectMech(autoMassBox_);
    connectMech(kinematicBox_);
    connectMech(shapeCombo_);
    for (QDoubleSpinBox* spin : extentSpin_) {
        connectMech(spin);
    }
    connectMech(radiusSpin_);
    connectMech(heightSpin_);
    connectMech(otherEdit_);
    connectMech(axisCombo_);
    connectMech(limitedBox_);
    connectMech(lowerSpin_);
    connectMech(upperSpin_);
    connectMech(driveCombo_);
    connectMech(targetSpin_);
    connectMech(maxSpeedSpin_);
    connectMech(accelSpin_);
    connectMech(beltSpeedSpin_);
    connectMech(rangeSpin_);
    connectMech(signalEdit_);
    connectMech(valueSignalEdit_);
    connectMech(intervalSpin_);
    connectMech(prototypeEdit_);
    connect(playAction_, &QAction::triggered, this, &MainWindow::playSimulation);
    connect(pauseAction_, &QAction::triggered, this, &MainWindow::pauseSimulation);
    connect(stopAction_, &QAction::triggered, this, &MainWindow::stopSimulation);
    connect(resetAction_, &QAction::triggered, this, &MainWindow::resetSimulation);
    connect(speedSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onSpeedChanged);
    connect(scriptCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onScriptChanged);
    simTimer_ = new QTimer(this);
    simTimer_->setInterval(16);
    connect(simTimer_, &QTimer::timeout, this, &MainWindow::onSimTick);
    simTimer_->start();
}

void MainWindow::bindScene() {
    scene_.onChanged = [this](ChangeKind kind) {
        if (kind == ChangeKind::Simulation) {
            viewport_->update();
            refreshSignals();
            shelfReload_ = false;
            refreshProperties();
            shelfReload_ = true;
            if (physicsShelf_ != nullptr) {
                physicsShelf_->setStats(scene_.physicsDoc.stats);
            }
            if (motion_ != nullptr) {
                motion_->refreshLive();
            }
            if (timeLabel_ != nullptr && simulation_) {
                timeLabel_->setText(trk("status.simTime").arg(simulation_->time(), 0, 'f', 2));
            }
            return;
        }
        if (kind == ChangeKind::Structure) {
            rebuildTree();
            refreshProperties();
            refreshSignals();
            if (scriptCombo_ != nullptr && scriptCombo_->count() != static_cast<int>(scene_.scripts.size())) {
                loadScriptEditor(-1);
            }
            if (motion_ != nullptr) {
                motion_->refresh();
            }
            if (floor_ != nullptr) {
                floor_->reload();
            }
            if (operations_ != nullptr) {
                operations_->reload();
            }
            if (gantt_ != nullptr) {
                gantt_->reload();
            }
        } else if (kind == ChangeKind::Selection) {
            refreshProperties();
            if (motion_ != nullptr) {
                motion_->syncSelection();
            }
            if (treeModel_->nodeId(tree_->currentIndex()) != scene_.selectedId()) {
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
    playAction_->setText(trk("action.play"));
    pauseAction_->setText(trk("action.pause"));
    stopAction_->setText(trk("action.stop"));
    resetAction_->setText(trk("action.reset"));
    if (auto* dock = findChild<QDockWidget*>("dock.signals")) {
        dock->setWindowTitle(trk("dock.signals"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.script")) {
        dock->setWindowTitle(trk("dock.script"));
    }
    if (auto* run = findChild<QPushButton*>("script.run")) {
        run->setText(trk("script.run"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.motion")) {
        dock->setWindowTitle(trk("dock.motion"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.collision")) {
        dock->setWindowTitle(trk("dock.collision"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.logic")) {
        dock->setWindowTitle(trk("dock.logic"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.path")) {
        dock->setWindowTitle(trk("dock.path"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.weld")) {
        dock->setWindowTitle(trk("dock.weld"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.plc")) {
        dock->setWindowTitle(trk("dock.plc"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.physics")) {
        dock->setWindowTitle(trk("dock.physics"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.floor")) {
        dock->setWindowTitle(trk("dock.floor"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.operations")) {
        dock->setWindowTitle(trk("dock.operations"));
    }
    if (auto* dock = findChild<QDockWidget*>("dock.gantt")) {
        dock->setWindowTitle(trk("dock.gantt"));
    }
    const bool zh = i18n_.locale() == "zh_CN";
    if (floor_ != nullptr) {
        floor_->setZh(zh);
    }
    if (operations_ != nullptr) {
        operations_->setZh(zh);
    }
    if (gantt_ != nullptr) {
        gantt_->setZh(zh);
    }
    if (physicsShelf_ != nullptr) {
        physicsShelf_->reload();
    }
    if (collision_ != nullptr) {
        collision_->retranslate();
    }
    if (weld_ != nullptr) {
        weld_->retranslate();
    }
    if (motion_ != nullptr) {
        motion_->retranslate();
    }
    signalTable_->setHorizontalHeaderLabels({trk("signal.name"), trk("signal.type"), trk("signal.value")});
    fillMechCombos();
    treeDock_->setWindowTitle(trk("dock.tree"));
    propertyDock_->setWindowTitle(trk("dock.properties"));
    treeModel_->setHeaders(trk("tree.name"), trk("tree.kind"));
    treeModel_->setWorldTitle(trk("tree.world"));
    cancelButton_->setText(trk("action.cancelImport"));
    if (auto* form = propertyDock_->findChild<QFormLayout*>()) {
        const char* labels[] = {"prop.empty",     "prop.name",       "prop.kind",      "prop.visible",
                                "prop.color",     "prop.tx",         "prop.ty",        "prop.tz",
                                "prop.rx",        "prop.ry",         "prop.rz",        "prop.vertices",
                                "prop.triangles", "mech.kind",       "mech.mass",      "mech.autoMass",
                                "mech.kinematic", "mech.shape",      "mech.hx",        "mech.hy",
                                "mech.hz",        "mech.radius",     "mech.height",    "mech.other",
                                "mech.axis",      "mech.limited",    "mech.lower",     "mech.upper",
                                "mech.drive",     "mech.target",     "mech.maxSpeed",  "mech.accel",
                                "mech.belt",      "mech.range",      "mech.signal",    "mech.valueSignal",
                                "mech.interval",  "mech.prototype"};
        for (int i = 0; i < form->rowCount() && i < 38; ++i) {
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
    if (chrome_ != nullptr) {
        chrome_->setLocaleZh(locale == "zh_CN");
    }
}

void MainWindow::rebuildTree() {
    const QString selected = QString::fromStdString(scene_.selectedId());
    treeModel_->setWorldTitle(trk("tree.world"));
    treeModel_->reload();
    const QModelIndex root = treeModel_->index(0, 0, {});
    if (root.isValid() && treeSearch_->text().trimmed().isEmpty() && treeModel_->rowCount(root) < 4000) {
        tree_->expand(root);
    }
    if (!selected.isEmpty()) {
        selectNode(selected, false);
    }
}

void MainWindow::selectNode(const QString& id, bool fromTree) {
    if (!fromTree) {
        const QModelIndex match = treeModel_->indexFor(id.toStdString());
        tree_->blockSignals(true);
        tree_->setCurrentIndex(match);
        if (match.isValid()) {
            tree_->scrollTo(match);
        }
        tree_->blockSignals(false);
    }
    if (fromTree && scene_.selectedId() != id.toStdString()) {
        scene_.setSelected(id.toStdString());
    }
    refreshProperties();
    refreshStatus();
    viewport_->update();
}

void MainWindow::onTreeCurrentChanged(const QModelIndex& current, const QModelIndex&) {
    if (!current.isValid()) {
        scene_.setSelected({});
        return;
    }
    selectNode(QString::fromStdString(treeModel_->nodeId(current)), true);
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
        if (physicsShelf_ != nullptr && shelfReload_) {
            physicsShelf_->reload();
        }
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
    const int mechIndex = mechCombo_->findData(static_cast<int>(node->mech.kind));
    mechCombo_->setCurrentIndex(std::max(0, mechIndex));
    massSpin_->setValue(node->mech.mass);
    autoMassBox_->setChecked(node->mech.autoMass);
    kinematicBox_->setChecked(node->mech.kinematic);
    const int shapeIndex = shapeCombo_->findData(static_cast<int>(node->mech.shape));
    shapeCombo_->setCurrentIndex(std::max(0, shapeIndex));
    extentSpin_[0]->setValue(node->mech.halfExtents.x);
    extentSpin_[1]->setValue(node->mech.halfExtents.y);
    extentSpin_[2]->setValue(node->mech.halfExtents.z);
    radiusSpin_->setValue(node->mech.radius);
    heightSpin_->setValue(node->mech.height);
    otherEdit_->setText(QString::fromUtf8(node->mech.other.c_str()));
    int axisIndex = 0;
    if (std::abs(node->mech.axis.y) > std::abs(node->mech.axis.x) && std::abs(node->mech.axis.y) >= std::abs(node->mech.axis.z)) {
        axisIndex = 1;
    } else if (std::abs(node->mech.axis.z) > std::abs(node->mech.axis.x)) {
        axisIndex = 2;
    }
    axisCombo_->setCurrentIndex(axisIndex);
    limitedBox_->setChecked(node->mech.limited);
    lowerSpin_->setValue(node->mech.lower);
    upperSpin_->setValue(node->mech.upper);
    driveCombo_->setCurrentIndex(std::max(0, driveCombo_->findData(static_cast<int>(node->mech.drive))));
    targetSpin_->setValue(node->mech.target);
    maxSpeedSpin_->setValue(node->mech.maxSpeed);
    accelSpin_->setValue(node->mech.maxAccel);
    beltSpeedSpin_->setValue(node->mech.speed);
    rangeSpin_->setValue(node->mech.range);
    signalEdit_->setText(QString::fromUtf8(node->mech.signal.c_str()));
    valueSignalEdit_->setText(QString::fromUtf8(node->mech.valueSignal.c_str()));
    intervalSpin_->setValue(node->mech.interval);
    prototypeEdit_->setText(QString::fromUtf8(node->mech.prototype.c_str()));
    refreshing_ = false;
    if (physicsShelf_ != nullptr && shelfReload_) {
        physicsShelf_->reload();
    }
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
    if (mechCombo_->currentData().isValid()) {
        node->mech.kind = static_cast<MechKind>(mechCombo_->currentData().toInt());
    }
    node->mech.mass = massSpin_->value();
    node->mech.autoMass = autoMassBox_->isChecked();
    node->mech.kinematic = kinematicBox_->isChecked();
    if (node->mech.kinematic && node->mech.motion == BodyMotion::Free) {
        node->mech.motion = BodyMotion::Driven;
    } else if (!node->mech.kinematic && (node->mech.motion == BodyMotion::Driven || node->mech.motion == BodyMotion::Surface)) {
        node->mech.motion = BodyMotion::Free;
    }
    if (shapeCombo_->currentData().isValid()) {
        node->mech.shape = static_cast<CollisionShape>(shapeCombo_->currentData().toInt());
    }
    node->mech.halfExtents = {extentSpin_[0]->value(), extentSpin_[1]->value(), extentSpin_[2]->value()};
    node->mech.radius = radiusSpin_->value();
    node->mech.height = heightSpin_->value();
    node->mech.other = otherEdit_->text().toUtf8().toStdString();
    if (axisCombo_->currentIndex() == 1) {
        node->mech.axis = {0, 1, 0};
    } else if (axisCombo_->currentIndex() == 2) {
        node->mech.axis = {0, 0, 1};
    } else {
        node->mech.axis = {1, 0, 0};
    }
    node->mech.limited = limitedBox_->isChecked();
    node->mech.lower = lowerSpin_->value();
    node->mech.upper = upperSpin_->value();
    if (driveCombo_->currentData().isValid()) {
        node->mech.drive = static_cast<DriveMode>(driveCombo_->currentData().toInt());
    }
    node->mech.target = targetSpin_->value();
    node->mech.maxSpeed = maxSpeedSpin_->value();
    node->mech.maxAccel = accelSpin_->value();
    node->mech.speed = beltSpeedSpin_->value();
    node->mech.direction = node->mech.axis;
    node->mech.range = rangeSpin_->value();
    node->mech.signal = signalEdit_->text().toUtf8().toStdString();
    node->mech.valueSignal = valueSignalEdit_->text().toUtf8().toStdString();
    node->mech.interval = intervalSpin_->value();
    node->mech.prototype = prototypeEdit_->text().toUtf8().toStdString();
    scene_.notify(ChangeKind::Transform);
    if (tree_->currentIndex().isValid()) {
        treeModel_->reload();
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
    if (simulation_) {
        simulation_->stop();
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
    if (simulation_) {
        simulation_->stop();
    }
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
                    if (simulation_) {
                        simulation_->stop();
                    }
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

void MainWindow::arrangeLineDocks() {
    if (floorDock_ == nullptr || operationDock_ == nullptr || ganttDock_ == nullptr) {
        return;
    }
    auto show = [](QDockWidget* dock, bool visible) {
        if (dock != nullptr) {
            dock->setVisible(visible);
        }
    };
    show(treeDock_, false);
    show(propertyDock_, false);
    show(motionDock_, false);
    show(logicDock_, false);
    show(pathDock_, false);
    show(collisionDock_, false);
    show(signalDock_, false);
    show(scriptDock_, false);
    show(plcDock_, false);
    show(physicsDock_, false);
    show(floorDock_, true);
    show(operationDock_, true);
    show(ganttDock_, true);
    show(weldDock_, lineFocus_ == QStringLiteral("weld"));
    if (lineFocus_ == QStringLiteral("weld")) {
        show(floorDock_, false);
        show(operationDock_, false);
        show(ganttDock_, false);
        show(weldDock_, true);
        resizeDocks({weldDock_}, {460}, Qt::Horizontal);
        if (weldDock_ != nullptr) {
            weldDock_->raise();
        }
    } else if (lineFocus_ == QStringLiteral("ops")) {
        resizeDocks({operationDock_, floorDock_}, {820, 280}, Qt::Horizontal);
        resizeDocks({ganttDock_}, {180}, Qt::Vertical);
        operationDock_->raise();
    } else if (lineFocus_ == QStringLiteral("gantt")) {
        resizeDocks({floorDock_, operationDock_}, {280, 280}, Qt::Horizontal);
        resizeDocks({ganttDock_}, {460}, Qt::Vertical);
        ganttDock_->raise();
    } else {
        resizeDocks({floorDock_, operationDock_}, {700, 320}, Qt::Horizontal);
        resizeDocks({ganttDock_}, {230}, Qt::Vertical);
        floorDock_->raise();
    }
}

void MainWindow::captureScreenshot() {
    if (lineDemo_) {
        arrangeLineDocks();
    }
    // update() only queues a paint. The simulation clock is advanced before the
    // event loop runs, so the widget FBO would still show the pose from fitAll.
    viewport_->repaint();
    QImage view = viewport_->grabFramebuffer();
    if (view.isNull()) {
        view = viewport_->captureView();
    }
    QImage window = grab().toImage();
    if (!view.isNull() && !window.isNull()) {
        const qreal dpr = devicePixelRatioF();
        const QPoint topLeft = viewport_->mapTo(this, QPoint(0, 0));
        QPainter painter(&window);
        painter.drawImage(QRectF(topLeft.x() * dpr, topLeft.y() * dpr, viewport_->width() * dpr, viewport_->height() * dpr),
                          view);
        auto stamp = [&](QWidget* floater) {
            if (floater != nullptr && floater->isVisible()) {
                const QPoint origin = floater->mapTo(this, QPoint(0, 0));
                painter.drawPixmap(QRectF(origin.x() * dpr, origin.y() * dpr, floater->width() * dpr, floater->height() * dpr),
                                   floater->grab(), QRectF(floater->rect()));
            }
        };
        stamp(palette_);
        stamp(intent_);
    }
    const QImage& output = window.isNull() ? view : window;
    const bool saved = !output.isNull() && output.save(screenshotPath_);
    QApplication::exit(saved ? 0 : 2);
}

QString MainWindow::mechLabel(MechKind kind) const {
    switch (kind) {
        case MechKind::None: return trk("mech.none");
        case MechKind::Rigid: return trk("mech.rigid");
        case MechKind::Collision: return trk("mech.collision");
        case MechKind::FixedJoint: return trk("mech.fixed");
        case MechKind::Hinge: return trk("mech.hinge");
        case MechKind::Slider: return trk("mech.slider");
        case MechKind::Ball: return trk("mech.ball");
        case MechKind::Conveyor: return trk("mech.conveyor");
        case MechKind::CollisionSensor: return trk("mech.collisionSensor");
        case MechKind::DistanceSensor: return trk("mech.distanceSensor");
        case MechKind::LightBarrier: return trk("mech.lightBarrier");
        case MechKind::Source: return trk("mech.source");
        case MechKind::Sink: return trk("mech.sink");
        case MechKind::Limit: return trk("mech.limit");
        case MechKind::Cylinder: return trk("mech.cylinder");
        case MechKind::Gripper: return trk("mech.gripper");
    }
    return trk("mech.none");
}

void MainWindow::fillMechCombos() {
    refreshing_ = true;
    const auto fill = [](QComboBox* combo, const QList<QPair<QString, int>>& items) {
        const int current = combo->currentData().toInt();
        combo->clear();
        for (const auto& item : items) {
            combo->addItem(item.first, item.second);
        }
        const int index = combo->findData(current);
        combo->setCurrentIndex(index < 0 ? 0 : index);
    };
    fill(mechCombo_, {{trk("mech.none"), static_cast<int>(MechKind::None)},
                      {trk("mech.rigid"), static_cast<int>(MechKind::Rigid)},
                      {trk("mech.collision"), static_cast<int>(MechKind::Collision)},
                      {trk("mech.fixed"), static_cast<int>(MechKind::FixedJoint)},
                      {trk("mech.hinge"), static_cast<int>(MechKind::Hinge)},
                      {trk("mech.slider"), static_cast<int>(MechKind::Slider)},
                      {trk("mech.ball"), static_cast<int>(MechKind::Ball)},
                      {trk("mech.conveyor"), static_cast<int>(MechKind::Conveyor)},
                      {trk("mech.collisionSensor"), static_cast<int>(MechKind::CollisionSensor)},
                      {trk("mech.distanceSensor"), static_cast<int>(MechKind::DistanceSensor)},
                      {trk("mech.lightBarrier"), static_cast<int>(MechKind::LightBarrier)},
                      {trk("mech.source"), static_cast<int>(MechKind::Source)},
                      {trk("mech.sink"), static_cast<int>(MechKind::Sink)},
                      {trk("mech.limit"), static_cast<int>(MechKind::Limit)},
                      {trk("mech.cylinder"), static_cast<int>(MechKind::Cylinder)},
                      {trk("mech.gripper"), static_cast<int>(MechKind::Gripper)}});
    fill(shapeCombo_, {{trk("shape.box"), static_cast<int>(CollisionShape::Box)},
                       {trk("shape.sphere"), static_cast<int>(CollisionShape::Sphere)},
                       {trk("shape.capsule"), static_cast<int>(CollisionShape::Capsule)},
                       {trk("shape.convex"), static_cast<int>(CollisionShape::ConvexHull)},
                       {trk("shape.cylinder"), static_cast<int>(CollisionShape::Cylinder)},
                       {trk("shape.extrusion"), static_cast<int>(CollisionShape::Extrusion)},
                       {trk("shape.mesh"), static_cast<int>(CollisionShape::Mesh)}});
    axisCombo_->clear();
    axisCombo_->addItems({trk("axis.x"), trk("axis.y"), trk("axis.z")});
    fill(driveCombo_, {{trk("drive.none"), static_cast<int>(DriveMode::None)},
                       {trk("drive.position"), static_cast<int>(DriveMode::Position)},
                       {trk("drive.speed"), static_cast<int>(DriveMode::Speed)}});
    refreshing_ = false;
    refreshSignals();
    loadScriptEditor(-1);
}

void MainWindow::refreshSignals() {
    if (signalTable_ == nullptr) {
        return;
    }
    signalTable_->setRowCount(static_cast<int>(scene_.signals.size()));
    for (int row = 0; row < signalTable_->rowCount(); ++row) {
        const Signal& signal = scene_.signals[static_cast<std::size_t>(row)];
        auto ensure = [&](int column, const QString& text) {
            if (signalTable_->item(row, column) == nullptr) {
                signalTable_->setItem(row, column, new QTableWidgetItem);
            }
            signalTable_->item(row, column)->setText(text);
        };
        ensure(0, QString::fromUtf8(signal.name.c_str()));
        ensure(1, QString::fromUtf8(signalTypeName(signal.type)));
        QString value;
        if (signal.type == SignalType::Bool) {
            value = signal.flag ? trk("signal.true") : trk("signal.false");
        } else if (signal.type == SignalType::Int) {
            value = QString::number(signal.integer);
        } else {
            value = QString::number(signal.real, 'f', 2);
        }
        ensure(2, value);
    }
}

void MainWindow::commitScriptEditor() {
    if (scriptIndex_ < 0 || scriptIndex_ >= static_cast<int>(scene_.scripts.size()) || scriptEdit_ == nullptr) {
        return;
    }
    ProjectScript& script = scene_.scripts[static_cast<std::size_t>(scriptIndex_)];
    const std::string next = scriptEdit_->toPlainText().toUtf8().toStdString();
    if (script.source == next) {
        return;
    }
    script.source = next;
    scene_.notify(ChangeKind::Transform);
}

void MainWindow::loadScriptEditor(int index) {
    if (scriptCombo_ == nullptr) {
        return;
    }
    refreshing_ = true;
    if (scriptCombo_->count() != static_cast<int>(scene_.scripts.size())) {
        scriptCombo_->clear();
        for (const ProjectScript& script : scene_.scripts) {
            scriptCombo_->addItem(QString::fromUtf8(script.name.c_str()));
        }
        scriptIndex_ = -1;
    }
    if (scene_.scripts.empty()) {
        scriptEdit_->clear();
        scriptIndex_ = -1;
        refreshing_ = false;
        return;
    }
    int resolved = index;
    if (resolved < 0) {
        resolved = 0;
        for (int i = 0; i < static_cast<int>(scene_.scripts.size()); ++i) {
            if (scene_.scripts[static_cast<std::size_t>(i)].name == scene_.activeScript) {
                resolved = i;
                break;
            }
        }
    }
    scriptCombo_->setCurrentIndex(resolved);
    scriptIndex_ = resolved;
    scriptEdit_->setPlainText(QString::fromUtf8(scene_.scripts[static_cast<std::size_t>(resolved)].source.c_str()));
    refreshing_ = false;
}

void MainWindow::onScriptChanged(int index) {
    if (refreshing_ || index < 0) {
        return;
    }
    commitScriptEditor();
    loadScriptEditor(index);
    if (scriptIndex_ >= 0 && scriptIndex_ < static_cast<int>(scene_.scripts.size())) {
        const std::string& name = scene_.scripts[static_cast<std::size_t>(scriptIndex_)].name;
        if (scene_.activeScript != name) {
            scene_.activeScript = name;
            scene_.notify(ChangeKind::Transform);
        }
    }
}

void MainWindow::playSimulation() {
    commitScriptEditor();
    if (scriptIndex_ >= 0 && scriptIndex_ < static_cast<int>(scene_.scripts.size())) {
        scene_.activeScript = scene_.scripts[static_cast<std::size_t>(scriptIndex_)].name;
    }
    std::string source;
    for (const ProjectScript& script : scene_.scripts) {
        if (script.name == scene_.activeScript || source.empty()) {
            source = script.source;
            if (script.name == scene_.activeScript) {
                break;
            }
        }
    }
    std::string error;
    if (!scripting_->load(source, error)) {
        scriptLog_->appendPlainText(QString::fromStdString(error));
    }
    if (simulation_->state() == SimState::Stopped) {
        jointRest_.clear();
        for (const KinematicResource& resource : scene_.resources) {
            for (const KinJoint& joint : resource.joints) {
                jointRest_.push_back(joint.position);
            }
        }
    }
    simulation_->setHooks([this] { scripting_->onStart(); },
                          [this](double dt) {
                              scripting_->onStep(dt);
                              logic_.step(scene_, dt);
                              if (robot_ != nullptr) {
                                  robot_->step(dt);
                              }
                          },
                          [this] { scripting_->onStop(); });
    simulation_->setAfterStep([this] { scripting_->onContacts(); });
    std::string robotError;
    robot_->build(scene_, robotError);
    const std::string path = !pendingPath_.empty() ? pendingPath_ : demoPath_;
    pendingPath_.clear();
    if (robot_ != nullptr) {
        robot_->setHideAllDuringQuery(kr420TrackDemo_);
    }
    const auto hidden = kr420TrackDemo_ ? hideAllMeshedParts(scene_)
                                        : (kr420Demo_ ? hidePartsOutsideProximity(scene_) : std::vector<std::shared_ptr<SceneNode>>{});
    if (!path.empty()) {
        if (!robot_->arm(path, robotError) && !robotError.empty()) {
            statusLabel_->setText(QString::fromStdString(robotError));
        }
    }
    for (const auto& node : hidden) {
        if (node) {
            node->visible = true;
        }
    }
    simulation_->play();
    if (!simulation_->lastError().empty()) {
        statusLabel_->setText(QString::fromStdString(simulation_->lastError()));
    }
}

void MainWindow::pauseSimulation() { simulation_->pause(); }

void MainWindow::stopSimulation() {
    simulation_->stop();
    std::size_t cursor = 0;
    for (KinematicResource& resource : scene_.resources) {
        for (KinJoint& joint : resource.joints) {
            if (cursor < jointRest_.size()) {
                joint.position = jointRest_[cursor++];
            }
        }
    }
    logic_.reset();
    if (timeLabel_ != nullptr) {
        timeLabel_->setText(trk("status.simTime").arg(0.0, 0, 'f', 2));
    }
    refreshSignals();
}

void MainWindow::resetSimulation() { stopSimulation(); }

void MainWindow::onSimTick() {
    if (simulation_ == nullptr || simulation_->state() != SimState::Playing) {
        return;
    }
    simulation_->tick(0.016);
    if (viewport_ != nullptr && simulation_ != nullptr) {
        viewport_->setSimTime(simulation_->time());
        const PhysicsStats& stats = scene_.physicsDoc.stats;
        viewport_->setPhysicsLine(trk("phys.hud")
                                      .arg(stats.contacts)
                                      .arg(stats.detectMs, 0, 'f', 3)
                                      .arg(stats.solveMs, 0, 'f', 3)
                                      .arg(stats.continuous)
                                      .arg(QString::fromLatin1(physics_->backendName())));
    }
    if (logicCanvas_ != nullptr) {
        logicCanvas_->update();
    }
    if (pathTimeline_ != nullptr) {
        pathTimeline_->update();
    }
    const std::string log = scripting_->takeLog();
    if (!log.empty() && scriptLog_ != nullptr) {
        scriptLog_->appendPlainText(QString::fromStdString(log));
    }
}

void MainWindow::onSpeedChanged(double speed) {
    if (simulation_) {
        simulation_->setSpeed(speed);
    }
}

void MainWindow::runScriptSnippet() {
    if (scripting_ == nullptr || scriptEdit_ == nullptr) {
        return;
    }
    const std::string code = scriptEdit_->toPlainText().toUtf8().toStdString();
    if (scene_.scripts.empty()) {
        scene_.scripts.push_back(ProjectScript{"snippet", code});
        scene_.activeScript = "snippet";
        scriptIndex_ = 0;
        if (scriptCombo_ != nullptr) {
            refreshing_ = true;
            scriptCombo_->addItem(QStringLiteral("snippet"));
            scriptCombo_->setCurrentIndex(0);
            refreshing_ = false;
        }
        scene_.notify(ChangeKind::Structure);
    } else {
        commitScriptEditor();
    }
    std::string output;
    std::string error;
    const bool ok = scripting_->execute(code, output, error);
    if (scriptLog_ == nullptr) {
        return;
    }
    if (!output.empty()) {
        scriptLog_->appendPlainText(QString::fromStdString(output));
    }
    if (!ok) {
        scriptLog_->appendPlainText(QString::fromStdString(error.empty() ? std::string("script failed") : error));
    }
}

void MainWindow::usePhysicsBackend(PhysicsBackend backend) {
    if (simulation_ && simulation_->state() != SimState::Stopped) {
        simulation_->stop();
    }
    physics_ = createPhysicsEngine(backend);
    simulation_ = std::make_unique<Simulation>(scene_, *physics_);
    if (scripting_ != nullptr) {
        scripting_->setPhysicsHooks([this](const std::string& id) { physics_->adoptSpawned(id); },
                                    [this](const std::string& id) { physics_->releaseBody(id); });
    }
    scene_.physicsDoc.engine = physicsBackendName(backend);
}

namespace {

bool graftKr420Model(Scene& scene, const std::string& stepPath, std::string& note) {
    Scene side;
    StepImporter importer;
    const ImportResult imported = importer.importFile(stepPath, side);
    if (!imported.ok) {
        note = imported.message;
        return false;
    }
    const SuggestResult suggestion = suggestLinkGroups(side);
    std::string error;
    if (!applyLinkSuggestions(side, "kr420", suggestion, kr420DatasheetLimits(), error)) {
        note = error.empty() ? suggestion.note : error;
        return false;
    }
    addKr420Demonstration(side);
    if (auto proxy = scene.findByName("KR420")) {
        scene.removeNode(proxy->id);
    }
    if (auto column = scene.findByName("KR420Column")) {
        scene.removeNode(column->id);
    }
    std::unordered_map<std::string, std::string> meshMap;
    std::unordered_map<std::string, std::string> idMap;
    std::function<void(const std::shared_ptr<SceneNode>&, const std::shared_ptr<SceneNode>&)> copy =
        [&](const std::shared_ptr<SceneNode>& from, const std::shared_ptr<SceneNode>& parent) {
            if (!from) {
                return;
            }
            for (const auto& child : from->children) {
                auto node = scene.makeNode(child->kind, child->name);
                node->local = child->local;
                node->color = child->color;
                node->visible = child->visible;
                node->faceAxes = child->faceAxes;
                node->mech = child->mech;
                if (!child->meshId.empty()) {
                    if (meshMap.count(child->meshId) == 0) {
                        meshMap[child->meshId] = scene.addMesh(side.mesh(child->meshId));
                    }
                    node->meshId = meshMap[child->meshId];
                }
                idMap[child->id] = node->id;
                scene.addChild(parent, node);
                copy(child, node);
            }
        };
    scene.beginEdit();
    if (side.root()) {
        copy(side.root(), scene.root());
    }
    scene.robotPaths.erase(std::remove_if(scene.robotPaths.begin(), scene.robotPaths.end(),
                                          [](const RobotPath& path) { return path.name == "cycle"; }),
                           scene.robotPaths.end());
    for (RobotPath path : side.robotPaths) {
        scene.robotPaths.push_back(std::move(path));
    }
    for (KinematicResource resource : side.resources) {
        for (KinLink& link : resource.links) {
            const auto mapped = idMap.find(link.nodeId);
            if (mapped != idMap.end()) {
                link.nodeId = mapped->second;
            }
        }
        bool replaced = false;
        for (KinematicResource& existing : scene.resources) {
            if (existing.name == resource.name) {
                existing = resource;
                replaced = true;
            }
        }
        if (!replaced) {
            scene.resources.push_back(std::move(resource));
        }
    }
    if (Station* weld = findStation(scene.layout, "st_weld")) {
        for (const KinematicResource& resource : scene.resources) {
            if (resource.name != "kr420" || resource.links.empty()) {
                continue;
            }
            if (auto node = scene.find(resource.links.front().nodeId)) {
                weld->nodeName = node->name;
            }
        }
    }
    scene.proximity = side.proximity;
    scene.endEdit();
    note = suggestion.note.empty() ? "KR420" : suggestion.note;
    return true;
}

}  // namespace

void MainWindow::loadDemo(const QString& name) {
    collisionDemo_ = name == QStringLiteral("collision");
    kr420Demo_ = name == QStringLiteral("kr420");
    kr420TrackDemo_ = name == QStringLiteral("kr420track");
    transferDemo_ = name == QStringLiteral("transfer");
    meshDemo_ = name == QStringLiteral("mesh");
    slipDemo_ = name == QStringLiteral("slip");
    mcdDemo_ = name == QStringLiteral("mcd");
    lineDemo_ = name == QStringLiteral("line") || name.startsWith(QStringLiteral("line-"));
    lineFocus_ = QStringLiteral("floor");
    if (name.endsWith(QStringLiteral("ops"))) {
        lineFocus_ = QStringLiteral("ops");
    } else if (name.endsWith(QStringLiteral("gantt"))) {
        lineFocus_ = QStringLiteral("gantt");
    } else if (name.endsWith(QStringLiteral("weld"))) {
        lineFocus_ = QStringLiteral("weld");
    }
    if (name == QStringLiteral("scara")) {
        buildScaraDemo(scene_);
        demoPath_ = "pick_place";
    } else if (collisionDemo_) {
        buildCollisionDemo(scene_);
        demoPath_.clear();
    } else if (kr420Demo_) {
        scene_.clear();
        demoPath_.clear();
        const QString stepPath = QString::fromUtf8(SIM_SOURCE_DIR "/testdata/local/KR420_R3080_KR280_R3080.stp");
        if (!QFile::exists(stepPath)) {
            statusLabel_->setText(trk("motion.kr420Missing"));
        } else {
            StepImporter importer;
            const ImportResult imported = importer.importFile(stepPath.toStdString(), scene_);
            if (!imported.ok) {
                statusLabel_->setText(QString::fromStdString(imported.message));
            } else {
                const SuggestResult suggestion = suggestLinkGroups(scene_);
                std::string error;
                if (!applyLinkSuggestions(scene_, "kr420", suggestion, kr420DatasheetLimits(), error)) {
                    statusLabel_->setText(QString::fromStdString(error.empty() ? suggestion.note : error));
                } else {
                    addKr420Demonstration(scene_);
                    demoPath_ = "cycle";
                    statusLabel_->setText(QString::fromStdString(suggestion.note));
                }
            }
        }
    } else if (kr420TrackDemo_) {
        scene_.clear();
        demoPath_.clear();
        const QString stepPath = QString::fromUtf8(SIM_SOURCE_DIR "/testdata/local/KR420_R3080_KR280_R3080.stp");
        if (!QFile::exists(stepPath)) {
            statusLabel_->setText(trk("motion.kr420Missing"));
        } else {
            StepImporter importer;
            const ImportResult imported = importer.importFile(stepPath.toStdString(), scene_);
            if (!imported.ok) {
                statusLabel_->setText(QString::fromStdString(imported.message));
            } else {
                const SuggestResult suggestion = suggestLinkGroups(scene_);
                std::string error;
                if (!applyLinkSuggestions(scene_, "kr420", suggestion, kr420DatasheetLimits(), error)) {
                    statusLabel_->setText(QString::fromStdString(error.empty() ? suggestion.note : error));
                } else if (!addKr420TrackDemonstration(scene_, error)) {
                    statusLabel_->setText(QString::fromStdString(error));
                } else {
                    demoPath_ = "long";
                    statusLabel_->setText(QString::fromStdString(suggestion.note));
                }
            }
        }
    } else if (transferDemo_) {
        usePhysicsBackend(PhysicsBackend::Builtin);
        buildTransferLine(scene_);
        demoPath_.clear();
        applyTaskMode(4);
    } else if (meshDemo_) {
        usePhysicsBackend(PhysicsBackend::Builtin);
        buildMeshGapDemo(scene_);
        demoPath_.clear();
        applyTaskMode(4);
    } else if (slipDemo_) {
        usePhysicsBackend(PhysicsBackend::Builtin);
        buildSlipDemo(scene_);
        demoPath_.clear();
        applyTaskMode(4);
    } else if (mcdDemo_) {
        usePhysicsBackend(PhysicsBackend::Builtin);
        buildDeviceDemo(scene_);
        demoPath_.clear();
        std::string error;
        logic_.reset();
        logic_.start(scene_, "cell", error);
        if (physics_ != nullptr && physics_->build(scene_, error)) {
            for (int i = 0; i < 72; ++i) {
                logic_.step(scene_, 1.0 / 60.0);
                physics_->step(1.0 / 60.0);
            }
        }
        const auto degreesOf = [](const std::shared_ptr<SceneNode>& node) {
            if (!node) {
                return 0.0;
            }
            const Quat q = node->local.rotation.normalized();
            return rad2deg(2.0 * std::atan2(q.z, q.w));
        };
        const double gear = degreesOf(scene_.findByName("GearOut"));
        const auto cylinder = scene_.findByName("Cylinder");
        const double stroke = cylinder ? cylinder->mech.command : 0;
        const auto gripper = scene_.findByName("Gripper");
        const bool held = gripper && !gripper->mech.held.empty();
        const Signal* limit = scene_.findSignal("at_limit");
        const Signal* collected = scene_.findSignal("collected_n");
        viewport_->setCallout(QStringLiteral("齿轮 %1° · 气缸 %2 mm · 抓取 %3 · 限位 %4 · 收集 %5")
                                  .arg(gear, 0, 'f', 0)
                                  .arg(stroke, 0, 'f', 0)
                                  .arg(held ? QStringLiteral("已持") : QStringLiteral("空"))
                                  .arg(limit != nullptr && limit->flag ? QStringLiteral("开") : QStringLiteral("关"))
                                  .arg(collected != nullptr ? static_cast<int>(collected->integer) : 0));
        if (cylinder) {
            scene_.setSelected(cylinder->id);
        }
        applyTaskMode(4);
        refreshSignals();
        if (signalDock_ != nullptr) {
            signalDock_->raise();
            resizeDocks({signalDock_}, {210}, Qt::Vertical);
        }
        if (physicsDock_ != nullptr) {
            physicsDock_->raise();
        }
    } else if (name == QStringLiteral("plc")) {
        demoPath_.clear();
        if (plc_ != nullptr) {
            plc_->demonstrateSecure();
        }
        viewport_->setCallout(QStringLiteral("Basic256Sha256 · SignAndEncrypt · operator"));
        applyTaskMode(5);
        refreshSignals();
        if (plcDock_ != nullptr) {
            plcDock_->show();
            plcDock_->raise();
            resizeDocks({plcDock_}, {460}, Qt::Horizontal);
        }
        if (signalDock_ != nullptr) {
            signalDock_->show();
            signalDock_->raise();
            resizeDocks({signalDock_}, {220}, Qt::Vertical);
        }
    } else if (lineDemo_) {
        buildExampleLine(scene_);
        demoPath_.clear();
        const QString stepPath = QString::fromUtf8(SIM_SOURCE_DIR "/testdata/local/KR420_R3080_KR280_R3080.stp");
        std::string note;
        const bool grafted = QFile::exists(stepPath) && graftKr420Model(scene_, stepPath.toStdString(), note);
        const MeasureResult measured = measureOperationTimes(scene_, false);
        statusLabel_->setText(QString::fromStdString(scene_.layout.title) + QStringLiteral(" · ") +
                              (grafted ? QStringLiteral("KR420 模型") : QStringLiteral("代理模型")) +
                              QStringLiteral(" · 实测 ") + QString::number(measured.updated));
        if (lineFocus_ == QStringLiteral("weld")) {
            prepareExampleWelds(scene_);
            const int tourPoints = planExampleWeldTour(scene_);
            KinematicResource* robot = findResource(scene_.resources, "kr420");
            const BaseRegion region = robot == nullptr ? BaseRegion{} : optimizeRobotBase(*robot, scene_.welds, 500, 3);
            if (weld_ != nullptr) {
                weld_->reload();
            }
            int reachable = 0;
            for (const WeldSpot& spot : scene_.welds) {
                if (spot.reachable) {
                    ++reachable;
                }
            }
            int envelopeSegments = 0;
            if (robot != nullptr) {
                std::vector<std::vector<double>> tourJoints;
                for (const RobotPath& path : scene_.robotPaths) {
                    if (path.name != "weld_tour") {
                        continue;
                    }
                    for (const PathPoint& point : path.points) {
                        if (!point.joints.empty()) {
                            tourJoints.push_back(point.joints);
                        }
                    }
                }
                if (tourJoints.size() >= 2) {
                    const MotionEnvelope envelope = buildLinkSweep(scene_, *robot, tourJoints, "weld_envelope");
                    envelopeSegments = static_cast<int>(envelope.nodes.size());
                }
                KinematicResource mate = *robot;
                mate.name = "kr280";
                KinJoint rail;
                rail.name = "rail";
                rail.motion = JointMotion::Prismatic;
                rail.axis = {0, 1, 0};
                rail.externalName = "E1";
                rail.externalKind = "track";
                rail.position = 350;
                rail.lower = -3000;
                rail.upper = 3000;
                mate.joints.push_back(rail);
                scene_.resources.push_back(std::move(mate));
                robot = findResource(scene_.resources, "kr420");
                KinematicResource* mateRobot = findResource(scene_.resources, "kr280");
                const Vec3 tcpA = robot == nullptr ? Vec3{} : forwardModel(*robot).position;
                const Vec3 tcpB = mateRobot == nullptr ? tcpA : forwardModel(*mateRobot).position;
                auto proxy = scene_.makeNode(NodeKind::Part, "KR280");
                proxy->meshId = scene_.addMesh(makeBoxMesh({160, 160, 420}));
                proxy->local.translation = tcpB;
                proxy->color = {0.86, 0.45, 0.18};
                scene_.addChild(scene_.root(), proxy);
                InterlockZone zone;
                zone.name = "weld_cell";
                zone.center = (tcpA + tcpB) * 0.5;
                zone.radius = 400;
                zone.releaseSeconds = 1.5;
                zone.resources = {"kr420", "kr280"};
                scene_.interlocks = {zone};
                evaluateInterlocks(scene_);
                if (robot != nullptr) {
                    for (WeldSpot& spot : scene_.welds) {
                        if (!spot.reachable) {
                            spot.advice = suggestWeldFix(*robot, spot).text;
                        }
                    }
                }
            }
            const QString base = region.found ? QStringLiteral("底座 X %1…%2")
                                                    .arg(region.minX, 0, 'f', 0)
                                                    .arg(region.maxX, 0, 'f', 0)
                                              : QStringLiteral("底座未全覆盖");
            int krlPoints = 0;
            if (robot != nullptr) {
                const std::unique_ptr<IOlpProgram> post = createOlpProgram("kuka");
                for (const RobotPath& path : scene_.robotPaths) {
                    if (path.name != "weld_tour" || post == nullptr) {
                        continue;
                    }
                    std::string src;
                    std::string dat;
                    std::string error;
                    RobotPath back;
                    if (post->exportProgram(*robot, path, src, dat, error) &&
                        post->importProgram(robot, src, dat, back, error)) {
                        krlPoints = static_cast<int>(back.points.size());
                    }
                }
            }
            const double hold = applyInterlockHold(scene_);
            const ScheduleReport schedule = scheduleOperations(scene_.operations, scene_.layout);
            if (weld_ != nullptr) {
                weld_->reload();
            }
            viewport_->setCallout(QStringLiteral("示例焊点 %1 · 可达 %2 · %3 · 避障 %4 点 · 连杆包络 %5 · 互锁保持 %6 s · 节拍 %7 s · KRL %8 点")
                                      .arg(static_cast<int>(scene_.welds.size()))
                                      .arg(reachable)
                                      .arg(base)
                                      .arg(tourPoints)
                                      .arg(envelopeSegments)
                                      .arg(hold, 0, 'f', 1)
                                      .arg(schedule.makespan, 0, 'f', 1)
                                      .arg(krlPoints));
            applyTaskMode(3);
        } else {
            applyTaskMode(0);
        }
    } else {
        buildTurntableLiftDemo(scene_);
        demoPath_.clear();
    }
    if (motion_ != nullptr) {
        motion_->refresh();
    }
    if (collision_ != nullptr && collisionDemo_) {
        collision_->reload();
        viewport_->setCallout(collision_->summaryText());
    }
    if (collisionDemo_) {
        applyTaskMode(4);
    } else if (kr420Demo_ || kr420TrackDemo_ || name == QStringLiteral("scara")) {
        applyTaskMode(3);
    } else if (name == QStringLiteral("turntable")) {
        applyTaskMode(1);
    }
    if (logicCanvas_ != nullptr) {
        logicCanvas_->update();
    }
    if (pathTimeline_ != nullptr) {
        pathTimeline_->update();
    }
    if (floor_ != nullptr) {
        floor_->reload();
    }
    if (operations_ != nullptr) {
        operations_->reload();
    }
    if (gantt_ != nullptr) {
        gantt_->reload();
    }
    if (lineDemo_) {
        arrangeLineDocks();
    }
    rebuildTree();
    viewport_->fitAll();
    if (lineFocus_ == QStringLiteral("weld") && !scene_.welds.empty()) {
        Vec3 center{0, 0, 0};
        for (const WeldSpot& spot : scene_.welds) {
            center = center + spot.position;
        }
        center = center / static_cast<double>(scene_.welds.size());
        viewport_->camera().target = center;
        viewport_->camera().distance = 1600;
        viewport_->camera().eyeDirection = {0.55, -0.72, 0.42};
    }
}

void MainWindow::runLargeSceneBench(const QString& screenshot) {
    benchShot_ = screenshot;
    benchTries_ = 0;
    benchPhase_ = 0;
    for (QDockWidget* dock : {propertyDock_, signalDock_, scriptDock_, collisionDock_, motionDock_, logicDock_, pathDock_,
                              plcDock_, physicsDock_, floorDock_, operationDock_, ganttDock_}) {
        if (dock != nullptr) {
            dock->hide();
        }
    }
    if (treeDock_ != nullptr) {
        treeDock_->show();
        resizeDocks({treeDock_}, {420}, Qt::Horizontal);
    }
    // The model reads the scene directly. A per-node callback would rebuild the view 100000 times.
    auto callback = std::move(scene_.onChanged);
    scene_.onChanged = nullptr;
    buildLargeField(scene_, 100000, 256);
    scene_.onChanged = std::move(callback);
    // show() has already returned, but the OpenGL widget is realized on the event loop.
    QTimer::singleShot(0, this, [this] { finishLargeSceneBench(); });
}

void MainWindow::finishLargeSceneBench() {
    if (benchPhase_ == 0) {
        QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        if (viewport_ == nullptr || !viewport_->isValid() || viewport_->width() < 32 || viewport_->height() < 32) {
            if (++benchTries_ < 40) {
                QTimer::singleShot(50, this, [this] { finishLargeSceneBench(); });
                return;
            }
            std::cout << "BENCH failed reason=viewport_not_realized tries=" << benchTries_ << std::endl;
            QApplication::exit(2);
            return;
        }
        if (treeModel_ != nullptr && tree_ != nullptr && treeSearch_ != nullptr) {
            treeSearch_->clear();
            treeModel_->reload();
            const QModelIndex root = treeModel_->index(0, 0, {});
            QElapsedTimer treeTimer;
            treeTimer.start();
            if (root.isValid()) {
                tree_->expand(root);
            }
            const QModelIndex firstChild = treeModel_->index(0, 0, root);
            const int childCount = treeModel_->rowCount(root);
            const QModelIndex lastChild = childCount > 0 ? treeModel_->index(childCount - 1, 0, root) : QModelIndex{};
            for (int pass = 0; pass < 8; ++pass) {
                tree_->scrollTo(pass % 2 == 0 ? lastChild : firstChild, QAbstractItemView::PositionAtTop);
            }
            benchScrollMs_ = static_cast<double>(treeTimer.nsecsElapsed()) / 1.0e6;
            std::vector<std::string> matches;
            treeTimer.restart();
            const int found = scene_.collectNames("p99999", matches, 10);
            benchTreeMs_ = static_cast<double>(treeTimer.nsecsElapsed()) / 1.0e6;
            benchTreeHits_ = found;
            treeSearch_->setText(QStringLiteral("p99999"));
            QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
            const QModelIndex filtered = treeModel_->index(0, 0, {});
            if (filtered.isValid()) {
                tree_->setCurrentIndex(filtered);
                tree_->scrollTo(filtered);
            }
            QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
            if (!benchShot_.isEmpty()) {
                QString treeShot = benchShot_;
                treeShot.replace(QStringLiteral("large-field"), QStringLiteral("large-tree"));
                if (treeShot == benchShot_) {
                    treeShot += QStringLiteral(".tree.png");
                }
                grab().save(treeShot);
            }
        }
        for (QDockWidget* dock : {treeDock_, propertyDock_}) {
            if (dock != nullptr) {
                dock->hide();
            }
        }
        viewport_->setStandardView(StandardView::Iso);
        viewport_->setForceDetail(true);
        viewport_->fitAll();
        benchPhase_ = 1;
        // A second repaint in the same slot does not enter paintGL. Time the next turn.
        QTimer::singleShot(0, this, [this] { finishLargeSceneBench(); });
        return;
    }

    QElapsedTimer timer;
    timer.start();
    viewport_->repaint();
    const double wallMs = static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    const double drawMs = viewport_->lastDrawMs();
    const double frameMs = wallMs > drawMs ? wallMs : drawMs;
    if (viewport_->lastSubmitted() <= 0) {
        if (++benchTries_ < 40) {
            QTimer::singleShot(50, this, [this] { finishLargeSceneBench(); });
            return;
        }
        std::cout << "BENCH failed reason=no_instances phase=" << benchPhase_ << " wall_ms=" << wallMs
                  << " draw_ms=" << drawMs << std::endl;
        QApplication::exit(2);
        return;
    }

    if (benchPhase_ == 1) {
        benchFitMs_ = frameMs;
        benchFitSubmitted_ = viewport_->lastSubmitted();
        benchFitCulled_ = viewport_->lastCulled();
        benchFitDraws_ = viewport_->lastDraws();
        benchFitDrawn_ = viewport_->lastDrawnTriangles();
        double rssKb = 0;
        std::ifstream status("/proc/self/status");
        std::string label;
        while (status >> label) {
            if (label == "VmRSS:") {
                status >> rssKb;
                break;
            }
            status.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        benchFitOccluded_ = viewport_->lastOccluded();
        viewport_->setCallout(QString("10万零件 · 全细节 %1 三角 · %2 ms · 遮挡 %3 · RSS %4 MB · MSAA %5")
                                  .arg(benchFitDrawn_)
                                  .arg(benchFitMs_, 0, 'f', 0)
                                  .arg(benchFitOccluded_)
                                  .arg(rssKb / 1024.0, 0, 'f', 0)
                                  .arg(QSurfaceFormat::defaultFormat().samples()));
        viewport_->setForceDetail(false);
        viewport_->camera().target = scene_.worldBounds().center();
        viewport_->camera().distance = 4500;
        benchPhase_ = 2;
        QTimer::singleShot(0, this, [this] { finishLargeSceneBench(); });
        return;
    }

    if (benchPhase_ == 2) {
        benchNearSubmitted_ = viewport_->lastSubmitted();
        benchNearCulled_ = viewport_->lastCulled();
        benchNearDrawn_ = viewport_->lastDrawnTriangles();
        benchNearOccluded_ = viewport_->lastOccluded();
        benchNearMs_ = frameMs;
        viewport_->camera().eyeDirection = {0.18, -0.97, 0.14};
        viewport_->camera().distance = 1600;
        viewport_->camera().target = scene_.worldBounds().center();
        // A slab in front of the field. Parts behind it are what the occlusion buffer can reject.
        // The fit-all and near frames already finished, and the slab is hidden again before the shot.
        if (scene_.findByName("OccluderWall") == nullptr) {
            const Vec3 center = scene_.worldBounds().center();
            const Vec3 towardEye = normalize(viewport_->camera().eyeDirection);
            auto wall = scene_.makeNode(NodeKind::Part, "OccluderWall");
            wall->meshId = scene_.addMesh(makeBoxMesh({6000, 80, 2500}));
            wall->local.translation = center + towardEye * 700.0;
            wall->color = {0.85, 0.45, 0.2};
            scene_.addChild(scene_.root(), wall);
        }
        benchPhase_ = 3;
        QTimer::singleShot(0, this, [this] { finishLargeSceneBench(); });
        return;
    }

    benchGrazeSubmitted_ = viewport_->lastSubmitted();
    benchGrazeOccluded_ = viewport_->lastOccluded();
    if (auto wall = scene_.findByName("OccluderWall")) {
        wall->visible = false;
    }
    viewport_->setStandardView(StandardView::Iso);
    viewport_->camera().target = scene_.worldBounds().center();
    viewport_->camera().distance = 4500;

    long long sceneTriangles = 0;
    int meshTriangles = 0;
    int parts = 0;
    if (const auto part = scene_.findByName("p0")) {
        if (const auto mesh = scene_.mesh(part->meshId)) {
            meshTriangles = static_cast<int>(mesh->triangleCount());
        }
    }
    if (scene_.root()) {
        for (const auto& child : scene_.root()->children) {
            if (child && child->name.rfind("p", 0) == 0) {
                ++parts;
            }
        }
    }
    sceneTriangles = static_cast<long long>(meshTriangles) * parts;
    double rssKb = 0;
    std::ifstream status("/proc/self/status");
    std::string label;
    while (status >> label) {
        if (label == "VmRSS:") {
            status >> rssKb;
            break;
        }
        status.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    const int msaa = QSurfaceFormat::defaultFormat().samples();
    const double rssMb = rssKb / 1024.0;
    const double nearMs = benchNearMs_;
    std::vector<Aabb> gpuBounds;
    if (scene_.root()) {
        gpuBounds.reserve(scene_.root()->children.size());
        for (const auto& child : scene_.root()->children) {
            if (!child) {
                continue;
            }
            Aabb box;
            box.expand(child->local.translation - Vec3{20, 20, 10});
            box.expand(child->local.translation + Vec3{20, 20, 20});
            gpuBounds.push_back(box);
        }
    }
    QElapsedTimer gpuTimer;
    gpuTimer.start();
    const int gpuKept = viewport_->gpuFrustumCull(gpuBounds);
    const double gpuMs = static_cast<double>(gpuTimer.nsecsElapsed()) / 1.0e6;
    std::cout << "BENCH parts=" << parts << " mesh_triangles=" << meshTriangles << " triangles=" << sceneTriangles
              << " submitted=" << benchFitSubmitted_ << " culled=" << benchFitCulled_ << " occluded=" << benchFitOccluded_
              << " draws=" << benchFitDraws_ << " drawn_triangles=" << benchFitDrawn_ << " frame_ms=" << benchFitMs_
              << " fps=" << (benchFitMs_ > 0 ? 1000.0 / benchFitMs_ : 0) << " rss_mb=" << rssMb << " msaa=" << msaa
              << " near_submitted=" << benchNearSubmitted_ << " near_culled=" << benchNearCulled_
              << " near_occluded=" << benchNearOccluded_ << " near_drawn=" << benchNearDrawn_
              << " near_frame_ms=" << nearMs << " near_fps=" << (nearMs > 0 ? 1000.0 / nearMs : 0)
              << " resident=" << viewport_->residentMeshes() << " tree_query_ms=" << benchTreeMs_
              << " tree_hits=" << benchTreeHits_ << " tree_scroll_ms=" << benchScrollMs_ << " gpu_kept=" << gpuKept
              << " gpu_ms=" << gpuMs << " gpu_count=" << gpuBounds.size()
              << " graze_submitted=" << benchGrazeSubmitted_ << " graze_occluded=" << benchGrazeOccluded_ << std::endl;
    if (benchFitMs_ < 1.0 || nearMs < 1.0) {
        std::cout << "BENCH failed reason=timer_did_not_cover_paint" << std::endl;
        QApplication::exit(2);
        return;
    }
    if (!benchShot_.isEmpty()) {
        setScreenshotPath(benchShot_);
        captureScreenshot();
    } else {
        QApplication::exit(0);
    }
}

void MainWindow::beginHeadlessPlayback() {
    QTimer::singleShot(200, this, [this] {
        resizeDocks({treeDock_, propertyDock_}, {280, 300}, Qt::Horizontal);
        if (auto* signals = findChild<QDockWidget*>("dock.signals")) {
            if (auto* scripts = findChild<QDockWidget*>("dock.script")) {
                resizeDocks({signals, scripts}, {380, 720}, Qt::Horizontal);
            }
        }
        viewport_->setStandardView(kr420TrackDemo_ ? StandardView::Front : StandardView::Iso);
        viewport_->fitAll();
        if (collisionDemo_ && collision_ != nullptr) {
            if (auto* dock = findChild<QDockWidget*>("dock.collision")) {
                if (auto* signals = findChild<QDockWidget*>("dock.signals")) {
                    tabifyDockWidget(signals, dock);
                }
                dock->show();
                dock->raise();
                resizeDocks({dock}, {280}, Qt::Vertical);
            }
            collision_->reload();
            viewport_->fitAll();
        }
        if (transferDemo_ || meshDemo_ || slipDemo_) {
            const char* focus = transferDemo_ ? "RubberBox" : meshDemo_ ? "Supported" : "GripBox";
            if (const auto box = scene_.findByName(focus)) {
                scene_.setSelected(box->id);
            }
            if (physicsDock_ != nullptr) {
                physicsDock_->show();
                physicsDock_->raise();
                resizeDocks({physicsDock_}, {360}, Qt::Horizontal);
            }
        } else if (const auto pusher = scene_.findByName("Pusher")) {
            scene_.setSelected(pusher->id);
        }
        playSimulation();
        if (screenshotPath_.isEmpty()) {
            return;
        }
        // Catch up on the simulation clock before the first software-GL paint.
        // A wall-clock timer expires during that paint and both shots land at time 0.
        const double target = std::max(headlessSeconds_, 0.0);
        int guard = 0;
        while (simulation_ && simulation_->time() + 1e-4 < target && guard < 8) {
            const double remain = target - simulation_->time();
            simulation_->tick(std::min(remain, 2.0));
            ++guard;
        }
        if (viewport_ != nullptr) {
            if (meshDemo_ || slipDemo_) {
                viewport_->fitAll();
            }
            const PhysicsStats& stats = scene_.physicsDoc.stats;
            viewport_->setPhysicsLine(trk("phys.hud")
                                          .arg(stats.contacts)
                                          .arg(stats.detectMs, 0, 'f', 3)
                                          .arg(stats.solveMs, 0, 'f', 3)
                                          .arg(stats.continuous)
                                          .arg(QString::fromLatin1(physics_ ? physics_->backendName() : "")));
        }
        QTimer::singleShot(0, this, &MainWindow::captureScreenshot);
    });
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    positionFloaters();
}

void MainWindow::positionFloaters() {
    if (palette_ != nullptr) {
        palette_->move(std::max(12, (width() - palette_->width()) / 2), 64);
    }
    if (intent_ != nullptr) {
        intent_->move(std::max(12, (width() - intent_->width()) / 2), std::max(80, height() - intent_->height() - 40));
    }
}

void MainWindow::setTaskModeName(const QString& mode) {
    const QString key = mode.trimmed().toLower();
    int index = 0;
    if (key == "kinematics" || key == "2") {
        index = 1;
    } else if (key == "logic" || key == "3") {
        index = 2;
    } else if (key == "robot" || key == "4") {
        index = 3;
    } else if (key == "debug" || key == "5") {
        index = 4;
    } else if (key == "commission" || key == "6") {
        index = 5;
    }
    applyTaskMode(index);
}

void MainWindow::applyTaskMode(int mode) {
    taskMode_ = std::clamp(mode, 0, 5);
    auto show = [this](QDockWidget* dock, bool visible) {
        if (dock != nullptr) {
            dock->setVisible(visible && !focusMode_);
        }
    };
    show(treeDock_, taskMode_ == 0 || taskMode_ == 1 || taskMode_ == 3 || taskMode_ == 4);
    show(propertyDock_, taskMode_ != 2);
    show(motionDock_, taskMode_ == 1 || taskMode_ == 3);
    show(logicDock_, taskMode_ == 2);
    show(pathDock_, taskMode_ == 3);
    show(collisionDock_, taskMode_ == 4 || taskMode_ == 5);
    show(signalDock_, taskMode_ == 2 || taskMode_ == 4 || taskMode_ == 5);
    show(scriptDock_, taskMode_ == 5);
    show(plcDock_, taskMode_ == 5);
    show(physicsDock_, taskMode_ == 0 || taskMode_ == 4);
    const bool showLine = !scene_.layout.stations.empty() && taskMode_ == 0;
    show(floorDock_, showLine);
    show(operationDock_, showLine);
    show(ganttDock_, showLine);
    show(weldDock_, taskMode_ == 3 || lineFocus_ == QStringLiteral("weld"));
    if (lineDemo_) {
        arrangeLineDocks();
    }
    if (chrome_ != nullptr) {
        chrome_->setMode(taskMode_);
    }
    if (logicCanvas_ != nullptr && taskMode_ == 2) {
        logicCanvas_->reload();
        resizeDocks({logicDock_}, {720}, Qt::Horizontal);
    }
    if (pathTimeline_ != nullptr && taskMode_ == 3) {
        pathTimeline_->reload();
        resizeDocks({pathDock_}, {260}, Qt::Vertical);
    }
    if (taskMode_ == 5 && plcDock_ != nullptr) {
        resizeDocks({plcDock_}, {300}, Qt::Horizontal);
    }
    saveWorkspace();
}

void MainWindow::setThemeName(const QString& theme) {
    themeName_ = theme == "light" ? "light" : "dark";
    QFile file(QString::fromLatin1(SIM_SOURCE_DIR) + "/resources/theme/" + themeName_ + ".qss");
    if (file.open(QIODevice::ReadOnly)) {
        qApp->setStyleSheet(QString::fromUtf8(file.readAll()));
    }
    if (viewport_ != nullptr) {
        viewport_->setDarkChrome(themeName_ == "dark");
    }
    if (logicCanvas_ != nullptr) {
        logicCanvas_->setDark(themeName_ == "dark");
    }
    if (pathTimeline_ != nullptr) {
        pathTimeline_->setDark(themeName_ == "dark");
    }
    if (floor_ != nullptr) {
        floor_->setDark(themeName_ == "dark");
    }
    if (gantt_ != nullptr) {
        gantt_->setDark(themeName_ == "dark");
    }
    if (operations_ != nullptr) {
        operations_->setDark(themeName_ == "dark");
    }
    saveWorkspace();
}

void MainWindow::showCommandPalette() {
    if (palette_ == nullptr) {
        return;
    }
    positionFloaters();
    palette_->openFor(&scene_);
}

void MainWindow::showPlcConnection() {
    // The task mode is applied before the event loop starts.
    applyTaskMode(5);
    for (QDockWidget* dock : {treeDock_, propertyDock_, signalDock_, scriptDock_, collisionDock_, motionDock_, logicDock_,
                              pathDock_, physicsDock_, floorDock_, operationDock_, ganttDock_}) {
        if (dock != nullptr) {
            dock->hide();
        }
    }
    if (plcDock_ != nullptr) {
        plcDock_->show();
        plcDock_->raise();
        resizeDocks({plcDock_}, {520}, Qt::Horizontal);
    }
    if (plc_ != nullptr) {
        plc_->connectOpcUa();
    }
}

void MainWindow::showIntentBar() {
    if (intent_ == nullptr) {
        return;
    }
    positionFloaters();
    if (viewport_ != nullptr) {
        viewport_->setPreviewMode(true);
        viewport_->setGhost(true);
    }
    intent_->openBar();
}

void MainWindow::runUiCommand(const std::string& id) {
    if (id.rfind("object:", 0) == 0) {
        selectNode(QString::fromStdString(id.substr(7)), false);
        return;
    }
    if (id.rfind("signal:", 0) == 0) {
        applyTaskMode(5);
        return;
    }
    if (id == "ui.new") {
        newProject();
    } else if (id == "ui.open") {
        openProjectInteractive();
    } else if (id == "ui.save") {
        saveProjectInteractive();
    } else if (id == "ui.import_step") {
        importStepInteractive();
    } else if (id == "ui.import_obj") {
        importObjInteractive();
    } else if (id == "ui.play" || id == "sim_play") {
        playSimulation();
    } else if (id == "ui.pause" || id == "sim_pause") {
        pauseSimulation();
    } else if (id == "ui.stop" || id == "sim_stop") {
        stopSimulation();
    } else if (id == "ui.reset") {
        resetSimulation();
    } else if (id == "ui.fit" || id == "ui.fit_all") {
        viewport_->fitAll();
    } else if (id == "ui.move") {
        moveAction_->setChecked(true);
        viewport_->setGizmoMode(GizmoMode::Translate);
    } else if (id == "ui.rotate") {
        rotateAction_->setChecked(true);
        viewport_->setGizmoMode(GizmoMode::Rotate);
    } else if (id == "ui.measure") {
        viewport_->setMeasureMode(true);
    } else if (id == "ui.focus") {
        focusMode_ = !focusMode_;
        applyTaskMode(taskMode_);
    } else if (id == "ui.theme") {
        setThemeName(themeName_ == "dark" ? "light" : "dark");
    } else if (id == "ui.mode.model") {
        applyTaskMode(0);
    } else if (id == "ui.mode.kinematics") {
        applyTaskMode(1);
    } else if (id == "ui.mode.logic") {
        applyTaskMode(2);
    } else if (id == "ui.mode.robot") {
        applyTaskMode(3);
    } else if (id == "ui.mode.debug") {
        applyTaskMode(4);
    } else if (id == "ui.mode.commission") {
        applyTaskMode(5);
    } else if (id == "ui.palette") {
        showCommandPalette();
    } else if (id == "ui.intent") {
        showIntentBar();
    } else if (id == "ui.teach") {
        if (!scene_.robotPaths.empty()) {
            auto node = scene_.find(scene_.selectedId());
            PathPoint point;
            point.name = "T" + std::to_string(scene_.robotPaths.front().points.size() + 1);
            point.move = PathMove::Lin;
            if (node) {
                const Mat4 world = node->worldMatrix();
                point.position = {world(3, 0), world(3, 1), world(3, 2)};
            }
            scene_.robotPaths.front().points.push_back(point);
            scene_.notify(ChangeKind::Structure);
            if (pathTimeline_ != nullptr) {
                pathTimeline_->reload();
            }
        }
    } else {
        CommandExecutor executor(scene_, simulation_.get(), physics_.get());
        executor.execute({{"op", id}});
    }
}

bool MainWindow::applyIntent(const std::string& text) {
    const CommandPlan plan = CommandRegistry::planIntent(text);
    if (plan.needsClarify) {
        return false;
    }
    intentPoses_.clear();
    std::vector<std::shared_ptr<SceneNode>> stack{scene_.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        intentPoses_.push_back({node->id, node->local});
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
    }
    intentSignals_ = scene_.signals;
    intentUndoValid_ = true;
    CommandExecutor executor(scene_, simulation_.get(), physics_.get());
    bool any = false;
    for (const PlanStep& step : plan.steps) {
        if (step.risk == "R3" || (step.risk == "R2" && intent_ != nullptr && !intent_->allowsStructuralEdits())) {
            continue;
        }
        nlohmann::json command = step.args;
        command["op"] = step.op;
        if (executor.execute(command).ok) {
            any = true;
        }
    }
    if (viewport_ != nullptr) {
        viewport_->setPreviewMode(false);
        viewport_->setGhost(false);
        viewport_->setSimTime(simulation_ ? simulation_->time() : 0);
    }
    refreshSignals();
    return any;
}

void MainWindow::undoIntent() {
    if (!intentUndoValid_) {
        if (intent_ != nullptr) {
            intent_->note("没有可撤销的意图事务");
        }
        return;
    }
    for (const auto& [id, pose] : intentPoses_) {
        if (auto node = scene_.find(id)) {
            node->local = pose;
        }
    }
    scene_.signals = intentSignals_;
    intentUndoValid_ = false;
    if (viewport_ != nullptr) {
        viewport_->setGhost(false);
        viewport_->setPreviewMode(false);
        viewport_->update();
    }
    if (intent_ != nullptr) {
        intent_->note("已撤销上一次意图事务");
    }
    refreshSignals();
}

void MainWindow::saveWorkspace() {
    QSettings settings;
    settings.setValue("ui/mode", taskMode_);
    settings.setValue("ui/focus", focusMode_);
    settings.setValue("ui/theme", themeName_);
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (!confirmDiscard()) {
        event->ignore();
        return;
    }
    QMainWindow::closeEvent(event);
}

}  // namespace sim
