#include "sim/app/MainWindow.hpp"

#include "sim/core/Version.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QFont>
#include <QFontDatabase>
#include <QIcon>
#include <QPalette>
#include <QStyleFactory>
#include <QSurfaceFormat>
#include <QTimer>

namespace {

void applyDarkPalette(QApplication& app) {
    app.setStyle(QStyleFactory::create("Fusion"));
    QPalette palette;
    const QColor window(27, 33, 51);
    const QColor base(18, 24, 42);
    const QColor button(42, 51, 80);
    const QColor text(230, 232, 242);
    const QColor highlight(44, 110, 206);
    palette.setColor(QPalette::Window, window);
    palette.setColor(QPalette::WindowText, text);
    palette.setColor(QPalette::Base, base);
    palette.setColor(QPalette::AlternateBase, window);
    palette.setColor(QPalette::ToolTipBase, base);
    palette.setColor(QPalette::ToolTipText, text);
    palette.setColor(QPalette::Text, text);
    palette.setColor(QPalette::Button, button);
    palette.setColor(QPalette::ButtonText, text);
    palette.setColor(QPalette::BrightText, Qt::white);
    palette.setColor(QPalette::Highlight, highlight);
    palette.setColor(QPalette::HighlightedText, Qt::white);
    palette.setColor(QPalette::PlaceholderText, QColor(160, 166, 184));
    app.setPalette(palette);
}

void applyCjkFont(QApplication& app) {
    // HarmonyOS Sans is listed first when installed. It is not bundled with the app.
    const QStringList preferred = {"HarmonyOS Sans SC", "Source Han Sans SC", "Noto Sans CJK SC", "Noto Sans SC",
                                   "Microsoft YaHei UI", "Microsoft YaHei", "PingFang SC", "WenQuanYi Micro Hei"};
    const QStringList families = QFontDatabase::families();
    for (const QString& family : preferred) {
        if (families.contains(family)) {
            QFont font(family);
            font.setPointSize(10);
            app.setFont(font);
            return;
        }
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication application(argc, argv);
    QApplication::setApplicationName(QString::fromLatin1(sim::kAppName));
    QApplication::setOrganizationName(QString::fromLatin1("SimStudio"));
    QApplication::setApplicationVersion(QString::fromLatin1(sim::kVersion));
    QIcon icon;
    icon.addFile(":/branding/logo_256.png");
    icon.addFile(":/branding/logo.png");
    icon.addFile(":/branding/app.ico");
    QApplication::setWindowIcon(icon);
    applyDarkPalette(application);
    applyCjkFont(application);

#ifdef _WIN32
    // A shipped zip keeps the CPython standard library in Lib/ next to the exe.
    // python3xx._pth (written by the packager) takes precedence; PYTHONHOME covers a bare layout.
    const QString appDir = QCoreApplication::applicationDirPath();
    if (QDir(appDir + QStringLiteral("/Lib")).exists()) {
        qputenv("PYTHONHOME", QDir::toNativeSeparators(appDir).toLocal8Bit());
    }
#endif

    QCommandLineParser parser;
    parser.setApplicationDescription("Sim Studio");
    parser.addHelpOption();
    parser.addVersionOption();
    QCommandLineOption importStep("import", "Import a STEP file on startup.", "file");
    QCommandLineOption openProject("open", "Open a .simproj file.", "file");
    QCommandLineOption screenshot("screenshot", "Save a window image and exit.", "png");
    QCommandLineOption language("lang", "UI locale: zh_CN or en.", "locale");
    QCommandLineOption play("play", "Start the simulation after opening a project.");
    QCommandLineOption simSeconds("sim-seconds", "Seconds to simulate before a screenshot.", "seconds", "2.5");
    QCommandLineOption demo("demo", "Built-in demo: turntable, scara, collision, transfer, kr420, or kr420track.", "name");
    QCommandLineOption mode("mode", "Task mode: model, kinematics, logic, robot, debug, commission.", "name");
    QCommandLineOption theme("theme", "Theme: dark or light.", "name");
    QCommandLineOption palette("palette", "Open the command palette before a screenshot.");
    QCommandLineOption intent("intent", "Open the AI intent bar before a screenshot.");
    parser.addOption(importStep);
    parser.addOption(openProject);
    parser.addOption(screenshot);
    parser.addOption(language);
    parser.addOption(play);
    parser.addOption(simSeconds);
    parser.addOption(demo);
    parser.addOption(mode);
    parser.addOption(theme);
    parser.addOption(palette);
    parser.addOption(intent);
    parser.process(application);

    sim::MainWindow window(parser.value(language));
    window.setWindowIcon(icon);
    window.show();
    if (parser.isSet(theme)) {
        window.setThemeName(parser.value(theme));
    }
    if (parser.isSet(demo)) {
        window.loadDemo(parser.value(demo));
    } else if (parser.isSet(openProject)) {
        window.openProject(parser.value(openProject));
    }
    if (parser.isSet(screenshot)) {
        window.setScreenshotPath(parser.value(screenshot));
    }
    if (parser.isSet(simSeconds)) {
        window.setHeadlessSeconds(parser.value(simSeconds).toDouble());
    }
    if (parser.isSet(mode)) {
        window.setTaskModeName(parser.value(mode));
    }
    if (parser.isSet(palette)) {
        window.showCommandPalette();
    }
    if (parser.isSet(intent)) {
        window.showIntentBar();
    }
    if (parser.isSet(importStep)) {
        window.importStepAsync(parser.value(importStep));
    } else if (parser.isSet(play)) {
        window.beginHeadlessPlayback();
    } else if (parser.isSet(screenshot)) {
        QTimer::singleShot(400, &window, [&window] { window.captureScreenshot(); });
    }
    return application.exec();
}
