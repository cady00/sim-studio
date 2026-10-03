#include "sim/app/MainWindow.hpp"

#include "sim/core/Version.hpp"

#include <QApplication>
#include <QCommandLineParser>
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
    const QColor highlight(108, 112, 216);
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
    const QStringList preferred = {"Noto Sans CJK SC", "Noto Sans SC", "Source Han Sans SC", "Microsoft YaHei UI",
                                   "Microsoft YaHei", "PingFang SC", "WenQuanYi Micro Hei"};
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
    icon.addFile(":/branding/logo.png");
    icon.addFile(":/branding/app.ico");
    QApplication::setWindowIcon(icon);
    applyDarkPalette(application);
    applyCjkFont(application);

    QCommandLineParser parser;
    parser.setApplicationDescription("Sim Studio phase 1");
    parser.addHelpOption();
    parser.addVersionOption();
    QCommandLineOption importStep("import", "Import a STEP file on startup.", "file");
    QCommandLineOption openProject("open", "Open a .simproj file.", "file");
    QCommandLineOption screenshot("screenshot", "Save a window image and exit.", "png");
    QCommandLineOption language("lang", "UI locale: zh_CN or en.", "locale");
    parser.addOption(importStep);
    parser.addOption(openProject);
    parser.addOption(screenshot);
    parser.addOption(language);
    parser.process(application);

    sim::MainWindow window(parser.value(language));
    window.show();
    if (parser.isSet(openProject)) {
        window.openProject(parser.value(openProject));
    }
    if (parser.isSet(screenshot)) {
        window.setScreenshotPath(parser.value(screenshot));
    }
    if (parser.isSet(importStep)) {
        window.importStepAsync(parser.value(importStep));
    } else if (parser.isSet(screenshot)) {
        QTimer::singleShot(400, &window, [&window] { window.captureScreenshot(); });
    }
    return application.exec();
}
