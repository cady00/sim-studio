#pragma once

#include "sim/io/RobotLibrary.hpp"

#include <QDialog>

#include <functional>
#include <vector>

class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;

namespace sim {

// Browses a local Process Simulate library. The chosen directory is kept in
// QSettings on this machine and is not written into the project file.
class RobotLibraryDialog : public QDialog {
    Q_OBJECT

public:
    using Translate = std::function<QString(const char*)>;
    using Importer = std::function<QString(const std::vector<RobotLibraryEntry>&)>;

    explicit RobotLibraryDialog(Translate translate, QWidget* parent = nullptr);

    void setImporter(Importer importer);
    void setRoot(const QString& path, bool persist);
    void setFilter(const QString& text);
    void selectName(const QString& name);
    QString root() const;

protected:
    void showEvent(QShowEvent* event) override;

private:
    void browse();
    void rescan();
    void refill();
    void showPreview();
    void importSelected();

    Translate translate_;
    Importer importer_;
    std::vector<RobotLibraryEntry> entries_;
    QLineEdit* pathEdit_ = nullptr;
    QLineEdit* filterEdit_ = nullptr;
    QComboBox* brandBox_ = nullptr;
    QListWidget* list_ = nullptr;
    QLabel* preview_ = nullptr;
    QLabel* status_ = nullptr;
};

}  // namespace sim
