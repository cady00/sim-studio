#pragma once

#include "sim/core/Scene.hpp"

#include <QAbstractItemModel>
#include <QString>

#include <vector>

namespace sim {

// Rows are scene nodes. Qt asks only for the rows on screen, so a flat list of
// 100000 parts does not allocate a widget item per part.
class SceneTreeModel : public QAbstractItemModel {
    Q_OBJECT

public:
    explicit SceneTreeModel(Scene& scene, QObject* parent = nullptr);

    void reload();
    void setFilter(const QString& text);
    void setHeaders(const QString& name, const QString& kind);
    void setWorldTitle(const QString& title);
    QModelIndex indexFor(const std::string& id) const;
    std::string nodeId(const QModelIndex& index) const;

    QModelIndex index(int row, int column, const QModelIndex& parent) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    int rowCount(const QModelIndex& parent) const override;
    int columnCount(const QModelIndex& parent) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

private:
    SceneNode* nodeAt(const QModelIndex& index) const;
    int rowInParent(const SceneNode* node) const;

    Scene& scene_;
    QString filter_;
    QString nameHeader_;
    QString kindHeader_;
    QString worldTitle_ = QStringLiteral("World");
    std::vector<std::string> matches_;
};

}  // namespace sim
