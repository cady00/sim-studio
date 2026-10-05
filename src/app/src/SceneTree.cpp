#include "sim/app/SceneTree.hpp"

namespace sim {
namespace {

QString nodeTitle(const SceneNode& node, const QString& worldTitle) {
    if (node.id == "root" && node.name == "World") {
        return worldTitle;
    }
    return QString::fromUtf8(node.name.c_str());
}

}  // namespace

SceneTreeModel::SceneTreeModel(Scene& scene, QObject* parent) : QAbstractItemModel(parent), scene_(scene) {}

void SceneTreeModel::reload() {
    beginResetModel();
    matches_.clear();
    if (!filter_.isEmpty()) {
        scene_.collectNames(filter_.toUtf8().toStdString(), matches_, 5000);
    }
    endResetModel();
}

void SceneTreeModel::setFilter(const QString& text) {
    filter_ = text.trimmed();
    reload();
}

void SceneTreeModel::setHeaders(const QString& name, const QString& kind) {
    nameHeader_ = name;
    kindHeader_ = kind;
    emit headerDataChanged(Qt::Horizontal, 0, 1);
}

void SceneTreeModel::setWorldTitle(const QString& title) {
    worldTitle_ = title;
}

QModelIndex SceneTreeModel::index(int row, int column, const QModelIndex& parent) const {
    if (row < 0 || column < 0 || column > 1 || scene_.root() == nullptr) {
        return {};
    }
    if (!filter_.isEmpty()) {
        if (parent.isValid() || row >= static_cast<int>(matches_.size())) {
            return {};
        }
        auto node = scene_.find(matches_[static_cast<std::size_t>(row)]);
        if (!node) {
            return {};
        }
        return createIndex(row, column, node.get());
    }
    if (!parent.isValid()) {
        return row == 0 ? createIndex(0, column, scene_.root().get()) : QModelIndex{};
    }
    SceneNode* parentNode = nodeAt(parent);
    if (parentNode == nullptr || row >= static_cast<int>(parentNode->children.size())) {
        return {};
    }
    return createIndex(row, column, parentNode->children[static_cast<std::size_t>(row)].get());
}

QModelIndex SceneTreeModel::parent(const QModelIndex& child) const {
    if (!child.isValid() || !filter_.isEmpty()) {
        return {};
    }
    SceneNode* node = nodeAt(child);
    if (node == nullptr || node->id == "root") {
        return {};
    }
    auto parentNode = node->parent.lock();
    if (!parentNode) {
        return {};
    }
    return createIndex(rowInParent(parentNode.get()), 0, parentNode.get());
}

int SceneTreeModel::rowCount(const QModelIndex& parent) const {
    if (!filter_.isEmpty()) {
        return parent.isValid() ? 0 : static_cast<int>(matches_.size());
    }
    if (!parent.isValid()) {
        return scene_.root() ? 1 : 0;
    }
    SceneNode* node = nodeAt(parent);
    return node == nullptr ? 0 : static_cast<int>(node->children.size());
}

int SceneTreeModel::columnCount(const QModelIndex&) const { return 2; }

QVariant SceneTreeModel::data(const QModelIndex& index, int role) const {
    SceneNode* node = nodeAt(index);
    if (node == nullptr || (role != Qt::DisplayRole && role != Qt::UserRole)) {
        return {};
    }
    if (role == Qt::UserRole) {
        return QString::fromStdString(node->id);
    }
    if (index.column() == 1) {
        if (node->mech.kind != MechKind::None) {
            return QString::fromLatin1(mechKindName(node->mech.kind));
        }
        return node->kind == NodeKind::Assembly ? QStringLiteral("assembly") : QStringLiteral("part");
    }
    return nodeTitle(*node, worldTitle_);
}

QVariant SceneTreeModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
        return {};
    }
    if (section == 0) {
        return nameHeader_.isEmpty() ? QStringLiteral("Name") : nameHeader_;
    }
    return kindHeader_.isEmpty() ? QStringLiteral("Kind") : kindHeader_;
}

QModelIndex SceneTreeModel::indexFor(const std::string& id) const {
    auto node = scene_.find(id);
    if (!node) {
        return {};
    }
    if (node->id == "root") {
        return index(0, 0, {});
    }
    if (!filter_.isEmpty()) {
        for (int row = 0; row < static_cast<int>(matches_.size()); ++row) {
            if (matches_[static_cast<std::size_t>(row)] == id) {
                return index(row, 0, {});
            }
        }
        return {};
    }
    std::vector<SceneNode*> chain;
    for (SceneNode* cursor = node.get(); cursor != nullptr && cursor->id != "root";) {
        chain.push_back(cursor);
        auto parentNode = cursor->parent.lock();
        cursor = parentNode.get();
    }
    QModelIndex current;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        current = index(rowInParent(*it), 0, current);
    }
    return current;
}

std::string SceneTreeModel::nodeId(const QModelIndex& index) const {
    SceneNode* node = nodeAt(index);
    return node == nullptr ? std::string{} : node->id;
}

SceneNode* SceneTreeModel::nodeAt(const QModelIndex& index) const {
    return index.isValid() ? static_cast<SceneNode*>(index.internalPointer()) : nullptr;
}

int SceneTreeModel::rowInParent(const SceneNode* node) const {
    if (node == nullptr) {
        return 0;
    }
    auto parentNode = node->parent.lock();
    if (!parentNode) {
        return 0;
    }
    for (int row = 0; row < static_cast<int>(parentNode->children.size()); ++row) {
        if (parentNode->children[static_cast<std::size_t>(row)].get() == node) {
            return row;
        }
    }
    return 0;
}

}  // namespace sim
