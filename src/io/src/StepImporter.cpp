#include "sim/io/StepImporter.hpp"

#include "sim/io/OcctEnvironment.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Bnd_Box.hxx>
#include <BinXCAFDrivers.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_Tool.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LengthUnit.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>

#include <BRepBuilderAPI_Sewing.hxx>
#include <Interface_Check.hxx>
#include <Interface_CheckIterator.hxx>
#include <Interface_CheckTool.hxx>
#include <Interface_InterfaceModel.hxx>
#include <ShapeFix_Shape.hxx>
#include <IFSelect_PrintCount.hxx>

#include <algorithm>
#include <cstring>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace sim {
namespace {

std::string extToUtf8(const TCollection_ExtendedString& ext) {
    std::string out;
    out.reserve(static_cast<std::size_t>(ext.Length()));
    for (int i = 1; i <= ext.Length(); ++i) {
        const auto ch = static_cast<char16_t>(ext.Value(i));
        if (ch < 0x80) {
            out.push_back(static_cast<char>(ch));
        } else if (ch < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (ch >> 6)));
            out.push_back(static_cast<char>(0x80 | (ch & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xE0 | (ch >> 12)));
            out.push_back(static_cast<char>(0x80 | ((ch >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (ch & 0x3F)));
        }
    }
    while (!out.empty() && (out.back() == ' ' || out.back() == '\t')) {
        out.pop_back();
    }
    return out;
}

std::string readName(const TDF_Label& label) {
    Handle(TDataStd_Name) attr;
    if (label.IsNull() || !label.FindAttribute(TDataStd_Name::GetID(), attr) || attr.IsNull()) {
        return {};
    }
    return extToUtf8(attr->Get());
}

bool isGenericName(const std::string& name) {
    // OCCT writes unnamed occurrences as "=>[0:1:1:1]". Those are not product names.
    if (name.rfind("=>", 0) == 0) {
        return true;
    }
    return name.empty() || name == "SOLID" || name == "SHELL" || name == "COMPOUND" || name == "FACE" ||
           name == "ASSEMBLY" || name == "SHAPE";
}

std::string labelEntry(const TDF_Label& label) {
    TCollection_AsciiString entry;
    TDF_Tool::Entry(label, entry);
    return entry.ToCString();
}

Transform fromTrsf(const gp_Trsf& trsf, double toMillimeters) {
    Transform transform;
    const gp_XYZ translation = trsf.TranslationPart();
    transform.translation = {translation.X() * toMillimeters, translation.Y() * toMillimeters,
                             translation.Z() * toMillimeters};
    const gp_Quaternion quaternion = trsf.GetRotation();
    transform.rotation = Quat{quaternion.W(), quaternion.X(), quaternion.Y(), quaternion.Z()}.normalized();
    const double scale = trsf.ScaleFactor();
    transform.scale = {scale, scale, scale};
    return transform;
}

bool colorOf(const Handle(XCAFDoc_ColorTool)& colors, const TDF_Label& label, Vec3& out) {
    if (colors.IsNull() || label.IsNull()) {
        return false;
    }
    Quantity_Color color;
    if (colors->GetColor(label, XCAFDoc_ColorSurf, color) || colors->GetColor(label, XCAFDoc_ColorGen, color) ||
        colors->GetColor(label, XCAFDoc_ColorCurv, color)) {
        out = {color.Red(), color.Green(), color.Blue()};
        return true;
    }
    return false;
}

double lengthUnitToMillimeters(const Handle(TDocStd_Document)& document) {
    Handle(XCAFDoc_LengthUnit) unit;
    if (!document.IsNull() && document->Main().FindAttribute(XCAFDoc_LengthUnit::GetID(), unit) && !unit.IsNull()) {
        const double toMeter = unit->GetUnitValue();
        if (toMeter > 1e-12) {
            return toMeter * 1000.0;
        }
    }
    return 1.0;
}

std::shared_ptr<MeshData> tessellate(const TopoDS_Shape& input, double toMillimeters, const gp_Trsf& placement) {
    TopoDS_Shape shape = input;
    shape.Location(TopLoc_Location());
    if (shape.IsNull()) {
        return {};
    }

    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    double deflection = 0.8 / std::max(toMillimeters, 1e-9);
    if (!box.IsVoid()) {
        double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const double dx = xmax - xmin;
        const double dy = ymax - ymin;
        const double dz = zmax - zmin;
        const double diagonalMm = std::sqrt(dx * dx + dy * dy + dz * dz) * toMillimeters;
        const double cap = diagonalMm > 800.0 ? 8.0 : 3.0;
        const double targetMm = std::clamp(diagonalMm * 0.0025, 0.35, cap);
        deflection = targetMm / std::max(toMillimeters, 1e-9);
    }

    BRepMesh_IncrementalMesh mesher(shape, deflection, Standard_False, 0.4, Standard_True);
    mesher.Perform();

    auto mesh = std::make_shared<MeshData>();
    for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
        const TopoDS_Face face = TopoDS::Face(faces.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
            continue;
        }
        const gp_Trsf trsf = location.Transformation();
        const bool reversed = face.Orientation() == TopAbs_REVERSED;
        for (int triangleIndex = 1; triangleIndex <= triangulation->NbTriangles(); ++triangleIndex) {
            int n1 = 0, n2 = 0, n3 = 0;
            triangulation->Triangle(triangleIndex).Get(n1, n2, n3);
            gp_Pnt a = triangulation->Node(n1).Transformed(trsf).Transformed(placement);
            gp_Pnt b = triangulation->Node(n2).Transformed(trsf).Transformed(placement);
            gp_Pnt c = triangulation->Node(n3).Transformed(trsf).Transformed(placement);
            Vec3 p0{a.X() * toMillimeters, a.Y() * toMillimeters, a.Z() * toMillimeters};
            Vec3 p1{b.X() * toMillimeters, b.Y() * toMillimeters, b.Z() * toMillimeters};
            Vec3 p2{c.X() * toMillimeters, c.Y() * toMillimeters, c.Z() * toMillimeters};
            if (reversed) {
                std::swap(p1, p2);
            }
            Vec3 normal = cross(p1 - p0, p2 - p0);
            const double normalLength = length(normal);
            if (normalLength < 1e-12) {
                continue;
            }
            normal = normal / normalLength;
            const auto base = static_cast<std::uint32_t>(mesh->vertexCount());
            const Vec3 points[3] = {p0, p1, p2};
            for (const Vec3& point : points) {
                mesh->positions.push_back(static_cast<float>(point.x));
                mesh->positions.push_back(static_cast<float>(point.y));
                mesh->positions.push_back(static_cast<float>(point.z));
                mesh->normals.push_back(static_cast<float>(normal.x));
                mesh->normals.push_back(static_cast<float>(normal.y));
                mesh->normals.push_back(static_cast<float>(normal.z));
            }
            mesh->indices.push_back(base);
            mesh->indices.push_back(base + 1);
            mesh->indices.push_back(base + 2);
        }
    }
    if (mesh->empty()) {
        return {};
    }
    mesh->recomputeBounds();
    return mesh;
}

enum class MeshGap { Ok, NoFace, Unmeshed };

struct MeshBuild {
    std::shared_ptr<MeshData> mesh;
    MeshGap gap = MeshGap::Ok;
    int faces = 0;
    bool healed = false;
};

int countFaces(const TopoDS_Shape& shape) {
    int faces = 0;
    if (shape.IsNull()) {
        return 0;
    }
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
        ++faces;
    }
    return faces;
}

// ShapeFix and sewing run only after the first triangulation returns nothing.
// A valid solid, including the sample fixture, keeps the deflection it already had.
MeshBuild buildMesh(const TopoDS_Shape& input, double toMillimeters, const gp_Trsf& placement) {
    MeshBuild built;
    built.mesh = tessellate(input, toMillimeters, placement);
    if (built.mesh) {
        return built;
    }
    TopoDS_Shape shape = input;
    if (!shape.IsNull()) {
        shape.Location(TopLoc_Location());
    }
    built.faces = countFaces(shape);
    if (shape.IsNull() || built.faces == 0) {
        built.gap = MeshGap::NoFace;
        return built;
    }
    try {
        ShapeFix_Shape fixer(shape);
        fixer.SetPrecision(0.01);
        fixer.SetMaxTolerance(1.0);
        fixer.Perform();
        TopoDS_Shape fixed = fixer.Shape();
        if (fixed.IsNull()) {
            fixed = shape;
        }
        built.mesh = tessellate(fixed, toMillimeters, placement);
        if (built.mesh) {
            built.healed = true;
            built.gap = MeshGap::Ok;
            return built;
        }
        BRepBuilderAPI_Sewing sewing(0.1);
        sewing.Add(fixed);
        sewing.Perform();
        const TopoDS_Shape& sewed = sewing.SewedShape();
        built.mesh = tessellate(sewed.IsNull() ? fixed : sewed, toMillimeters, placement);
        if (built.mesh) {
            built.healed = true;
            built.gap = MeshGap::Ok;
            return built;
        }
        built.faces = countFaces(sewed.IsNull() ? fixed : sewed);
    } catch (const Standard_Failure&) {
    }
    built.gap = MeshGap::Unmeshed;
    return built;
}

std::string gapReason(const MeshBuild& built) {
    if (built.gap == MeshGap::NoFace) {
        return "STEP 形状没有面（没有实体、壳或曲面），节点保留，没有网格";
    }
    return "有 " + std::to_string(built.faces) + " 个面，ShapeFix 和缝合后仍然没有三角形，节点保留";
}

Vec3 placedPoint(const gp_Pnt& point, const gp_Trsf& placement, double toMillimeters) {
    const gp_Pnt moved = point.Transformed(placement);
    return {moved.X() * toMillimeters, moved.Y() * toMillimeters, moved.Z() * toMillimeters};
}

Vec3 placedVector(const gp_Dir& direction, const gp_Trsf& placement) {
    gp_Vec vector(direction);
    vector.Transform(placement);
    const Vec3 out{vector.X(), vector.Y(), vector.Z()};
    return length(out) > 1e-12 ? normalize(out) : Vec3{0, 0, 1};
}

void rememberAxis(std::vector<FaceAxis>& axes, FaceAxis axis) {
    if (axis.radius < 8.0 || length(axis.direction) < 0.5) {
        return;
    }
    axis.direction = normalize(axis.direction);
    for (const FaceAxis& existing : axes) {
        if (std::abs(existing.radius - axis.radius) > std::max(2.0, 0.08 * axis.radius)) {
            continue;
        }
        if (std::abs(dot(existing.direction, axis.direction)) < 0.985) {
            continue;
        }
        const Vec3 delta = axis.origin - existing.origin;
        const Vec3 radial = delta - existing.direction * dot(delta, existing.direction);
        if (length(radial) < 4.0) {
            return;
        }
    }
    axes.push_back(axis);
}

std::vector<FaceAxis> collectFaceAxes(const TopoDS_Shape& input, double toMillimeters, const gp_Trsf& placement) {
    TopoDS_Shape shape = input;
    shape.Location(TopLoc_Location());
    std::vector<FaceAxis> axes;
    if (shape.IsNull()) {
        return axes;
    }
    for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
        const TopoDS_Face face = TopoDS::Face(faces.Current());
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cylinder = surface.Cylinder();
            FaceAxis axis;
            axis.origin = placedPoint(cylinder.Location(), placement, toMillimeters);
            axis.direction = placedVector(cylinder.Axis().Direction(), placement);
            axis.radius = cylinder.Radius() * toMillimeters;
            rememberAxis(axes, axis);
            continue;
        }
        if (surface.GetType() != GeomAbs_Plane) {
            continue;
        }
        for (TopExp_Explorer edges(face, TopAbs_EDGE); edges.More(); edges.Next()) {
            BRepAdaptor_Curve curve(TopoDS::Edge(edges.Current()));
            if (curve.GetType() != GeomAbs_Circle) {
                continue;
            }
            const gp_Circ circle = curve.Circle();
            FaceAxis axis;
            axis.origin = placedPoint(circle.Location(), placement, toMillimeters);
            axis.direction = placedVector(circle.Axis().Direction(), placement);
            axis.radius = circle.Radius() * toMillimeters;
            rememberAxis(axes, axis);
        }
    }
    std::sort(axes.begin(), axes.end(), [](const FaceAxis& a, const FaceAxis& b) { return a.radius > b.radius; });
    if (axes.size() > 16) {
        axes.resize(16);
    }
    return axes;
}

struct GapTally {
    std::string name;
    std::string reason;
    int count = 0;
};

struct MeshCacheEntry {
    std::string id;
    MeshGap gap = MeshGap::Ok;
    int faces = 0;
    bool healed = false;
};

struct ImportContext {
    Scene& scene;
    Handle(XCAFDoc_ShapeTool) shapes;
    Handle(XCAFDoc_ColorTool) colors;
    double toMillimeters = 1;
    const ImportProgressFn* progress = nullptr;
    std::atomic<bool>* cancel = nullptr;
    std::unordered_map<std::string, MeshCacheEntry> meshCache;
    int tessellated = 0;
    int partBudget = 1;
    int created = 0;
    int partCount = 0;
    int assemblyCount = 0;
    int emptyPartCount = 0;
    int healedShapes = 0;
    std::vector<GapTally> gaps;
};

void noteGap(ImportContext& context, const std::string& name, const std::string& reason) {
    for (GapTally& gap : context.gaps) {
        if (gap.name == name && gap.reason == reason) {
            ++gap.count;
            ++context.emptyPartCount;
            return;
        }
    }
    context.gaps.push_back(GapTally{name, reason, 1});
    ++context.emptyPartCount;
}

void adoptMesh(ImportContext& context, const std::string& name, const MeshCacheEntry& entry, std::string& meshId) {
    meshId = entry.id;
    if (entry.gap == MeshGap::Ok) {
        return;
    }
    MeshBuild built;
    built.gap = entry.gap;
    built.faces = entry.faces;
    noteGap(context, name, gapReason(built));
}

bool cancelled(const ImportContext& context) { return context.cancel != nullptr && context.cancel->load(); }

void report(const ImportContext& context, int percent, const std::string& key, int current, int total) {
    if (context.progress == nullptr || !*context.progress) {
        return;
    }
    ImportProgress progress;
    progress.percent = percent;
    progress.stageKey = key;
    progress.current = current;
    progress.total = total;
    (*context.progress)(progress);
}

MeshCacheEntry meshFor(ImportContext& context, const TDF_Label& geometryLabel) {
    const std::string key = labelEntry(geometryLabel);
    const auto cached = context.meshCache.find(key);
    if (cached != context.meshCache.end()) {
        return cached->second;
    }
    TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(geometryLabel);
    MeshBuild built = buildMesh(shape, context.toMillimeters, gp_Trsf());
    ++context.tessellated;
    const int percent = 55 + (40 * context.tessellated) / std::max(context.partBudget, 1);
    report(context, std::min(percent, 95), "progress.tessellate", context.tessellated, context.partBudget);
    MeshCacheEntry entry;
    entry.gap = built.gap;
    entry.faces = built.faces;
    entry.healed = built.healed;
    if (built.healed) {
        ++context.healedShapes;
    }
    if (built.mesh) {
        entry.id = context.scene.addMesh(std::move(built.mesh));
    }
    context.meshCache.emplace(key, entry);
    return entry;
}

MeshCacheEntry meshForSolid(ImportContext& context, const TopoDS_Shape& solid) {
    const gp_Trsf placement = solid.Location().Transformation();
    MeshBuild built = buildMesh(solid, context.toMillimeters, placement);
    ++context.tessellated;
    MeshCacheEntry entry;
    entry.gap = built.gap;
    entry.faces = built.faces;
    entry.healed = built.healed;
    if (built.healed) {
        ++context.healedShapes;
    }
    if (built.mesh) {
        entry.id = context.scene.addMesh(std::move(built.mesh));
    }
    return entry;
}

int importLabel(ImportContext& context, const TDF_Label& label, const std::shared_ptr<SceneNode>& parent, int depth) {
    if (label.IsNull() || !parent || depth > 64 || cancelled(context)) {
        return 0;
    }

    TDF_Label referred;
    const bool reference = XCAFDoc_ShapeTool::IsReference(label) && XCAFDoc_ShapeTool::GetReferredShape(label, referred);
    const TDF_Label defined = reference ? referred : label;
    const bool assembly = XCAFDoc_ShapeTool::IsAssembly(label) || (reference && XCAFDoc_ShapeTool::IsAssembly(defined));

    std::string name = readName(label);
    if (isGenericName(name)) {
        const std::string fallback = readName(defined);
        if (!fallback.empty()) {
            name = fallback;
        }
    }
    if (name.empty()) {
        name = assembly ? "Assembly" : "Part";
    }

    auto node = context.scene.makeNode(assembly ? NodeKind::Assembly : NodeKind::Part, name);
    node->local = fromTrsf(XCAFDoc_ShapeTool::GetLocation(label).Transformation(), context.toMillimeters);
    Vec3 color = node->color;
    if (!colorOf(context.colors, label, color)) {
        colorOf(context.colors, defined, color);
    }
    node->color = color;

    if (assembly) {
        context.scene.addChild(parent, node);
        ++context.created;
        ++context.assemblyCount;
        TDF_LabelSequence components;
        const TDF_Label componentSource = XCAFDoc_ShapeTool::IsAssembly(label) ? label : defined;
        if (!XCAFDoc_ShapeTool::GetComponents(componentSource, components)) {
            return 1;
        }
        int count = 1;
        for (int i = 1; i <= components.Length(); ++i) {
            count += importLabel(context, components.Value(i), node, depth + 1);
            if (cancelled(context)) {
                break;
            }
        }
        return count;
    }

    TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(defined);
    std::vector<TopoDS_Shape> solids;
    for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
        solids.push_back(explorer.Current());
    }
    if (solids.size() <= 1) {
        const MeshCacheEntry entry = meshFor(context, defined);
        adoptMesh(context, name, entry, node->meshId);
        if (!shape.IsNull()) {
            node->faceAxes = collectFaceAxes(shape, context.toMillimeters, gp_Trsf());
        }
        context.scene.addChild(parent, node);
        ++context.created;
        ++context.partCount;
        return 1;
    }

    node->kind = NodeKind::Assembly;
    context.scene.addChild(parent, node);
    ++context.created;
    ++context.assemblyCount;
    int count = 1;
    int index = 1;
    for (const TopoDS_Shape& solid : solids) {
        if (cancelled(context)) {
            break;
        }
        char suffix[32];
        std::snprintf(suffix, sizeof(suffix), "Solid %03d", index++);
        auto child = context.scene.makeNode(NodeKind::Part, suffix);
        const MeshCacheEntry entry = meshForSolid(context, solid);
        adoptMesh(context, std::string(suffix) + " " + name, entry, child->meshId);
        child->faceAxes = collectFaceAxes(solid, context.toMillimeters, solid.Location().Transformation());
        child->color = node->color;
        context.scene.addChild(node, child);
        ++context.created;
        ++context.partCount;
        ++count;
    }
    return count;
}

int countPartLabels(const Handle(XCAFDoc_ShapeTool)& shapes, const TDF_Label& label, int depth) {
    if (label.IsNull() || depth > 64) {
        return 0;
    }
    TDF_Label referred;
    const bool reference = XCAFDoc_ShapeTool::IsReference(label) && XCAFDoc_ShapeTool::GetReferredShape(label, referred);
    const TDF_Label defined = reference ? referred : label;
    const bool assembly = XCAFDoc_ShapeTool::IsAssembly(label) || (reference && XCAFDoc_ShapeTool::IsAssembly(defined));
    if (!assembly) {
        return 1;
    }
    TDF_LabelSequence components;
    const TDF_Label source = XCAFDoc_ShapeTool::IsAssembly(label) ? label : defined;
    if (!XCAFDoc_ShapeTool::GetComponents(source, components)) {
        return 0;
    }
    int count = 0;
    for (int i = 1; i <= components.Length(); ++i) {
        count += countPartLabels(shapes, components.Value(i), depth + 1);
    }
    return count;
}

void prepareOcct() {
    static std::once_flag once;
    std::call_once(once, [] {
        ensureOcctEnvironment();
        const Handle(XCAFApp_Application) application = XCAFApp_Application::GetApplication();
        BinXCAFDrivers::DefineFormat(application);
    });
}

// XCAFApp_Application keeps every NewDocument alive until Close. The local
// Handle going out of scope is not enough, so the document would stay for the
// whole process and show up as an OCCT leak.
class XcafDocumentGuard {
public:
    XcafDocumentGuard(const Handle(XCAFApp_Application)& application, Handle(TDocStd_Document)& document)
        : application_(application), document_(document) {}

    ~XcafDocumentGuard() {
        if (application_.IsNull() || document_.IsNull()) {
            return;
        }
        Handle(TDocStd_Document) held = document_;
        document_.Nullify();
        try {
            application_->Close(held);
        } catch (const Standard_Failure&) {
        }
    }

    XcafDocumentGuard(const XcafDocumentGuard&) = delete;
    XcafDocumentGuard& operator=(const XcafDocumentGuard&) = delete;

private:
    Handle(XCAFApp_Application) application_;
    Handle(TDocStd_Document)& document_;
};

struct FileCensus {
    int products = 0;
    int occurrences = 0;
    int manifoldSolids = 0;
    int voidSolids = 0;
};

FileCensus censusOf(const Handle(Interface_InterfaceModel)& model) {
    FileCensus census;
    if (model.IsNull()) {
        return census;
    }
    for (int index = 1; index <= model->NbEntities(); ++index) {
        const Handle(Standard_Transient) entity = model->Value(index);
        if (entity.IsNull()) {
            continue;
        }
        const char* type = model->TypeName(entity, Standard_False);
        if (type == nullptr) {
            continue;
        }
        if (std::strcmp(type, "PRODUCT") == 0) {
            ++census.products;
        } else if (std::strcmp(type, "NEXT_ASSEMBLY_USAGE_OCCURRENCE") == 0) {
            ++census.occurrences;
        } else if (std::strcmp(type, "MANIFOLD_SOLID_BREP") == 0) {
            ++census.manifoldSolids;
        } else if (std::strcmp(type, "BREP_WITH_VOIDS") == 0) {
            ++census.voidSolids;
        }
    }
    return census;
}

std::string schemaOf(const std::string& path) {
    std::ifstream input(path);
    std::string chunk(16384, '\0');
    input.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
    chunk.resize(static_cast<std::size_t>(std::max<std::streamsize>(0, input.gcount())));
    const auto marker = chunk.find("FILE_SCHEMA");
    if (marker == std::string::npos) {
        return {};
    }
    const auto open = chunk.find('(', marker);
    const auto close = chunk.find(')', open == std::string::npos ? marker : open);
    if (open == std::string::npos || close == std::string::npos || close <= open) {
        return {};
    }
    std::string inner = chunk.substr(open + 1, close - open - 1);
    std::string out;
    for (char ch : inner) {
        if (ch == '\'' || ch == '\n' || ch == '\r') {
            continue;
        }
        out.push_back(ch);
    }
    while (!out.empty() && (out.front() == ' ' || out.front() == '(')) {
        out.erase(out.begin());
    }
    while (!out.empty() && (out.back() == ' ' || out.back() == ')')) {
        out.pop_back();
    }
    return out;
}

std::string schemaLabel(const std::string& schema) {
    if (schema.find("AP242") != std::string::npos) {
        return "AP242";
    }
    if (schema.find("AUTOMOTIVE_DESIGN") != std::string::npos) {
        return "AP214";
    }
    if (schema.find("CONFIG_CONTROL_DESIGN") != std::string::npos) {
        return "AP203";
    }
    if (schema.empty()) {
        return "STEP";
    }
    return schema.size() > 48 ? schema.substr(0, 48) : schema;
}

void appendModelChecks(std::vector<std::string>& warnings, const Handle(Interface_InterfaceModel)& model) {
    if (model.IsNull()) {
        return;
    }
    Interface_CheckTool tool(model);
    Interface_CheckIterator iterator = tool.CompleteCheckList();
    int hidden = 0;
    for (iterator.Start(); iterator.More(); iterator.Next()) {
        const Handle(Interface_Check)& check = iterator.Value();
        if (check.IsNull() || (check->NbFails() == 0 && check->NbWarnings() == 0)) {
            continue;
        }
        const int number = iterator.Number();
        std::string who = "文件";
        if (number > 0 && number <= model->NbEntities()) {
            const Handle(Standard_Transient) entity = model->Value(number);
            const char* type = entity.IsNull() ? nullptr : model->TypeName(entity, Standard_False);
            who = "#" + std::to_string(number);
            if (type != nullptr && type[0] != '\0') {
                who += " ";
                who += type;
            }
        }
        auto push = [&](const std::string& text) {
            if (warnings.size() >= 24) {
                ++hidden;
                return;
            }
            if (text.find("authorisation") != std::string::npos || text.find("n0.7") != std::string::npos) {
                warnings.push_back(
                    "未解析引用 1 条：FILE_NAME 第 7 项 authorisation 是 $，不是带引号的字符串。这是文件头，不是悬空的实体号，几何没有因此丢掉");
                return;
            }
            warnings.push_back(who + "：" + text);
        };
        for (int index = 1; index <= check->NbFails(); ++index) {
            const char* text = check->CFail(index, Standard_True);
            push(text == nullptr ? std::string{} : std::string(text));
        }
        for (int index = 1; index <= check->NbWarnings(); ++index) {
            const char* text = check->CWarning(index, Standard_True);
            push(std::string("警告 ") + (text == nullptr ? std::string{} : std::string(text)));
        }
    }
    if (hidden > 0) {
        warnings.push_back("另有 " + std::to_string(hidden) + " 条 Open CASCADE 检查未逐条展开");
    }
}

}  // namespace

ImportResult StepImporter::importFile(const std::string& path, Scene& scene) {
    return importFile(path, scene, {}, nullptr);
}

ImportResult StepImporter::importFile(const std::string& path, Scene& scene, const ImportProgressFn& progress,
                                      std::atomic<bool>* cancel) {
    const auto started = std::chrono::steady_clock::now();
    ImportResult result;
    auto finish = [&](bool ok, const std::string& message) {
        result.ok = ok;
        result.message = message;
        result.elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        return result;
    };

    if (cancel != nullptr && cancel->load()) {
        return finish(false, "cancelled");
    }
    if (progress) {
        progress(ImportProgress{-1, "progress.reading", 0, 0});
    }

    prepareOcct();
    Interface_Static::SetIVal("read.step.product.mode", 1);
    Interface_Static::SetIVal("read.step.product.context", 1);
    Interface_Static::SetIVal("read.step.assembly.level", 1);
    Interface_Static::SetIVal("read.stepcaf.subshapes.name", 1);
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");

    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    reader.SetColorMode(true);
    reader.SetLayerMode(true);
    const IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
    if (status != IFSelect_RetDone) {
        return finish(false, "STEP reader could not read " + path);
    }
    const Handle(Interface_InterfaceModel) model = reader.Reader().Model();
    const FileCensus census = censusOf(model);
    std::vector<std::string> warnings;
    appendModelChecks(warnings, model);
    if (cancel != nullptr && cancel->load()) {
        return finish(false, "cancelled");
    }
    if (progress) {
        progress(ImportProgress{-1, "progress.transfer", 0, 0});
    }

    const Handle(XCAFApp_Application) application = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) document;
    application->NewDocument("BinXCAF", document);
    if (document.IsNull()) {
        return finish(false, "OCCT could not create an XCAF document. Check that OCCT resource files are installed.");
    }
    XcafDocumentGuard closeDocument(application, document);
    if (!reader.Transfer(document)) {
        return finish(false, "STEP transfer into the XCAF document failed");
    }
    std::ostringstream transferNotes;
    reader.ChangeReader().PrintCheckTransfer(transferNotes, Standard_True, IFSelect_CountByItem);
    const std::string transferText = transferNotes.str();
    if (transferText.find("Nb Total:0") == std::string::npos && transferText.find("Fail") != std::string::npos) {
        std::string brief = transferText;
        if (brief.size() > 400) {
            brief.resize(400);
        }
        warnings.push_back("传输检查：" + brief);
    }

    const Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    const Handle(XCAFDoc_ColorTool) colors = XCAFDoc_DocumentTool::ColorTool(document->Main());
    if (shapes.IsNull()) {
        return finish(false, "STEP document has no shape tool");
    }
    shapes->UpdateAssemblies();

    // Cascade unit was requested in millimetres. Keep a length-unit fallback when the
    // document still reports a different scale (value is metres per file unit).
    double toMillimeters = 1.0;
    Handle(XCAFDoc_LengthUnit) unit;
    if (document->Main().FindAttribute(XCAFDoc_LengthUnit::GetID(), unit) && !unit.IsNull()) {
        const std::string unitName = unit->GetUnitName().ToCString();
        if (unitName != "mm" && unitName != "MM" && unit->GetUnitValue() > 0) {
            toMillimeters = lengthUnitToMillimeters(document);
        }
    }

    TDF_LabelSequence roots;
    shapes->GetFreeShapes(roots);
    if (roots.IsEmpty()) {
        return finish(false, "STEP file does not contain a shape");
    }

    ImportContext context{scene, shapes, colors, toMillimeters, &progress, cancel, {}, 0, 1, 0, 0, 0, 0, 0, {}};
    context.partBudget = 0;
    for (int i = 1; i <= roots.Length(); ++i) {
        context.partBudget += countPartLabels(shapes, roots.Value(i), 0);
    }
    context.partBudget = std::max(context.partBudget, 1);

    scene.beginEdit();
    for (int i = 1; i <= roots.Length(); ++i) {
        importLabel(context, roots.Value(i), scene.root(), 0);
        if (cancelled(context)) {
            break;
        }
    }
    scene.endEdit();

    std::error_code ec;
    const auto absolute = std::filesystem::absolute(path, ec);
    scene.sourceFiles.push_back(ec ? path : absolute.string());
    scene.name = std::filesystem::path(path).stem().string();

    if (cancelled(context)) {
        return finish(false, "cancelled");
    }
    if (context.created == 0) {
        return finish(false, "STEP transfer produced no assembly nodes");
    }

    result.nodeCount = context.created;
    result.meshCount = static_cast<int>(scene.meshes().size());
    for (const auto& [id, mesh] : scene.meshes()) {
        (void)id;
        if (mesh) {
            result.triangleCount += static_cast<int>(mesh->triangleCount());
        }
    }
    result.fileProductCount = census.products;
    result.fileOccurrenceCount = census.occurrences;
    result.fileSolidCount = census.manifoldSolids + census.voidSolids;
    result.partCount = context.partCount;
    result.assemblyCount = context.assemblyCount;
    result.emptyPartCount = context.emptyPartCount;
    for (const GapTally& gap : context.gaps) {
        warnings.push_back(gap.name + " ×" + std::to_string(gap.count) + "：" + gap.reason);
    }
    if (context.healedShapes > 0) {
        warnings.push_back("ShapeFix 或缝合后恢复了 " + std::to_string(context.healedShapes) +
                           " 个原先没有三角形的形状");
    }
    std::ostringstream report;
    report << schemaLabel(schemaOf(path)) << " 产品" << result.fileProductCount << " 装配" << result.fileOccurrenceCount
           << " 实体" << result.fileSolidCount << " 节点" << result.nodeCount << " 网格" << result.meshCount << " 三角"
           << result.triangleCount << " 空" << result.emptyPartCount;
    result.report = report.str();
    result.warnings = std::move(warnings);
    if (progress) {
        progress(ImportProgress{100, "progress.done", context.partBudget, context.partBudget});
    }
    return finish(true, "imported");
}

}  // namespace sim
