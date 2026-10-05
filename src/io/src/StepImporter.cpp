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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <mutex>
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

struct ImportContext {
    Scene& scene;
    Handle(XCAFDoc_ShapeTool) shapes;
    Handle(XCAFDoc_ColorTool) colors;
    double toMillimeters = 1;
    const ImportProgressFn* progress = nullptr;
    std::atomic<bool>* cancel = nullptr;
    std::unordered_map<std::string, std::string> meshCache;
    int tessellated = 0;
    int partBudget = 1;
    int created = 0;
};

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

std::string meshFor(ImportContext& context, const TDF_Label& geometryLabel) {
    const std::string key = labelEntry(geometryLabel);
    const auto cached = context.meshCache.find(key);
    if (cached != context.meshCache.end()) {
        return cached->second;
    }
    TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(geometryLabel);
    auto mesh = tessellate(shape, context.toMillimeters, gp_Trsf());
    ++context.tessellated;
    const int percent = 55 + (40 * context.tessellated) / std::max(context.partBudget, 1);
    report(context, std::min(percent, 95), "progress.tessellate", context.tessellated, context.partBudget);
    if (!mesh) {
        context.meshCache.emplace(key, std::string{});
        return {};
    }
    const std::string id = context.scene.addMesh(std::move(mesh));
    context.meshCache.emplace(key, id);
    return id;
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
        node->meshId = meshFor(context, defined);
        if (!shape.IsNull()) {
            node->faceAxes = collectFaceAxes(shape, context.toMillimeters, gp_Trsf());
        }
        context.scene.addChild(parent, node);
        ++context.created;
        return 1;
    }

    node->kind = NodeKind::Assembly;
    context.scene.addChild(parent, node);
    ++context.created;
    int count = 1;
    int index = 1;
    for (const TopoDS_Shape& solid : solids) {
        if (cancelled(context)) {
            break;
        }
        char suffix[32];
        std::snprintf(suffix, sizeof(suffix), "Solid %03d", index++);
        auto child = context.scene.makeNode(NodeKind::Part, suffix);
        const gp_Trsf placement = solid.Location().Transformation();
        child->meshId = [&] {
            auto mesh = tessellate(solid, context.toMillimeters, placement);
            ++context.tessellated;
            if (!mesh) {
                return std::string{};
            }
            return context.scene.addMesh(std::move(mesh));
        }();
        child->faceAxes = collectFaceAxes(solid, context.toMillimeters, placement);
        child->color = node->color;
        context.scene.addChild(node, child);
        ++context.created;
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

    ImportContext context{scene, shapes, colors, toMillimeters, &progress, cancel, {}, 0, 1, 0};
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
    if (progress) {
        progress(ImportProgress{100, "progress.done", context.partBudget, context.partBudget});
    }
    return finish(true, "imported");
}

}  // namespace sim
