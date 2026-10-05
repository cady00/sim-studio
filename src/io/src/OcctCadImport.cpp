#include "sim/io/AssetImport.hpp"

#include "sim/io/OcctEnvironment.hpp"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Reader.hxx>
#include <IGESControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>

namespace sim {
namespace {

std::shared_ptr<MeshData> tessellate(const TopoDS_Shape& input) {
    if (input.IsNull()) {
        return {};
    }
    TopoDS_Shape shape = input;
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    double deflection = 0.8;
    if (!box.IsVoid()) {
        double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const double diagonal = std::sqrt((xmax - xmin) * (xmax - xmin) + (ymax - ymin) * (ymax - ymin) +
                                          (zmax - zmin) * (zmax - zmin));
        deflection = std::clamp(diagonal * 0.01, 0.2, 8.0);
    }
    BRepMesh_IncrementalMesh mesher(shape, deflection, Standard_False, 0.5, Standard_True);
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
            gp_Pnt a = triangulation->Node(n1).Transformed(trsf);
            gp_Pnt b = triangulation->Node(n2).Transformed(trsf);
            gp_Pnt c = triangulation->Node(n3).Transformed(trsf);
            Vec3 p0{a.X(), a.Y(), a.Z()};
            Vec3 p1{b.X(), b.Y(), b.Z()};
            Vec3 p2{c.X(), c.Y(), c.Z()};
            if (reversed) {
                std::swap(p1, p2);
            }
            Vec3 normal = cross(p1 - p0, p2 - p0);
            const double normalLength = length(normal);
            if (normalLength < 1e-9) {
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

AssetImportResult adoptShape(const TopoDS_Shape& shape, Scene& scene, const std::string& name, const char* format) {
    AssetImportResult result;
    result.format = format;
    auto mesh = tessellate(shape);
    if (!mesh) {
        result.message = std::string(format) + " produced an empty mesh";
        return result;
    }
    scene.beginEdit();
    auto node = scene.makeNode(NodeKind::Part, name);
    node->meshId = scene.addMesh(mesh);
    node->color = {0.62, 0.66, 0.72};
    if (scene.root()) {
        scene.addChild(scene.root(), node);
    }
    scene.endEdit();
    result.ok = true;
    result.nodeCount = 1;
    result.meshCount = 1;
    result.triangleCount = static_cast<int>(mesh->triangleCount());
    result.message = "imported";
    return result;
}

class IgesImporter final : public IAssetImporter {
public:
    const char* format() const override { return "iges"; }
    std::vector<std::string> extensions() const override { return {"igs", "iges"}; }
    bool probeContent(const std::string& head) const override {
        return head.size() >= 80 && head[72] == 'S';
    }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        AssetImportResult result;
        result.format = format();
        ensureOcctEnvironment();
        IGESControl_Reader reader;
        if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) {
            result.message = "iges was not read";
            return result;
        }
        reader.TransferRoots();
        return adoptShape(reader.OneShape(), scene, "iges", format());
    }
};

class BrepImporter final : public IAssetImporter {
public:
    const char* format() const override { return "brep"; }
    std::vector<std::string> extensions() const override { return {"brep"}; }
    bool probeContent(const std::string& head) const override {
        return head.find("CASCADE Topology") != std::string::npos || head.find("DBRep_DrawableShape") != std::string::npos;
    }
    AssetImportResult importFile(const std::string& path, Scene& scene) const override {
        AssetImportResult result;
        result.format = format();
        ensureOcctEnvironment();
        TopoDS_Shape shape;
        BRep_Builder builder;
        if (!BRepTools::Read(shape, path.c_str(), builder) || shape.IsNull()) {
            result.message = "brep was not read";
            return result;
        }
        return adoptShape(shape, scene, "brep", format());
    }
};

}  // namespace

void registerOcctAssetImporters(AssetRegistry& registry) {
    registry.add(std::make_unique<IgesImporter>());
    registry.add(std::make_unique<BrepImporter>());
}

bool writeSampleShape(const std::string& format, const std::string& path, std::string& error) {
    ensureOcctEnvironment();
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    if (format == "step" || format == "stp") {
        Interface_Static::SetCVal("write.step.unit", "MM");
        STEPControl_Writer writer;
        if (writer.Transfer(box, STEPControl_AsIs) != IFSelect_RetDone || writer.Write(path.c_str()) != IFSelect_RetDone) {
            error = "step sample was not written";
            return false;
        }
        error.clear();
        return true;
    }
    if (format == "iges" || format == "igs") {
        IGESControl_Writer writer;
        writer.AddShape(box);
        writer.ComputeModel();
        if (!writer.Write(path.c_str())) {
            error = "iges sample was not written";
            return false;
        }
        error.clear();
        return true;
    }
    if (format == "brep") {
        if (!BRepTools::Write(box, path.c_str())) {
            error = "brep sample was not written";
            return false;
        }
        error.clear();
        return true;
    }
    error = "sample format is not step, iges, or brep";
    return false;
}

}  // namespace sim
