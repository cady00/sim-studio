#include "sim/io/OcctEnvironment.hpp"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BinXCAFDrivers.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string path = argc > 1 ? argv[1] : "samples/step/sample_fixture.step";
    sim::ensureOcctEnvironment();
    Handle(XCAFApp_Application) application = XCAFApp_Application::GetApplication();
    BinXCAFDrivers::DefineFormat(application);
    Handle(TDocStd_Document) document;
    application->NewDocument("BinXCAF", document);
    if (document.IsNull()) {
        std::cerr << "failed to create XCAF document\n";
        return 1;
    }

    Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    Handle(XCAFDoc_ColorTool) colors = XCAFDoc_DocumentTool::ColorTool(document->Main());
    TopoDS_Shape base = BRepPrimAPI_MakeBox(120.0, 80.0, 12.0).Shape();
    TopoDS_Shape pin = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp::DZ()), 6.0, 25.0).Shape();
    TDF_Label baseLabel = shapes->AddShape(base, false);
    TDF_Label pinLabel = shapes->AddShape(pin, false);
    TDataStd_Name::Set(baseLabel, "BasePlate");
    TDataStd_Name::Set(pinLabel, "Pin");
    colors->SetColor(baseLabel, Quantity_Color(0.17, 0.22, 0.40, Quantity_TOC_RGB), XCAFDoc_ColorGen);
    colors->SetColor(pinLabel, Quantity_Color(0.57, 0.58, 0.95, Quantity_TOC_RGB), XCAFDoc_ColorGen);

    TDF_Label assembly = shapes->NewShape();
    TDataStd_Name::Set(assembly, "Fixture");
    shapes->AddComponent(assembly, baseLabel, TopLoc_Location());
    gp_Trsf pinA;
    pinA.SetTranslation(gp_Vec(30, 25, 12));
    TDF_Label pinALabel = shapes->AddComponent(assembly, pinLabel, TopLoc_Location(pinA));
    TDataStd_Name::Set(pinALabel, "Pin_A");
    gp_Trsf pinB;
    pinB.SetTranslation(gp_Vec(90, 55, 12));
    TDF_Label pinBLabel = shapes->AddComponent(assembly, pinLabel, TopLoc_Location(pinB));
    TDataStd_Name::Set(pinBLabel, "Pin_B");
    shapes->UpdateAssemblies();

    Interface_Static::SetCVal("write.step.schema", "AP214");
    Interface_Static::SetCVal("write.step.unit", "MM");
    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    writer.SetColorMode(true);
    if (!writer.Transfer(document)) {
        std::cerr << "STEP transfer failed\n";
        return 1;
    }
    if (writer.Write(path.c_str()) != IFSelect_RetDone) {
        std::cerr << "STEP write failed: " << path << "\n";
        return 1;
    }
    std::cout << "wrote " << path << "\n";
    return 0;
}
