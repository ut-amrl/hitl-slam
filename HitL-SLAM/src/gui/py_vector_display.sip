%Import QtCore/QtCoremod.sip
%Import QtGui/QtGuimod.sip

%Module(name=py_vector_display, version=1)

class PyVectorDisplay {

%TypeHeaderCode
#include "../py_vector_display.h"
%End

public:
  static QWidget* initializeForPythonUI();
  static void AutoLocalize(double x, double y, double theta);
  static void SetInteractionPan();
  static void SetInteractionZoom();
  static void SetInteractionGoto();
  static void SetInteractionLocalize();
  static void SetInteractionClick();
  static void ZoomIn();
  static void ZoomOut();
  static void shutdown();
  static void EnableDrawing();
  static void DisableDrawing();
};
