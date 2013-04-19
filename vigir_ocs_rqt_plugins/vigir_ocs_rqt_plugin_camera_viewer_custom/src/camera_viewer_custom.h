#ifndef vigir_ocs_rqt__CameraViewerCustom_H
#define vigir_ocs_rqt__CameraViewerCustom_H

#include <rqt_gui_cpp/plugin.h>
#include "ui/camera_viewer_custom_widget.h"

#include <QDockWidget>

namespace rqt {

class CameraViewerCustom
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  CameraViewerCustom();

  ~CameraViewerCustom();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

protected:

  qt_gui_cpp::PluginContext* context_;

  CameraViewerCustomWidget* widget_;
  
};

}

#endif // vigir_ocs_rqt__CameraViewerCustom_H
