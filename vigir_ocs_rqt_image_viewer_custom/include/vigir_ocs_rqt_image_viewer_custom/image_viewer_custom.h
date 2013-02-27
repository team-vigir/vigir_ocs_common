#ifndef vigir_ocs_rqt__ImageViewerCustom_H
#define vigir_ocs_rqt__ImageViewerCustom_H

#include <rqt_gui_cpp/plugin.h>
#include "ui/camera_viewer_custom_widget.h"

#include <QDockWidget>

namespace vigir_ocs_rqt_image_viewer_custom {

class ImageViewerCustom
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  ImageViewerCustom();

  ~ImageViewerCustom();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

protected:

  qt_gui_cpp::PluginContext* context_;

  CameraViewerCustomWidget* widget_;
  
};

}

#endif // vigir_ocs_rqt__ImageViewerCustom_H
