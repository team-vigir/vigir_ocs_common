#ifndef vigir_ocs_rqt__ImageViewerCustom_H
#define vigir_ocs_rqt__ImageViewerCustom_H

#include <rqt_gui_cpp/plugin.h>
#include "ui/image_viewer_custom_widget.h"

#include <QDockWidget>

namespace rqt {

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

  ImageViewerCustomWidget* widget_;
  
};

}

#endif // vigir_ocs_rqt__ImageViewerCustom_H
