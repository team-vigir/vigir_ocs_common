#ifndef vigir_ocs_rqt__Main3DViewCustom_H
#define vigir_ocs_rqt__Main3DViewCustom_H

#include <rqt_gui_cpp/plugin.h>
#include "ui/main_3d_view_widget.h"

#include <QDockWidget>

namespace rqt {

class Main3DViewCustom
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  Main3DViewCustom();

  ~Main3DViewCustom();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

protected:

  qt_gui_cpp::PluginContext* context_;

  Main3DViewWidget* widget_;
  
};

}

#endif // vigir_ocs_rqt__Main3DViewCustom_H
