#ifndef vigir_ocs_rqt__MapViewCustom_H
#define vigir_ocs_rqt__MapViewCustom_H

#include <rqt_gui_cpp/plugin.h>
#include "ui/map_view_widget.h"

#include <QDockWidget>

namespace rqt {

class MapViewCustom
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  MapViewCustom();

  ~MapViewCustom();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

protected:

  qt_gui_cpp::PluginContext* context_;

  MapViewWidget* widget_;
  
};

}

#endif // vigir_ocs_rqt__MapViewCustom_H
