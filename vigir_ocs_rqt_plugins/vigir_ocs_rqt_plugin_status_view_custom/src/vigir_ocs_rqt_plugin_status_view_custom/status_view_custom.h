#ifndef vigir_ocs_rqt__StatusViewCustom_H
#define vigir_ocs_rqt__StatusViewCustom_H

#include <rqt_gui_cpp/plugin.h>
#include "status_window.h"

#include <QDockWidget>

namespace rqt {

class StatusViewCustom
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  StatusViewCustom();

  ~StatusViewCustom();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

protected:

  qt_gui_cpp::PluginContext* context_;

  status_window* widget_;
  
};

}

#endif // vigir_ocs_rqt__StatusViewCustom_H
