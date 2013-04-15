#ifndef vigir_ocs_rqt__JoystickCustom_H
#define vigir_ocs_rqt__JoystickCustom_H

#include <rqt_gui_cpp/plugin.h>
#include "ui/joystick_widget.h"

#include <QDockWidget>

namespace rqt {

class JoystickCustom
        : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:

    JoystickCustom();

    ~JoystickCustom();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    virtual bool eventFilter(QObject* watched, QEvent* event);

protected:

    qt_gui_cpp::PluginContext* context_;

    JoystickWidget* widget_;
};

}

#endif // vigir_ocs_rqt__JoystickCustom_H
