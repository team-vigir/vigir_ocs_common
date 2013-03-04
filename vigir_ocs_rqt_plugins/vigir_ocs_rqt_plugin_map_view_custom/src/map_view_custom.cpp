#include <map_view_custom.h>

#include <pluginlib/class_list_macros.h>

#include <QCloseEvent>
#include <QMenuBar>

namespace rqt {

MapViewCustom::MapViewCustom()
  : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
{
  setObjectName("MapViewCustom");
}

MapViewCustom::~MapViewCustom()
{
}

void MapViewCustom::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context_ = &context;

  widget_ = new MapViewWidget();
  context.addWidget(widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);
}

bool MapViewCustom::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == widget_ && event->type() == QEvent::Close)
  {
    event->ignore();
    context_->closePlugin();
    return true;
  }

  return QObject::eventFilter(watched, event);
}

}

PLUGINLIB_EXPORT_CLASS(rqt::MapViewCustom, rqt_gui_cpp::Plugin)
