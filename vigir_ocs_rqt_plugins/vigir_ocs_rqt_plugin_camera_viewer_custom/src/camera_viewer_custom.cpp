#include <camera_viewer_custom.h>

#include <pluginlib/class_list_macros.h>

#include <QCloseEvent>
#include <QMenuBar>

namespace rqt {

CameraViewerCustom::CameraViewerCustom()
  : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
{
  setObjectName("CameraViewerCustom");
}

CameraViewerCustom::~CameraViewerCustom()
{
}

void CameraViewerCustom::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context_ = &context;

  widget_ = new CameraViewerCustomWidget();
  context.addWidget(widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);
}

bool CameraViewerCustom::eventFilter(QObject* watched, QEvent* event)
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

PLUGINLIB_EXPORT_CLASS(rqt::CameraViewerCustom, rqt_gui_cpp::Plugin)
