#include <image_viewer_custom.h>

#include <pluginlib/class_list_macros.h>

#include <QCloseEvent>
#include <QMenuBar>

namespace rqt {

ImageViewerCustom::ImageViewerCustom()
  : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
{
  setObjectName("ImageViewerCustom");
}

ImageViewerCustom::~ImageViewerCustom()
{
}

void ImageViewerCustom::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context_ = &context;

  widget_ = new ImageViewerCustomWidget();
  context.addWidget(widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);
}

bool ImageViewerCustom::eventFilter(QObject* watched, QEvent* event)
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

PLUGINLIB_EXPORT_CLASS(rqt::ImageViewerCustom, rqt_gui_cpp::Plugin)
