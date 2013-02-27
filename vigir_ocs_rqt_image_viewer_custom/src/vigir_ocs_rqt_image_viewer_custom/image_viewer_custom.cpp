#include <vigir_ocs_rqt_image_viewer_custom/image_viewer_custom.h>

#include <pluginlib/class_list_macros.h>

#include <QCloseEvent>
#include <QMenuBar>

namespace vigir_ocs_rqt_image_viewer_custom {

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

  widget_ = new CameraViewerCustomWidget();

  /*widget_->setWindowTitle("CameraViewerCustom");
  if (context.serialNumber() != 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }*/
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

PLUGINLIB_EXPORT_CLASS(vigir_ocs_rqt_image_viewer_custom::ImageViewerCustom, rqt_gui_cpp::Plugin)
