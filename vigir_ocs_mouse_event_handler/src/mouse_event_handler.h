#ifndef RVIZ_MOUSE_EVENT_HANDLER_H
#define RVIZ_MOUSE_EVENT_HANDLER_H

#include <QApplication>
#include <QMouseEvent>
#include <QWidget>

namespace vigir_ocs
{

/**
 * a handler for mouse events in general
 * is pa
 */
class MouseEventHandler : public QObject
{
Q_OBJECT
public:
  MouseEventHandler( QObject* parent = 0 );
  virtual ~MouseEventHandler();

Q_SIGNALS:
  void mouseLeftButton( bool, int, int );
  void mouseLeftButtonCtrl( bool, int, int );
  void mouseLeftButtonShift( bool, int, int );
  void mouseRightButton( bool, int, int );

public Q_SLOTS:
  void mousePressEvent( QMouseEvent* event );
  void mouseReleaseEvent( QMouseEvent* event );

private:
  int xo, yo;
};

} // namespace rviz

#endif

