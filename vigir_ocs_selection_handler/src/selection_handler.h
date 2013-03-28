#ifndef RVIZ_SELECTION_HANDLER_H
#define RVIZ_SELECTION_HANDLER_H

#include <QApplication>
#include <QMouseEvent>
#include <QWidget>

namespace vigir_ocs
{

/**
 * a handler for mouse-based selection
 */
class SelectionHandler : public QObject
{
Q_OBJECT
public:
  SelectionHandler( QObject* parent = 0 );
  virtual ~SelectionHandler();

Q_SIGNALS:
  void select( int, int );
  void selectROI( int, int, int, int );

public Q_SLOTS:
  void mousePressEvent( QMouseEvent* event );

private:
  int xo, yo;
};

} // namespace rviz

#endif

