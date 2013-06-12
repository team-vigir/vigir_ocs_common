#ifndef RVIZ_RENDER_PANEL_CUSTOM_H
#define RVIZ_RENDER_PANEL_CUSTOM_H

#include <ros/ros.h>
#include "rviz/render_panel.h"

namespace rviz
{

/**
 * A widget which shows an OGRE-rendered scene in RViz.
 *
 * RenderPanelCustom displays a scene and forwards mouse and key events to
 * the DisplayContext (which further forwards them to the active
 * Tool, etc.)
 */
class RenderPanelCustom: public RenderPanel
{
Q_OBJECT
public:
  /** Constructor.  Ogre::Root::createRenderWindow() is called within. */
  RenderPanelCustom( QWidget* parent = 0 );
  virtual ~RenderPanelCustom();

  //Ogre::Camera* getCamera() {return customCamera;};

  void setEventFilters(int function, bool block, Qt::KeyboardModifiers block_key_mod=Qt::NoModifier, Qt::MouseButtons block_mouse_buttons=Qt::NoButton);

  static enum
  {
      MOUSE_MOVE_EVENT=0,
      MOUSE_PRESS_EVENT=1,
      MOUSE_RELEASE_EVENT=2,
      MOUSE_DOUBLE_CLICK_EVENT=3,
      MOUSE_WHEEL_EVENT=4,
      MOUSE_LEAVE_EVENT=5,
      KEY_PRESS_EVENT=6
  } Events;

Q_SIGNALS:
  void signalMouseMoveEvent( QMouseEvent* event );
  void signalMousePressEvent( QMouseEvent* event );
  void signalMouseReleaseEvent( QMouseEvent* event );
  void signalMouseDoubleClickEvent( QMouseEvent* event );
  void signalMouseWheelEvent( QWheelEvent* event );
  void signalMouseLeaveEvent ( QEvent * event );
  void signalKeyPressEvent( QKeyEvent* event );

protected:
  // Override from QWidget
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void mouseDoubleClickEvent( QMouseEvent* event );
  virtual void wheelEvent( QWheelEvent* event );
  virtual void leaveEvent ( QEvent * event );
  virtual void keyPressEvent( QKeyEvent* event );

private:
  //Ogre::Camera customCamera;

  // define what will be sent over to rviz, gives total flexibility
  typedef struct {
      bool block;
      Qt::KeyboardModifiers block_key_modifiers;
      Qt::MouseButtons block_mouse_buttons;
  } BlockConfig;
  std::map<int,BlockConfig> block_config_;
};

} // namespace rviz

#endif

