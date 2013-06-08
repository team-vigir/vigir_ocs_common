#ifndef RVIZ_RENDER_PANEL_CUSTOM_H
#define RVIZ_RENDER_PANEL_CUSTOM_H

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
  bool block_mouse_move_;
  Qt::KeyboardModifiers block_mouse_move_key_modifiers_;
  Qt::MouseButtons block_mouse_move_mouse_buttons_;
  bool block_mouse_press_;
  Qt::KeyboardModifiers block_mouse_press_key_modifiers_;
  Qt::MouseButtons block_mouse_press_mouse_buttons_;
  bool block_mouse_release_;
  Qt::KeyboardModifiers block_mouse_release_key_modifiers_;
  Qt::MouseButtons block_mouse_release_mouse_buttons_;
  bool block_mouse_double_click_;
  Qt::KeyboardModifiers block_mouse_double_click_key_modifiers_;
  Qt::MouseButtons block_mouse_double_click_mouse_buttons_;
  bool block_mouse_wheel_;
  Qt::KeyboardModifiers block_mouse_wheel_key_modifiers_;
  Qt::MouseButtons block_mouse_wheel_mouse_buttons_;
  bool block_mouse_leave_;
  Qt::KeyboardModifiers block_mouse_leave_key_modifiers_;
  Qt::MouseButtons block_mouse_leave_mouse_buttons_;
  bool block_mouse_key_press_;
  Qt::KeyboardModifiers block_mouse_key_press_key_modifiers_;
  Qt::MouseButtons block_mouse_key_press_mouse_buttons_;
};

} // namespace rviz

#endif

