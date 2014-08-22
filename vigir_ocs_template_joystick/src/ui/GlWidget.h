#ifndef GLWIDGET_H
#define GLWIDGET_H

#endif // GLWIDGET_H
#include <QtOpenGL>
#include <QDebug>

class GlWidget : public QGLWidget
{
  Q_OBJECT

  public:
    GlWidget(QWidget *parent = 0);
    ~GlWidget();
   // QSize sizeHint() const;
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);

  protected:
    void initializeGL();
    void resizeGL(int width,int height);
    void paintGL();
//    void mousePressEvent(QMouseEvent * event);
//    void mouseMoveEvent(QMouseEvent * event);
//    void wheelEvent(QWheelEvent * event);

  private:
    QMatrix4x4 pMatrix;
    QGLShaderProgram shaderProgram;
    QVector<QVector3D> vertices;
    double alpha;
    double beta;
    double distance;
    QPoint lastMousePosition;
    QVector<QVector3D> colors;

    void processMultipleKeys();

    //xyz for moving/rotating mMatrix
    double newX;
    double newY;
    double newZ;

    //handle multiple keys at once
    QSet<Qt::Key> keysPressed;
};
