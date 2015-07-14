/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
@TODO_ADD_AUTHOR_INFO
#include "GlWidget.h"


GlWidget::GlWidget(QWidget * parent)
    : QGLWidget(QGLFormat(/*format options */),parent)
{
    alpha = 0;
    beta = 0;
    distance = 2.5;

    newX = 1;
    newY = 1;
    newZ = 1;


}

GlWidget:: ~GlWidget()
{

}

//QSize QWidget::sizeHint() const
//{
//    return QSize(640, 480);
//}

void GlWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    qglClearColor(QColor(Qt::black));

    shaderProgram.addShaderFromSourceFile(QGLShader::Vertex, "vertexShader.vert");
    shaderProgram.addShaderFromSourceFile(QGLShader::Fragment, "fragmentShader.frag");
    shaderProgram.link();

    vertices << QVector3D(-0.5,-0.5,0.5) << QVector3D(0.5,-0.5,0.5) << QVector3D(0.5,0.5,0.5)
             << QVector3D(0.5,0.5,0.5) << QVector3D(-0.5,0.5,0.5)<< QVector3D(-0.5,-0.5,0.5)
             << QVector3D(0.5,-0.5,-0.5) << QVector3D(-0.5,-0.5,-0.5)<< QVector3D(-0.5,0.5,-0.5)
             << QVector3D(-0.5,0.5,-0.5) << QVector3D(0.5,0.5,-0.5)<< QVector3D(0.5,-0.5,-0.5)
             << QVector3D(-0.5,-0.5,-0.5) << QVector3D(-0.5,-0.5,0.5)<< QVector3D(-0.5,0.5,0.5)
             << QVector3D(-0.5,0.5,0.5) << QVector3D(-0.5,-0.5,-0.5)<< QVector3D(-0.5,-0.5,-0.5)
             << QVector3D(0.5,-0.5,0.5) << QVector3D(0.5,-0.5,-0.5)<< QVector3D(0.5,0.5,-0.5)
             << QVector3D(0.5,0.5,-0.5) << QVector3D(0.5,0.5,0.5)<< QVector3D(0.5,-0.5,0.5)
             << QVector3D(-0.5,0.5,0.5) << QVector3D(0.5,0.5,0.5)<< QVector3D(0.5,0.5,-0.5)
             << QVector3D(0.5,0.5,-0.5) << QVector3D(-0.5,0.5,-0.5)<< QVector3D(-0.5,0.5,0.5)
             << QVector3D(-0.5,-0.5,-0.5) << QVector3D(0.5,-0.5,-0.5)<< QVector3D(0.5,-0.5,0.5)
             << QVector3D(0.5,-0.5,0.5) << QVector3D(-0.5,-0.5,0.5)<< QVector3D(-0.5,0.5,0.5);


}

void GlWidget::resizeGL(int width, int height)
{
    if(height <=0)
        height = 1;

    pMatrix.setToIdentity();
    pMatrix.perspective(60.0,(float) width/ (float) height, 0.001,1000);

    glViewport(0,0,width,height);
}

//motion: rotate and translate object. no moving camera
void GlWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    QMatrix4x4 mMatrix;
    QMatrix4x4 vMatrix;

    //QVector3D modelPosition = mMatrix * QVector3D(newX,newY,newZ);
    //QVector3D modelUpPosition = mMatrix * QVector3D(0,1,0);
    mMatrix.translate(newX,newY,newZ);

    mMatrix.rotate(alpha,0,1,0);
    mMatrix.rotate(beta,0,0,1);



    QMatrix4x4 cameraTransformation;
    //cameraTransformation.setToIdentity();
    //cameraTransformation.rotate(alpha,0,1,0);
    //cameraTransformation.rotate(beta,1,0,0);

    QVector3D cameraPosition = cameraTransformation * QVector3D(0,0,5);
    QVector3D cameraUpDirection = cameraTransformation * QVector3D(0,1,0);
    vMatrix.lookAt(cameraPosition,QVector3D(0,0,0), cameraUpDirection);

    shaderProgram.bind();

    shaderProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
    shaderProgram.setUniformValue("color", QColor(Qt::white));

    shaderProgram.setAttributeArray("vertex",vertices.constData());
    shaderProgram.enableAttributeArray("vertex");

    glDrawArrays(GL_TRIANGLES,0,vertices.size());

    shaderProgram.disableAttributeArray("vertex");

    shaderProgram.release();

}

//void GlWidget::mousePressEvent(QMouseEvent * event)
//{
//    lastMousePosition = event->pos();
//    event->accept();
//}

//void GlWidget::mouseMoveEvent(QMouseEvent *event)
//{
//    int deltaX = event->x() - lastMousePosition.x();
//    int deltaY = event->y() - lastMousePosition.y();

//    if (event->buttons() & Qt::LeftButton)
//    {
//        alpha -= deltaX;
//        while(alpha < 0)
//        {
//            alpha += 360;
//        }
//        while(alpha >= 360)
//        {
//            alpha -=360;
//        }
//        beta -= deltaY;
//        if(beta < -90)
//            beta = -90;
//        if(beta > 90)
//            beta = 90;

//        updateGL();
//        lastMousePosition  = event->pos();
//        event->accept();
//    }

//}

//void GlWidget::wheelEvent(QWheelEvent *event)
//{
//    int delta = event->delta();

//    if(event->orientation() == Qt::Vertical)
//    {
//        if(delta <0)
//            distance *= 1.1;
//        else if (delta >0)
//            distance *= .9;
//        updateGL();
//    }
//    event->accept();
//}

void GlWidget::keyPressEvent(QKeyEvent * event)
{

    keysPressed.insert((Qt::Key) event->key());
   // qDebug() <<"keys amt= "<< keysPressed.count();
    if(keysPressed.count() >1)
    {
        processMultipleKeys();
        return;
    }
    //qDebug() << "key: "<< QKeySequence(keysPressed.begin()).toString();

    if(event->key() == Qt::Key_W )
    {
       // qDebug()<<"key w pressed";
        //distance *= 1.1;     //zoom out
        //move forward in z
        newZ += .1;

    }
    else if (event->key() == Qt::Key_A)
    {
       // qDebug() << "key a pressed";
        //move back in x
        newX -=.1;
    }
    else if (event->key() == Qt::Key_S)
    {
        newZ -= .1;
    }
    else if (event->key() == Qt::Key_D)
        newX +=.1;
    else if (event->key() == Qt::Key_I)
    {
        beta += 5;
    }
    else if (event->key() == Qt::Key_J)
        alpha -=5;
    else if (event->key() == Qt::Key_K)
        beta -= 5;
    else if (event->key() == Qt::Key_L)
        alpha +=5;
    updateGL();

    //event->accept();
}

void GlWidget::keyReleaseEvent(QKeyEvent *event)
{
    //remove key from set
    keysPressed.remove((Qt::Key)event->key());

    //releasing interrupts call to processing other keys that may be down
//    if(keysPressed.count()>1)
//        processMultipleKeys();
//    else if (keysPressed.count() ==1)

}

void GlWidget::processMultipleKeys()
{
    qDebug() << "handling multiKey input";
    //need to handle multiple calls to this? ex. press w and a , then press d     will the call invoking d check for w and a again to build upon existing effects? could do 8 bools
    if(keysPressed.contains(Qt::Key_W))
    {
        newZ += .1;
    }
    if(keysPressed.contains(Qt::Key_A))
    {
        newX -=.1;
    }
    if(keysPressed.contains(Qt::Key_S))
    {
        newZ -=.1;
    }
    if(keysPressed.contains(Qt::Key_D))
    {
        newX +=.1;
    }
    if(keysPressed.contains(Qt::Key_I))
    {
        beta += 5;
    }
    if(keysPressed.contains(Qt::Key_J))
    {
        alpha -=5;
    }
    if(keysPressed.contains(Qt::Key_K))
    {
        beta -=5;
    }
    if(keysPressed.contains(Qt::Key_L))
    {
        alpha+=5;
    }
    updateGL();

}
