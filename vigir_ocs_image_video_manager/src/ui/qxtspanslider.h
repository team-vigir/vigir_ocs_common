00001 /****************************************************************************
00002  **
00003  ** Copyright (C) Qxt Foundation. Some rights reserved.
00004  **
00005  ** This file is part of the QxtGui module of the Qxt library.
00006  **
00007  ** This library is free software; you can redistribute it and/or modify it
00008  ** under the terms of the Common Public License, version 1.0, as published
00009  ** by IBM, and/or under the terms of the GNU Lesser General Public License,
00010  ** version 2.1, as published by the Free Software Foundation.
00011  **
00012  ** This file is provided "AS IS", without WARRANTIES OR CONDITIONS OF ANY
00013  ** KIND, EITHER EXPRESS OR IMPLIED INCLUDING, WITHOUT LIMITATION, ANY
00014  ** WARRANTIES OR CONDITIONS OF TITLE, NON-INFRINGEMENT, MERCHANTABILITY OR
00015  ** FITNESS FOR A PARTICULAR PURPOSE.
00016  **
00017  ** You should have received a copy of the CPL and the LGPL along with this
00018  ** file. See the LICENSE file and the cpl1.0.txt/lgpl-2.1.txt files
00019  ** included with the source distribution for more information.
00020  ** If you did not receive a copy of the licenses, contact the Qxt Foundation.
00021  **
00022  ** <http://libqxt.org>  <foundation@libqxt.org>
00023  **
00024  ****************************************************************************/
00025 #ifndef QXTSPANSLIDER_H
00026 #define QXTSPANSLIDER_H
00027 
00028 #include <QSlider>
00029 #include "qxtnamespace.h"
00030 #include "qxtglobal.h"
00031 #include "qxtpimpl.h"
00032 
00033 class QxtSpanSliderPrivate;
00034 
00035 class QXT_GUI_EXPORT QxtSpanSlider : public QSlider
00036 {
00037     Q_OBJECT
00038     QXT_DECLARE_PRIVATE(QxtSpanSlider);
00039     Q_PROPERTY(int lowerValue READ lowerValue WRITE setLowerValue)
00040     Q_PROPERTY(int upperValue READ upperValue WRITE setUpperValue)
00041     Q_PROPERTY(int lowerPosition READ lowerPosition WRITE setLowerPosition)
00042     Q_PROPERTY(int upperPosition READ upperPosition WRITE setUpperPosition)
00043     Q_PROPERTY(HandleMovementMode handleMovementMode READ handleMovementMode WRITE setHandleMovementMode)
00044     Q_ENUMS(HandleMovementMode)
00045 
00046 public:
00047     explicit QxtSpanSlider(QWidget* parent = 0);
00048     explicit QxtSpanSlider(Qt::Orientation orientation, QWidget* parent = 0);
00049     virtual ~QxtSpanSlider();
00050 
00051     enum HandleMovementMode
00052     {
00053         FreeMovement,
00054         NoCrossing,
00055         NoOverlapping
00056     };
00057 
00058     HandleMovementMode handleMovementMode() const;
00059     void setHandleMovementMode(HandleMovementMode mode);
00060 
00061     int lowerValue() const;
00062     int upperValue() const;
00063 
00064     int lowerPosition() const;
00065     int upperPosition() const;
00066 
00067 public Q_SLOTS:
00068     void setLowerValue(int lower);
00069     void setUpperValue(int upper);
00070     void setSpan(int lower, int upper);
00071 
00072     void setLowerPosition(int lower);
00073     void setUpperPosition(int upper);
00074 
00075 Q_SIGNALS:
00076     void spanChanged(int lower, int upper);
00077     void lowerValueChanged(int lower);
00078     void upperValueChanged(int upper);
00079 
00080     void lowerPositionChanged(int lower);
00081     void upperPositionChanged(int upper);
00082 
00083 protected:
00084     virtual void keyPressEvent(QKeyEvent* event);
00085     virtual void mousePressEvent(QMouseEvent* event);
00086     virtual void mouseMoveEvent(QMouseEvent* event);
00087     virtual void mouseReleaseEvent(QMouseEvent* event);
00088     virtual void paintEvent(QPaintEvent* event);
00089 };
00090 
00091 #endif // QXTSPANSLIDER_H

