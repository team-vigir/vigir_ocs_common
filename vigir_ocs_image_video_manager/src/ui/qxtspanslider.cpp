
/*0002  **
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
00025 #include "qxtspanslider.h"
00026 #include "qxtspanslider_p.h"
00027 #include <QKeyEvent>
00028 #include <QMouseEvent>
00029 #include <QApplication>
00030 #include <QStylePainter>
00031 #include <QStyleOptionSlider>
00032 
00033 QxtSpanSliderPrivate::QxtSpanSliderPrivate() :
00034         lower(0),
00035         upper(0),
00036         lowerPos(0),
00037         upperPos(0),
00038         offset(0),
00039         position(0),
00040         lastPressed(NoHandle),
00041         mainControl(LowerHandle),
00042         lowerPressed(QStyle::SC_None),
00043         upperPressed(QStyle::SC_None),
00044         movement(QxtSpanSlider::FreeMovement),
00045         firstMovement(false),
00046         blockTracking(false)
00047 {
00048 }
00049 
00050 // TODO: get rid of this in Qt 4.3
00051 void QxtSpanSliderPrivate::initStyleOption(QStyleOptionSlider* option, SpanHandle handle) const
00052 {
00053     if (!option)
00054         return;
00055 
00056     const QSlider* p = &qxt_p();
00057     option->initFrom(p);
00058     option->subControls = QStyle::SC_None;
00059     option->activeSubControls = QStyle::SC_None;
00060     option->orientation = p->orientation();
00061     option->maximum = p->maximum();
00062     option->minimum = p->minimum();
00063     option->tickPosition = p->tickPosition();
00064     option->tickInterval = p->tickInterval();
00065     option->upsideDown = (p->orientation() == Qt::Horizontal) ?
00066                          (p->invertedAppearance() != (option->direction == Qt::RightToLeft)) : (!p->invertedAppearance());
00067     option->direction = Qt::LeftToRight; // we use the upsideDown option instead
00068     option->sliderPosition = (handle == LowerHandle ? lowerPos : upperPos);
00069     option->sliderValue = (handle == LowerHandle ? lower : upper);
00070     option->singleStep = p->singleStep();
00071     option->pageStep = p->pageStep();
00072     if (p->orientation() == Qt::Horizontal)
00073         option->state |= QStyle::State_Horizontal;
00074 }
00075 
00076 int QxtSpanSliderPrivate::pixelPosToRangeValue(int pos) const
00077 {
00078     QStyleOptionSlider opt;
00079     initStyleOption(&opt);
00080 
00081     int sliderMin = 0;
00082     int sliderMax = 0;
00083     int sliderLength = 0;
00084     const QSlider* p = &qxt_p();
00085     const QRect gr = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, p);
00086     const QRect sr = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, p);
00087     if (p->orientation() == Qt::Horizontal)
00088     {
00089         sliderLength = sr.width();
00090         sliderMin = gr.x();
00091         sliderMax = gr.right() - sliderLength + 1;
00092     }
00093     else
00094     {
00095         sliderLength = sr.height();
00096         sliderMin = gr.y();
00097         sliderMax = gr.bottom() - sliderLength + 1;
00098     }
00099     return QStyle::sliderValueFromPosition(p->minimum(), p->maximum(), pos - sliderMin,
00100                                            sliderMax - sliderMin, opt.upsideDown);
00101 }
00102 
00103 void QxtSpanSliderPrivate::handleMousePress(const QPoint& pos, QStyle::SubControl& control, int value, SpanHandle handle)
00104 {
00105     QStyleOptionSlider opt;
00106     initStyleOption(&opt, handle);
00107     QSlider* p = &qxt_p();
00108     const QStyle::SubControl oldControl = control;
00109     control = p->style()->hitTestComplexControl(QStyle::CC_Slider, &opt, pos, p);
00110     const QRect sr = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, p);
00111     if (control == QStyle::SC_SliderHandle)
00112     {
00113         position = value;
00114         offset = pick(pos - sr.topLeft());
00115         lastPressed = handle;
00116         p->setSliderDown(true);
00117     }
00118     if (control != oldControl)
00119         p->update(sr);
00120 }
00121 
00122 void QxtSpanSliderPrivate::setupPainter(QPainter* painter, Qt::Orientation orientation, qreal x1, qreal y1, qreal x2, qreal y2) const
00123 {
00124     QColor highlight = qxt_p().palette().color(QPalette::Highlight);
00125     QLinearGradient gradient(x1, y1, x2, y2);
00126     gradient.setColorAt(0, highlight.dark(120));
00127     gradient.setColorAt(1, highlight.light(108));
00128     painter->setBrush(gradient);
00129 
00130     if (orientation == Qt::Horizontal)
00131         painter->setPen(QPen(highlight.dark(130), 0));
00132     else
00133         painter->setPen(QPen(highlight.dark(150), 0));
00134 }
00135 
00136 void QxtSpanSliderPrivate::drawSpan(QStylePainter* painter, const QRect& rect) const
00137 {
00138     QStyleOptionSlider opt;
00139     initStyleOption(&opt);
00140     const QSlider* p = &qxt_p();
00141 
00142     // area
00143     QRect groove = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, p);
00144     if (opt.orientation == Qt::Horizontal)
00145         groove.adjust(0, 0, -1, 0);
00146     else
00147         groove.adjust(0, 0, 0, -1);
00148 
00149     // pen & brush
00150     painter->setPen(QPen(p->palette().color(QPalette::Dark).light(110), 0));
00151     if (opt.orientation == Qt::Horizontal)
00152         setupPainter(painter, opt.orientation, groove.center().x(), groove.top(), groove.center().x(), groove.bottom());
00153     else
00154         setupPainter(painter, opt.orientation, groove.left(), groove.center().y(), groove.right(), groove.center().y());
00155 
00156     // draw groove
00157 #if QT_VERSION >= 0x040200
00158     painter->drawRect(rect.intersected(groove));
00159 #else // QT_VERSION < 0x040200
00160     painter->drawRect(rect.intersect(groove));
00161 #endif // QT_VERSION
00162 }
00163 
00164 void QxtSpanSliderPrivate::drawHandle(QStylePainter* painter, SpanHandle handle) const
00165 {
00166     QStyleOptionSlider opt;
00167     initStyleOption(&opt, handle);
00168     opt.subControls = QStyle::SC_SliderHandle;
00169     QStyle::SubControl pressed = (handle == LowerHandle ? lowerPressed : upperPressed);
00170     if (pressed == QStyle::SC_SliderHandle)
00171     {
00172         opt.activeSubControls = pressed;
00173         opt.state |= QStyle::State_Sunken;
00174     }
00175     painter->drawComplexControl(QStyle::CC_Slider, opt);
00176 }
00177 
00178 void QxtSpanSliderPrivate::triggerAction(QAbstractSlider::SliderAction action, bool main)
00179 {
00180     int value = 0;
00181     bool no = false;
00182     bool up = false;
00183     const int min = qxt_p().minimum();
00184     const int max = qxt_p().maximum();
00185     const SpanHandle altControl = (mainControl == LowerHandle ? UpperHandle : LowerHandle);
00186 
00187     blockTracking = true;
00188 
00189     switch (action)
00190     {
00191     case QAbstractSlider::SliderSingleStepAdd:
00192         if ((main && mainControl == UpperHandle) || (!main && altControl == UpperHandle))
00193         {
00194             value = qBound(min, upper + qxt_p().singleStep(), max);
00195             up = true;
00196             break;
00197         }
00198         value = qBound(min, lower + qxt_p().singleStep(), max);
00199         break;
00200     case QAbstractSlider::SliderSingleStepSub:
00201         if ((main && mainControl == UpperHandle) || (!main && altControl == UpperHandle))
00202         {
00203             value = qBound(min, upper - qxt_p().singleStep(), max);
00204             up = true;
00205             break;
00206         }
00207         value = qBound(min, lower - qxt_p().singleStep(), max);
00208         break;
00209     case QAbstractSlider::SliderToMinimum:
00210         value = min;
00211         if ((main && mainControl == UpperHandle) || (!main && altControl == UpperHandle))
00212             up = true;
00213         break;
00214     case QAbstractSlider::SliderToMaximum:
00215         value = max;
00216         if ((main && mainControl == UpperHandle) || (!main && altControl == UpperHandle))
00217             up = true;
00218         break;
00219     case QAbstractSlider::SliderMove:
00220         if ((main && mainControl == UpperHandle) || (!main && altControl == UpperHandle))
00221             up = true;
00222     case QAbstractSlider::SliderNoAction:
00223         no = true;
00224         break;
00225     default:
00226         qWarning("QxtSpanSliderPrivate::triggerAction: Unknown action");
00227         break;
00228     }
00229 
00230     if (!no && !up)
00231     {
00232         if (movement == QxtSpanSlider::NoCrossing)
00233             value = qMin(value, upper);
00234         else if (movement == QxtSpanSlider::NoOverlapping)
00235             value = qMin(value, upper - 1);
00236 
00237         if (movement == QxtSpanSlider::FreeMovement && value > upper)
00238         {
00239             swapControls();
00240             qxt_p().setUpperPosition(value);
00241         }
00242         else
00243         {
00244             qxt_p().setLowerPosition(value);
00245         }
00246     }
00247     else if (!no)
00248     {
00249         if (movement == QxtSpanSlider::NoCrossing)
00250             value = qMax(value, lower);
00251         else if (movement == QxtSpanSlider::NoOverlapping)
00252             value = qMax(value, lower + 1);
00253 
00254         if (movement == QxtSpanSlider::FreeMovement && value < lower)
00255         {
00256             swapControls();
00257             qxt_p().setLowerPosition(value);
00258         }
00259         else
00260         {
00261             qxt_p().setUpperPosition(value);
00262         }
00263     }
00264 
00265     blockTracking = false;
00266     qxt_p().setLowerValue(lowerPos);
00267     qxt_p().setUpperValue(upperPos);
00268 }
00269 
00270 void QxtSpanSliderPrivate::swapControls()
00271 {
00272     qSwap(lower, upper);
00273     qSwap(lowerPressed, upperPressed);
00274     lastPressed = (lastPressed == LowerHandle ? UpperHandle : LowerHandle);
00275     mainControl = (mainControl == LowerHandle ? UpperHandle : LowerHandle);
00276 }
00277 
00278 void QxtSpanSliderPrivate::updateRange(int min, int max)
00279 {
00280     Q_UNUSED(min);
00281     Q_UNUSED(max);
00282     // setSpan() takes care of keeping span in range
00283     qxt_p().setSpan(lower, upper);
00284 }
00285 
00286 void QxtSpanSliderPrivate::movePressedHandle()
00287 {
00288     switch (lastPressed)
00289     {
00290         case QxtSpanSliderPrivate::LowerHandle:
00291             if (lowerPos != lower)
00292             {
00293                 bool main = (mainControl == QxtSpanSliderPrivate::LowerHandle);
00294                 triggerAction(QAbstractSlider::SliderMove, main);
00295             }
00296             break;
00297         case QxtSpanSliderPrivate::UpperHandle:
00298             if (upperPos != upper)
00299             {
00300                 bool main = (mainControl == QxtSpanSliderPrivate::UpperHandle);
00301                 triggerAction(QAbstractSlider::SliderMove, main);
00302             }
00303             break;
00304         default:
00305             break;
00306     }
00307 }
00308 
00419 QxtSpanSlider::QxtSpanSlider(QWidget* parent) : QSlider(parent)
00420 {
00421     QXT_INIT_PRIVATE(QxtSpanSlider);
00422     connect(this, SIGNAL(rangeChanged(int, int)), &qxt_d(), SLOT(updateRange(int, int)));
00423     connect(this, SIGNAL(sliderReleased()), &qxt_d(), SLOT(movePressedHandle()));
00424 }
00425 
00429 QxtSpanSlider::QxtSpanSlider(Qt::Orientation orientation, QWidget* parent) : QSlider(orientation, parent)
00430 {
00431     QXT_INIT_PRIVATE(QxtSpanSlider);
00432     connect(this, SIGNAL(rangeChanged(int, int)), &qxt_d(), SLOT(updateRange(int, int)));
00433     connect(this, SIGNAL(sliderReleased()), &qxt_d(), SLOT(movePressedHandle()));
00434 }
00435 
00439 QxtSpanSlider::~QxtSpanSlider()
00440 {
00441 }
00442 
00447 QxtSpanSlider::HandleMovementMode QxtSpanSlider::handleMovementMode() const
00448 {
00449     return qxt_d().movement;
00450 }
00451 
00452 void QxtSpanSlider::setHandleMovementMode(QxtSpanSlider::HandleMovementMode mode)
00453 {
00454     qxt_d().movement = mode;
00455 }
00456 
00461 int QxtSpanSlider::lowerValue() const
00462 {
00463     return qMin(qxt_d().lower, qxt_d().upper);
00464 }
00465 
00466 void QxtSpanSlider::setLowerValue(int lower)
00467 {
00468     setSpan(lower, qxt_d().upper);
00469 }
00470 
00475 int QxtSpanSlider::upperValue() const
00476 {
00477     return qMax(qxt_d().lower, qxt_d().upper);
00478 }
00479 
00480 void QxtSpanSlider::setUpperValue(int upper)
00481 {
00482     setSpan(qxt_d().lower, upper);
00483 }
00484 
00489 void QxtSpanSlider::setSpan(int lower, int upper)
00490 {
00491     const int low = qBound(minimum(), qMin(lower, upper), maximum());
00492     const int upp = qBound(minimum(), qMax(lower, upper), maximum());
00493     if (low != qxt_d().lower || upp != qxt_d().upper)
00494     {
00495         if (low != qxt_d().lower)
00496         {
00497             qxt_d().lower = low;
00498             qxt_d().lowerPos = low;
00499             emit lowerValueChanged(low);
00500         }
00501         if (upp != qxt_d().upper)
00502         {
00503             qxt_d().upper = upp;
00504             qxt_d().upperPos = upp;
00505             emit upperValueChanged(upp);
00506         }
00507         emit spanChanged(qxt_d().lower, qxt_d().upper);
00508         update();
00509     }
00510 }
00511 
00516 int QxtSpanSlider::lowerPosition() const
00517 {
00518     return qxt_d().lowerPos;
00519 }
00520 
00521 void QxtSpanSlider::setLowerPosition(int lower)
00522 {
00523     if (qxt_d().lowerPos != lower)
00524     {
00525         qxt_d().lowerPos = lower;
00526         if (!hasTracking())
00527             update();
00528         if (isSliderDown())
00529             emit lowerPositionChanged(lower);
00530         if (hasTracking() && !qxt_d().blockTracking)
00531         {
00532             bool main = (qxt_d().mainControl == QxtSpanSliderPrivate::LowerHandle);
00533             qxt_d().triggerAction(SliderMove, main);
00534         }
00535     }
00536 }
00537 
00542 int QxtSpanSlider::upperPosition() const
00543 {
00544     return qxt_d().upperPos;
00545 }
00546 
00547 void QxtSpanSlider::setUpperPosition(int upper)
00548 {
00549     if (qxt_d().upperPos != upper)
00550     {
00551         qxt_d().upperPos = upper;
00552         if (!hasTracking())
00553             update();
00554         if (isSliderDown())
00555             emit upperPositionChanged(upper);
00556         if (hasTracking() && !qxt_d().blockTracking)
00557         {
00558             bool main = (qxt_d().mainControl == QxtSpanSliderPrivate::UpperHandle);
00559             qxt_d().triggerAction(SliderMove, main);
00560         }
00561     }
00562 }
00563 
00567 void QxtSpanSlider::keyPressEvent(QKeyEvent* event)
00568 {
00569     QSlider::keyPressEvent(event);
00570 
00571     bool main = true;
00572     SliderAction action = SliderNoAction;
00573     switch (event->key())
00574     {
00575     case Qt::Key_Left:
00576         main   = (orientation() == Qt::Horizontal);
00577         action = !invertedAppearance() ? SliderSingleStepSub : SliderSingleStepAdd;
00578         break;
00579     case Qt::Key_Right:
00580         main   = (orientation() == Qt::Horizontal);
00581         action = !invertedAppearance() ? SliderSingleStepAdd : SliderSingleStepSub;
00582         break;
00583     case Qt::Key_Up:
00584         main   = (orientation() == Qt::Vertical);
00585         action = invertedControls() ? SliderSingleStepSub : SliderSingleStepAdd;
00586         break;
00587     case Qt::Key_Down:
00588         main   = (orientation() == Qt::Vertical);
00589         action = invertedControls() ? SliderSingleStepAdd : SliderSingleStepSub;
00590         break;
00591     case Qt::Key_Home:
00592         main   = (qxt_d().mainControl == QxtSpanSliderPrivate::LowerHandle);
00593         action = SliderToMinimum;
00594         break;
00595     case Qt::Key_End:
00596         main   = (qxt_d().mainControl == QxtSpanSliderPrivate::UpperHandle);
00597         action = SliderToMaximum;
00598         break;
00599     default:
00600         event->ignore();
00601         break;
00602     }
00603 
00604     if (action)
00605         qxt_d().triggerAction(action, main);
00606 }
00607 
00611 void QxtSpanSlider::mousePressEvent(QMouseEvent* event)
00612 {
00613     if (minimum() == maximum() || (event->buttons() ^ event->button()))
00614     {
00615         event->ignore();
00616         return;
00617     }
00618 
00619     qxt_d().handleMousePress(event->pos(), qxt_d().upperPressed, qxt_d().upper, QxtSpanSliderPrivate::UpperHandle);
00620     if (qxt_d().upperPressed != QStyle::SC_SliderHandle)
00621         qxt_d().handleMousePress(event->pos(), qxt_d().lowerPressed, qxt_d().lower, QxtSpanSliderPrivate::LowerHandle);
00622 
00623     qxt_d().firstMovement = true;
00624     event->accept();
00625 }
00626 
00630 void QxtSpanSlider::mouseMoveEvent(QMouseEvent* event)
00631 {
00632     if (qxt_d().lowerPressed != QStyle::SC_SliderHandle && qxt_d().upperPressed != QStyle::SC_SliderHandle)
00633     {
00634         event->ignore();
00635         return;
00636     }
00637 
00638     QStyleOptionSlider opt;
00639     qxt_d().initStyleOption(&opt);
00640     const int m = style()->pixelMetric(QStyle::PM_MaximumDragDistance, &opt, this);
00641     int newPosition = qxt_d().pixelPosToRangeValue(qxt_d().pick(event->pos()) - qxt_d().offset);
00642     if (m >= 0)
00643     {
00644         const QRect r = rect().adjusted(-m, -m, m, m);
00645         if (!r.contains(event->pos()))
00646         {
00647             newPosition = qxt_d().position;
00648         }
00649     }
00650 
00651     // pick the preferred handle on the first movement
00652     if (qxt_d().firstMovement)
00653     {
00654         if (qxt_d().lower == qxt_d().upper)
00655         {
00656             if (newPosition < lowerValue())
00657             {
00658                 qxt_d().swapControls();
00659                 qxt_d().firstMovement = false;
00660             }
00661         }
00662         else
00663         {
00664             qxt_d().firstMovement = false;
00665         }
00666     }
00667 
00668     if (qxt_d().lowerPressed == QStyle::SC_SliderHandle)
00669     {
00670         if (qxt_d().movement == NoCrossing)
00671             newPosition = qMin(newPosition, upperValue());
00672         else if (qxt_d().movement == NoOverlapping)
00673             newPosition = qMin(newPosition, upperValue() - 1);
00674 
00675         if (qxt_d().movement == FreeMovement && newPosition > qxt_d().upper)
00676         {
00677             qxt_d().swapControls();
00678             setUpperPosition(newPosition);
00679         }
00680         else
00681         {
00682             setLowerPosition(newPosition);
00683         }
00684     }
00685     else if (qxt_d().upperPressed == QStyle::SC_SliderHandle)
00686     {
00687         if (qxt_d().movement == NoCrossing)
00688             newPosition = qMax(newPosition, lowerValue());
00689         else if (qxt_d().movement == NoOverlapping)
00690             newPosition = qMax(newPosition, lowerValue() + 1);
00691 
00692         if (qxt_d().movement == FreeMovement && newPosition < qxt_d().lower)
00693         {
00694             qxt_d().swapControls();
00695             setLowerPosition(newPosition);
00696         }
00697         else
00698         {
00699             setUpperPosition(newPosition);
00700         }
00701     }
00702     event->accept();
00703 }
00704 
00708 void QxtSpanSlider::mouseReleaseEvent(QMouseEvent* event)
00709 {
00710     QSlider::mouseReleaseEvent(event);
00711     setSliderDown(false);
00712     qxt_d().lowerPressed = QStyle::SC_None;
00713     qxt_d().upperPressed = QStyle::SC_None;
00714     update();
00715 }
00716 
00720 void QxtSpanSlider::paintEvent(QPaintEvent* event)
00721 {
00722     Q_UNUSED(event);
00723     QStylePainter painter(this);
00724 
00725     // ticks
00726     QStyleOptionSlider opt;
00727     qxt_d().initStyleOption(&opt);
00728     opt.subControls = QStyle::SC_SliderTickmarks;
00729     painter.drawComplexControl(QStyle::CC_Slider, opt);
00730 
00731     // groove
00732     opt.sliderValue = 0;
00733     opt.sliderPosition = 0;
00734     opt.subControls = QStyle::SC_SliderGroove;
00735     painter.drawComplexControl(QStyle::CC_Slider, opt);
00736 
00737     // handle rects
00738     opt.sliderPosition = qxt_d().lowerPos;
00739     const QRect lr = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
00740     const int lrv  = qxt_d().pick(lr.center());
00741     opt.sliderPosition = qxt_d().upperPos;
00742     const QRect ur = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
00743     const int urv  = qxt_d().pick(ur.center());
00744 
00745     // span
00746     const int minv = qMin(lrv, urv);
00747     const int maxv = qMax(lrv, urv);
00748     const QPoint c = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, this).center();
00749     QRect spanRect;
00750     if (orientation() == Qt::Horizontal)
00751         spanRect = QRect(QPoint(minv, c.y() - 2), QPoint(maxv, c.y() + 1));
00752     else
00753         spanRect = QRect(QPoint(c.x() - 2, minv), QPoint(c.x() + 1, maxv));
00754     qxt_d().drawSpan(&painter, spanRect);
00755 
00756     // handles
00757     switch (qxt_d().lastPressed)
00758     {
00759     case QxtSpanSliderPrivate::LowerHandle:
00760         qxt_d().drawHandle(&painter, QxtSpanSliderPrivate::UpperHandle);
00761         qxt_d().drawHandle(&painter, QxtSpanSliderPrivate::LowerHandle);
00762         break;
00763     case QxtSpanSliderPrivate::UpperHandle:
00764     default:
00765         qxt_d().drawHandle(&painter, QxtSpanSliderPrivate::LowerHandle);
00766         qxt_d().drawHandle(&painter, QxtSpanSliderPrivate::UpperHandle);
00767         break;
00768     }
00769 }

Generated on Tue Oct 19 2010 16:19:42 for mitk by  doxygen 1.7.2

