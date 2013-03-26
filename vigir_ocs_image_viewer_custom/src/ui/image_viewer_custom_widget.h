#ifndef ImageViewerCustomWidget_H
#define ImageViewerCustomWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>


namespace Ui {
class ImageViewerCustomWidget;
}

class ImageViewerCustomWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit ImageViewerCustomWidget(QWidget *parent = 0);
    ~ImageViewerCustomWidget();
    
private:
    Ui::ImageViewerCustomWidget*ui;

public Q_SLOTS:
    void alterDisplay(int);
    void updatePitch(int);
    void updateFeedFPS(int);
    void updateSelectedFPS(int);
    void scan();
    void isLocked();
    void alterChoices(int);


};

#endif // ImageViewerCustomWidget_H
