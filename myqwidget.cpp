#include "myqwidget.h"

#include <QHBoxLayout>
#include <QLabel>

MyQWidget::MyQWidget(QWidget *parent) :
    QWidget(parent)
{

}
void MyQWidget::setPixmap(QPixmap* pixmap){
    QLabel *label = new QLabel();
    _pixmap = pixmap;
    label->setPixmap(*pixmap);
    QHBoxLayout *layout = new QHBoxLayout();
    layout->addWidget(label);
    setLayout(layout);
}
