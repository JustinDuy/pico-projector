#ifndef MYQWIDGET_H
#define MYQWIDGET_H

#include <QWidget>

class MyQWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MyQWidget(QWidget *parent = 0);
    void setPixmap(QPixmap* q);
private:
    QPixmap* _pixmap;
};

#endif // MYQWIDGET_H
