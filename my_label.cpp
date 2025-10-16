#include "my_label.h"

my_label::my_label(QWidget *parent) : QLabel(parent)
{
    this->setMouseTracking(true);
}

void my_label::mouseMoveEvent(QMouseEvent *ev)
{
    QPointF pos = ev->position();
    if (pos.x() >= 0 && pos.y() >= 0 && pos.x() < this->width() && pos.y() < this->height()) {
        QPoint mousepos = pos.toPoint();
        emit sendMousePosition(mousepos);
    }
}

void my_label::mousePressEvent(QMouseEvent *ev)
{
    if (ev->button() == Qt::LeftButton) {
        x = ev->position().x();
        y = ev->position().y();
        emit Mouse_Pos();
    }
}
