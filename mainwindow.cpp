#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QDebug>
#include <QtMath>
#include <QElapsedTimer>
#include <QThread>
#include <QCoreApplication>
#include <algorithm>
#include <QSet>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    this->line_choose = 0;
    ui->setupUi(this);

    lastPoint1 = QPoint(-1, -1);
    lastPoint2 = QPoint(-1, -1);

    QPixmap pix(ui->frame->width(), ui->frame->height());
    pix.fill(Qt::black);
    ui->frame->setPixmap(pix);

    connect(ui->frame, SIGNAL(Mouse_Pos()), this, SLOT(Mouse_Pressed()));
    connect(ui->frame, SIGNAL(sendMousePosition(QPoint&)), this, SLOT(showMousePosition(QPoint&)));
    this->radius_circle = 5;
    this->grid_box = 10;
    this->grid_size = ui->frame->frameSize().width();
    this->draw_clicked = false;
    this->center = 0;
    this->currPoly.col = Qt::blue;
    this->polygonFlag = false;
    currPoly.vertices.clear();

}
Mat3 MatOps::matIdentity() {
    Mat3 M;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) M[i][j] = (i==j)?1.0:0.0;
    return M;
}

Mat3 MatOps::matTranslate(double dx, double dy) {
    Mat3 M = matIdentity();
    M[0][2] = dx;
    M[1][2] = dy;
    return M;
}

Mat3 MatOps::matRotate(double angleDeg) {
    double a = angleDeg * M_PI / 180.0;
    double c = cos(a), s = sin(a);
    Mat3 M = matIdentity();
    M[0][0] =  c; M[0][1] = -s;
    M[1][0] =  s; M[1][1] =  c;
    return M;
}

Mat3 MatOps::matScale(double sx, double sy) {
    Mat3 M = matIdentity();
    M[0][0] = sx; M[1][1] = sy;
    return M;
}

Mat3 MatOps::matShear(double shx, double shy) {
    Mat3 M = matIdentity();
    M[0][1] = shx;
    M[1][0] = shy;
    return M;
}

Mat3 MatOps::matReflectX() {
    Mat3 M = matIdentity();
    M[1][1] = -1.0;
    return M;
}

Mat3 MatOps::matReflectY() {
    Mat3 M = matIdentity();
    M[0][0] = -1.0;
    return M;
}

Mat3 MatOps::matMultiply(const Mat3 &A, const Mat3 &B) {
    Mat3 C;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++){
            C[i][j] = 0.0;
            for (int k=0;k<3;k++) C[i][j] += A[i][k] * B[k][j];
        }
    return C;
}

QPointF MatOps::applyMatToPoint(const Mat3 &M, const QPointF &pt) {
    double x = pt.x(), y = pt.y();
    double nx = M[0][0]*x + M[0][1]*y + M[0][2]*1.0;
    double ny = M[1][0]*x + M[1][1]*y + M[1][2]*1.0;
    double w  = M[2][0]*x + M[2][1]*y + M[2][2]*1.0;
    if (fabs(w) > 1e-12) { nx /= w; ny /= w; }
    return QPointF(nx, ny);
}
Mat3 MatOps::matReflectAboutLine(const QPointF &p1, const QPointF &p2) {
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    double theta = atan2(dy, dx)  * 180.0 / M_PI;

    Mat3 T1 = matTranslate(-p1.x(), -p1.y());
    Mat3 R1 = matRotate(-theta);
    Mat3 RefX = matReflectX();
    Mat3 R2 = matRotate(theta);
    Mat3 T2 = matTranslate(p1.x(), p1.y());

    return matMultiply(T2, matMultiply(R2, matMultiply(RefX, matMultiply(R1, T1))));
}


Mat3 MatOps::matRotateAboutPoint(double angleDeg, double cx, double cy) {
    Mat3 T1 = matTranslate(-cx, -cy);
    Mat3 R  = matRotate(angleDeg);
    Mat3 T2 = matTranslate(cx, cy);
    Mat3 M = matMultiply(T2, matMultiply(R, T1));
    return M;
}

void MatOps::applyMatrixToPolygon(Polygon &poly, const Mat3 &M, bool inplace) {
    QVector<QPoint> out;
    out.reserve(poly.vertices.size());
    for (const QPoint &p : poly.vertices) {
        QPointF pf = applyMatToPoint(M, QPointF(p.x(), p.y()));
        out.push_back(QPoint(qRound(pf.x()), qRound(pf.y())));
    }
    if (inplace) {
        poly.vertices = out;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showMousePosition(QPoint &pos)
{
    sc_x = pos.x();
    sc_y = pos.y();

    int X = (sc_x / this->grid_box) - this->center;
    int Y = -(sc_y / this->grid_box) + this->center;

    ui->mouse_movement->setText("X : " + QString::number(X) +
                                ", Y : " + QString::number(Y));
}

void MainWindow::Mouse_Pressed()
{
    org_x = sc_x;
    org_y = sc_y;

    int X = (sc_x / this->grid_box) - this->center;
    int Y = -(sc_y / this->grid_box) + this->center;

    if(polygonFlag){
        currPoly.vertices.push_back(QPoint(X,Y));
    }
    ui->mouse_pressed->setText("X : " + QString::number(X) +
                               ", Y : " + QString::number(Y));

    lastPoint1 = lastPoint2;
    lastPoint2 = QPoint(org_x, org_y);

    addPoint(org_x, org_y);
}

int MainWindow::plotCircleSymmetry(int screen_cx, int screen_cy, int x, int y,
                                   QPainter& painter, Qt::GlobalColor col)
{
    auto drawCell = [&](int gx, int gy) {
        QRect rect(gx * this->grid_box, gy * this->grid_box,
                   this->grid_box, this->grid_box);
        painter.fillRect(rect, QBrush(col, Qt::SolidPattern));
    };
    int pointsPlotted = 0;

    drawCell(screen_cx + x, screen_cy - y); pointsPlotted++;
    drawCell(screen_cx - x, screen_cy - y); pointsPlotted++;
    drawCell(screen_cx + x, screen_cy + y); pointsPlotted++;
    drawCell(screen_cx - x, screen_cy + y); pointsPlotted++;
    drawCell(screen_cx + y, screen_cy - x); pointsPlotted++;
    drawCell(screen_cx - y, screen_cy - x); pointsPlotted++;
    drawCell(screen_cx + y, screen_cy + x); pointsPlotted++;
    drawCell(screen_cx - y, screen_cy + x); pointsPlotted++;
    return pointsPlotted;

}

QVector<double> MainWindow::draw_circle_grid(int CX, int CY, int R,
                                             QPainter& painter, QPixmap &pm,
                                             Qt::GlobalColor col,
                                             bool mid, bool animate)
{
    if (R <= 0) return {0.0, 0};

    int screen_cx = CX + this->center;
    int screen_cy = this->center - CY;

    QElapsedTimer timer;
    auto maybeAnimate = [&]() {
        if (animate) {
            ui->frame->setPixmap(pm);
            QCoreApplication::processEvents();
            QThread::msleep(100);
        }
    };
    qint64 elapsed = 0;
    int totalPoints = 0;

    if (mid) {
        timer.start();
        int x = 0;
        int y = R;
        int d = 1 - R;

        while (x <= y) {
            totalPoints += plotCircleSymmetry(screen_cx, screen_cy, x, y, painter, col);
            maybeAnimate();

            if (d < 0) {
                d += 2*x + 3;
            } else {
                d += 2*(x - y) + 5;
                --y;
            }
            ++x;
        }
        elapsed = timer.nsecsElapsed();
    } else {
        timer.start();
        int x = 0;
        int y = R;
        while (x <= y) {
            totalPoints += plotCircleSymmetry(screen_cx, screen_cy, x, y, painter, col);
            maybeAnimate();
            x++;
            y = qRound(qSqrt(R*R - x*x));
        }
        elapsed = timer.nsecsElapsed();
    }
    ui->frame->setPixmap(pm);
    return {elapsed/1e6, (double)totalPoints};
}


void MainWindow::addPoint(int x, int y)
{
    QPixmap pm = ui->frame->pixmap();
    if (pm.isNull()) return;

    QPainter painter(&pm);
    painter.setPen(QPen(Qt::white, 5));

    int gx = qFloor(x / this->grid_box);
    int gy = qFloor(y / this->grid_box);

    QRect rect = QRect(gx * this->grid_box, gy * this->grid_box,
                       this->grid_box, this->grid_box);
    painter.fillRect(rect, QBrush(Qt::yellow, Qt::SolidPattern));

    painter.end();
    ui->frame->setPixmap(pm);
}

double MainWindow::draw_line_grid(int X1, int X2, int Y1, int Y2,
                                  QPainter& painter, Qt::GlobalColor col, bool bla)
{
    int gx1 = X1 + this->center;
    int gy1 = this->center - Y1;
    int gx2 = X2 + this->center;
    int gy2 = this->center - Y2;

    qint64 elapsed = 0;

    const int cells = this->grid_size / this->grid_box;

    if (bla) {
        QElapsedTimer timer;
        QSet<QRect> rects;

        timer.start();
        int dx = abs(gx2 - gx1);
        int dy = abs(gy2 - gy1);
        int sx = (gx1 < gx2) ? 1 : -1;
        int sy = (gy1 < gy2) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (gy1 >= 0 && gy1 < cells && gx1 >= 0 && gx1 < cells) {
                rects.insert(QRect(gx1*this->grid_box, gy1*this->grid_box,
                                   this->grid_box, this->grid_box));
            }
            if (gx1 == gx2 && gy1 == gy2) break;
            int e2 = 2*err;
            if (e2 > -dy) { err -= dy; gx1 += sx; }
            if (e2 < dx)  { err += dx; gy1 += sy; }
        }

        elapsed = timer.nsecsElapsed();
        for (const QRect &rect : rects) {
            painter.fillRect(rect, QBrush(col, Qt::SolidPattern));
        }
    }
    else {
        QElapsedTimer timer;
        QSet<QRect> rects;

        timer.start();

        int dx = gx2 - gx1;
        int dy = gy2 - gy1;
        int steps = std::max(abs(dx), abs(dy));
        float x_inc = (steps == 0) ? 0 : dx / (float)steps;
        float y_inc = (steps == 0) ? 0 : dy / (float)steps;


        float x = gx1;
        float y = gy1;

        for (int i = 0; i <= steps; i++) {
            int xi = qRound(x);
            int yi = qRound(y);
            if (yi >= 0 && yi < cells && xi >= 0 && xi < cells) {
                rects.insert(QRect(xi*this->grid_box, yi*this->grid_box,
                                   this->grid_box, this->grid_box));
            }
            x += x_inc;
            y += y_inc;
        }

        elapsed = timer.nsecsElapsed();
        for (const QRect &rect : rects) {
            painter.fillRect(rect, QBrush(col, Qt::SolidPattern));
        }
    }

    return elapsed / 1e6;
}

void MainWindow::on_clear_clicked()
{
    lastPoint1 = QPoint(-1, -1);
    lastPoint2 = QPoint(-1, -1);
    this->points.clear();
    this->circles.clear();
    this->ellipses.clear();
    this->polygons.clear();
    QPixmap pix(ui->frame->width(), ui->frame->height());
    pix.fill(Qt::black);
    ui->frame->setPixmap(pix);

    this->on_draw_grid_clicked();
}

void MainWindow::refresh()
{
    QPixmap pix(ui->frame->width(), ui->frame->height());
    pix.fill(Qt::black);
    ui->frame->setPixmap(pix);
    this->on_draw_grid_clicked();

    if (points.empty() && circles.empty()) {
        lastPoint1 = QPoint(-1, -1);
        lastPoint2 = QPoint(-1, -1);
        return;
    }
    if (!points.empty()) {
        auto [X1, X2, Y1, Y2, bla] = this->points.back();
        lastPoint1 = QPoint((X1 + this->center) * this->grid_box,
                            (this->center - Y1) * this->grid_box);
        lastPoint2 = QPoint((X2 + this->center) * this->grid_box,
                            (this->center - Y2) * this->grid_box);
    }
}
int MainWindow::plotEllipseSymmetry(int screen_cx, int screen_cy, int x, int y,
                                    QPainter& painter, Qt::GlobalColor col) {
    auto drawCell = [&](int gx, int gy) {
        QRect rect(gx * this->grid_box, gy * this->grid_box,
                   this->grid_box, this->grid_box);
        painter.fillRect(rect, QBrush(col, Qt::SolidPattern));
    };

    int pointsPlotted = 0;
    drawCell(screen_cx + x, screen_cy - y); pointsPlotted++;
    drawCell(screen_cx - x, screen_cy - y); pointsPlotted++;
    drawCell(screen_cx + x, screen_cy + y); pointsPlotted++;
    drawCell(screen_cx - x, screen_cy + y); pointsPlotted++;
    return pointsPlotted;
}
double MainWindow::draw_ellipse_polar(int CX, int CY, int RX, int RY,
                                      QPainter& painter, QPixmap &pm,
                                      Qt::GlobalColor col, bool animate,
                                      int &pointsCount) {
    pointsCount = 0;
    if (RX <= 0 || RY <= 0) return 0.0;

    int screen_cx = CX + this->center;
    int screen_cy = this->center - CY;

    QElapsedTimer timer;
    timer.start();

    auto maybeAnimate = [&]() {
        if (animate) {
            ui->frame->setPixmap(pm);
            QCoreApplication::processEvents();
            QThread::msleep(50);
        }
    };

    const double step = 1.0 / std::max(RX, RY);
    for (double theta = 0; theta <= M_PI/2; theta += step) {
        int x = qRound(RX * qCos(theta));
        int y = qRound(RY * qSin(theta));
        pointsCount += plotEllipseSymmetry(screen_cx, screen_cy, x, y, painter, col);
        maybeAnimate();
    }

    pointsCount += plotEllipseSymmetry(screen_cx, screen_cy, RX, 0, painter, col);
    pointsCount += plotEllipseSymmetry(screen_cx, screen_cy, 0, RY, painter, col);

    ui->frame->setPixmap(pm);
    return timer.nsecsElapsed();
}
double MainWindow::draw_ellipse_midpoint(int CX, int CY, int RX, int RY,
                                         QPainter& painter, QPixmap &pm,
                                         Qt::GlobalColor col, bool animate,
                                         int &pointsCount) {
    pointsCount = 0;
    if (RX <= 0 || RY <= 0) return 0.0;

    int screen_cx = CX + this->center;
    int screen_cy = this->center - CY;

    QElapsedTimer timer;
    timer.start();

    auto maybeAnimate = [&]() {
        if (animate) {
            ui->frame->setPixmap(pm);
            QCoreApplication::processEvents();
            QThread::msleep(50);
        }
    };
    long long Rx2 = (long long)RX * RX;
    long long Ry2 = (long long)RY * RY;

    bool swapped = false;
    if(RY > RX) {
        std::swap(RX,RY);
        std::swap(Rx2, Ry2);
        swapped = true;
    }
    int x = 0, y = RY;


    long double d1 = Ry2 - (Rx2 * RY) + (0.25 * Rx2);
    while ((Rx2*(y-0.5) > Ry2*(x+1))) {
        if(swapped)
            pointsCount += plotEllipseSymmetry(screen_cx, screen_cy, y, x, painter, col);
        else
            pointsCount += plotEllipseSymmetry(screen_cx, screen_cy, x, y, painter, col);
        maybeAnimate();
        qInfo() <<QString::number(d1, 'g', 17) << "X:-" << x << " " << "Y:- " << y << "\n";

        if (d1 < 0) {
            d1 += (Ry2*(2*x+3));
        } else {
            d1 += (Ry2*(2*x+3) + Rx2*(-2*y+2));
            y--;
        }

        x++;
    }

    long long d2 = (Ry2) * (x + 0.5) * (x + 0.5) + (Rx2) * (y - 1) * (y - 1) - (Rx2 * Ry2);
    while (y >= 0) {
        if(swapped)
            pointsCount += plotEllipseSymmetry(screen_cx, screen_cy, y, x, painter, col);
        else
            pointsCount += plotEllipseSymmetry(screen_cx, screen_cy, x, y, painter, col);
        maybeAnimate();
        if(d2 < 0){
            d2 += (Ry2*(2*x+2) + Rx2*(-2*y + 3));
            x++;
        }
        else{
            d2 += (Ry2*(-2*y+3));

        }
        y--;
    }

    ui->frame->setPixmap(pm);
    return timer.nsecsElapsed();
}

void MainWindow::tick_draw_lines() {
    if (this->points.empty() && this->circles.empty() && this->ellipses.empty() && this->polygons.empty()) return;

    QPixmap pm = ui->frame->pixmap();
    QPainter painter(&pm);

    if(!this->points.empty()){
        for (const auto& point : std::as_const(points)) {
            auto [X1, X2, Y1, Y2, col] = point;
            this->draw_line_grid(X1, X2, Y1, Y2, painter, col, (col == Qt::red));
        }
    }

    if(!this->circles.empty()){
        for (const auto &c : std::as_const(circles)) {
            this->draw_circle_grid(c.CX, c.CY, c.R, painter, pm,
                                   c.col, 1, false);
        }
    }

    if(!this->ellipses.empty()){
        for (const auto &e : std::as_const(ellipses)) {
            int pts = 0;
            draw_ellipse_midpoint(e.CX, e.CY, e.RX, e.RY, painter, pm,
                                  e.col, false, pts);
        }
    }

    if(!this->polygons.empty()){
        for (const auto &poly : std::as_const(polygons)) {
            if(poly.vertices.size() >= 3){
                draw_polygon(painter, poly);
            }
        }
    }

    painter.end();
    ui->frame->setPixmap(pm);
    for (const auto &poly : std::as_const(polygons)) {
        if(poly.isFilled){
            scanlineFill(poly, poly.fillCol, false);
        }
    }
}


void MainWindow::on_draw_grid_clicked()
{
    this->draw_clicked = true;

    QPixmap pm = ui->frame->pixmap();
    QPainter painter(&pm);

    const int total_cells = this->grid_size / this->grid_box;
    this->center = total_cells / 2;

    this->draw_axes(painter);

    painter.setPen(QPen(Qt::white, 1));
    for (int i = 0; i <= total_cells; i++) {
        int pos = i * this->grid_box;
        painter.drawLine(QPoint(pos, 0), QPoint(pos, this->grid_size));
        painter.drawLine(QPoint(0, pos), QPoint(this->grid_size, pos));
    }

    painter.end();
    ui->frame->setPixmap(pm);
}

void MainWindow::on_grid_size_valueChanged(int grid)
{
    if (grid < 2) return;

    this->grid_box = grid;

    if (!this->draw_clicked) return;

    this->refresh();
    this->tick_draw_lines();
}

void MainWindow::on_algo_select_currentIndexChanged(int index)
{
    this->line_choose = index;
}

void MainWindow::draw_axes(QPainter& painter)
{
    int c = this->center;
    this->draw_line_grid(-c, c, 0, 0, painter, Qt::white, true);
    this->draw_line_grid(0, 0, -c, c, painter, Qt::white, true);
}
double MainWindow::draw_circle_cartesian(int CX, int CY, int R,
                                         QPainter& painter, QPixmap &pm,
                                         Qt::GlobalColor col,
                                         bool animate,
                                         int &pointsCount)
{
    pointsCount = 0;
    if (R <= 0) return 0.0;

    int screen_cx = CX + this->center;
    int screen_cy = this->center - CY;

    QElapsedTimer timer;
    timer.start();

    auto maybeAnimate = [&]() {
        if (animate) {
            ui->frame->setPixmap(pm);
            QCoreApplication::processEvents();
            QThread::msleep(50);
        }
    };

    int x = 0;
    while (x <= R) {
        int y = qRound(qSqrt(R*R - x*x));
        plotCircleSymmetry(screen_cx, screen_cy, x, y, painter, col);
        pointsCount += 8;
        maybeAnimate();
        x++;
    }

    ui->frame->setPixmap(pm);
    return timer.nsecsElapsed() / 1e6;
}

void MainWindow::on_draw_line_clicked()
{
    if (lastPoint1.x() == -1 || lastPoint2.x() == -1) return;

    int X1 = (lastPoint1.x() / this->grid_box) - this->center;
    int Y1 = -(lastPoint1.y() / this->grid_box) + this->center;

    int X2 = (lastPoint2.x() / this->grid_box) - this->center;
    int Y2 = -(lastPoint2.y() / this->grid_box) + this->center;

    bool use_bresenham = (this->line_choose == 0);
    this->points.push_back({X1, X2, Y1, Y2, use_bresenham ? Qt::red : Qt::green});

    QPixmap pm = ui->frame->pixmap();
    QPainter painter(&pm);

    double time = this->draw_line_grid(X1, X2, Y1, Y2, painter,
                                       (use_bresenham ? Qt::red : Qt::green), use_bresenham);
    painter.end();

    ui->time_algo->setText(QString("Execution time: %1 ms").arg(time, 0, 'f', 6));
    ui->frame->setPixmap(pm);
}

void MainWindow::on_draw_circle_clicked()
{
    QPoint centerPoint = (lastPoint2.x() != -1) ? lastPoint2 : lastPoint1;

    bool useCartesian = ui->algo_select->currentText().contains("Cartesian");

    if (centerPoint.x() == -1) return;
    bool useMidpoint = ui->algo_select->currentText().contains("Midpoint");

    int CX = (centerPoint.x() / this->grid_box) - this->center;
    int CY = -(centerPoint.y() / this->grid_box) + this->center;


    QPixmap pm = ui->frame->pixmap();
    QPainter painter(&pm);
    if(useCartesian){
        circles.push_back({CX, CY, this->radius_circle, Qt::green});

        int pts = 0;
        double time = draw_circle_cartesian(CX, CY, this->radius_circle, painter, pm,
                                            Qt::green,
                                            true, pts);
        painter.end();
        ui->time_algo->setText(QString("Execution time: %1 ms, Points: %2")
                                   .arg(time/1e6, 0, 'f', 6)
                                   .arg(pts));
        ui->frame->setPixmap(pm);
        return;
    }

    circles.push_back({CX, CY, this->radius_circle, (useMidpoint ? Qt::cyan : Qt::magenta)});

    QVector<double> time = draw_circle_grid(CX, CY, this->radius_circle, painter, pm,
                                            (useMidpoint ? Qt::cyan : Qt::magenta),
                                            useMidpoint,
                                            true);

    painter.end();
    ui->time_algo->setText(QString("Execution time: %1 ms, Points: %2")
                               .arg(time[0]/1e6, 0, 'f', 6)
                               .arg(time[1]));    ui->frame->setPixmap(pm);
}


void MainWindow::on_radius_valueChanged(int arg1)
{
    this->radius_circle = arg1;
    if(!circles.empty()){
        auto circle = circles.back();
        circles.pop_back();
        circles.push_back({circle.CX, circle.CY, this->radius_circle, circle.col});

        QPixmap pix(ui->frame->width(), ui->frame->height());
        pix.fill(Qt::black);
        ui->frame->setPixmap(pix);

        this->on_draw_grid_clicked();
        this->tick_draw_lines();
    }

}

void MainWindow::on_draw_ellipse_clicked() {
    QPoint centerPoint = (lastPoint2.x() != -1) ? lastPoint2 : lastPoint1;
    if (centerPoint.x() == -1) return;

    int CX = (centerPoint.x() / this->grid_box) - this->center;
    int CY = -(centerPoint.y() / this->grid_box) + this->center;

    int RX = ui->radius_rx->value();
    int RY = ui->radius_ry->value();
    if (RX <= 0 || RY <= 0) return;


    bool useMidpoint = ui->algo_select->currentText().contains("Midpoint");
    ellipses.push_back({CX, CY, RX, RY, useMidpoint ? Qt::cyan : Qt::yellow});

    QPixmap pm = ui->frame->pixmap();
    QPainter painter(&pm);

    int pts = 0;
    double time = 0;

    if (useMidpoint) {
        time = draw_ellipse_midpoint(CX, CY, RX, RY, painter, pm, Qt::cyan, true, pts);
    } else {
        time = draw_ellipse_polar(CX, CY, RX, RY, painter, pm, Qt::yellow, true, pts);
    }

    painter.end();
    ui->time_algo->setText(QString("Execution time: %1 ns, Points: %2")
                               .arg(time)
                               .arg(pts));
    ui->frame->setPixmap(pm);
}



void MainWindow::on_radius_rx_valueChanged(int arg1)
{
    if(!ellipses.empty()){
        auto ellipse = ellipses.back();
        ellipses.pop_back();
        ellipses.push_back({ellipse.CX, ellipse.CY, arg1, ellipse.RY, ellipse.col});

        QPixmap pix(ui->frame->width(), ui->frame->height());
        pix.fill(Qt::black);
        ui->frame->setPixmap(pix);

        this->on_draw_grid_clicked();
        this->tick_draw_lines();
    }
}

void MainWindow::on_radius_ry_valueChanged(int arg1)
{
    if(!ellipses.empty()){
        auto ellipse = ellipses.back();
        ellipses.pop_back();
        ellipses.push_back({ellipse.CX, ellipse.CY, ellipse.RX, arg1, ellipse.col});

        QPixmap pix(ui->frame->width(), ui->frame->height());
        pix.fill(Qt::black);
        ui->frame->setPixmap(pix);

        this->on_draw_grid_clicked();
        this->tick_draw_lines();
    }
}

void MainWindow::draw_polygon(QPainter& painter, const Polygon& poly) {
    if (poly.vertices.size() < 2) return;

    for (int i = 0; i < poly.vertices.size() - 1; ++i) {
        QPoint p1 = poly.vertices[i];
        QPoint p2 = poly.vertices[i + 1];

        int X1 = p1.x();
        int Y1 = p1.y();
        int X2 = p2.x();
        int Y2 = p2.y();

        this->draw_line_grid(X1, X2, Y1, Y2, painter, poly.col, true);
    }

    // Close the polygon
    QPoint p_last = poly.vertices.back();
    QPoint p_first = poly.vertices.front();

    this->draw_line_grid(p_last.x(), p_first.x(), p_last.y(), p_first.y(), painter, poly.col, true);
}



void MainWindow::on_start_polygon_clicked()
{
    // if(lastPoint2 == QPoint(-1, -1) || lastPoint1 == QPoint(-1,-1)) return;

    if(ui->start_polygon->text().contains("Start")){
        this->polygonFlag = true;
        ui->start_polygon->setText("Close Polygon");
    }
    else if(ui->start_polygon->text().contains("Close")){
        if (currPoly.vertices.size() < 3) {
            qDebug() << "At least 3 points are needed to form a polygon.";
            this->polygonFlag = false;
            currPoly.vertices.clear();
            return;
        }
        this->currPoly.col = Qt::blue;
        this->polygonFlag = false;
        this->polygons.push_back(currPoly);

        QPixmap pm = ui->frame->pixmap();
        QPainter painter(&pm);

        this->draw_polygon(painter, currPoly);

        painter.end();
        ui->frame->setPixmap(pm);

        currPoly.vertices.clear();


        ui->start_polygon->setText("Start Polygon");
    }
}


void MainWindow::boundaryFill(int x, int y, Qt::GlobalColor newColor, Qt::GlobalColor boundaryColor, bool dir_8) {
    QPixmap pm = ui->frame->pixmap();
    if (pm.isNull()) return;

    const int cells = this->grid_size / this->grid_box;
    QImage img = pm.toImage();
    QColor fillQ(newColor);
    QColor boundaryQ(boundaryColor);

    int gx0 = qFloor(x / (double)this->grid_box);
    int gy0 = qFloor(y / (double)this->grid_box);

    if (gx0 < 0 || gy0 < 0 || gx0 >= cells || gy0 >= cells) return;

    auto cellHasBoundaryOrFill = [&](int gx, int gy) -> bool {
        if (gx < 0 || gy < 0 || gx >= cells || gy >= cells) return true;
        for (int dx = 0; dx < grid_box; ++dx)
            for (int dy = 0; dy < grid_box; ++dy) {
                QColor c = img.pixelColor(gx * grid_box + dx, gy * grid_box + dy);
                if (c == boundaryQ || c == fillQ) return true;
            }
        return false;
    };

    if (cellHasBoundaryOrFill(gx0, gy0)) return;

    QSet<QPair<int,int>> visited;
    QVector<QPoint> stack;
    stack.append(QPoint(gx0, gy0));

    QPainter painter(&pm);

    auto maybeAnimate = [&]() {
        ui->frame->setPixmap(pm);
        QCoreApplication::processEvents();
        QThread::msleep(30);
    };

    const QVector<QPoint> directions4 = {{1,0}, {-1,0}, {0,1}, {0,-1}};
    const QVector<QPoint> directions8 = {{1,1}, {-1,1}, {1,-1}, {-1,-1}};

    while (!stack.isEmpty()) {
        QPoint p = stack.takeLast();
        int gx = p.x();
        int gy = p.y();

        if (visited.contains(qMakePair(gx, gy))) continue;
        visited.insert(qMakePair(gx, gy));

        if (cellHasBoundaryOrFill(gx, gy)) continue;

        QRect rect(gx*grid_box, gy*grid_box, grid_box, grid_box);
        painter.fillRect(rect, QBrush(fillQ, Qt::SolidPattern));

        for(int dx=0; dx<grid_box; ++dx)
            for(int dy=0; dy<grid_box; ++dy)
                img.setPixelColor(gx*grid_box + dx, gy*grid_box + dy, fillQ);

        maybeAnimate();

        for (const auto &d : directions4) {
            int nx = gx + d.x();
            int ny = gy + d.y();
            if (!visited.contains(qMakePair(nx, ny)) && !cellHasBoundaryOrFill(nx, ny))
                stack.append(QPoint(nx, ny));
        }

        if(dir_8){
            for (const auto &d : directions8) {
                int nx = gx + d.x();
                int ny = gy + d.y();

                int adj1x = gx + d.x();
                int adj1y = gy;
                int adj2x = gx;
                int adj2y = gy + d.y();

                if (!visited.contains(qMakePair(nx, ny)) &&
                    !cellHasBoundaryOrFill(nx, ny) &&
                    !cellHasBoundaryOrFill(adj1x, adj1y) &&
                    !cellHasBoundaryOrFill(adj2x, adj2y))
                {
                    stack.append(QPoint(nx, ny));
                }
            }
        }
    }

    painter.end();
    ui->frame->setPixmap(pm);
}

void MainWindow::scanlineFill(const Polygon& poly, Qt::GlobalColor fillColor, bool animate) {
    if (poly.vertices.size() < 3) return;

    QColor fillQ(fillColor);

    int ymin = poly.vertices.front().y();
    int ymax = ymin;
    for (const QPoint &v : poly.vertices) {
        ymin = std::min(ymin, v.y());
        ymax = std::max(ymax, v.y());
    }
    QPixmap pm = ui->frame->pixmap();
    if (pm.isNull()) return;
    QPainter painter(&pm);

    const int cells = this->grid_size / this->grid_box;
    int n = poly.vertices.size();

    QImage img = pm.toImage();

    auto maybeAnimate = [&]() {
        if(animate){
            ui->frame->setPixmap(pm);
            QCoreApplication::processEvents();
            QThread::msleep(30);
        }
    };

    for (int y = ymin; y <= ymax; ++y) {
        QVector<double> interX;
        for (int i = 0; i < n; ++i) {
            QPoint p1 = poly.vertices[i];
            QPoint p2 = poly.vertices[(i + 1) % n];

            int x1 = p1.x(), y1 = p1.y();
            int x2 = p2.x(), y2 = p2.y();

            if (y1 == y2) continue;

            int yminEdge = std::min(y1, y2);
            int ymaxEdge = std::max(y1, y2);

            if ( (y >= yminEdge) && (y < ymaxEdge) ) {
                double t = ( (y + 0.5) - y1 ) / double(y2 - y1);
                double ix = x1 + t * (x2 - x1);
                interX.append(ix);
            }
        }

        if (interX.isEmpty()) continue;
        std::sort(interX.begin(), interX.end());

        for (int k = 0; k + 1 < interX.size(); k += 2) {
            int xLeft = (int)qCeil(interX[k]);
            int xRight = (int)qFloor(interX[k+1]);
            if (xRight < xLeft) continue;

            for (int lx = xLeft; lx <= xRight; ++lx) {
                int gx = lx + this->center;
                int gy = this->center - y;
                if (gx < 0 || gy < 0 || gx >= cells || gy >= cells) continue;

                int cx = gx * this->grid_box + this->grid_box / 2;
                int cy = gy * this->grid_box + this->grid_box / 2;
                QColor cur = img.pixelColor(cx, cy);

                if (cur == poly.col) continue;

                QRect rect(gx * this->grid_box, gy * this->grid_box,
                           this->grid_box, this->grid_box);
                painter.fillRect(rect, QBrush(fillQ, Qt::SolidPattern));
                maybeAnimate();

            }
        }
    }

    painter.end();
    ui->frame->setPixmap(pm);
}
void MainWindow::floodFill(int x, int y, Qt::GlobalColor newColor, bool dir_8) {
    QPixmap pm = ui->frame->pixmap();
    if (pm.isNull()) return;

    const int cells = this->grid_size / this->grid_box;
    int gx0 = qFloor(x / (double)this->grid_box);
    int gy0 = qFloor(y / (double)this->grid_box);

    if (gx0 < 0 || gy0 < 0 || gx0 >= cells || gy0 >= cells) return;

    QImage img = pm.toImage().convertToFormat(QImage::Format_ARGB32);

    QColor oldColor(Qt::black);
    QColor fillQ(newColor);

    QSet<QPair<int,int>> visited;
    QVector<QPoint> stack;
    stack.append(QPoint(gx0, gy0));

    QPainter painter(&img);

    auto maybeAnimate = [&]() {
        ui->frame->setPixmap(QPixmap::fromImage(img));
        QCoreApplication::processEvents();
        QThread::msleep(30);
    };

    auto cellCenterColor = [&](int gx, int gy) -> QColor {
        int cx = gx * this->grid_box + this->grid_box / 2;
        int cy = gy * this->grid_box + this->grid_box / 2;
        cx = qBound(0, cx, img.width() - 1);
        cy = qBound(0, cy, img.height() - 1);
        QColor cur = img.pixelColor(cx, cy);
        if (cur == Qt::GlobalColor::yellow) cur = Qt::GlobalColor::black;
        return cur;
    };

    auto isFillable = [&](int gx, int gy) -> bool {
        if (gx < 0 || gy < 0 || gx >= cells || gy >= cells) return false;
        QColor cur = cellCenterColor(gx, gy);
        if (cur == fillQ) return false;
        return (cur == oldColor);
    };

    while (!stack.isEmpty()) {
        QPoint p = stack.takeLast();
        int gx = p.x(), gy = p.y();

        if (gx < 0 || gy < 0 || gx >= cells || gy >= cells) continue;

        QPair<int,int> pair(gx, gy);
        if (visited.contains(pair)) continue;
        visited.insert(pair);

        if (!isFillable(gx, gy)) continue;

        QRect rect(gx * this->grid_box, gy * this->grid_box, this->grid_box, this->grid_box);
        painter.fillRect(rect, QBrush(fillQ, Qt::SolidPattern));

        maybeAnimate();

        if (isFillable(gx + 1, gy)) stack.append(QPoint(gx + 1, gy));
        if (isFillable(gx - 1, gy)) stack.append(QPoint(gx - 1, gy));
        if (isFillable(gx, gy + 1)) stack.append(QPoint(gx, gy + 1));
        if (isFillable(gx, gy - 1)) stack.append(QPoint(gx, gy - 1));

        if (dir_8) {
            if (isFillable(gx + 1, gy) && isFillable(gx, gy + 1) && isFillable(gx + 1, gy + 1))
                stack.append(QPoint(gx + 1, gy + 1));
            if (isFillable(gx - 1, gy) && isFillable(gx, gy + 1) && isFillable(gx - 1, gy + 1))
                stack.append(QPoint(gx - 1, gy + 1));
            if (isFillable(gx + 1, gy) && isFillable(gx, gy - 1) && isFillable(gx + 1, gy - 1))
                stack.append(QPoint(gx + 1, gy - 1));
            if (isFillable(gx - 1, gy) && isFillable(gx, gy - 1) && isFillable(gx - 1, gy - 1))
                stack.append(QPoint(gx - 1, gy - 1));
        }
    }

    painter.end();
    ui->frame->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::on_fill_polygon_clicked()
{

    if(ui->algo_select->currentText().toLower().contains("boundary")){
        if(lastPoint2 == QPoint(-1, -1)) return;
        this->polygons.back().isFilled = true;
        this->polygons.back().fillCol = Qt::gray;
        qInfo() << lastPoint2.x() << " " << lastPoint2.y();

        this->boundaryFill(lastPoint2.x(), lastPoint2.y(), Qt::gray, this->polygons.back().col, !ui->algo_select->currentText().toLower().contains("4"));
    }
    if (ui->algo_select->currentText().toLower().contains("flood")) {
        if (lastPoint2 == QPoint(-1, -1)) return;
        this->polygons.back().isFilled = true;
        this->polygons.back().fillCol = Qt::red;

        qInfo() << lastPoint2.x() << " " << lastPoint2.y();
        this->floodFill(lastPoint2.x(), lastPoint2.y(), Qt::red, !ui->algo_select->currentText().toLower().contains("4"));
    }

    if(ui->algo_select->currentText().toLower().contains("scan")){
        this->polygons.back().isFilled = true;
        this->polygons.back().fillCol = Qt::green;

        this->scanlineFill(this->polygons.back(), Qt::green, true);
    }
}


void MainWindow::on_clear_fill_clicked()
{
    auto polygon_copy = polygons;
    this->on_clear_clicked();
    for (auto &poly : polygon_copy) {
        if(poly.isFilled){
            poly.isFilled = false;
        }
    }
    this->polygons = polygon_copy;

    this->tick_draw_lines();
}


void translatePolygon(Polygon &poly, int dx, int dy) {
    QVector<QPoint> result;
    for (auto &p : poly.vertices) {


    }
    poly.vertices = result;
}
void translatePolygon(Polygon &poly, double dx, double dy) {
    Mat3 T = MatOps::matTranslate(dx, dy);
    MatOps::applyMatrixToPolygon(poly, T, true);
}

void MainWindow::on_translate_button_clicked()
{
    if(polygons.empty()) return;
    translatePolygon(this->polygons.back(), (double)ui->input_dx->value(), (double)ui->input_dy->value());
    this->refresh();
    this->tick_draw_lines();
}
void rotatePolygon(Polygon &poly, double angleDeg) {
    qDebug() << "AngleDeg =" << angleDeg;
    Mat3 R = MatOps::matRotate(angleDeg);
    MatOps::applyMatrixToPolygon(poly, R, true);
}
void scalePolygon(Polygon &poly, double sx, double sy) {
    Mat3 S = MatOps::matScale(sx, sy);
    MatOps::applyMatrixToPolygon(poly, S, true);
}

void shearPolygon(Polygon &poly, double sx, double sy) {
    Mat3 S = MatOps::matShear(sx, sy);
    MatOps::applyMatrixToPolygon(poly, S, true);
}
void rotatePolygonAboutPoint(Polygon &poly, double angleDeg, QPoint p1) {
    Mat3 R = MatOps::matRotateAboutPoint(angleDeg, p1.x(), p1.y());
    MatOps::applyMatrixToPolygon(poly, R, true);
}
void MainWindow::on_rotate_button_clicked()
{
    if(polygons.empty()) return;
    if(ui->rotate_select->currentText().contains("origin")){
        qInfo() << ui->input_angle->value() << "\n";
        rotatePolygon(this->polygons.back(),(double)ui->input_angle->value());
        this->refresh();
        this->tick_draw_lines();
    }
    else{
        int X = (this->lastPoint2.x() / this->grid_box) - this->center;
        int Y = -(this->lastPoint2.y() / this->grid_box) + this->center;
        rotatePolygonAboutPoint(this->polygons.back(),(double)ui->input_angle->value(), QPoint(X,Y));
        QPixmap pix(ui->frame->width(), ui->frame->height());
        pix.fill(Qt::black);
        ui->frame->setPixmap(pix);
        this->on_draw_grid_clicked();
        this->tick_draw_lines();
    }
}


void MainWindow::on_scale_button_clicked()
{
    if(polygons.empty()) return;
    scalePolygon(this->polygons.back(), (double)ui->input_sx->value(), (double)ui->input_sy->value());
    this->refresh();
    this->tick_draw_lines();
}


void MainWindow::on_shear_button_clicked()
{
    if(polygons.empty()) return;
    shearPolygon(this->polygons.back(), (double)ui->input_sx->value(), (double)ui->input_sy->value());
    this->refresh();
    this->tick_draw_lines();
}
void reflectX(Polygon &poly) {
    Mat3 S = MatOps::matReflectX();
    MatOps::applyMatrixToPolygon(poly, S, true);
}
void reflectY(Polygon &poly) {
    Mat3 S = MatOps::matReflectY();
    MatOps::applyMatrixToPolygon(poly, S, true);
}
void reflectLine(Polygon &poly, QPointF& p1, QPointF& p2) {
    Mat3 S = MatOps::matReflectAboutLine(p1, p2);
    MatOps::applyMatrixToPolygon(poly, S, true);
}
void MainWindow::on_reflect_button_clicked()
{
    if(this->polygons.empty()) return;
    if(ui->reflect_select->currentText().contains("X")){
        reflectX(this->polygons.back());
    }
    else if(ui->reflect_select->currentText().contains("Y")){
        reflectY(this->polygons.back());
    }
    else{
        if(!points.empty()){
            auto [x1, x2, y1, y2, algo] = points.back();

            QPointF p1(x1, y1);
            QPointF p2(x2, y2);

            reflectLine(this->polygons.back(), p1, p2);

        }
    }
    this->refresh();
    this->tick_draw_lines();
}
void MainWindow::on_set_clipping_window_clicked()
{
    if (lastPoint1.x() == -1 || lastPoint2.x() == -1) return;

    int X1 = (lastPoint1.x() / grid_box) - center;
    int Y1 = -(lastPoint1.y() / grid_box) + center;
    int X2 = (lastPoint2.x() / grid_box) - center;
    int Y2 = -(lastPoint2.y() / grid_box) + center;

    clipTopLeft = QPoint(std::min(X1, X2), std::max(Y1, Y2));
    clipBottomRight = QPoint(std::max(X1, X2), std::min(Y1, Y2));
    clipWindowSet = true;

    int xLeft = clipTopLeft.x();
    int xRight = clipBottomRight.x();
    int yTop = clipTopLeft.y();
    int yBottom = clipBottomRight.y();

    QPixmap pm = ui->frame->pixmap();
    QPainter painter(&pm);

    draw_line_grid(xLeft, xRight, yTop, yTop, painter, Qt::yellow, true);     // Top
    draw_line_grid(xRight, xRight, yBottom, yTop, painter, Qt::yellow, true); // Right
    draw_line_grid(xRight, xLeft, yBottom, yBottom, painter, Qt::yellow, true); // Bottom
    draw_line_grid(xLeft, xLeft, yBottom, yTop, painter, Qt::yellow, true);   // Left

    painter.end();
    ui->frame->setPixmap(pm);


    this->points.push_back({xLeft, xRight, yTop, yTop, Qt::yellow});       // Top edge
    this->points.push_back({xRight, xRight, yBottom, yTop, Qt::yellow});   // Right edge
    this->points.push_back({xRight, xLeft, yBottom, yBottom, Qt::yellow}); // Bottom edge
    this->points.push_back({xLeft, xLeft, yBottom, yTop, Qt::yellow});     // Left edge
}

int MainWindow::computeCode(int x, int y) {
    int code = INSIDE;
    if (x < clipTopLeft.x()) code |= LEFT;
    else if (x > clipBottomRight.x()) code |= RIGHT;
    if (y < clipBottomRight.y()) code |= BOTTOM;
    else if (y > clipTopLeft.y()) code |= TOP;
    return code;
}


bool MainWindow::cohenSutherlandClip(int &x1, int &y1, int &x2, int &y2) {
    if (!clipWindowSet) return false;

    int code1 = computeCode(x1, y1);
    int code2 = computeCode(x2, y2);
    bool accept = false;

    while (true) {
        if ((code1 | code2) == 0) { // both inside
            accept = true;
            break;
        } else if (code1 & code2) { // both outside
            break;
        } else {
            int codeOut = code1 ? code1 : code2;
            double x, y;

            if (codeOut & TOP) {
                x = x1 + (x2 - x1) * (clipTopLeft.y() - y1) / double(y2 - y1);
                y = clipTopLeft.y();
            } else if (codeOut & BOTTOM) {
                x = x1 + (x2 - x1) * (clipBottomRight.y() - y1) / double(y2 - y1);
                y = clipBottomRight.y();
            } else if (codeOut & RIGHT) {
                y = y1 + (y2 - y1) * (clipBottomRight.x() - x1) / double(x2 - x1);
                x = clipBottomRight.x();
            } else if (codeOut & LEFT) {
                y = y1 + (y2 - y1) * (clipTopLeft.x() - x1) / double(x2 - x1);
                x = clipTopLeft.x();
            }

            if (codeOut == code1) {
                x1 = qRound(x);
                y1 = qRound(y);
                code1 = computeCode(x1, y1);
            } else {
                x2 = qRound(x);
                y2 = qRound(y);
                code2 = computeCode(x2, y2);
            }
        }
    }
    return accept;
}
bool MainWindow::liangBarskyClip(int &x1, int &y1, int &x2, int &y2) {
    if (!clipWindowSet) return false;

    double xMin = std::min(clipTopLeft.x(), clipBottomRight.x());
    double xMax = std::max(clipTopLeft.x(), clipBottomRight.x());
    double yMin = std::min(clipTopLeft.y(), clipBottomRight.y());
    double yMax = std::max(clipTopLeft.y(), clipBottomRight.y());

    double dx = x2 - x1;
    double dy = y2 - y1;

    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x1 - xMin, xMax - x1, y1 - yMin, yMax - y1};

    double u1 = 0.0, u2 = 1.0;
    for (int i = 0; i < 4; i++) {
        if (fabs(p[i]) < 1e-9) {
            if (q[i] < 0) return false;  // parallel & outside
        } else {
            double r = q[i] / p[i];
            if (p[i] < 0)
                u1 = std::max(u1, r);
            else
                u2 = std::min(u2, r);
            if (u1 > u2) return false;
        }
    }

    double newX1 = x1 + u1 * dx;
    double newY1 = y1 + u1 * dy;
    double newX2 = x1 + u2 * dx;
    double newY2 = y1 + u2 * dy;

    x1 = static_cast<int>(std::lround(newX1));
    y1 = static_cast<int>(std::lround(newY1));
    x2 = static_cast<int>(std::lround(newX2));
    y2 = static_cast<int>(std::lround(newY2));

    return true;
}

void MainWindow::clip_line() {
    if (points.empty() || !clipWindowSet) return;

    auto [X1, X2, Y1, Y2, col] = points.back();
    this->points.pop_back();

    bool accepted = false;
    QString algo = ui->clip_algo_select->currentText().toLower();

    if (algo.contains("cohen"))
        accepted = cohenSutherlandClip(X1, Y1, X2, Y2);
    else if (algo.contains("liang"))
        accepted = liangBarskyClip(X1, Y1, X2, Y2);

    if (accepted) {
        this->points.push_back({X1, X2, Y1, Y2, col});
    }

    this->refresh();
    this->tick_draw_lines();
}

QVector<QVector<QPoint>> MainWindow::clipPolygonWeilerAtherton(const QVector<QPoint> &subjectPoly) {
    if (!clipWindowSet || subjectPoly.isEmpty()) return {subjectPoly};

    int xMin = std::min(clipTopLeft.x(), clipBottomRight.x());
    int xMax = std::max(clipTopLeft.x(), clipBottomRight.x());
    int yMin = std::min(clipTopLeft.y(), clipBottomRight.y());
    int yMax = std::max(clipTopLeft.y(), clipBottomRight.y());

    QVector<QPoint> clipPoly = {
        QPoint(xMin, yMin), QPoint(xMin, yMax),
        QPoint(xMax, yMax), QPoint(xMax, yMin)
    };
    // std::reverse(clipPoly.begin(), clipPoly.end());

    struct Node {
        QPoint pt;
        bool isIntersection = false;
        bool isEntry = false;
        bool visited = false;
        int neighborIndex = -1;
    };

    auto intersection = [&](QPoint p1, QPoint p2, QPoint q1, QPoint q2, QPoint &out) -> bool {
        double A1 = p2.y() - p1.y();
        double B1 = p1.x() - p2.x();
        double C1 = A1 * p1.x() + B1 * p1.y();

        double A2 = q2.y() - q1.y();
        double B2 = q1.x() - q2.x();
        double C2 = A2 * q1.x() + B2 * q1.y();

        double det = A1 * B2 - A2 * B1;
        if (fabs(det) < 1e-9) return false;

        double x = (B2 * C1 - B1 * C2) / det;
        double y = (A1 * C2 - A2 * C1) / det;

        if (x < std::min(p1.x(), p2.x()) - 1 || x > std::max(p1.x(), p2.x()) + 1 ||
            y < std::min(p1.y(), p2.y()) - 1 || y > std::max(p1.y(), p2.y()) + 1)
            return false;
        if (x < std::min(q1.x(), q2.x()) - 1 || x > std::max(q1.x(), q2.x()) + 1 ||
            y < std::min(q1.y(), q2.y()) - 1 || y > std::max(q1.y(), q2.y()) + 1)
            return false;

        out = QPoint((int)std::round(x), (int)std::round(y));
        return true;
    };

    auto inside = [&](const QPoint &p) {
        return (p.x() >= xMin && p.x() <= xMax && p.y() >= yMin && p.y() <= yMax);
    };

    QVector<Node> subjNodes;
    for (auto p : subjectPoly) subjNodes.push_back({p});

    QVector<Node> clipNodes;
    for (auto p : clipPoly) clipNodes.push_back({p});

    // --- Find and insert intersections ---
    QVector<QPair<int, Node>> subjInserts, clipInserts;
    for (int i = 0; i < subjectPoly.size(); ++i) {
        QPoint p1 = subjectPoly[i];
        QPoint p2 = subjectPoly[(i + 1) % subjectPoly.size()];

        for (int j = 0; j < clipPoly.size(); ++j) {
            QPoint q1 = clipPoly[j];
            QPoint q2 = clipPoly[(j + 1) % clipPoly.size()];

            QPoint inter;
            if (intersection(p1, p2, q1, q2, inter)) {
                bool entry = inside(p1) && !inside(p2);
                subjInserts.push_back({i + 1, {inter, true, entry, false, -1}});
                clipInserts.push_back({j + 1, {inter, true, entry, false, -1}});
            }
        }
    }

    // Insert intersections in reverse (so indices remain correct)
    std::sort(subjInserts.begin(), subjInserts.end(),
              [](auto &a, auto &b) { return a.first > b.first; });
    for (auto &p : subjInserts)
        subjNodes.insert(subjNodes.begin() + p.first, p.second);

    std::sort(clipInserts.begin(), clipInserts.end(),
              [](auto &a, auto &b) { return a.first > b.first; });
    for (auto &p : clipInserts)
        clipNodes.insert(clipNodes.begin() + p.first, p.second);

    // --- Link intersections ---
    for (int i = 0; i < subjNodes.size(); ++i)
        if (subjNodes[i].isIntersection)
            for (int j = 0; j < clipNodes.size(); ++j)
                if (clipNodes[j].isIntersection && subjNodes[i].pt == clipNodes[j].pt) {
                    subjNodes[i].neighborIndex = j;
                    clipNodes[j].neighborIndex = i;
                }

    // --- Traverse all intersection entry points ---
    QVector<QVector<QPoint>> resultPolys;
    for (int i = 0; i < subjNodes.size(); ++i) {
        if (subjNodes[i].isIntersection && subjNodes[i].isEntry && !subjNodes[i].visited) {
            QVector<QPoint> outPoly;
            int idx = i;
            bool inSubject = true;

            do {
                Node &curr = inSubject ? subjNodes[idx] : clipNodes[idx];
                curr.visited = true;
                outPoly.push_back(curr.pt);

                if (curr.isIntersection) {
                    inSubject = !inSubject;
                    idx = inSubject ? curr.neighborIndex : curr.neighborIndex;
                }

                if (inSubject)
                    idx = (idx + 1) % subjNodes.size();
                else
                    idx = (idx + 1) % clipNodes.size();

            } while (!(outPoly.size() > 1 && outPoly.front() == outPoly.back()));

            if (outPoly.size() > 3)
                resultPolys.push_back(outPoly);
        }
    }

    // If polygon is fully inside the clip region
    if (resultPolys.isEmpty() && inside(subjectPoly[0]))
        resultPolys.push_back(subjectPoly);

    return resultPolys;
}



QVector<QPoint> MainWindow::clipPolygonSuthHodg(const QVector<QPoint> &poly) {
    if (!clipWindowSet) return poly;

    QVector<QPoint> output = poly;

    QPoint tl = clipTopLeft;
    QPoint br = clipBottomRight;
    QPoint tr(br.x(), tl.y());
    QPoint bl(tl.x(), br.y());

    auto intersection = [&](QPoint p1, QPoint p2, QPoint p3, QPoint p4) {
        double A1 = p2.y() - p1.y();
        double B1 = p1.x() - p2.x();
        double C1 = A1 * p1.x() + B1 * p1.y();

        double A2 = p4.y() - p3.y();
        double B2 = p3.x() - p4.x();
        double C2 = A2 * p3.x() + B2 * p3.y();

        double det = A1 * B2 - A2 * B1;
        if (fabs(det) < 1e-9) return QPoint(0, 0);
        double x = (B2 * C1 - B1 * C2) / det;
        double y = (A1 * C2 - A2 * C1) / det;
        return QPoint(qRound(x), qRound(y));
    };

    auto clipEdge = [&](const QVector<QPoint> &inList, auto inside, QPoint e1, QPoint e2) {
        QVector<QPoint> out;
        for (int i = 0; i < inList.size(); ++i) {
            QPoint curr = inList[i];
            QPoint prev = inList[(i + inList.size() - 1) % inList.size()];

            bool in1 = inside(prev);
            bool in2 = inside(curr);

            if (in1 && in2) out.push_back(curr);
            else if (in1 && !in2)
                out.push_back(intersection(prev, curr, e1, e2));
            else if (!in1 && in2) {
                out.push_back(intersection(prev, curr, e1, e2));
                out.push_back(curr);
            }
        }
        return out;
    };

    output = clipEdge(output,
                      [&](const QPoint &pt) { return pt.x() >= tl.x(); },
                      tl, bl);

    output = clipEdge(output,
                      [&](const QPoint &pt) { return pt.x() <= br.x(); },
                      tr, br);

    output = clipEdge(output,
                      [&](const QPoint &pt) { return pt.y() >= br.y(); },
                      br, bl);

    output = clipEdge(output,
                      [&](const QPoint &pt) { return pt.y() <= tl.y(); },
                      tl, tr);

    return output;
}

void MainWindow::clip_polygon() {
    if (!clipWindowSet || this->polygons.empty()) return;

    QString algo = ui->clip_algo_select->currentText().toLower();
    QVector<QVector<QPoint>> clippedPolys;

    if (algo.contains("hodgeman")) {
        clippedPolys = {clipPolygonSuthHodg(this->polygons.back().vertices)};
    }
    else if (algo.contains("weiler")) {
        clippedPolys = clipPolygonWeilerAtherton(this->polygons.back().vertices);
    }

    auto col = this->polygons.back().col;
    this->polygons.pop_back();

    for (const auto poly : clippedPolys) {
        Polygon p;
        p.vertices = poly;
        for(auto v:poly){
            qDebug() << v.x() << " " << v.y() << " ";
        }
        p.col = col;
        this->polygons.push_back(p);
    }

    this->refresh();
    this->tick_draw_lines();
}


void MainWindow::on_clip_button_clicked()
{
    if(ui->clip_algo_select->currentText().toLower().contains("cohen") || ui->clip_algo_select->currentText().toLower().contains("liang")) {
        this->clip_line();
    }
    if(ui->clip_algo_select->currentText().toLower().contains("hodgeman") || ui->clip_algo_select->currentText().toLower().contains("weiler")){
        this->clip_polygon();
    }
}
