#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPoint>
#include <qpainter.h>
#include <array>
#include <cmath>

struct CircleRec {
    int CX, CY, R;
    Qt::GlobalColor col;
};

struct Ellipse{
    int CX, CY;
    int RX, RY;
    Qt::GlobalColor col;
};


struct Polygon{
    QVector<QPoint> vertices;
    Qt::GlobalColor col;
    Qt::GlobalColor fillCol;
    bool isFilled = false;
};

using Mat3 = std::array<std::array<double,3>,3>;

namespace MatOps{
static Mat3 matIdentity();
static Mat3 matTranslate(double dx, double dy);
static Mat3 matRotate(double angleDeg);
static Mat3 matScale(double sx, double sy);
static Mat3 matShear(double shx, double shy);
static Mat3 matReflectX();
static Mat3 matReflectY();

static Mat3 matReflectAboutLine(const QPointF &p1, const QPointF &p2);
static Mat3 matRotateAboutPoint(double angleDeg, double cx, double cy);

static Mat3 matMultiply(const Mat3 &A, const Mat3 &B);
static QPointF applyMatToPoint(const Mat3 &M, const QPointF &pt);

void extracted(Polygon &poly, const Mat3 &M, QVector<QPoint> &out);
void applyMatrixToPolygon(Polygon &poly, const Mat3 &M,
                          bool inplace = true);
};
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void Mouse_Pressed();
    void showMousePosition(QPoint &pos);
    void on_clear_clicked();

    void on_draw_grid_clicked();
    void on_grid_size_valueChanged(int arg1);
    void tick_draw_lines();
    void refresh();

    void on_algo_select_currentIndexChanged(int index);

    void on_draw_line_clicked();

    void on_draw_circle_clicked();
    void on_radius_valueChanged(int arg1);

    void on_draw_ellipse_clicked();

    void on_radius_rx_valueChanged(int arg1);

    void on_radius_ry_valueChanged(int arg1);

    void on_start_polygon_clicked();


    void on_fill_polygon_clicked();

    void on_clear_fill_clicked();

    void on_translate_button_clicked();

    void on_rotate_button_clicked();

    void on_scale_button_clicked();

    void on_shear_button_clicked();

    void on_reflect_button_clicked();

    void on_set_clipping_window_clicked();

    void on_clip_button_clicked();

private:
    int line_choose;
    Ui::MainWindow *ui;
    void addPoint(int x, int y);
    QVector<std::tuple<int, int, int, int, Qt::GlobalColor>> points;
    QVector<CircleRec> circles;
    QVector<Ellipse> ellipses;
    QVector<Polygon> polygons;

    QPoint lastPoint1;
    QPoint lastPoint2;

    QPoint clipBottomRight;
    QPoint clipTopLeft;
    bool clipWindowSet;
    Polygon currPoly;
    int sc_x, sc_y;
    int org_x, org_y;
    int grid_box;
    bool draw_clicked;
    int grid_size;
    int center;
    int radius_circle;

    bool polygonFlag;
private:
    const int INSIDE = 0; // 0000
    const int LEFT = 1;   // 0001
    const int RIGHT = 2;  // 0010
    const int BOTTOM = 4; // 0100
    const int TOP = 8;    // 1000

    int plotEllipseSymmetry(int screen_cx, int screen_cy, int x, int y,
                            QPainter& painter, Qt::GlobalColor col);
    double draw_ellipse_polar(int CX, int CY, int RX, int RY,
                              QPainter& painter, QPixmap &pm,
                              Qt::GlobalColor col, bool animate,
                              int &pointsCount);

    double draw_ellipse_midpoint(int CX, int CY, int RX, int RY,
                                 QPainter& painter, QPixmap &pm,
                                 Qt::GlobalColor col, bool animate,
                                 int &pointsCount);
    double draw_line_grid(int x1, int x2, int y1, int y2, QPainter& painter, Qt::GlobalColor col, bool bla);
    int plotCircleSymmetry(int CX, int CY, int x, int y,
                           QPainter& painter, Qt::GlobalColor col);
    double draw_circle_cartesian(int CX, int CY, int R,
                                 QPainter& painter, QPixmap &pm,
                                 Qt::GlobalColor col,
                                 bool animate,
                                 int &pointsCount);
    QVector<double> draw_circle_grid(int CX, int CY, int R, QPainter& painter, QPixmap &pm, Qt::GlobalColor col, bool mid, bool animate);
    void draw_axes(QPainter& painter);
    void scanlineFill(const Polygon& poly, Qt::GlobalColor fillColor, bool animate = true);
    bool isBoundaryPoint(int x, int y);
    void boundaryFill(int x, int y, Qt::GlobalColor fillColor, Qt::GlobalColor boundaryColor, bool dir_8 = true);
    void floodFill(int x, int y, Qt::GlobalColor newColor, bool dir_8 = true);
    void draw_polygon(QPainter& painter, const Polygon& poly);
    bool cohenSutherlandClip(int &x1, int &y1, int &x2, int &y2);
    QVector<QPoint> clipPolygonSuthHodg(const QVector<QPoint> &poly);
    QVector<QVector<QPoint>> clipPolygonWeilerAtherton(const QVector<QPoint> &poly);
    void clip_polygon();
    bool liangBarskyClip(int &x1, int &y1, int &x2, int &y2);
    void clip_line();
    int computeCode(int x, int y);

};

#endif // MAINWINDOW_H
