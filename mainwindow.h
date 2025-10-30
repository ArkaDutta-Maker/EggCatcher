#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPixmap>
#include <QTimer>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QVector>
#include <QPointF>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:
    void gameTick();

private:
    Ui::MainWindow *ui;
    void drawGame(float alpha);
    void drawGameOver();
    bool gameOver;
    // --- Grid / display ---
    int grid_box;
    int grid_size;
    int cols, rows;
    QPixmap background;
    QVector<int> dropColumns;     // 4 centered columns
    int currentColumnIndex;        // which column will spawn next
    float globalSpawnTimer;        // counts time since last spawn
    float spawnInterval;           // delay between spawns
    QVector<float> columnTimers;       // timer accumulator per column
    QVector<float> columnDelays;       // spawn delay per column (seconds)

    // --- Basket ---
    QPointF basket;     // subpixel position
    float basketXVelocity = 0.0f;
    float basketTargetVel = 0.0f;
    float basketAccel = 20.0f;   // acceleration factor for smoothing
    float basketMaxVel = 12.0f;  // max speed in cells/sec
    float lastEdgeSpawnTime;
    float edgeSpawnCooldown;
    float globalTime;
    // --- Eggs ---
    QVector<QPointF> eggs;

    // --- Game state ---
    int score;
    int lives;

    // --- Timer / frame control ---
    QTimer *gameTimer;
    void updatePhysics(float dt);

    // --- Input flags ---
    bool moveLeft;
    bool moveRight;
    QElapsedTimer frameClock;
    float accumulator = 0.0f;
    float fixedDelta = 1.0f / 120.0f;  // logic update at 120 Hz (high precision)

};

#endif // MAINWINDOW_H
