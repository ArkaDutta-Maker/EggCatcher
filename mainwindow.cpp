#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QtMath>
#include <algorithm>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
    moveLeft(false), moveRight(false)
{
    ui->setupUi(this);
    setFocusPolicy(Qt::StrongFocus);


    // --- Grid setup ---
    grid_box = 20;
    grid_size = ui->frame->frameSize().width();
    cols = grid_size / grid_box;
    rows = grid_size / grid_box;

    // --- Background grid ---
    QPixmap bg(grid_size, grid_size);
    bg.fill(Qt::black);
    QPainter g(&bg);
    g.setPen(QPen(Qt::darkGray, 1));
    for (int i = 0; i <= cols; i++)
        g.drawLine(i * grid_box, 0, i * grid_box, grid_size);
    for (int j = 0; j <= rows; j++)
        g.drawLine(0, j * grid_box, grid_size, j * grid_box);
    g.end();
    background = bg;
    ui->frame->setPixmap(background);

    // --- Game setup ---
    score = 0;
    lives = 3;
    basket = QPointF(cols / 2.0f, rows - 3);

    basketXVelocity = 0.0f;
    basketTargetVel = 0.0f;
    basketAccel = 20.0f;
    basketMaxVel = 12.0f;

    // --- Timer setup ---
    gameTimer = new QTimer(this);
    connect(gameTimer, &QTimer::timeout, this, &MainWindow::gameTick);
    gameTimer->start(16);  // ~60 FPS
    frameClock.start();
    fixedDelta = 1.0f / 120.0f;
    accumulator = 0.0f;
    gameOver = false;
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    if (event->isAutoRepeat()) return;

    if (gameOver && event->key() == Qt::Key_R) {
        resetGame();
        return;
    }

    if (!gameRunning && event->key() == Qt::Key_Return) {
        gameRunning = true;
        return;
    }

    if (gameOver) return;
    if (!gameRunning) return;

    if (event->key() == Qt::Key_A)
        moveLeft = true;
    else if (event->key() == Qt::Key_D)
        moveRight = true;
}

void MainWindow::keyReleaseEvent(QKeyEvent *event) {
    if (event->isAutoRepeat()) return;
    if (!gameRunning || gameOver) return;
    if (gameOver) return;

    if (event->key() == Qt::Key_A)
        moveLeft = false;
    else if (event->key() == Qt::Key_D)
        moveRight = false;
}

void MainWindow::resetGame() {
    score = 0;
    lives = 3;
    eggs.clear();
    basket = QPointF(cols / 2.0f, rows - 3);
    basketXVelocity = 0.0f;
    basketTargetVel = 0.0f;
    gameOver = false;
    accumulator = 0.0f;
    frameClock.restart();
    gameRunning = true;
    gameTimer->start();
}


void MainWindow::gameTick() {
    if (gameOver) {
        drawGameOver();
        return;
    }

    // If game hasn't started yet → just draw start screen
    if (!gameRunning) {
        drawStartScreen();
        return;
    }

    float dt = frameClock.restart() / 1000.0f;
    accumulator += dt;

    while (accumulator >= fixedDelta) {
        updatePhysics(fixedDelta);
        accumulator -= fixedDelta;
    }

    float alpha = accumulator / fixedDelta;
    drawGame(alpha);

    // --- Check game over ---
    if (lives <= 0) {
        gameOver = true;
        drawGameOver();
        gameTimer->stop();
    }
}

void MainWindow::drawStartScreen() {
    QPixmap pix = background;
    QPainter p(&pix);

    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 20, QFont::Bold));
    p.drawText(pix.rect(), Qt::AlignCenter,
               "EGG CATCHER\n\nPress ENTER to Start");

    p.end();
    ui->frame->setPixmap(pix);
}


void MainWindow::drawGameOver() {
    QPixmap pix = background;
    QPainter p(&pix);
    p.setPen(Qt::red);
    p.setFont(QFont("Arial", 24, QFont::Bold));
    p.drawText(pix.rect(), Qt::AlignCenter, "GAME OVER\nPress R to Restart");
    p.end();
    ui->frame->setPixmap(pix);
}
void MainWindow::updatePhysics(float dt) {
    if (gameOver) return;

    // --- Basket momentum ---
    if (moveLeft && !moveRight)
        basketTargetVel = -basketMaxVel;
    else if (moveRight && !moveLeft)
        basketTargetVel = basketMaxVel;
    else
        basketTargetVel = 0.0f;

    float blend = 1.0f - qExp(-basketAccel * dt);
    basketXVelocity += (basketTargetVel - basketXVelocity) * blend;

    basket.setX(basket.x() + basketXVelocity * dt);
    basket.setX(std::clamp((float)basket.x(), 0.0f, float(cols - 1)));

    // --- Egg falling ---
    const float fallSpeed = 3.0f;
    for (auto &egg : eggs)
        egg.setY(egg.y() + fallSpeed * dt);

    // --- Collision detection (inverted bowl) ---
    QVector<QPointF> newEggs;
    int bx = qRound(basket.x());
    int by = qRound(basket.y());

    QVector<QPoint> basketCells = {
        {bx-2, by}, {bx-1, by}, {bx, by}, {bx+1, by}, {bx+2, by}, // top row
        {bx-1, by+1}, {bx, by+1}, {bx+1, by+1},                   // middle row
        {bx, by+2}                                                 // bottom tip
    };

    // create basket rectangles (logical grid units)
    QVector<QRectF> basketRects;
    for (auto &cell : basketCells) {
        int cx = std::clamp(cell.x(), 0, cols-1);
        int cy = std::clamp(cell.y(), 0, rows-1);
        basketRects.push_back(QRectF(cx - 0.1f, cy - 0.1f, 1.2f, 1.2f)); // small margin for safety
    }

    for (auto &egg : eggs) {
        if (egg.y() >= rows - 1) {
            lives--;
            continue;
        }

        QRectF eggRect(egg.x(), egg.y(), 1.0f, 1.0f);
        bool caught = false;
        for (auto &bRect : basketRects) {
            if (eggRect.intersects(bRect)) {
                score++;
                caught = true;
                break;
            }
        }


        if (!caught)
            newEggs.push_back(egg);
    }

    eggs = newEggs;

    // --- Random egg spawn ---
    if (QRandomGenerator::global()->bounded(100) > 98) {
        float newX = QRandomGenerator::global()->bounded(float(cols - 1));
        eggs.append(QPointF(newX, 0.0f));
    }
}

void MainWindow::drawGame(float alpha) {
    QPixmap framePix = background;
    QPainter painter(&framePix);

    auto snapToGrid = [&](float coord) { return qRound(coord) * grid_box; };

    // --- Interpolated basket render ---
    float basketRenderX = basket.x() + basketXVelocity * alpha * fixedDelta;
    int bx = qRound(basketRenderX);
    int by = qRound(basket.y());

    QVector<QPoint> basketCells = {
        {bx-2, by}, {bx-1, by}, {bx, by}, {bx+1, by}, {bx+2, by},
        {bx-1, by+1}, {bx, by+1}, {bx+1, by+1},
        {bx, by+2}
    };

    painter.setBrush(Qt::blue);
    painter.setPen(Qt::NoPen);
    for (auto &cell : basketCells) {
        int cx = std::clamp(cell.x(), 0, cols-1);
        int cy = std::clamp(cell.y(), 0, rows-1);
        QRectF rect(cx * grid_box, cy * grid_box, grid_box, grid_box);
        painter.fillRect(rect, Qt::blue);
    }

    // --- Eggs render ---
    const float fallSpeed = 6.0f;
    for (auto &egg : eggs) {
        float eggRenderY = egg.y() + fallSpeed * alpha * fixedDelta;
        QRectF eggRect(
            snapToGrid(egg.x()),
            snapToGrid(eggRenderY),
            grid_box,
            grid_box
            );
        painter.fillRect(eggRect, Qt::yellow);
    }

    // --- Score / Lives HUD ---
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 10));
    painter.drawText(10, 20,
                     QString("Score: %1   Lives: %2").arg(score).arg(lives));

    painter.end();
    ui->frame->setPixmap(framePix);
}
