#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QtMath>
#include <algorithm>
#include <QDebug>

/* ============================================================
    HELPER — Draw egg (semi + ellipse)
============================================================ */
static void drawEggShape(QPainter &p, int cx, int cy, int box)
{
    p.setBrush(Qt::white);
    p.setPen(Qt::NoPen);

    auto plot = [&](int gx, int gy){
        p.fillRect(gx * box, gy * box, box, box, Qt::white);
    };

    /* ======================================================
       SEMICIRCLE BOTTOM
       (midpoint circle)
    ====================================================== */
    int r = box;
    int x = 0;
    int y = r;
    int d = 1 - r;

    QVector<QPoint> boundary;

    while (x <= y)
    {
        int px[4] = { x,  y, -x, -y };
        int py[4] = { y,  x,  y,  x };

        for (int i = 0; i < 4; i++)
        {
            int gx = cx + px[i];
            int gy = cy + py[i];
            if (gy >= cy)
                boundary.push_back({gx, gy});
        }

        x++;
        if (d < 0) d += 2*x + 1;
        else { y--; d += 2*(x - y) + 1; }
    }

    /* ======================================================
       TOP ELLIPSE
       (midpoint ellipse)
    ====================================================== */
    int rx = box;
    int ry = box * 1.5;
    int rx2 = rx * rx;
    int ry2 = ry * ry;
    int ex = 0;
    int ey = ry;

    float d1 = ry2 - rx2 * ry + (0.25f * rx2);
    int dx = 2 * ry2 * ex;
    int dy = 2 * rx2 * ey;

    while (dx < dy)
    {
        boundary.push_back({cx + ex, cy - ey});
        boundary.push_back({cx - ex, cy - ey});

        if (d1 < 0) {
            ex++;
            dx += 2*ry2;
            d1 += dx + ry2;
        } else {
            ex++; ey--;
            dx += 2*ry2;
            dy -= 2*rx2;
            d1 += dx - dy + ry2;
        }
    }

    float d2 =
        (ry2)*(ex+0.5f)*(ex+0.5f) +
        (rx2)*(ey-1)*(ey-1) -
        (rx2*ry2);

    while (ey >= 0)
    {
        boundary.push_back({cx + ex, cy - ey});
        boundary.push_back({cx - ex, cy - ey});

        if (d2 > 0) {
            ey--;
            dy -= 2*rx2;
            d2 += rx2 - dy;
        } else {
            ey--; ex++;
            dx += 2*ry2;
            dy -= 2*rx2;
            d2 += dx - dy + rx2;
        }
    }

    /* ======================================================
       FILL SCANLINES INSIDE
    ====================================================== */
    std::sort(boundary.begin(), boundary.end(),
              [](auto &a, auto &b){
                  return (a.y() == b.y()) ? a.x() < b.x() : a.y() < b.y();
              });

    int i = 0;
    while (i < boundary.size())
    {
        int y = boundary[i].y();
        QVector<int> xs;

        while (i < boundary.size() && boundary[i].y() == y) {
            xs.push_back(boundary[i].x());
            i++;
        }

        std::sort(xs.begin(), xs.end());
        for (int k = 0; k + 1 < xs.size(); k += 2) {
            for (int xF = xs[k]; xF <= xs[k+1]; xF++)
                plot(xF, y);
        }
    }
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
    moveLeft(false), moveRight(false)
{
    ui->setupUi(this);
    setFocusPolicy(Qt::StrongFocus);

    /* ------------------------------------------------------------
        GRID
    ------------------------------------------------------------ */
    grid_box = 5;
    grid_size = ui->frame->frameSize().width();
    cols = grid_size / grid_box;
    rows = grid_size / grid_box;

    /* ------------------ background grid --------------------- */
    QPixmap bg(grid_size, grid_size);
    bg.fill(Qt::green);
    {
        QPainter g(&bg);
        g.setPen(QPen(Qt::darkGray, 1));
        for (int i = 0; i <= cols; i++)
            g.drawLine(i * grid_box, 0, i * grid_box, grid_size);
        for (int j = 0; j <= rows; j++)
            g.drawLine(0, j * grid_box, grid_size, j * grid_box);
    }
    background = bg;
    ui->frame->setPixmap(background);

    /* ------------------------------------------------------------
        GAME VARS
    ------------------------------------------------------------ */
    score = 0;
    lives = 3;
    basket = QPointF(cols / 2.0f, rows - 3);

    basketXVelocity = 0.0f;
    basketTargetVel = 0.0f;
    basketAccel     = 20.0f;
    basketMaxVel    = 12.0f;

    /* ------------------------------------------------------------
        SOUNDS
    ------------------------------------------------------------ */
    soundCatch.setSource(QUrl::fromLocalFile("/Users/pavel/Documents/Graphics/EggCatcher/EggCatcher/sfx/catch.wav"));
    soundCatch.setVolume(0.8f);

    soundLose.setSource(QUrl::fromLocalFile("/Users/pavel/Documents/Graphics/EggCatcher/EggCatcher/sfx/lose.wav"));
    soundLose.setVolume(0.9f);

    qDebug() << "Catch status:" << soundCatch.status();
    qDebug() << "Lose status:" << soundLose.status();

    /* ------------------------------------------------------------
        TIMER
    ------------------------------------------------------------ */
    gameTimer = new QTimer(this);
    connect(gameTimer, &QTimer::timeout, this, &MainWindow::gameTick);
    gameTimer->start(16);
    frameClock.start();
    fixedDelta = 1.0f / 120.0f;
    accumulator = 0.0f;
    gameOver = false;


    /* ------------------------------------------------------------
        DROP COLUMNS (MORE SPACED)
    ------------------------------------------------------------ */
    dropColumns.clear();
    int mid = cols / 2;
    int gap = 40;  // increase to increase spacing
    dropColumns = { mid - gap, mid - (gap/2), mid + (gap/2), mid + gap };
    std::sort(dropColumns.begin(), dropColumns.end());

    columnTimers.resize(dropColumns.size());
    columnDelays.resize(dropColumns.size());
    for (int i = 0; i < dropColumns.size(); ++i) {
        columnTimers[i] = 0.0f;
        columnDelays[i] = 6.0f + QRandomGenerator::global()->bounded(2.0f);
    }

    currentColumnIndex = 0;
    globalSpawnTimer   = 0.0f;
    spawnInterval      = 1.0f;
    globalTime         = 0.0f;

    lastEdgeSpawnTime  = -100.0f;
    edgeSpawnCooldown  = 4.0f;
}


MainWindow::~MainWindow()
{
    delete ui;
}



/* ============================================================
    INPUT
============================================================ */
void MainWindow::keyPressEvent(QKeyEvent *event)
{
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

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) return;
    if (gameOver) return;
    if (!gameRunning) return;

    if (event->key() == Qt::Key_A)
        moveLeft = false;
    else if (event->key() == Qt::Key_D)
        moveRight = false;
}



/* ============================================================
    RESET
============================================================ */
void MainWindow::resetGame()
{
    ui->scoreLabel->show();
    ui->livesLabel->show();
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



/* ============================================================
    START SCREEN
============================================================ */
void MainWindow::drawStartScreen()
{
    QPixmap pix = background;
    QPainter p(&pix);

    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 20, QFont::Bold));
    p.drawText(pix.rect(), Qt::AlignCenter,
               "EGG CATCHER\n\nPress ENTER to Start");

    p.end();
    ui->frame->setPixmap(pix);
}



/* ============================================================
    GAME OVER
============================================================ */
void MainWindow::drawGameOver()
{
    ui->scoreLabel->hide();
    ui->livesLabel->hide();
    QPixmap pix = background;
    QPainter p(&pix);

    p.setPen(Qt::red);
    p.setFont(QFont("Arial", 24, QFont::Bold));
    p.drawText(pix.rect(), Qt::AlignCenter,
               "GAME OVER\nPress R to Restart");

    p.end();
    ui->frame->setPixmap(pix);
}



/* ============================================================
    MAIN TICK
============================================================ */
void MainWindow::gameTick()
{
    if (gameOver) {
        drawGameOver();
        return;
    }

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

    if (lives <= 0) {
        gameOver = true;
        drawGameOver();
        gameTimer->stop();
    }
}



/* ============================================================
    PHYSICS
============================================================ */
void MainWindow::updatePhysics(float dt)
{
    if (gameOver) return;

    globalTime      += dt;
    globalSpawnTimer += dt;


    /* ------------------- SPAWNING ------------------- */
    if (globalSpawnTimer >= spawnInterval) {

        int col = dropColumns[currentColumnIndex];
        bool isEdgeCol = (col == dropColumns.front() || col == dropColumns.back());
        bool canSpawn  = true;

        float dynamicEdgeCooldown =
            std::max(1.0f, edgeSpawnCooldown - 0.03f * score);

        if (isEdgeCol && (globalTime - lastEdgeSpawnTime < dynamicEdgeCooldown))
            canSpawn = false;

        if (canSpawn) {
            eggs.append(QPointF(float(col), 0.0f));
            if (isEdgeCol)
                lastEdgeSpawnTime = globalTime;
        }

        currentColumnIndex = (currentColumnIndex + 1) % dropColumns.size();
        globalSpawnTimer   = 0.0f;
        spawnInterval      = 2.0f + QRandomGenerator::global()->bounded(1.0f);
    }


    /* ------------------- BASKET MOVEMENT ------------------- */
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


    /* ------------------- EGGS FALL ------------------- */
    float fallSpeed = 3.0f + 0.15f * score;
    fallSpeed = std::min(fallSpeed, 25.0f);

    for (auto &egg : eggs)
        egg.setY(egg.y() + fallSpeed * dt);


    /* ------------------- COLLISION CHECK ------------------- */
    QVector<QPointF> newEggs;
    int bx = qRound(basket.x());
    int by = qRound(basket.y());

    // enlarged basket mask
    QVector<QPoint> basketCells = {
        {bx-3, by}, {bx-2, by}, {bx-1, by}, {bx, by}, {bx+1, by}, {bx+2, by}, {bx+3, by},
        {bx-2, by+1}, {bx-1, by+1}, {bx, by+1}, {bx+1, by+1}, {bx+2, by+1},
        {bx-1, by+2}, {bx, by+2}, {bx+1, by+2},
        {bx, by+3}
    };

    QVector<QRectF> basketRects;
    for (auto &cell : basketCells) {
        int cx = std::clamp(cell.x(), 0, cols-1);
        int cy = std::clamp(cell.y(), 0, rows-1);
        basketRects.push_back(QRectF(cx - 0.1f, cy - 0.1f, 1.2f, 1.2f));
    }

    for (auto &egg : eggs) {
        if (egg.y() >= rows - 1) {
            lives--;
            soundLose.play();
            continue;
        }

        QRectF eggRect(egg.x(), egg.y(), 1.0f, 1.0f);
        bool caught = false;

        for (auto &bRect : basketRects) {
            if (eggRect.intersects(bRect)) {
                score++;
                soundCatch.play();
                caught = true;
                break;
            }
        }

        if (!caught)
            newEggs.push_back(egg);
    }

    eggs = newEggs;


    /* ------------------- PER COLUMN SPAWN ------------------- */
    for (int i = 0; i < dropColumns.size(); ++i) {

        bool hasEgg = false;
        for (auto &egg : eggs)
            if (int(egg.x()) == dropColumns[i])
                hasEgg = true;

        if (!hasEgg) {
            columnTimers[i] += dt;
            if (columnTimers[i] >= columnDelays[i]) {
                eggs.append(QPointF(float(dropColumns[i]), 0.0f));
                columnTimers[i] = 0.0f;
                columnDelays[i] = 3.0f + QRandomGenerator::global()->bounded(2.0f);
            }
        }
    }
}



/* ============================================================
    DRAW
============================================================ */
void MainWindow::drawGame(float alpha)
{
    QPixmap framePix = background;
    QPainter painter(&framePix);

    auto snap = [&](float c){ return qRound(c) * grid_box; };


    /* ------------------- BASKET ------------------- */
    float basketRenderX = basket.x() + basketXVelocity * alpha * fixedDelta;
    int bx = qRound(basketRenderX);
    int by = qRound(basket.y());

    painter.setBrush(Qt::blue);
    painter.setPen(Qt::NoPen);

    int cx = bx;        // basket center (grid coords)
    int cy = by;
    int r  = 12;        // radius in grid cells → adjust size

    int x = 0;
    int y = r;
    int d = 1 - r;

    /* midpoint circle — only lower half stored & plotted */
    while (x <= y)
    {
        // four symmetric points for lower hemisphere
        int px[4] = {  x,   y,  -x,  -y };
        int py[4] = {  y,   x,   y,   x };

        for (int i = 0; i < 4; i++)
        {
            int gx = cx + px[i];
            int gy = cy + py[i];

            if (gy >= cy)   // only lower half
            {
                gx = std::clamp(gx, 0, cols-1);
                gy = std::clamp(gy, 0, rows-1);

                painter.fillRect(gx * grid_box,
                                 gy * grid_box,
                                 grid_box, grid_box,
                                 Qt::blue);
            }
        }

        x++;
        if (d < 0) d += 2*x + 1;
        else { y--; d += 2*(x - y) + 1; }
    }


    /* ------------------- EGGS ------------------- */
    float fallSpeed = 3.0f + 0.15f * score;
    fallSpeed = std::min(fallSpeed, 25.0f);

    painter.setBrush(Qt::yellow);
    painter.setPen(Qt::NoPen);

    for (auto &egg : eggs) {

        float eggRenderY = egg.y() + fallSpeed * alpha * fixedDelta;

        // ✅ IMPORTANT:
        // Pass grid coords (not pixel coords) to midpoint algorithm
        int cx = qRound(egg.x());        // GRID coordinate
        int cy = qRound(eggRenderY);     // GRID coordinate

        drawEggShape(painter, cx, cy, grid_box);
    }


    /* ------------------- HUD ------------------- */
    ui->scoreLabel->setText(QString("Score: %1").arg(score));
    ui->livesLabel->setText(QString("Lives: %1").arg(lives));

    painter.end();
    ui->frame->setPixmap(framePix);
}
