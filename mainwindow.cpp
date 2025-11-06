#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPainter>
#include <QRandomGenerator>
#include <QtMath>
#include <algorithm>
#include <QThread>
#include <QDebug>
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

#include <QPainterPath>
#include <QKeyEvent>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    gameTimer(nullptr),
    grid_box(6),
    highScore(0),
    grid_size(600),
    cols(0),
    rows(0),
    globalTime(0.0f),
    globalSpawnTimer(0.0f),
    spawnInterval(1.0f),
    lastEdgeSpawnTime(-100.0f),
    edgeSpawnCooldown(4.0f),
    currentColumnIndex(0),
    basket(0, 0),
    prevBasketX(0.0f),
    basketXVelocity(0.0f),
    basketTargetVel(0.0f),
    basketAccel(25.0f),
    basketMaxVel(50.0f),
    moveLeft(false),
    moveRight(false),
    fixedDelta(1.0f / 120.0f),
    accumulator(0.0f),
    gameOver(false),
    gameRunning(false),
    score(0),
    lives(3),
    flashAlpha(0.0f),
    flashFadeSpeed(2.5f)
{
    ui->setupUi(this);
    setFocusPolicy(Qt::StrongFocus);

    // Size grid from frame widget if available
    if (ui->frame) {
        grid_size = ui->frame->frameSize().width();
        if (grid_size <= 0)
            grid_size = 600;
    }
    loadHighScore();
    cols = qMax(40, grid_size / grid_box);
    rows = qMax(30, grid_size / grid_box);

    // background (simple green field)
    background = QPixmap(grid_size, grid_size);
    background.fill(QColor(48, 120, 48)); // nicer green
    {
        QPainter g(&background);
        g.setPen(QPen(QColor(25, 100, 25, 80), 1));
        for (int i = 0; i <= cols; ++i)
            g.drawLine(i * grid_box, 0, i * grid_box, grid_size);
        for (int j = 0; j <= rows; ++j)
            g.drawLine(0, j * grid_box, grid_size, j * grid_box);
    }
    ui->frame->setPixmap(background);

    // Game defaults
    score = 0;
    lives = 3;
    basket = QPointF(cols / 2.0f, rows - 3.0f);
    prevBasketX = basket.x();

    // Sounds
    soundCatch.setSource(QUrl::fromLocalFile("C:/Projects/EggCatcher/sfx/catch.wav"));
    soundCatch.setVolume(0.8f);
    soundLose.setSource(QUrl::fromLocalFile("C:/Projects/EggCatcher/sfx/lose.wav"));
    soundLose.setVolume(0.9f);

    // Drop columns
    dropColumns.clear();
    int mid = cols / 2;
    dropColumns = {mid - 25, mid - 5, mid + 5, mid + 25};
    std::sort(dropColumns.begin(), dropColumns.end());

    columnTimers.resize(dropColumns.size());
    columnDelays.resize(dropColumns.size());
    for (int i = 0; i < dropColumns.size(); ++i) {
        columnTimers[i] = 0.0f;
        columnDelays[i] = 3.0f + QRandomGenerator::global()->bounded(2.0f);
    }

    // Timer
    gameTimer = new QTimer(this);
    gameTimer->setTimerType(Qt::PreciseTimer);
    connect(gameTimer, &QTimer::timeout, this, &MainWindow::gameTick);
    gameTimer->start(1000 / 60);  // ≈ 16.67 ms per frame

    frameClock.start();

    // start paused
    gameRunning = false;
    gameOver = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

#include <QFile>
#include <QTextStream>

void MainWindow::loadHighScore() {
    QFile file("highscore.txt");
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        int savedScore = 0;
        in >> savedScore;
        highScore = savedScore;
        file.close();
    }
}

void MainWindow::saveHighScore() {
    QFile file("highscore.txt");
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        out << highScore;
        file.close();
    }
}


void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    if (gameOver && event->key() == Qt::Key_R) {
        resetGame();
        return;
    }

    if (!gameRunning && event->key() == Qt::Key_Return) {
        gameRunning = true;
        return;
    }

    if (gameOver || !gameRunning)
        return;

    if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left)
        moveLeft = true;
    else if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right)
        moveRight = true;
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;
    if (gameOver || !gameRunning)
        return;

    if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left)
        moveLeft = false;
    else if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right)
        moveRight = false;
}

void MainWindow::resetGame()
{
    ui->scoreLabel->show();
    ui->livesLabel->show();
    score = 0;
    lives = 3;
    eggs.clear();
    basket = QPointF(cols / 2.0f, rows - 3.0f);
    prevBasketX = basket.x();
    basketXVelocity = 0.0f;
    basketTargetVel = 0.0f;
    gameOver = false;
    saveHighScore();
    moveRight = false;
    moveLeft = false;
    accumulator = 0.0f;
    flashColor = QColor();
    frameClock.restart();
    gameRunning = true;
    gameTimer->start();
}

void MainWindow::drawStartScreen()
{
    QPixmap pix = background;
    QPainter p(&pix);
    p.setRenderHint(QPainter::Antialiasing, true);

    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 24, QFont::Bold));
    p.drawText(pix.rect(), Qt::AlignCenter,
               "EGG CATCHER\n\nPress ENTER to Start");

    p.end();
    ui->frame->setPixmap(pix);
}

void MainWindow::drawGameOver()
{
    ui->scoreLabel->hide();
    ui->livesLabel->hide();
    QPixmap pix = background;
    QPainter p(&pix);
    p.setRenderHint(QPainter::Antialiasing, true);

    p.setPen(Qt::red);
    p.setFont(QFont("Arial", 28, QFont::Bold));
    p.drawText(pix.rect(), Qt::AlignCenter,
               "GAME OVER\nPress R to Restart");

    p.end();
    ui->frame->setPixmap(pix);
}

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

    // Elapsed real time
    float dt = frameClock.restart() / 1000.0f;
    dt = qBound(0.001f, dt, 0.05f);   // clamp: min 1ms, max 50ms
    accumulator += dt;

    // Run physics in fixed steps
    const float fixedStep = fixedDelta;  // 1/120 s
    while (accumulator >= fixedStep) {
        updatePhysics(fixedStep);
        accumulator -= fixedStep;
    }

    float alpha = accumulator / fixedStep;

    drawGame(alpha);

    const int targetFrameMS = 16; // ~60fps
    int elapsed = frameClock.elapsed();
    if (elapsed < targetFrameMS)
        QThread::msleep(targetFrameMS - elapsed);

    static int frameCount = 0;
    static float fpsTimer = 0;
    frameCount++;
    fpsTimer += dt;
    if (fpsTimer >= 1.0f) {
        qDebug() << "FPS:" << frameCount;
        frameCount = 0;
        fpsTimer = 0;
    }


    if (scoreAnimTimer > 0.0f) {
        scoreAnimTimer -= dt;
        float t = 1.0f - scoreAnimTimer / 0.2f;
        scoreScale = 1.0f + 0.5f * (1.0f - t*t); // ease-out
    } else {
        scoreScale = 1.0f;
        scoreChanged = false;
    }

    // Lives pulse animation decay
    if (livesPulseTimer > 0.0f) {
        livesPulseTimer -= dt;
    }

}


void MainWindow::updatePhysics(float dt)
{
    if (gameOver)
        return;

    globalTime += dt;
    globalSpawnTimer += dt;

    // -------------------- EGG SPAWN --------------------
    if (globalSpawnTimer >= spawnInterval) {
        int col = dropColumns[currentColumnIndex];
        bool isEdgeCol = (col == dropColumns.front() || col == dropColumns.back());
        bool canSpawn = true;

        float dynamicEdgeCooldown = qMax(0.6f, edgeSpawnCooldown - 0.03f * score);
        if (isEdgeCol && (globalTime - lastEdgeSpawnTime < dynamicEdgeCooldown))
            canSpawn = false;

        if (canSpawn) {
            Egg e;
            e.pos = QPointF(float(col), 0.0f);
            e.yVelocity = 0.0f;
            e.state = "falling";

            // ------------------ NEW TYPE SYSTEM ------------------
            int r = QRandomGenerator::global()->bounded(100);
            if (r < 75) {
                e.type = "normal";
                e.color = QColor(Qt::white);
            } else if (r < 95) {
                e.type = "bad";
                e.color = QColor(200, 50, 50); // red tint
            } else {
                e.type = "life";
                e.color = QColor(255, 105, 180); // pink/green
            }

            eggs.append(e);
            if (isEdgeCol)
                lastEdgeSpawnTime = globalTime;
        }

        currentColumnIndex = (currentColumnIndex + 1) % dropColumns.size();
        globalSpawnTimer = 0.0f;
        spawnInterval = 0.8f + QRandomGenerator::global()->bounded(0.6f);
    }

    // -------------------- BASKET MOVEMENT --------------------
    if (moveLeft && !moveRight)
        basketTargetVel = -basketMaxVel;
    else if (moveRight && !moveLeft)
        basketTargetVel = basketMaxVel;
    else
        basketTargetVel = 0.0f;

    float blend = 1.0f - qExp(-basketAccel * dt);
    basketXVelocity += (basketTargetVel - basketXVelocity) * qMin(1.0f, dt * basketAccel);

    // **MOVE basket position**
    basket.setX(basket.x() + basketXVelocity * dt);
    basket.setX(std::clamp((float)basket.x(), 0.0f, float(cols - 1)));
    prevBasketX = basket.x();

    // -------------------- EGG PHYSICS --------------------
    float baseGravity = 18.0f + score* 0.1f;
    float maxFallSpeed = 35.0f;

    spawnInterval = qMax(0.3f, 0.8f - score * 0.02f);


    QVector<Egg> survivors;
    bool caughtAny = false, lostAny = false;
    for (auto &egg : eggs) {
        egg.prevY = egg.pos.y();
        float gravity = baseGravity;
        if (egg.type == "life") gravity *= 0.5f; // slower fall
        if (egg.type == "bad") gravity *= 1.2f;  // slightly faster

        if (egg.state == "falling") {
            egg.yVelocity += gravity * dt;
            egg.yVelocity = qMin(egg.yVelocity, maxFallSpeed);
            egg.pos.setY(egg.pos.y() + egg.yVelocity * dt);

            int basketWidth = 16;
            int basketHeight = 6;

            QRectF basketRect(
                basket.x() - basketWidth / 2.0f,
                basket.y() - 0.5f,     // lifted upward to catch earlier
                basketWidth,
                basketHeight + 1.5f    // increased height slightly
                );

            QRectF eggRect(egg.pos.x(), egg.pos.y(), 1.0f, 1.0f);

            if (eggRect.intersects(basketRect)) {
                egg.state = "caught";
                egg.animTimer = 0;

                if (egg.type == "bad") {
                    lives = std::max(0, lives - 1);
                    lostAny = true;
                    flashColor = QColor(255, 0, 0);  // pure red
                    flashAlpha = 0.0f;
                    flashTimer = 0.0f;
                }
                else if (egg.type == "life") {
                    lives = std::min(5, lives + 1);
                    flashColor = QColor(0, 255, 0);  // green
                    flashAlpha = 0.0f;
                    flashTimer = 0.0f;
                }
                else {
                    score++;
                }

                caughtAny = true;
            }
            else if (egg.pos.y() >= rows - 1) {
                egg.state = "splat";
                egg.animTimer = 0;
                if(egg.type != "bad")
                    lostAny = true;
            }
            survivors.push_back(egg);
        }
        else if (egg.state == "caught") {
            egg.animTimer += dt;
            egg.scale = 1.0f - egg.animTimer * 3.0f;
            egg.alpha = 1.0f - egg.animTimer * 2.0f;
            if (egg.animTimer >= 0.5f)
                continue;
            survivors.push_back(egg);
        }
        else if (egg.state == "splat" && egg.animTimer < 1) { // spawn once
            int numParticles = 12;
            int scale = 1000;
            for (int i = 0; i < numParticles; ++i) {
                int angleDeg = QRandomGenerator::global()->bounded(0, 360);
                double rad = angleDeg * M_PI / 180.0;

                int speed = QRandomGenerator::global()->bounded(500, 1500); // scaled by 1000

                Particle p;
                p.pos = QPoint(int(egg.pos.x() * scale), int(egg.pos.y() * scale));
                p.velocity = QPoint(int(cos(rad) * speed), int(sin(rad) * speed));
                p.lifetime = QRandomGenerator::global()->bounded(30, 60); // ticks
                p.alpha = 255;
                p.color = egg.color;
                particles.append(p);
            }
        }

    }

    eggs = survivors;
    if (caughtAny) score++;
    if (lostAny) lives--;
    if (score > highScore) highScore = score;  // <--- update high score
    if(lives <= 0) gameOver = true;


    // -------- FLASH ANIMATION UPDATE --------
    if (flashColor.isValid()) {
        flashTimer += dt;
        float fadeInDur = 0.2f;
        float fadeOutDur = 0.5f;

        if (flashTimer < fadeInDur)
            flashAlpha = flashTimer / fadeInDur;
        else if (flashTimer < fadeInDur + fadeOutDur)
            flashAlpha = 1.0f - (flashTimer - fadeInDur) / fadeOutDur;
        else {
            flashColor = QColor();
            flashAlpha = 0.0f;
        }
    }


    if (caughtAny && !gameOver) {
        scoreAnimTimer = 0.2f;   // duration of pulse
        scoreScale = 1.5f;       // scale factor
        scoreChanged = true;
    }

    if (lostAny || std::any_of(eggs.begin(), eggs.end(), [](const Egg &e){ return e.type == "life"; })) {
        livesPulseTimer = 0.3f;
        livesChanged = true;
    }


    QVector<Particle> aliveParticles;
    int scale = 1000;
    for (auto &p : particles) {
        p.pos += p.velocity / 60;   // divide by FPS for movement
        p.lifetime--;
        p.alpha = std::max(0, (p.lifetime * 255) / 60); // fade out
        if (p.lifetime > 0)
            aliveParticles.push_back(p);
    }
    particles = aliveParticles;

}

void MainWindow::drawEggShape(QPainter &p, const Egg &egg, float cellSize)
{
    p.setRenderHint(QPainter::Antialiasing, false); // pixelated look

    QPointF center((egg.pos.x() + 0.5f) * cellSize, (egg.pos.y() + 0.5f) * cellSize);

    float baseW = cellSize *2.5f* 1.5f; // width scaling
    float baseH = cellSize *2.5f* 2.0f; // height scaling
    float w = baseW * egg.scale;
    float h = baseH * egg.scale;

    QColor fillColor = egg.color;
    fillColor.setAlphaF(egg.alpha);
    QColor outlineColor = Qt::yellow;
    outlineColor.setAlphaF(egg.alpha);

    int step = 1; // pixel step

    // Lambda to draw a pixel
    auto plot = [&](int gx, int gy) {
        p.fillRect(center.x() + gx, center.y() + gy, step, step, fillColor);
    };


    for (int yi = -h/2; yi <= h/2; yi += step)
    {
        float yf = float(yi) / (h/2);

        float modifier = (yf < 0) ? (1.0f - 0.3f * yf * yf) : (1.0f + 0.1f * yf);

        int xSpan = int((w/2) * sqrt(1 - yf * yf) * modifier);

        for (int xi = -xSpan; xi <= xSpan; xi += step)
        {
            plot(xi, yi);
        }
    }

    p.setBrush(outlineColor);
    for (int yi = -h/2; yi <= h/2; yi += step)
    {
        float yf = float(yi) / (h/2);
        float modifier = (yf < 0) ? (1.0f - 0.3f * yf * yf) : (1.0f + 0.1f * yf);
        int xSpan = int((w/2) * sqrt(1 - yf * yf) * modifier);

        // left & right edges
        p.fillRect(center.x() - xSpan, center.y() + yi, step, step, outlineColor);
        p.fillRect(center.x() + xSpan, center.y() + yi, step, step, outlineColor);
    }

    if (egg.state == "splat")
    {
        int splatW = int(w);
        int splatH = int(h * 0.4f);
        QRectF splatRect(center.x() - splatW / 2, center.y() - splatH / 2, splatW, splatH);
        p.fillRect(splatRect, fillColor);
        p.setPen(QPen(outlineColor, 2.0));
        p.drawRect(splatRect);
    }
}



void MainWindow::drawGame(float alpha)
{

    QPixmap framePix = background;
    QPainter painter(&framePix);
    painter.setRenderHint(QPainter::Antialiasing, true);

    float basketRenderX = prevBasketX + (basket.x() - prevBasketX) * alpha;
    float basketRenderY = basket.y();

    int basketWidthCells = 16;
    int basketHeightCells = 6;

    QColor basketFill(205, 133, 63);
    QColor basketOutline(120, 60, 20);
    // -------- FLASH RENDERING --------
    if (flashColor.isValid() && flashAlpha > 0.0f) {
        QColor overlay = flashColor;
        overlay.setAlphaF(flashAlpha * 0.5f);  // subtle transparency
        painter.fillRect(framePix.rect(), overlay);
    }

    painter.setBrush(basketFill);
    painter.setPen(Qt::NoPen);

    for (int y = 0; y < basketHeightCells; ++y) {
        float rowY = basketRenderY + y;
        int taper = std::min(y / 1, basketWidthCells / 6);
        int startX = -basketWidthCells / 2 + taper;
        int endX = basketWidthCells / 2 - taper;
        for (int x = startX; x <= endX; ++x) {
            float cx = basketRenderX + x;
            float px = cx * grid_box;
            float py = rowY * grid_box;
            painter.fillRect(px, py, grid_box, grid_box, basketFill);
        }
    }

    painter.setBrush(Qt::NoBrush);
    QPen rimPen(basketOutline);
    rimPen.setWidthF(4.0);
    rimPen.setCapStyle(Qt::RoundCap);
    rimPen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(rimPen);

    QPainterPath rimPath;
    QPointF leftPoint((basketRenderX - basketWidthCells / 2.0f) * grid_box, basketRenderY * grid_box);
    QPointF rightPoint((basketRenderX + basketWidthCells / 2.0f) * grid_box, basketRenderY * grid_box);
    QPointF control(basketRenderX * grid_box, (basketRenderY - basketHeightCells * 0.5f) * grid_box);
    rimPath.moveTo(leftPoint);
    rimPath.quadTo(control, rightPoint);
    painter.drawPath(rimPath);

    int trailLength = 6;
    for (int i = 1; i <= trailLength; ++i) {
        int fade = qMax(10, 120 - i * 18);
        QColor trailColor(160, 82, 45, fade);
        float trailX = basketRenderX - basketXVelocity * (i * 0.02f);
        painter.fillRect((trailX - basketWidthCells / 2.0f) * grid_box,
                         basketRenderY * grid_box,
                         basketWidthCells * grid_box,
                         basketHeightCells * grid_box,
                         trailColor);
    }

    // Draw Eggs
    for (auto &egg : eggs) {
        Egg renderEgg = egg;
        renderEgg.pos.setY(egg.prevY + (egg.pos.y() - egg.prevY) * alpha);
        drawEggShape(painter, renderEgg, (float)grid_box);
    }

    // HUD
    // Score with pulse
    painter.setFont(QFont("Comic Sans MS", 24, QFont::Bold));
    QColor scoreColor(255, 215, 0); // gold
    painter.setPen(scoreColor);

    painter.save();
    QPointF scorePos(30, 45);
    painter.translate(scorePos);
    painter.scale(scoreScale, scoreScale);
    painter.drawText(QPointF(0, 0), QString("Score: %1").arg(score));
    painter.restore();


    QFont highFont("Arial", 18, QFont::Bold);
    painter.setFont(highFont);
    QColor highColor(200, 200, 255); // soft blue
    painter.setPen(highColor);
    painter.drawText(30, 75, QString("High Score: %1").arg(highScore));

    // Lives Display (hearts with pulse)
    int heartSize = 24;
    float pulseScale = 1.0f + 0.5f * (livesPulseTimer / 0.3f); // scale while pulse active
    for (int i = 0; i < lives; ++i) {
        int x = framePix.width() - 40 - i * (heartSize + 5);
        int y = 20;

        QPainterPath heartPath;
        heartPath.moveTo(x + heartSize/2.0, y + heartSize/5.0);
        heartPath.cubicTo(x + heartSize/2.0, y, x, y, x, y + heartSize/3.0);
        heartPath.cubicTo(x, y + heartSize*0.8, x + heartSize/2.0, y + heartSize, x + heartSize/2.0, y + heartSize*0.9);
        heartPath.cubicTo(x + heartSize/2.0, y + heartSize, x + heartSize, y + heartSize*0.8, x + heartSize, y + heartSize/3.0);
        heartPath.cubicTo(x + heartSize, y, x + heartSize/2.0, y, x + heartSize/2.0, y + heartSize/5.0);

        painter.save();
        painter.translate(x + heartSize/2.0, y + heartSize/2.0);
        painter.scale(pulseScale, pulseScale);
        painter.translate(-(x + heartSize/2.0), -(y + heartSize/2.0));
        painter.setBrush(Qt::red);
        painter.setPen(Qt::NoPen);
        painter.drawPath(heartPath);
        painter.restore();
    }

    painter.setPen(Qt::NoPen);
    for (auto &p : particles) {
        QColor c = p.color;
        c.setAlpha(p.alpha);
        painter.setBrush(c);
        painter.setPen(Qt::NoPen);
        int size = grid_box / 3;
        painter.drawEllipse(QPointF(p.pos.x() / 1000.0, p.pos.y() / 1000.0) * grid_box, size, size);
    }



    painter.end();
    ui->frame->setPixmap(framePix);
}
