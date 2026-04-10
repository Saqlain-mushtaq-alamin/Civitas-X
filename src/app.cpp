#include "civitasx/app.hpp"

#include "engine/input.h"

#include <GL/freeglut.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace civitasx
{

    namespace
    {
        constexpr float kIntroLoadDurationSeconds = 1.35f;

        int textWidth(void *font, const std::string &text)
        {
            int width = 0;
            for (char c : text)
            {
                width += glutBitmapWidth(font, c);
            }
            return width;
        }

        void drawText(float x, float y, const std::string &text, void *font)
        {
            glRasterPos2f(x, y);
            for (char c : text)
            {
                glutBitmapCharacter(font, c);
            }
        }

        void drawCenteredText(float x, float y, const std::string &text, void *font)
        {
            const float w = static_cast<float>(textWidth(font, text));
            drawText(x - (w * 0.5f), y, text, font);
        }
    } // namespace

    App *App::instance_ = nullptr;

    bool App::initialize(int &argc, char **argv)
    {
        if (instance_ != nullptr)
        {
            return false;
        }

        instance_ = this;

        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
        glutInitWindowSize(width_, height_);
        glutCreateWindow("Civitas-X | Phase 1 - Autonomous Cars");

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        // Enable VSync so each swap waits for the monitor's vertical blank.
        // This stabilises the effective dt to one display-refresh interval and
        // eliminates the tearing and stutter that an uncapped loop produces.
        glutSwapInterval(1);

        engine::registerInputCallbacks();

        glutDisplayFunc(&App::displayCallback);
        glutReshapeFunc(&App::reshapeCallback);
        glutIdleFunc(&App::idleCallback);

        lastTimeSeconds_ = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
        introStartedSeconds_ = lastTimeSeconds_;
        introLoadProgress_ = 0.0f;
        simulationStarted_ = false;
        return true;
    }

    void App::run()
    {
        glutMainLoop();
    }

    void App::displayCallback()
    {
        if (instance_ != nullptr)
        {
            instance_->onDisplay();
        }
    }

    void App::reshapeCallback(int width, int height)
    {
        if (instance_ != nullptr)
        {
            instance_->onReshape(width, height);
        }
    }

    void App::idleCallback()
    {
        if (instance_ != nullptr)
        {
            instance_->onIdle();
        }
    }

    void App::onDisplay()
    {
        const float now = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
        const float elapsed = std::max(0.0f, now - introStartedSeconds_);

        if (!simulationStarted_)
        {
            const float t = std::clamp(elapsed / kIntroLoadDurationSeconds, 0.0f, 1.0f);
            // Ease out so loading looks fast but smooth.
            const float eased = 1.0f - std::pow(1.0f - t, 2.8f);
            introLoadProgress_ = std::clamp(eased, 0.0f, 1.0f);

            if (introLoadProgress_ >= 1.0f && engine::consumeStartPressed())
            {
                simulationStarted_ = true;
            }

            drawStartScreen(elapsed);
            glutSwapBuffers();
            return;
        }

        renderer_.render(width_, height_);
        glutSwapBuffers();
    }

    void App::onReshape(int width, int height)
    {
        width_ = (width <= 0) ? 1 : width;
        height_ = (height <= 0) ? 1 : height;
        glViewport(0, 0, width_, height_);
    }

    void App::onIdle()
    {
        // Only request a redraw once at least one display-refresh interval has
        // passed.  Without this guard the idle callback fires at CPU speed,
        // producing thousands of near-zero dt frames per second which makes
        // every moving object appear to stutter.  1/120 s is half a 60Hz frame
        // so we don't add visible lag while still keeping dt stable.
        constexpr float kMinFrameSeconds = 1.0f / 120.0f;
        const float now = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
        if ((now - lastTimeSeconds_) < kMinFrameSeconds)
        {
            return;
        }
        lastTimeSeconds_ = now;
        glutPostRedisplay();
    }

    void App::drawStartScreen(float elapsedSeconds) const
    {
        const float w = static_cast<float>(width_);
        const float h = static_cast<float>(height_);
        const float clampedProgress = std::clamp(introLoadProgress_, 0.0f, 1.0f);

        glViewport(0, 0, width_, height_);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.0, static_cast<double>(width_), static_cast<double>(height_), 0.0, -1.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        const float pulse = 0.5f + (0.5f * std::sin(elapsedSeconds * 1.7f));
        glClearColor(0.03f + (0.02f * pulse), 0.08f + (0.03f * pulse), 0.12f + (0.02f * pulse), 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Sky glow backdrop.
        glBegin(GL_QUADS);
        glColor3f(0.07f, 0.16f, 0.24f);
        glVertex2f(0.0f, 0.0f);
        glVertex2f(w, 0.0f);
        glColor3f(0.01f, 0.06f, 0.10f);
        glVertex2f(w, h);
        glVertex2f(0.0f, h);
        glEnd();

        // Distant city strips.
        glColor4f(0.10f, 0.22f, 0.30f, 0.8f);
        glBegin(GL_QUADS);
        glVertex2f(0.0f, h * 0.58f);
        glVertex2f(w, h * 0.58f);
        glVertex2f(w, h * 0.84f);
        glVertex2f(0.0f, h * 0.84f);
        glEnd();

        glColor4f(0.05f, 0.14f, 0.20f, 0.95f);
        glBegin(GL_QUADS);
        glVertex2f(0.0f, h * 0.74f);
        glVertex2f(w, h * 0.74f);
        glVertex2f(w, h);
        glVertex2f(0.0f, h);
        glEnd();

        // Road lane.
        glColor3f(0.09f, 0.10f, 0.12f);
        glBegin(GL_QUADS);
        glVertex2f(0.0f, h * 0.80f);
        glVertex2f(w, h * 0.80f);
        glVertex2f(w, h);
        glVertex2f(0.0f, h);
        glEnd();

        glColor3f(0.89f, 0.78f, 0.25f);
        const float laneOffset = std::fmod(elapsedSeconds * 240.0f, 170.0f);
        for (float x = -170.0f + laneOffset; x < w + 170.0f; x += 170.0f)
        {
            glBegin(GL_QUADS);
            glVertex2f(x, h * 0.90f);
            glVertex2f(x + 86.0f, h * 0.90f);
            glVertex2f(x + 86.0f, h * 0.915f);
            glVertex2f(x, h * 0.915f);
            glEnd();
        }

        // Stylized animated car.
        const float carX = std::fmod(elapsedSeconds * 175.0f, w + 280.0f) - 240.0f;
        const float carY = h * 0.83f;
        const float carBounce = std::sin(elapsedSeconds * 8.0f) * 2.5f;

        glColor3f(0.86f, 0.30f, 0.20f);
        glBegin(GL_QUADS);
        glVertex2f(carX + 20.0f, carY + 42.0f + carBounce);
        glVertex2f(carX + 190.0f, carY + 42.0f + carBounce);
        glVertex2f(carX + 190.0f, carY + 88.0f + carBounce);
        glVertex2f(carX + 20.0f, carY + 88.0f + carBounce);
        glEnd();

        glColor3f(0.70f, 0.22f, 0.15f);
        glBegin(GL_POLYGON);
        glVertex2f(carX + 54.0f, carY + 42.0f + carBounce);
        glVertex2f(carX + 102.0f, carY + 12.0f + carBounce);
        glVertex2f(carX + 154.0f, carY + 12.0f + carBounce);
        glVertex2f(carX + 186.0f, carY + 42.0f + carBounce);
        glEnd();

        glColor3f(0.60f, 0.83f, 0.93f);
        glBegin(GL_QUADS);
        glVertex2f(carX + 104.0f, carY + 18.0f + carBounce);
        glVertex2f(carX + 150.0f, carY + 18.0f + carBounce);
        glVertex2f(carX + 165.0f, carY + 41.0f + carBounce);
        glVertex2f(carX + 90.0f, carY + 41.0f + carBounce);
        glEnd();

        glColor3f(0.08f, 0.08f, 0.08f);
        for (int i = 0; i < 2; ++i)
        {
            const float wheelX = carX + (i == 0 ? 60.0f : 154.0f);
            const float wheelY = carY + 93.0f + carBounce;
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(wheelX, wheelY);
            for (int seg = 0; seg <= 24; ++seg)
            {
                const float a = static_cast<float>(seg) * 0.2617994f;
                glVertex2f(wheelX + (13.0f * std::cos(a)), wheelY + (13.0f * std::sin(a)));
            }
            glEnd();
        }

        // Stylized NPC silhouette with subtle walk loop.
        const float npcX = w * 0.72f + (std::sin(elapsedSeconds * 2.0f) * 18.0f);
        const float npcY = h * 0.69f;
        const float step = std::sin(elapsedSeconds * 7.2f) * 7.0f;

        glColor3f(0.94f, 0.78f, 0.60f);
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(npcX, npcY + 16.0f);
        for (int seg = 0; seg <= 22; ++seg)
        {
            const float a = static_cast<float>(seg) * 0.285599f;
            glVertex2f(npcX + (12.0f * std::cos(a)), npcY + 16.0f + (12.0f * std::sin(a)));
        }
        glEnd();

        glColor3f(0.20f, 0.55f, 0.83f);
        glBegin(GL_QUADS);
        glVertex2f(npcX - 10.0f, npcY + 30.0f);
        glVertex2f(npcX + 10.0f, npcY + 30.0f);
        glVertex2f(npcX + 14.0f, npcY + 84.0f);
        glVertex2f(npcX - 14.0f, npcY + 84.0f);
        glEnd();

        glColor3f(0.09f, 0.11f, 0.15f);
        glBegin(GL_QUADS);
        glVertex2f(npcX - 8.0f, npcY + 84.0f);
        glVertex2f(npcX - 2.0f, npcY + 84.0f);
        glVertex2f(npcX - 3.0f + step, npcY + 116.0f);
        glVertex2f(npcX - 12.0f + step, npcY + 116.0f);
        glEnd();

        glBegin(GL_QUADS);
        glVertex2f(npcX + 2.0f, npcY + 84.0f);
        glVertex2f(npcX + 8.0f, npcY + 84.0f);
        glVertex2f(npcX + 12.0f - step, npcY + 116.0f);
        glVertex2f(npcX + 3.0f - step, npcY + 116.0f);
        glEnd();

        const float centerX = w * 0.5f;
        const float titleY = h * 0.26f;
        const float panelWidth = std::min(700.0f, w * 0.84f);
        const float panelLeft = centerX - (panelWidth * 0.5f);
        const float panelTop = h * 0.12f;
        const float panelBottom = h * 0.52f;

        glColor4f(0.02f, 0.05f, 0.08f, 0.78f);
        glBegin(GL_QUADS);
        glVertex2f(panelLeft, panelTop);
        glVertex2f(panelLeft + panelWidth, panelTop);
        glVertex2f(panelLeft + panelWidth, panelBottom);
        glVertex2f(panelLeft, panelBottom);
        glEnd();

        glColor3f(0.26f, 0.76f, 0.90f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(panelLeft, panelTop);
        glVertex2f(panelLeft + panelWidth, panelTop);
        glVertex2f(panelLeft + panelWidth, panelBottom);
        glVertex2f(panelLeft, panelBottom);
        glEnd();

        glColor3f(0.93f, 0.97f, 0.99f);
        drawCenteredText(centerX, titleY, "CIVITAS-X", GLUT_BITMAP_TIMES_ROMAN_24);

        glColor3f(0.67f, 0.84f, 0.91f);
        drawCenteredText(centerX, titleY + 34.0f, "Autonomous City Simulation", GLUT_BITMAP_HELVETICA_18);

        const float barWidth = std::min(620.0f, w * 0.70f);
        const float barHeight = 26.0f;
        const float barX = centerX - (barWidth * 0.5f);
        const float barY = h * 0.40f;

        glColor3f(0.08f, 0.14f, 0.17f);
        glBegin(GL_QUADS);
        glVertex2f(barX, barY);
        glVertex2f(barX + barWidth, barY);
        glVertex2f(barX + barWidth, barY + barHeight);
        glVertex2f(barX, barY + barHeight);
        glEnd();

        const float fillWidth = (barWidth - 6.0f) * clampedProgress;
        glColor3f(0.19f, 0.80f, 0.64f);
        glBegin(GL_QUADS);
        glVertex2f(barX + 3.0f, barY + 3.0f);
        glVertex2f(barX + 3.0f + fillWidth, barY + 3.0f);
        glVertex2f(barX + 3.0f + fillWidth, barY + barHeight - 3.0f);
        glVertex2f(barX + 3.0f, barY + barHeight - 3.0f);
        glEnd();

        glColor3f(0.79f, 0.91f, 0.96f);
        const int percent = static_cast<int>(clampedProgress * 100.0f + 0.5f);
        drawCenteredText(centerX, barY + 44.0f, "Loading systems... " + std::to_string(percent) + "%", GLUT_BITMAP_8_BY_13);

        if (clampedProgress >= 1.0f)
        {
            const float blink = 0.35f + (0.65f * (0.5f + (0.5f * std::sin(elapsedSeconds * 4.4f))));
            glColor3f(0.97f * blink, 0.97f * blink, 0.97f * blink);
            drawCenteredText(centerX, barY + 86.0f, "Press Enter to Start", GLUT_BITMAP_HELVETICA_18);
        }
    }

} // namespace civitasx
