#include "civitasx/app.hpp"

#include "engine/input.h"
#include "graphics/shape_drawer.h"

#include <GL/freeglut.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

#ifdef _WIN32
#include <mmsystem.h>
#include <windows.h>
#endif

namespace civitasx
{

    namespace
    {
        constexpr int kFrameIntervalMs = 1000 / 60;
        constexpr float kIntroLoadDurationSeconds = 1.35f;
        const char *kIntroMusicFilePath = "assets/audio/start_screen_music.wav";

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

        void drawCenteredStrokeText(float centerX, float baselineY, const std::string &text, float scale)
        {
            float strokeWidth = 0.0f;
            for (char c : text)
            {
                strokeWidth += static_cast<float>(glutStrokeWidth(GLUT_STROKE_ROMAN, c));
            }

            glPushMatrix();
            glTranslatef(centerX - ((strokeWidth * scale) * 0.5f), baselineY, 0.0f);
            // Projection Y grows downward on this screen, so invert Y scale to keep stroke text upright.
            glScalef(scale, -scale, 1.0f);
            for (char c : text)
            {
                glutStrokeCharacter(GLUT_STROKE_ROMAN, c);
            }
            glPopMatrix();
        }

        void drawDetailedCar(float centerX, float baseY, float bob)
        {
            const float bodyWidth = 250.0f;
            const float bodyHeight = 54.0f;
            const float halfW = bodyWidth * 0.5f;
            const float left = centerX - halfW;
            const float right = centerX + halfW;

            glColor3f(0.86f, 0.30f, 0.20f);
            glBegin(GL_QUADS);
            glVertex2f(left + 8.0f, baseY + 35.0f + bob);
            glVertex2f(right - 10.0f, baseY + 35.0f + bob);
            glVertex2f(right - 8.0f, baseY + 35.0f + bodyHeight + bob);
            glVertex2f(left + 4.0f, baseY + 35.0f + bodyHeight + bob);
            glEnd();

            glColor3f(0.72f, 0.23f, 0.16f);
            glBegin(GL_POLYGON);
            glVertex2f(left + 52.0f, baseY + 35.0f + bob);
            glVertex2f(left + 106.0f, baseY + 3.0f + bob);
            glVertex2f(right - 98.0f, baseY + 3.0f + bob);
            glVertex2f(right - 34.0f, baseY + 35.0f + bob);
            glEnd();

            glColor3f(0.58f, 0.84f, 0.95f);
            glBegin(GL_QUADS);
            glVertex2f(left + 110.0f, baseY + 10.0f + bob);
            glVertex2f(right - 103.0f, baseY + 10.0f + bob);
            glVertex2f(right - 78.0f, baseY + 31.0f + bob);
            glVertex2f(left + 89.0f, baseY + 31.0f + bob);
            glEnd();

            glColor3f(0.95f, 0.90f, 0.64f);
            glBegin(GL_QUADS);
            glVertex2f(right - 14.0f, baseY + 53.0f + bob);
            glVertex2f(right + 4.0f, baseY + 53.0f + bob);
            glVertex2f(right + 4.0f, baseY + 68.0f + bob);
            glVertex2f(right - 14.0f, baseY + 68.0f + bob);
            glEnd();

            glColor3f(0.78f, 0.15f, 0.14f);
            glBegin(GL_QUADS);
            glVertex2f(left - 4.0f, baseY + 54.0f + bob);
            glVertex2f(left + 10.0f, baseY + 54.0f + bob);
            glVertex2f(left + 10.0f, baseY + 67.0f + bob);
            glVertex2f(left - 4.0f, baseY + 67.0f + bob);
            glEnd();

            glColor3f(0.08f, 0.08f, 0.08f);
            graphics::drawFilledCircle({left + 70.0f, baseY + 95.0f + bob}, 19.0f, 30);
            graphics::drawFilledCircle({right - 68.0f, baseY + 95.0f + bob}, 19.0f, 30);
            glColor3f(0.78f, 0.80f, 0.84f);
            graphics::drawFilledCircle({left + 70.0f, baseY + 95.0f + bob}, 7.0f, 20);
            graphics::drawFilledCircle({right - 68.0f, baseY + 95.0f + bob}, 7.0f, 20);
        }

        void drawDetailedNpc(float x, float y, float walkSwing)
        {
            glColor3f(0.95f, 0.79f, 0.61f);
            graphics::drawFilledCircle({x, y + 12.0f}, 13.0f, 28);

            glColor3f(0.17f, 0.20f, 0.25f);
            graphics::drawFilledCircle({x, y + 6.0f}, 12.0f, 28);

            glColor3f(0.21f, 0.58f, 0.86f);
            glBegin(GL_QUADS);
            glVertex2f(x - 15.0f, y + 24.0f);
            glVertex2f(x + 15.0f, y + 24.0f);
            glVertex2f(x + 19.0f, y + 84.0f);
            glVertex2f(x - 19.0f, y + 84.0f);
            glEnd();

            glColor3f(0.95f, 0.79f, 0.61f);
            glBegin(GL_QUADS);
            glVertex2f(x - 26.0f, y + 34.0f);
            glVertex2f(x - 16.0f, y + 34.0f);
            glVertex2f(x - 20.0f - walkSwing * 0.35f, y + 66.0f);
            glVertex2f(x - 31.0f - walkSwing * 0.35f, y + 66.0f);
            glEnd();

            glBegin(GL_QUADS);
            glVertex2f(x + 16.0f, y + 34.0f);
            glVertex2f(x + 26.0f, y + 34.0f);
            glVertex2f(x + 31.0f + walkSwing * 0.35f, y + 66.0f);
            glVertex2f(x + 20.0f + walkSwing * 0.35f, y + 66.0f);
            glEnd();

            glColor3f(0.10f, 0.12f, 0.16f);
            glBegin(GL_QUADS);
            glVertex2f(x - 10.0f, y + 84.0f);
            glVertex2f(x - 3.0f, y + 84.0f);
            glVertex2f(x - 1.0f + walkSwing, y + 120.0f);
            glVertex2f(x - 12.0f + walkSwing, y + 120.0f);
            glEnd();

            glBegin(GL_QUADS);
            glVertex2f(x + 3.0f, y + 84.0f);
            glVertex2f(x + 10.0f, y + 84.0f);
            glVertex2f(x + 12.0f - walkSwing, y + 120.0f);
            glVertex2f(x + 1.0f - walkSwing, y + 120.0f);
            glEnd();
        }
    } // namespace

    App *App::instance_ = nullptr;

    App::~App()
    {
        stopIntroMusic();
    }

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
        glutTimerFunc(kFrameIntervalMs, &App::timerCallback, 0);

        introStartedSeconds_ = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
        introLoadProgress_ = 0.0f;
        simulationStarted_ = false;
        startIntroMusic();
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

    void App::timerCallback(int value)
    {
        if (instance_ != nullptr)
        {
            instance_->onTimer(value);
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
                stopIntroMusic();
            }

            drawStartScreen(elapsed);
            glutSwapBuffers();
            return;
        }

        renderer_.render(width_, height_);
        glutSwapBuffers();
    }

    void App::startIntroMusic()
    {
#ifdef _WIN32
        // Placeholder behavior: once you upload this WAV file, it will auto-loop on the start screen.
        if (std::filesystem::exists(kIntroMusicFilePath))
        {
            PlaySoundA(kIntroMusicFilePath, nullptr, SND_FILENAME | SND_ASYNC | SND_LOOP | SND_NODEFAULT);
        }
#endif
    }

    void App::stopIntroMusic()
    {
#ifdef _WIN32
        PlaySoundA(nullptr, nullptr, SND_ASYNC);
#endif
    }

    void App::onReshape(int width, int height)
    {
        width_ = (width <= 0) ? 1 : width;
        height_ = (height <= 0) ? 1 : height;
        glViewport(0, 0, width_, height_);
    }

    void App::onTimer(int value)
    {
        (void)value;
        glutPostRedisplay();
        glutTimerFunc(kFrameIntervalMs, &App::timerCallback, 0);
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

        // Car moves forward and wraps around after leaving the right side.
        const float carLoopX = std::fmod(elapsedSeconds * 185.0f, w + 500.0f) - 260.0f;
        const float carY = h * 0.77f;
        const float carBounce = std::sin(elapsedSeconds * 8.0f) * 2.1f;
        drawDetailedCar(carLoopX, carY, carBounce);

        // NPC with more detail using shape utility circles.
        const float npcX = w * 0.76f + (std::sin(elapsedSeconds * 2.0f) * 22.0f);
        const float npcY = h * 0.58f;
        const float step = std::sin(elapsedSeconds * 7.0f) * 7.5f;
        drawDetailedNpc(npcX, npcY, step);

        const float centerX = w * 0.5f;
        const float titleY = h * 0.24f;
        const float panelWidth = std::min(860.0f, w * 0.90f);
        const float panelLeft = centerX - (panelWidth * 0.5f);
        const float panelTop = h * 0.09f;
        const float panelBottom = h * 0.56f;

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
        drawCenteredStrokeText(centerX, titleY, "CIVITAS-X", 0.43f);

        glColor3f(0.67f, 0.84f, 0.91f);
        const float subtitleY = titleY + 52.0f;
        drawCenteredText(centerX, subtitleY, "Autonomous City Simulation", GLUT_BITMAP_HELVETICA_18);

        const float barWidth = std::min(620.0f, w * 0.70f);
        const float barHeight = 26.0f;
        const float barX = centerX - (barWidth * 0.5f);
        const float barY = h * 0.43f;

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
            const float pressEnterY = subtitleY + ((barY - subtitleY) * 0.52f);
            drawCenteredText(centerX, pressEnterY, "Press Enter to Start", GLUT_BITMAP_HELVETICA_18);
        }
    }

} // namespace civitasx
