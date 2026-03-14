#include "civitasx/app.hpp"

#include "engine/input.h"

#include <GL/freeglut.h>

namespace civitasx
{

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

        engine::registerInputCallbacks();

        glutDisplayFunc(&App::displayCallback);
        glutReshapeFunc(&App::reshapeCallback);
        glutIdleFunc(&App::idleCallback);

        lastTimeSeconds_ = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
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
        const float now = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
        lastTimeSeconds_ = now;
        glutPostRedisplay();
    }

} // namespace civitasx
