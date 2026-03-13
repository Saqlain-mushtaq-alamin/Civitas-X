#include "civitasx/app.hpp"

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

        glClearColor(0.04f, 0.06f, 0.08f, 1.0f);

        world_.initialize();

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
        renderer_.render(world_, width_, height_);
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
        const float delta = now - lastTimeSeconds_;
        lastTimeSeconds_ = now;

        world_.update((delta > 0.0f) ? delta : 0.0f);
        glutPostRedisplay();
    }

} // namespace civitasx
