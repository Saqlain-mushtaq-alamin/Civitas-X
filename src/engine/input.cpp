#include "engine/input.h"

#include <GL/freeglut.h>

#include <algorithm>
#include <cmath>

namespace civitasx
{

    namespace engine
    {

        namespace
        {
            struct NavigationState
            {
                bool moveLeft = false;
                bool moveRight = false;
                bool moveUp = false;
                bool moveDown = false;

                bool hasMouse = false;
                int mouseX = 0;
                int mouseY = 0;

                float velocityX = 0.0f;
                float velocityY = 0.0f;

                float centerX = 0.0f;
                float centerY = 0.0f;

                float zoom = 1.0f;
                float targetZoom = 1.0f;

                float lastUpdateTimeSeconds = 0.0f;
                bool timingReady = false;
                bool cameraInitialized = false;
            };

            NavigationState g_navigation;

            constexpr float kMinZoom = 0.45f;
            constexpr float kMaxZoom = 4.0f;
            constexpr float kKeyboardPanSpeed = 900.0f;
            constexpr float kMousePanSpeed = 850.0f;
            constexpr float kVelocitySmoothing = 11.0f;
            constexpr float kZoomSmoothing = 13.0f;
            constexpr float kMouseDeadZone = 0.08f;

            float signPreservingDeadZone(float value)
            {
                const float absValue = std::fabs(value);
                if (absValue <= kMouseDeadZone)
                {
                    return 0.0f;
                }

                const float scaled = (absValue - kMouseDeadZone) / (1.0f - kMouseDeadZone);
                return (value < 0.0f) ? -scaled : scaled;
            }

            void setNormalKeyState(unsigned char key, bool pressed)
            {
                switch (key)
                {
                case 'a':
                case 'A':
                    g_navigation.moveLeft = pressed;
                    break;
                case 'd':
                case 'D':
                    g_navigation.moveRight = pressed;
                    break;
                case 'w':
                case 'W':
                    g_navigation.moveUp = pressed;
                    break;
                case 's':
                case 'S':
                    g_navigation.moveDown = pressed;
                    break;
                default:
                    break;
                }
            }

            void onKeyDown(unsigned char key, int x, int y)
            {
                static_cast<void>(x);
                static_cast<void>(y);
                setNormalKeyState(key, true);
            }

            void onKeyUp(unsigned char key, int x, int y)
            {
                static_cast<void>(x);
                static_cast<void>(y);
                setNormalKeyState(key, false);
            }

            void onSpecialDown(int key, int x, int y)
            {
                static_cast<void>(x);
                static_cast<void>(y);
                switch (key)
                {
                case GLUT_KEY_LEFT:
                    g_navigation.moveLeft = true;
                    break;
                case GLUT_KEY_RIGHT:
                    g_navigation.moveRight = true;
                    break;
                case GLUT_KEY_UP:
                    g_navigation.moveUp = true;
                    break;
                case GLUT_KEY_DOWN:
                    g_navigation.moveDown = true;
                    break;
                default:
                    break;
                }
            }

            void onSpecialUp(int key, int x, int y)
            {
                static_cast<void>(x);
                static_cast<void>(y);
                switch (key)
                {
                case GLUT_KEY_LEFT:
                    g_navigation.moveLeft = false;
                    break;
                case GLUT_KEY_RIGHT:
                    g_navigation.moveRight = false;
                    break;
                case GLUT_KEY_UP:
                    g_navigation.moveUp = false;
                    break;
                case GLUT_KEY_DOWN:
                    g_navigation.moveDown = false;
                    break;
                default:
                    break;
                }
            }

            void setMousePosition(int x, int y)
            {
                g_navigation.hasMouse = true;
                g_navigation.mouseX = x;
                g_navigation.mouseY = y;
            }

            void onMouseMove(int x, int y)
            {
                setMousePosition(x, y);
            }

            void applyScrollDirection(int direction)
            {
                const float zoomStep = (direction > 0) ? 1.12f : (1.0f / 1.12f);
                g_navigation.targetZoom *= zoomStep;
                g_navigation.targetZoom = std::clamp(g_navigation.targetZoom, kMinZoom, kMaxZoom);
            }

            void onMouseButton(int button, int state, int x, int y)
            {
                setMousePosition(x, y);

                if (state != GLUT_DOWN)
                {
                    return;
                }

                if (button == 3)
                {
                    applyScrollDirection(1);
                }
                else if (button == 4)
                {
                    applyScrollDirection(-1);
                }
            }

            void onMouseWheel(int wheel, int direction, int x, int y)
            {
                static_cast<void>(wheel);
                setMousePosition(x, y);
                applyScrollDirection(direction);
            }

            float clampf(float value, float minValue, float maxValue)
            {
                return std::max(minValue, std::min(value, maxValue));
            }
        } // namespace

        void registerInputCallbacks()
        {
            glutKeyboardFunc(&onKeyDown);
            glutKeyboardUpFunc(&onKeyUp);
            glutSpecialFunc(&onSpecialDown);
            glutSpecialUpFunc(&onSpecialUp);
            glutPassiveMotionFunc(&onMouseMove);
            glutMotionFunc(&onMouseMove);
            glutMouseFunc(&onMouseButton);
            glutMouseWheelFunc(&onMouseWheel);
        }

        void updateNavigation(int viewportWidth, int viewportHeight, int mapWidthPixels, int mapHeightPixels)
        {
            const int safeViewportWidth = (viewportWidth <= 0) ? 1 : viewportWidth;
            const int safeViewportHeight = (viewportHeight <= 0) ? 1 : viewportHeight;
            const int safeMapWidth = (mapWidthPixels <= 0) ? 1 : mapWidthPixels;
            const int safeMapHeight = (mapHeightPixels <= 0) ? 1 : mapHeightPixels;

            if (!g_navigation.cameraInitialized)
            {
                g_navigation.centerX = static_cast<float>(safeMapWidth) * 0.5f;
                g_navigation.centerY = static_cast<float>(safeMapHeight) * 0.5f;
                g_navigation.cameraInitialized = true;
            }

            const float now = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
            float dt = 0.0f;
            if (g_navigation.timingReady)
            {
                dt = now - g_navigation.lastUpdateTimeSeconds;
            }
            g_navigation.lastUpdateTimeSeconds = now;
            g_navigation.timingReady = true;

            if (dt <= 0.0f)
            {
                return;
            }

            dt = clampf(dt, 0.0f, 0.05f);

            const float keyboardX =
                (g_navigation.moveRight ? 1.0f : 0.0f) - (g_navigation.moveLeft ? 1.0f : 0.0f);
            const float keyboardY =
                (g_navigation.moveDown ? 1.0f : 0.0f) - (g_navigation.moveUp ? 1.0f : 0.0f);

            float mouseX = 0.0f;
            float mouseY = 0.0f;
            if (g_navigation.hasMouse)
            {
                const float normalizedX =
                    (static_cast<float>(g_navigation.mouseX) / static_cast<float>(safeViewportWidth)) * 2.0f - 1.0f;
                const float normalizedY =
                    (static_cast<float>(g_navigation.mouseY) / static_cast<float>(safeViewportHeight)) * 2.0f - 1.0f;
                mouseX = signPreservingDeadZone(clampf(normalizedX, -1.0f, 1.0f));
                mouseY = signPreservingDeadZone(clampf(normalizedY, -1.0f, 1.0f));
            }

            const float targetVelocityX = (keyboardX * kKeyboardPanSpeed) + (mouseX * kMousePanSpeed);
            const float targetVelocityY = (keyboardY * kKeyboardPanSpeed) + (mouseY * kMousePanSpeed);

            const float velocityBlend = 1.0f - std::exp(-kVelocitySmoothing * dt);
            g_navigation.velocityX += (targetVelocityX - g_navigation.velocityX) * velocityBlend;
            g_navigation.velocityY += (targetVelocityY - g_navigation.velocityY) * velocityBlend;

            g_navigation.centerX += (g_navigation.velocityX * dt) / g_navigation.zoom;
            g_navigation.centerY += (g_navigation.velocityY * dt) / g_navigation.zoom;

            const float zoomBlend = 1.0f - std::exp(-kZoomSmoothing * dt);
            g_navigation.zoom += (g_navigation.targetZoom - g_navigation.zoom) * zoomBlend;
            g_navigation.zoom = clampf(g_navigation.zoom, kMinZoom, kMaxZoom);

            const float aspect = static_cast<float>(safeViewportWidth) / static_cast<float>(safeViewportHeight);
            const float halfViewHeight = (static_cast<float>(safeMapHeight) * 0.5f) / g_navigation.zoom;
            const float halfViewWidth = halfViewHeight * aspect;

            const float mapCenterX = static_cast<float>(safeMapWidth) * 0.5f;
            const float mapCenterY = static_cast<float>(safeMapHeight) * 0.5f;
            const float minCenterX = halfViewWidth;
            const float maxCenterX = static_cast<float>(safeMapWidth) - halfViewWidth;
            const float minCenterY = halfViewHeight;
            const float maxCenterY = static_cast<float>(safeMapHeight) - halfViewHeight;

            if (minCenterX > maxCenterX)
            {
                g_navigation.centerX = mapCenterX;
            }
            else
            {
                g_navigation.centerX = clampf(g_navigation.centerX, minCenterX, maxCenterX);
            }

            if (minCenterY > maxCenterY)
            {
                g_navigation.centerY = mapCenterY;
            }
            else
            {
                g_navigation.centerY = clampf(g_navigation.centerY, minCenterY, maxCenterY);
            }
        }

        CameraState cameraState()
        {
            CameraState state;
            state.centerX = g_navigation.centerX;
            state.centerY = g_navigation.centerY;
            state.zoom = g_navigation.zoom;
            return state;
        }

    } // namespace engine

} // namespace civitasx
