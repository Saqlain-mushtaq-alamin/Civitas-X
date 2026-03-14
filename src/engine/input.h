#pragma once

namespace civitasx
{

    namespace engine
    {

        struct CameraState
        {
            float centerX = 0.0f;
            float centerY = 0.0f;
            float zoom = 1.0f;
        };

        void registerInputCallbacks();
        void updateNavigation(int viewportWidth, int viewportHeight, int mapWidthPixels, int mapHeightPixels);
        CameraState cameraState();
        bool consumeLeftClick(int &x, int &y);

    } // namespace engine

} // namespace civitasx
