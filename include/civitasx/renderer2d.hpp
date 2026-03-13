#pragma once

namespace civitasx
{

    class World;

    class Renderer2D
    {
    public:
        void render(const World &world, int viewportWidth, int viewportHeight) const;
    };

} // namespace civitasx
