#include "agents/npc_agent.h"

namespace civitasx
{

    namespace agents
    {

        NpcAgent makeDefaultNpc(int id, const glm::vec2 &spawnPosition)
        {
            NpcAgent npc;
            npc.id = id;
            npc.position = spawnPosition;
            npc.mood = 1.0f;
            return npc;
        }

    } // namespace agents

} // namespace civitasx
