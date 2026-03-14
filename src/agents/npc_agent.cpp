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
            npc.money = 100;
            npc.state = NpcState::Sleeping;
            npc.home = spawnPosition;
            npc.work = spawnPosition;
            npc.food = spawnPosition;
            npc.target = spawnPosition;
            npc.accessAnchor = spawnPosition;
            npc.cycleStage = 0;
            npc.dwellSeconds = 0.0f;
            npc.finalApproach = false;
            npc.hasAccessAnchor = false;
            return npc;
        }

    } // namespace agents

} // namespace civitasx
