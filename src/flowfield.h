#include "fpath.h"
#include "movedef.h"
#include "lib/framework/frame.h" // for statsdef.h
#include "statsdef.h"
#include <deque>
#include <glm/gtx/transform.hpp>

/*
	Concept: http://www.gameaipro.com/GameAIPro/GameAIPro_Chapter23_Crowd_Pathfinding_and_Steering_Using_Flow_Field_Tiles.pdf
*/

///////////////////////////////////////////////////////////////////// SCRATCHPAD
// Useful functions found:
// map_TileHeightSurface(x, y) -> height
// map_coord(x, y) -> tile
// world_coord(tile) -> x, y
// getTileMaxMin <- maybe?
// fpathBlockingTile(x, y, PROPULSION_TYPE_WHEELED) - is field blocking for given propulsion
// mapTile(x, y) -> MAPTILE
// worldTile(x, y) -> MAPTILE
// TileIsOccupied <- maybe, to check if there is a building on that tile
// TileIsKnownOccupied <- like above
// tileIsExplored

// MAPTILE.limitedContinent - if I understand it correctly, if there is no ground path between two tiles, then limitedContinent1 != limitedContinent2
// MAPTILE.hoverContinent - like above, but what is continent for hover?
// extern SDWORD	mapWidth, mapHeight;
// extern MAPTILE *psMapTiles;

// isDanger = auxTile(x, y, type.owner) & AUXBITS_THREAT;

/////////////////////////////////////////////////////////////////////


// TODO: avoiding tiles marked as "threat" (only AI, or anyone? It would be nice if your own droids would prefer to avoid enemy when retreating)
// TODO: maybe prefer visible tiles, or just discovered tiles (for player)
// Both things would go into integration field probably. Note that adding visibility stuff would quickly require most integration and flow fields to be thrown away, since visibility changes all the time.

// TODO: support marking sectors as dirty: portals needs to be rebuild, cost fields recalculated, and integration and flow fields for that sector must be removed from cache.

// TODO: disable before merge with main branch
constexpr const bool DEBUG_BUILD = true;

// FEEL FREE TO DISABLE THOSE AS THEY OBSCURE THE VIEW
constexpr const bool PORTALS_DEBUG = true;
constexpr const bool VECTOR_FIELD_DEBUG = true;
constexpr const bool DEBUG_A_STAR = true;
constexpr const bool DEBUG_THREAD_POOL = true;

void flowfieldEnable();
bool isFlowfieldEnabled();

void flowfieldInit();
void flowfieldDestroy();

/**
 * Public interface same as fpathRoute for compatibility.
 * Most fields are not yet used in current flowfield implementation, but I wanted it to be compatible with fpath.cpp
 */
void calculateFlowfieldsAsync(int startX, int startY, int targetX, int targetY, PROPULSION_TYPE propulsion);

std::deque<unsigned int> getFlowfieldPathFromCache(unsigned startX, unsigned startY, unsigned tX, unsigned tY, PROPULSION_TYPE propulsion);

Vector2f getFlowfieldMovementVector(unsigned int nextPortalId, unsigned currentX, unsigned currentY, PROPULSION_TYPE propulsion);

std::vector<Vector2i> flowfieldPortalPathToCoordsPath(const std::deque<unsigned int>& path, DROID* psDroid);

void debugDrawFlowfields(const glm::mat4 &mvp);

/** Initialise the path-finding module.
 */
bool ffpathInitialise();

/** Shutdown the path-finding module.
 */
void ffpathShutdown();
