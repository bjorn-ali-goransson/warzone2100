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

void flowfieldEnable();
bool isFlowfieldEnabled();

void flowfieldInit();
void flowfieldDestroy();

bool tryGetFlowfieldForTarget(unsigned int targetX, unsigned int targetY, PROPULSION_TYPE propulsion, unsigned int &flowfieldId);
void calculateFlowfieldAsync(unsigned int targetX, unsigned int targetY, PROPULSION_TYPE propulsion);

void debugDrawFlowfields(const glm::mat4 &mvp);

bool ffpathInitialise();
void ffpathShutdown();
