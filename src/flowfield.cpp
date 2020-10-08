// Must be before some stuff from headers from flowfield.h, otherwise "std has no member 'mutex'"
// That simply means someone else have messed up
#include <mutex>

#include "flowfield.h"

#include <future>
#include <map>
#include <set>
#include <typeinfo>
#include <vector>
#include <memory>

#include "lib/framework/debug.h"
#include "lib/ivis_opengl/pieblitfunc.h"
#include "lib/ivis_opengl/piepalette.h"
#include "lib/ivis_opengl/textdraw.h"
#include "lib/ivis_opengl/piematrix.h"

#include "display3d.h"
#include "map.h"
#include "lib/framework/wzapp.h"
#include <glm/gtx/transform.hpp>
#include "lib/framework/opengl.h"
#include "lib/ivis_opengl/piedef.h"
#include "lib/ivis_opengl/piefunc.h"
#include "lib/ivis_opengl/piestate.h"
#include "lib/ivis_opengl/piemode.h"
#include "lib/ivis_opengl/pieblitfunc.h"
#include "lib/ivis_opengl/pieclip.h"

static bool flowfieldEnabled = false;

void flowfieldEnable() {
	flowfieldEnabled = true;
}

bool isFlowfieldEnabled() {
	return flowfieldEnabled;
}

//

struct ComparableVector2i : Vector2i {
	ComparableVector2i(int x, int y) : Vector2i(x, y) {}
	ComparableVector2i(Vector2i value) : Vector2i(value) {}

	inline bool operator<(const ComparableVector2i& b) const {
		if(x < b.x){
			return true;
		}
		if(x > b.x){
			return false;
		}
		if(y < b.y){
			return true;
		}
		return false;
    }
};

#define FF_MAP_WIDTH 256
#define FF_MAP_HEIGHT 256
#define FF_MAP_AREA FF_MAP_WIDTH*FF_MAP_HEIGHT
#define FF_TILE_SIZE 128

constexpr const unsigned short NOT_PASSABLE = std::numeric_limits<unsigned short>::max();
constexpr const unsigned short COST_MIN = 1;

// Decides how much slopes should be avoided
constexpr const float SLOPE_COST_BASE = 0.2f;
// Decides when terrain height delta is considered a slope
// TODO: I do not have much knowledge about WZ, but why almost every tile has different heights?
constexpr const unsigned short SLOPE_THRESOLD = 2;

/*
* How many portal-level A* paths to cache (LRU)
* Notice: this is per type of propulsion
*/
constexpr const unsigned short PORTAL_PATH_CACHE_MAX = 50;

/*
* How many sector-level entries to cache (LRU)
* Total number of unique sector-flowfields depends directly on (number_of_portals ^ 2)
* Each flowfield takes SECTOR_SIZE^2 * sizeof(VectorT)
* For example, max cache size for defaults could be 64 * 2*4 * 8096 = 4MB
* Notice: this is per type of propulsion
*/
constexpr const unsigned short FLOWFIELD_CACHE_MAX = 8096;

/*
* How much to avoid adjacent static obstacles. 1.0 means no avoidance, > 1.0 means avoidance.
* Used in vector field generation for both map edges and non-passable terrain.
* The result is used as integer, so for small cost and small coefficient it makes no difference
*/
constexpr const float OBSTACLE_AVOIDANCE_COEFF = 1.5f;

struct Tile {
	unsigned short cost = COST_MIN;
	bool Tile::isBlocking() const
	{
		return cost == NOT_PASSABLE;
	}
};

struct VectorT {
	float x;
	float y;

	void normalize() {
		const float length = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

		if (length != 0) {
			x /= length;
			y /= length;
		}
	}
};

struct Flowfield {
	std::array<glm::i8vec2, FF_MAP_AREA> vectors;

	void setVector(Vector2i p, glm::i8vec2 vector) {
		vectors[p.y * FF_MAP_HEIGHT + p.x] = vector;
	}
	glm::i8vec2 getVector(Vector2i p) const {
		return vectors[p.y * FF_MAP_HEIGHT + p.x];
	}
};

void initCostFields();
void destroyCostFields();
void destroyFlowfieldCache();

Tile createTile(Vector2i p, PROPULSION_TYPE propulsion);

std::deque<unsigned int> getFlowfieldPathFromCache(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion);
Vector2f getMovementVector(unsigned int nextPortalId, Vector2i currentPosition, PROPULSION_TYPE propulsion);

struct FLOWFIELDREQUEST
{
	/// Source position
	Vector2i mapSource;
	/// Target position
	Vector2i mapGoal;
	PROPULSION_TYPE propulsion;
};

void flowfieldInit() {
	if (!isFlowfieldEnabled()) return;

	if(mapWidth == 0 || mapHeight == 0){
		// called by both stageTwoInitialise() and stageThreeInitialise().
		// in the case of both these being called, map will be unavailable the first time.
		return;
	}

	initCostFields();
}

void flowfieldDestroy() {
	if (!isFlowfieldEnabled()) return;

	destroyCostFields();
	destroyFlowfieldCache();
}

std::deque<unsigned int> getFlowfieldPathFromCache(unsigned startX, unsigned startY, unsigned tX, unsigned tY, const PROPULSION_TYPE propulsion) {
	Vector2i source { map_coord(startX), map_coord(startY) };
	Vector2i goal { map_coord(tX), map_coord(tY) };
	
	unsigned int sourcePortalId, goalPortalId;

	return getFlowfieldPathFromCache(sourcePortalId, goalPortalId, propulsion);
}

Vector2f getMovementVector(unsigned int nextPortalId, unsigned currentX, unsigned currentY, PROPULSION_TYPE propulsion) {
	return getMovementVector(nextPortalId, { currentX, currentY }, propulsion);
}

std::vector<ComparableVector2i> toComparableVectors(std::vector<Vector2i> values){
	std::vector<ComparableVector2i> result;

	for(auto value : values){
		result.push_back(*new ComparableVector2i(value));
	}

	return result;
}

// If the path finding system is shutdown or not
static volatile bool ffpathQuit = false;

// threading stuff
static WZ_THREAD        *ffpathThread = nullptr;
static WZ_MUTEX         *ffpathMutex = nullptr;
static WZ_SEMAPHORE     *ffpathSemaphore = nullptr;
static std::list<wz::packaged_task<FLOWFIELDREQUEST()>>    flowfieldRequests;

void processFlowfield(FLOWFIELDREQUEST request);

void calculateFlowfieldsAsync(int startX, int startY, int targetX, int targetY, PROPULSION_TYPE propulsion) {
	Vector2i source { map_coord(startX), map_coord(startY) };
	Vector2i goal { map_coord(targetX), map_coord(targetY) };

	FLOWFIELDREQUEST request;
	request.mapSource = source;
	request.mapGoal = goal;
	request.propulsion = propulsion;

	wz::packaged_task<FLOWFIELDREQUEST()> requestTask([request]() { return processFlowfield(request); });

	wzMutexLock(ffpathMutex);
	bool isFirstRequest = flowfieldRequests.empty();
	flowfieldRequests.push_back(std::move(requestTask));
	wzMutexUnlock(ffpathMutex);

	if (isFirstRequest)
	{
		wzSemaphorePost(ffpathSemaphore);  // Wake up processing thread.
	}
}

/** This runs in a separate thread */
static int ffpathThreadFunc(void *)
{
	wzMutexLock(ffpathMutex);

	while (!ffpathQuit)
	{
		if (flowfieldRequests.empty())
		{
			wzMutexUnlock(ffpathMutex);
			wzSemaphoreWait(ffpathSemaphore);  // Go to sleep until needed.
			wzMutexLock(ffpathMutex);
			continue;
		}

		if(!flowfieldRequests.empty())
		{
			// Copy the first request from the queue.
			auto flowfieldRequest = std::move(flowfieldRequests.front());
			flowfieldRequests.pop_front();

			wzMutexUnlock(ffpathMutex);
			flowfieldRequest();
			wzMutexLock(ffpathMutex);
		}
	}
	wzMutexUnlock(ffpathMutex);
	return 0;
}


// initialise the findpath module
bool ffpathInitialise()
{
	// The path system is up
	ffpathQuit = false;

	if (!ffpathThread)
	{
		printf("Initialising thread\n");
		ffpathMutex = wzMutexCreate();
		ffpathSemaphore = wzSemaphoreCreate(0);
		ffpathThread = wzThreadCreate(ffpathThreadFunc, nullptr);
		wzThreadStart(ffpathThread);
	}

	return true;
}


void ffpathShutdown()
{
	if (ffpathThread)
	{
		// Signal the path finding thread to quit
		ffpathQuit = true;
		wzSemaphorePost(ffpathSemaphore);  // Wake up thread.

		wzThreadJoin(ffpathThread);
		ffpathThread = nullptr;
		wzMutexDestroy(ffpathMutex);
		ffpathMutex = nullptr;
		wzSemaphoreDestroy(ffpathSemaphore);
		ffpathSemaphore = nullptr;
	}
}

// Propulsion mapping FOR READING DATA ONLY! See below.
const std::map<PROPULSION_TYPE, int> propulsionToIndex
{
	// All these share the same flowfield, because they are different types of ground-only
	{PROPULSION_TYPE_WHEELED, 0},
	{PROPULSION_TYPE_TRACKED, 0},
	{PROPULSION_TYPE_LEGGED, 0},
	{PROPULSION_TYPE_HALF_TRACKED, 0},
	//////////////////////////////////
	{PROPULSION_TYPE_PROPELLOR, 1},
	{PROPULSION_TYPE_HOVER, 2},
	{PROPULSION_TYPE_LIFT, 3}
};

// Propulsion used in for-loops FOR WRITING DATA. We don't want to process "0" index multiple times.
const std::map<PROPULSION_TYPE, int> propulsionToIndexUnique
{
	{PROPULSION_TYPE_WHEELED, 0},
	{PROPULSION_TYPE_PROPELLOR, 1},
	{PROPULSION_TYPE_HOVER, 2},
	{PROPULSION_TYPE_LIFT, 3}
};

// Cost fields for ground, hover and lift movement types
std::array<std::array<uint8_t, FF_MAP_AREA>, 4> costFields;

// Mutex for sector-level vector field cache
std::mutex flowfieldMutex;

// Caches
typedef std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>> flowfieldCacheT;

std::array<std::unique_ptr<flowfieldCacheT>, 4> flowfieldCache {
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT()),
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT()),
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT()),
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT())
};

//////////////////////////////////////////////////////////////////////////////////////

struct Node {
	unsigned short predecessorCost;
	unsigned int index;

	bool operator<(const Node& other) const {
		// We want top element to have lowest cost
		return predecessorCost > other.predecessorCost;
	}
};

void calculateIntegrationField(const std::vector<ComparableVector2i>& points, const sectorListT& sectors, AbstractSector* sector, Sector* integrationField);
void integrateFlowfieldPoints(std::priority_queue<Node>& openSet, const sectorListT& sectors, AbstractSector* sector, Sector* integrationField);
void calculateFlowfield(Flowfield* flowField, Sector* integrationField);
unsigned short getCostOrElse(Sector* integrationField, Vector2i coords, unsigned short elseCost);

void processFlowfield(FLOWFIELDREQUEST request) {

	// NOTE for us noobs!!!! This function is executed on its own thread!!!!

	printf("### Process flowfield from (%i, %i) to (%i, %i)\n", request.mapSource.x, request.mapSource.y, request.mapGoal.x, request.mapGoal.y);

	auto& sectors = costFields[propulsionToIndex.at(request.propulsion)];
	auto& localFlowfieldCache = *flowfieldCache[propulsionToIndex.at(request.propulsion)];

	Vector2i localStartPoint = request.mapSource;




	// Final goal task
	// TODO: in future with better integration with Warzone, there might be multiple goals for a formation, so droids don't bump into each other
	std::vector<ComparableVector2i> finalGoals { request.mapGoal };

	if (localFlowfieldCache.count(finalGoals) == 0) {










		Flowfield* flowField = new Flowfield();
		auto sectorId = AbstractSector::getIdByCoords(*goals.begin());
		printf("Processing flowfield [%i] (%i, %i)\n", (int)goals.size(), goals[0].x, goals[0].y);
		flowField->sectorId = sectorId;
		auto& sector = sectors[sectorId];

		// NOTE: Vector field for given might have been calculated by the time this task have chance to run.
		// I don't care, since this task has proven to be short, and I want to avoid lock contention when checking cache

		Sector* integrationField = new Sector();
		printf("----start\n");
		calculateIntegrationField(goals, sectors, sector.get(), integrationField);
		printf("----end\n");

		calculateFlowfield(flowField, integrationField);

		{
			std::lock_guard<std::mutex> lock(flowfieldMutex);
			auto cache = flowfieldCache[propulsionToIndex.at(propulsion)].get();

			printf("Inserting (%i, %i)[+%i] into cache\n", goals[0].x, goals[0].y, (int)goals.size() - 1);

			cache->insert(std::make_pair(goals, std::unique_ptr<Flowfield>(flowField)));
		}







		printf("Processing flowfield (%i, %i)\n", finalGoals[0].x, finalGoals[0].y);
		processFlowfield(finalGoals, portals, sectors, request.propulsion);
	}
	printf("Finished processing flowfield (%i, %i)\n", finalGoals[0].x, finalGoals[0].y);

}

void calculateIntegrationField(const std::vector<ComparableVector2i>& points, const sectorListT& sectors, AbstractSector* sector, Sector* integrationField) {
	// TODO: here do checking if given tile contains a building (instead of doing that in cost field)
	// TODO: split NOT_PASSABLE into a few constants, for terrain, buildings and maybe sth else
	for (unsigned int x = 0; x < SECTOR_SIZE; x++) {
		for (unsigned int y = 0; y < SECTOR_SIZE; y++) {
			Tile tile;
			tile.cost = NOT_PASSABLE;
			integrationField->setTile({x, y}, tile);
		}
	}

	// Thanks to priority queue, we have "water pouring effect".
	// First we go where cost is the lowest, so we don't discover better path later.
	std::priority_queue<Node> openSet;

	for (auto& point : points) {
		openSet.push({ 0, pointToIndex(point) });
	}

	while (!openSet.empty()) {
		integrateFlowfieldPoints(openSet, sectors, sector, integrationField);
		openSet.pop();
	}
}

void integrateFlowfieldPoints(std::priority_queue<Node>& openSet, const sectorListT& sectors, AbstractSector* sector, Sector* integrationField) {
	const Node& node = openSet.top();
	Vector2i nodePoint = getPointByFlatIndex(node.index);
	Tile nodeTile = sector->getTile(nodePoint);

	if (nodeTile.isBlocking()) {
		return;
	}

	unsigned short nodeCostFromCostField = nodeTile.cost;

	// Go to the goal, no matter what
	if (node.predecessorCost == 0) {
		nodeCostFromCostField = COST_MIN;
	}

	const unsigned short newCost = node.predecessorCost + nodeCostFromCostField;
	const unsigned short nodeOldCost = integrationField->getTile(nodePoint).cost;

	if (newCost < nodeOldCost) {
		Tile tile;
		tile.cost = newCost;
		integrationField->setTile(nodePoint, tile);

		for (unsigned int neighbor : AbstractSector::getNeighbors(sectors, nodePoint)) {
			openSet.push({ newCost, neighbor });
		}
	}
}

void calculateFlowfield(Flowfield* flowField, Sector* integrationField) {
	for (int y = 0; y < SECTOR_SIZE; y++) {
		for (int x = 0; x < SECTOR_SIZE; x++) {
			Vector2i p = {x, y};
			Tile tile = integrationField->getTile(p);
			if (tile.isBlocking() || tile.cost == COST_MIN) {
				// Skip goal and non-passable
				// TODO: probably 0.0 should be only for actual goals, not intermediate goals when crossing sectors
				flowField->setVector(p, VectorT { 0.0f, 0.0f });
				continue;
			}

			// Use current tile cost when no cost available.
			// This will either keep the vector horizontal or vertical, or turn away from higher-cost neighbor
			// NOTICE: Flow field on sector borders might be not optimal
			const unsigned short leftCost = getCostOrElse(integrationField, {x - 1, y}, tile.cost);
			const unsigned short rightCost = getCostOrElse(integrationField, {x + 1, y}, tile.cost);

			const unsigned short topCost = getCostOrElse(integrationField, {x, y - 1}, tile.cost);
			const unsigned short bottomCost = getCostOrElse(integrationField, {x, y + 1}, tile.cost);

			VectorT vector;
			vector.x = leftCost - rightCost;
			vector.y = topCost - bottomCost;
			vector.normalize();

			if (std::abs(vector.x) < 0.01f && std::abs(vector.y) < 0.01f) {
				// Local optima. Tilt the vector in any direction.
				vector.x = 0.1f;
				vector.y = 0.1f;
			}

			flowField->setVector(p, vector);
		}
	}
}

unsigned short getCostOrElse(Sector* integrationField, Vector2i coords, unsigned short elseCost) {
	if (coords.x < 0 || coords.y < 0 || coords.x >= SECTOR_SIZE || coords.y >= SECTOR_SIZE) {
		// if near sector border, assume its safe to go to nearby sector
		return std::max<short>(static_cast<short>(elseCost), static_cast<short>(elseCost) - static_cast<short>(1));
	}

	const Tile& tile = integrationField->getTile(coords);
	if (tile.isBlocking()) {
		return elseCost * OBSTACLE_AVOIDANCE_COEFF;
	}

	return tile.cost;
}

void initCostFields()
{
	numSectorsHorizontal = ceil(mapWidth * 1.f / SECTOR_SIZE);
	numSectorsVertical = ceil(mapHeight * 1.f / SECTOR_SIZE);
	numSectors = numSectorsHorizontal * numSectorsVertical;
	
	// Reserve and fill cost fields with empty sectors
	for (auto& sectors : costFields)
	{
		sectors.reserve(numSectors);

		for(auto y = 0; y < numSectorsVertical; y++){
			for(auto x = 0; x < numSectorsHorizontal; x++){
				auto sector = new Sector();
				sector->position = { x * SECTOR_SIZE, y * SECTOR_SIZE };
				sectors.push_back(std::unique_ptr<Sector>(sector));
			}
		}
	}

	numHorizontalTiles = numSectorsHorizontal * SECTOR_SIZE;
	numVerticalTiles = numSectorsVertical * SECTOR_SIZE;
	
	// Fill tiles in sectors
	for (int x = 0; x < numHorizontalTiles; x++)
	{
		for (int y = 0; y < numVerticalTiles; y++)
		{
			Vector2i p = {x, y};
			const unsigned int sectorId = Sector::getIdByCoords(p);

			for (auto&& propType : propulsionToIndexUnique)
			{
				const Tile groundTile = createTile(p, propType.first);
				costFields[propType.second][sectorId]->setTile(p, groundTile);
			}
		}
	}
}

void destroyCostFields()
{
	for (auto& sectors : costFields)
	{
		sectors.clear();
	}
}

void destroyPortals()
{
	for (auto& portal : portalArr)
	{
		portal.clear();
	}
}

void destroyFlowfieldCache() {
	for (auto&& pair : propulsionToIndexUnique) {
		portalPathCache[pair.second]->clear();
		flowfieldCache[pair.second]->clear();
	}
}

Tile createTile(Vector2i p, PROPULSION_TYPE propulsion)
{
	unsigned short cost = NOT_PASSABLE;
	const bool isBlocking = fpathBlockingTile(p.x, p.y, propulsion);

	// TODO: Current impl forbids VTOL from flying over short buildings
	if (!isBlocking)
	{
		int pMax, pMin;
		getTileMaxMin(p.x, p.y, &pMax, &pMin);

		const auto delta = static_cast<unsigned short>(pMax - pMin);

		if (propulsion != PROPULSION_TYPE_LIFT && delta > SLOPE_THRESOLD)
		{
			// Yes, the cost is integer and we do not care about floating point tail
			cost = std::max(COST_MIN, static_cast<unsigned short>(SLOPE_COST_BASE * delta));
		}
		else
		{
			cost = COST_MIN;
		}
	}

	Tile tile;
	tile.cost = cost;
	return tile;
}

unsigned int pointToIndex(Vector2i p) {
	return p.y * numHorizontalTiles + p.x;
}

Vector2i getPointByFlatIndex(unsigned int index) {
	const unsigned int y = index / numHorizontalTiles;
	const unsigned int x = index % numHorizontalTiles;
	return Vector2i { x, y };
}

unsigned int straightLineDistance(Vector2i source, Vector2i destination) {
	const unsigned int dx = abs(static_cast<int>(source.x) - static_cast<int>(destination.x));
	const unsigned int dy = abs(static_cast<int>(source.y) - static_cast<int>(destination.y));
	return 1 * sqrt(dx * dx + dy * dy);
}

std::deque<unsigned int> getFlowfieldPathFromCache(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion) {
	std::lock_guard<std::mutex> lock(portalPathMutex);
	auto& localPortalPathCache = *portalPathCache[propulsionToIndex.at(propulsion)];

	if(localPortalPathCache.count({sourcePortalId, goalPortalId}) == 0){
		return std::deque<unsigned int> {};
	}

	return localPortalPathCache.find({sourcePortalId, goalPortalId})->second;
}

Vector2f getMovementVector(unsigned int nextPortalId, Vector2i currentPosition, PROPULSION_TYPE propulsion) {
	auto&& portals = portalArr[propulsionToIndex.at(propulsion)];
	const Portal& nextPortal = portals.at(nextPortalId);
	std::vector<ComparableVector2i> goals = portalToGoals(nextPortal, currentPosition);

	std::lock_guard<std::mutex> lock(flowfieldMutex);
	flowfieldCacheT& localFlowfieldCache = *flowfieldCache[propulsionToIndex.at(propulsion)];

	if(localFlowfieldCache.count(goals) == 0){
		// (0,0) vector considered invalid
		return { 0.0f, 0.0f };
	}

	VectorT vector = localFlowfieldCache.find(goals)->second->getVector(currentPosition);

	return { vector.x, vector.y };
}

void debugDrawFlowfield(const glm::mat4 &mvp) {
	pie_SetRendMode(REND_OPAQUE);

	const auto playerXTile = map_coord(player.p.x);
	const auto playerZTile = map_coord(player.p.z);
	
	const auto& groundSectors = costFields[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)];

	std::map<unsigned int, bool> sectorsInView;

	for (auto deltaX = -6; deltaX <= 6; deltaX++)
	{
		const auto x = playerXTile + deltaX;

		if(x < 0){
			continue;
		}
		
		for (auto deltaZ = -6; deltaZ <= 6; deltaZ++)
		{
			const auto z = playerZTile + deltaZ;

			if(z < 0){
				continue;
			}

			auto sectorId = AbstractSector::getIdByCoords({x, z});
			auto sector = groundSectors[sectorId].get();
			if(!sectorsInView.count(sectorId)){
				sectorsInView[sectorId] = true;
			}

			const float XA = world_coord(x);
			const float XB = world_coord(x + 1);
			const float ZA = world_coord(z);
			const float ZB = world_coord(z + 1);
			
			float height = map_TileHeight(x, z);

			// tile

			iV_PolyLine({
				{ XA, height, -ZA },
				{ XA, height, -ZB },
				{ XB, height, -ZB },
				{ XB, height, -ZA },
				{ XA, height, -ZA },
			}, mvp, WZCOL_GREY);

			// cost

			const Vector3i a = { (XA + XB) / 2, height, -(ZA + ZB) / 2 };
			Vector2i b;

			pie_RotateProject(&a, mvp, &b);
			auto cost = sector->getTile({x, z}).cost;
			if(cost != NOT_PASSABLE){
				WzText costText(std::to_string(cost), font_medium);
				costText.render(b.x, b.y, WZCOL_TEXT_BRIGHT);
			}

			// position

			if(x < 999 && z < 999){
				char positionString[7];
				ssprintf(positionString, "%i,%i", x, z);
				const Vector3i positionText3dCoords = { (XA + 20), height, -(ZB - 20) };
				Vector2i positionText2dCoords;

				pie_RotateProject(&positionText3dCoords, mvp, &positionText2dCoords);
				WzText positionText(positionString, font_small);
				positionText.render(positionText2dCoords.x, positionText2dCoords.y, WZCOL_LBLUE);
			}
	 	}
	}

	// flowfields

	auto cache = flowfieldCache[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)].get();

	for (auto const& cacheEntry: *cache) {
		auto sector = groundSectors[cacheEntry.second->sectorId].get();
		
		auto& flowfield = cacheEntry.second;
		for (int y = 0; y < SECTOR_SIZE; y++) {
			for (int x = 0; x < SECTOR_SIZE; x++) {
				auto vector = flowfield->getVector({x, y});
				
				auto startPointX = world_coord(sector->position.x + x) + FF_TILE_SIZE / 2;
				auto startPointY = world_coord(sector->position.y + y) + FF_TILE_SIZE / 2;

				auto portalHeight = map_TileHeight(startPointX, startPointY);

				// origin

				iV_PolyLine({
					{ startPointX - 10, portalHeight + 10, -startPointY - 10 },
					{ startPointX - 10, portalHeight + 10, -startPointY + 10 },
					{ startPointX + 10, portalHeight + 10, -startPointY + 10 },
					{ startPointX + 10, portalHeight + 10, -startPointY - 10 },
					{ startPointX - 10, portalHeight + 10, -startPointY - 10 },
				}, mvp, WZCOL_WHITE);
				
				// direction

				iV_PolyLine({
					{ startPointX, portalHeight + 10, -startPointY },
					{ startPointX + vector.x * 75, portalHeight + 10, -startPointY - vector.y * 75 },
				}, mvp, WZCOL_WHITE);
			}
		}
		
		// goal

		for (auto&& goal : cacheEntry.first) {
			auto goalX = world_coord(goal.x) + FF_TILE_SIZE / 2;
			auto goalY = world_coord(goal.y) + FF_TILE_SIZE / 2;
			auto portalHeight = map_TileHeight(goalX, goalY);
			iV_PolyLine({
				{ goalX - 10 + 3, portalHeight + 10, -goalY - 10 + 3 },
				{ goalX - 10 + 3, portalHeight + 10, -goalY + 10 + 3 },
				{ goalX + 10 + 3, portalHeight + 10, -goalY + 10 + 3 },
				{ goalX + 10 + 3, portalHeight + 10, -goalY - 10 + 3 },
				{ goalX - 10 + 3, portalHeight + 10, -goalY - 10 + 3 },
			}, mvp, WZCOL_RED);
		}
	}
}

void debugDrawFlowfields(const glm::mat4 &mvp) {
	if (!isFlowfieldEnabled()) return;

	if (VECTOR_FIELD_DEBUG) {
		debugDrawFlowfield(mvp);
	}
}
