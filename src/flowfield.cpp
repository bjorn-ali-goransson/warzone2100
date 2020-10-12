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

/// This type is ONLY needed for adding vectors as key to eg. a map.
/// because GLM's vector implementation does not include an is-less-than operator overload,
/// which is required by std::map.
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

constexpr const unsigned short COST_NOT_PASSABLE = std::numeric_limits<unsigned short>::max();
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

void initCostFields();
void destroyCostFields();
void destroyflowfieldCaches();

struct FLOWFIELDREQUEST
{
	/// Target position
	Vector2i goal;
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
	destroyflowfieldCaches();
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

void calculateFlowfieldAsync(unsigned int targetX, unsigned int targetY, PROPULSION_TYPE propulsion) {
	Vector2i goal { map_coord(targetX), map_coord(targetY) };

	FLOWFIELDREQUEST request;
	request.goal = goal;
	request.propulsion = propulsion;

	wz::packaged_task<FLOWFIELDREQUEST()> requestTask([request]() { processFlowfield(request); return request; });

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

// Mutex for sector-level vector field cache
std::mutex flowfieldMutex;

//////////////////////////////////////////////////////////////////////////////////////

inline unsigned int coordinateToArrayIndex(unsigned short x, unsigned short y) { return y * FF_MAP_WIDTH + x; }
inline Vector2i arrayIndexToCoordinate(unsigned int index) { return Vector2i { index % FF_MAP_WIDTH, index / FF_MAP_WIDTH }; }

struct IntegrationField {
	unsigned short cost[FF_MAP_AREA];
	void setCost(unsigned int x, unsigned int y, unsigned short cost){
		this->cost[coordinateToArrayIndex(x, y)] = cost;
	}
	unsigned short getCost(unsigned int x, unsigned int y){
		return this->cost[coordinateToArrayIndex(x, y)];
	}
	void setCost(unsigned int index, unsigned short cost){
		this->cost[index] = cost;
	}
	unsigned short getCost(unsigned int index){
		return this->cost[index];
	}
};

struct CostField {
	unsigned short cost[FF_MAP_AREA];
	void setCost(unsigned int x, unsigned int y, unsigned short cost){
		this->cost[coordinateToArrayIndex(x, y)] = cost;
	}
	unsigned short getCost(unsigned int x, unsigned int y){
		return this->cost[coordinateToArrayIndex(x, y)];
	}
	void setCost(unsigned int index, unsigned short cost){
		this->cost[index] = cost;
	}
	unsigned short getCost(unsigned int index){
		return this->cost[index];
	}
};

// TODO: Remove this in favor of glm.
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

unsigned int flowfieldIdIncrementor = 0;
IntegrationField* latestIntegrationField = nullptr;

struct Flowfield {
	unsigned int id;
	std::array<VectorT, FF_MAP_AREA> vectors;

	void setVector(unsigned short x, unsigned short y, VectorT vector) {
		vectors[coordinateToArrayIndex(x, y)] = vector;
	}
	VectorT getVector(unsigned short x, unsigned short y) const {
		return vectors[coordinateToArrayIndex(x, y)];
	}
};

// Cost fields for ground, hover and lift movement types
std::array<std::unique_ptr<CostField>, 4> costFields {
	std::unique_ptr<CostField>(new CostField()),
	std::unique_ptr<CostField>(new CostField()),
	std::unique_ptr<CostField>(new CostField()),
	std::unique_ptr<CostField>(new CostField()),
};

// Flow field cache for ground, hover and lift movement types
std::array<std::unique_ptr<std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>>, 4> flowfieldCaches {
	std::unique_ptr<std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>>(new std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>()),
	std::unique_ptr<std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>>(new std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>()),
	std::unique_ptr<std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>>(new std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>()),
	std::unique_ptr<std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>>(new std::map<std::vector<ComparableVector2i>, std::unique_ptr<Flowfield>>())
};

bool tryGetFlowfieldForTarget(unsigned int targetX, unsigned int targetY, PROPULSION_TYPE propulsion, unsigned int &flowfieldId){
	const auto flowfieldCache = flowfieldCaches[propulsionToIndex.at(propulsion)].get();

	std::vector<ComparableVector2i> goals { { map_coord(targetX), map_coord(targetY) } };

 	// this check is already done in fpath.cpp.
	// TODO: we should perhaps refresh the flowfield instead of just bailing here.
	if (!flowfieldCache->count(goals)) {
		return false;
	}

	const auto flowfield = flowfieldCache->at(goals).get();

	flowfieldId = flowfield->id;

	return true;
}

std::map<unsigned int, Flowfield*> flowfieldById;
bool tryGetFlowfieldVector(unsigned int flowfieldId, int x, int y, Vector2f& vector){
	if(!flowfieldById.count(flowfieldId)){
		return false;
	}

	auto flowfield = flowfieldById.at(flowfieldId);

	auto v = flowfield->getVector(x, y);
	vector = { v.x, v.y };

	return true;
}

struct Node {
	unsigned short predecessorCost;
	unsigned int index;

	bool operator<(const Node& other) const {
		// We want top element to have lowest cost
		return predecessorCost > other.predecessorCost;
	}
};

void calculateIntegrationField(const std::vector<ComparableVector2i>& points, IntegrationField* integrationField, CostField* costField);
void calculateFlowfield(Flowfield* flowField, IntegrationField* integrationField);

void processFlowfield(FLOWFIELDREQUEST request) {
	// NOTE for us noobs!!!! This function is executed on its own thread!!!!

	const auto flowfieldCache = flowfieldCaches[propulsionToIndex.at(request.propulsion)].get();
	const auto costField = costFields[propulsionToIndex.at(request.propulsion)].get();

	std::vector<ComparableVector2i> goals { request.goal }; // TODO: multiple goals for formations

 	// this check is already done in fpath.cpp.
	// TODO: we should perhaps refresh the flowfield instead of just bailing here.
	if (flowfieldCache->count(goals)) {
		return;
	}

	IntegrationField* integrationField = new IntegrationField();
	latestIntegrationField = integrationField;
	calculateIntegrationField(goals, integrationField, costField); // TODO: measure time

	Flowfield* flowfield = new Flowfield();
	flowfield->id = flowfieldIdIncrementor++;
	flowfieldById.insert(std::make_pair(flowfield->id, flowfield));
	calculateFlowfield(flowfield, integrationField); // TODO: measure time

	{
		std::lock_guard<std::mutex> lock(flowfieldMutex);
		flowfieldCache->insert(std::make_pair(goals, std::unique_ptr<Flowfield>(flowfield)));
	}
}

void integrateFlowfieldPoints(std::priority_queue<Node>& openSet, IntegrationField* integrationField, CostField* costField);

void calculateIntegrationField(const std::vector<ComparableVector2i>& points, IntegrationField* integrationField, CostField* costField) {
	// TODO: here do checking if given tile contains a building (instead of doing that in cost field)
	// TODO: split COST_NOT_PASSABLE into a few constants, for terrain, buildings and maybe sth else
	for (unsigned int x = 0; x < mapWidth; x++) {
		for (unsigned int y = 0; y < mapHeight; y++) {
			integrationField->setCost(x, y, COST_NOT_PASSABLE);
		}
	}

	// Thanks to priority queue, we get the water ripple effect - closest tile first.
	// First we go where cost is the lowest, so we don't discover better path later.
	std::priority_queue<Node> openSet;

	for (auto& point : points) {
		openSet.push({ 0, coordinateToArrayIndex(point.x, point.y) });
	}

	while (!openSet.empty()) {
		integrateFlowfieldPoints(openSet, integrationField, costField);
		openSet.pop();
	}
}

void integrateFlowfieldPoints(std::priority_queue<Node>& openSet, IntegrationField* integrationField, CostField* costField) {
	const Node& node = openSet.top();
	auto cost = costField->getCost(node.index);

	if (cost == COST_NOT_PASSABLE) {
		return;
	}

	// Go to the goal, no matter what
	if (node.predecessorCost == 0) {
		cost = COST_MIN;
	}

	const unsigned short integrationCost = node.predecessorCost + cost;
	const unsigned short oldIntegrationCost = integrationField->getCost(node.index);

	if (integrationCost < oldIntegrationCost) {
		integrationField->setCost(node.index, integrationCost);

		auto coordinate = arrayIndexToCoordinate(node.index);

		// North
		if(coordinate.y > 0){
			openSet.push({ integrationCost, coordinateToArrayIndex(coordinate.x, coordinate.y - 1) });
		}
		// East
		if(coordinate.x < mapWidth){
			openSet.push({ integrationCost, coordinateToArrayIndex(coordinate.x + 1, coordinate.y) });
		}
		// South
		if(coordinate.y < mapHeight){
			openSet.push({ integrationCost, coordinateToArrayIndex(coordinate.x, coordinate.y + 1) });
		}
		// West
		if(coordinate.x > 0){
			openSet.push({ integrationCost, coordinateToArrayIndex(coordinate.x - 1, coordinate.y) });
		}
	}
}

unsigned short getCostOrDefault(IntegrationField* integrationField, Vector2i coords, unsigned short defaultCost) {
	const auto cost = integrationField->getCost(coords.x, coords.y);
	
	if (cost == COST_NOT_PASSABLE) {
		return defaultCost * OBSTACLE_AVOIDANCE_COEFF;
	}

	return cost;
}

void calculateFlowfield(Flowfield* flowField, IntegrationField* integrationField) {
	for (int y = 0; y < mapHeight; y++) {
		for (int x = 0; x < mapWidth; x++) {
			const auto cost = integrationField->getCost(x, y);
			if (cost == COST_NOT_PASSABLE || cost == COST_MIN) {
				// Skip goal and non-passable
				// TODO: probably 0.0 should be only for actual goals, not intermediate goals when crossing sectors
				flowField->setVector(x, y, VectorT { 0.0f, 0.0f });
				continue;
			}

			// Use current tile cost when no cost available.
			// This will either keep the vector horizontal or vertical, or turn away from higher-cost neighbor
			// NOTICE: Flow field on sector borders might be not optimal
			const unsigned short leftCost = getCostOrDefault(integrationField, {x - 1, y}, cost);
			const unsigned short rightCost = getCostOrDefault(integrationField, {x + 1, y}, cost);

			const unsigned short topCost = getCostOrDefault(integrationField, {x, y - 1}, cost);
			const unsigned short bottomCost = getCostOrDefault(integrationField, {x, y + 1}, cost);

			VectorT vector;
			vector.x = leftCost - rightCost;
			vector.y = topCost - bottomCost;
			vector.normalize();

			if (std::abs(vector.x) < 0.01f && std::abs(vector.y) < 0.01f) {
				// Local optima. Tilt the vector in any direction.
				vector.x = 0.1f;
				vector.y = 0.1f;
			}

			flowField->setVector(x, y, vector);
		}
	}
}

unsigned short calculateTileCost(unsigned short x, unsigned short y, PROPULSION_TYPE propulsion)
{
	// TODO: Current impl forbids VTOL from flying over short buildings
	if (!fpathBlockingTile(x, y, propulsion))
	{
		int pMax, pMin;
		getTileMaxMin(x, y, &pMax, &pMin);

		const auto delta = static_cast<unsigned short>(pMax - pMin);

		if (propulsion != PROPULSION_TYPE_LIFT && delta > SLOPE_THRESOLD)
		{
			// Yes, the cost is integer and we do not care about floating point tail
			return std::max(COST_MIN, static_cast<unsigned short>(SLOPE_COST_BASE * delta));
		}
		else
		{
			return COST_MIN;
		}
	}

	return COST_NOT_PASSABLE;
}

void initCostFields()
{
	for (int x = 0; x < mapWidth; x++)
	{
		for (int y = 0; y < mapHeight; y++)
		{
			for (auto&& propType : propulsionToIndexUnique)
			{
				auto cost = calculateTileCost(x, y, propType.first);
				costFields[propType.second]->setCost(x, y, cost);
			}
		}
	}
}

void destroyCostFields()
{
	// ?
}

void destroyflowfieldCaches() {
	for (auto&& pair : propulsionToIndexUnique) {
		flowfieldCaches[pair.second]->clear();
	}
}

void debugDrawFlowfield(const glm::mat4 &mvp) {
	pie_SetRendMode(REND_OPAQUE);

	const auto playerXTile = map_coord(player.p.x);
	const auto playerZTile = map_coord(player.p.z);
	
	const auto& costField = costFields[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)];

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

			const Vector3i a = { (XA + 20), height, -(ZA + 20) };
			Vector2i b;

			pie_RotateProject(&a, mvp, &b);
			auto cost = costField->getCost(x, z);
			if(cost != COST_NOT_PASSABLE){
				WzText costText(std::to_string(cost), font_small);
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

			// integration fields

			if(latestIntegrationField != nullptr){
				const Vector3i integrationFieldText3dCoordinates = { (XA + 20), height, -(ZA + 40) };
				Vector2i integrationFieldText2dCoordinates;

				pie_RotateProject(&integrationFieldText3dCoordinates, mvp, &integrationFieldText2dCoordinates);
				auto cost = latestIntegrationField->getCost(x, z);
				WzText costText(std::to_string(cost), font_small);
				costText.render(integrationFieldText2dCoordinates.x, integrationFieldText2dCoordinates.y, WZCOL_TEXT_BRIGHT);
			}

			// flowfields

			auto cache = flowfieldCaches[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)].get();

			for (auto const& cacheEntry: *cache) {
				auto& flowfield = cacheEntry.second;
				auto vector = flowfield->getVector(x, z);
				
				auto startPointX = XA + FF_TILE_SIZE / 2;
				auto startPointY = ZA + FF_TILE_SIZE / 2;

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
	}

	// flowfields

	auto cache = flowfieldCaches[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)].get();

	for (auto const& cacheEntry: *cache) {
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

#define VECTOR_FIELD_DEBUG 1

void debugDrawFlowfields(const glm::mat4 &mvp) {
	if (!isFlowfieldEnabled()) return;

	if (VECTOR_FIELD_DEBUG) {
		debugDrawFlowfield(mvp);
	}
}
