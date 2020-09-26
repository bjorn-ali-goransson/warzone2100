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

struct ComparableVector2i : Vector2i {
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

// Sector is a square with side length of SECTOR_SIZE. 
constexpr const unsigned int SECTOR_SIZE = 16;

constexpr const unsigned short NOT_PASSABLE = std::numeric_limits<unsigned short>::max();
constexpr const unsigned short COST_MIN = 1;

// Decides how much slopes should be avoided
constexpr const float SLOPE_COST_BASE = 0.2f;
// Decides when terrain height delta is considered a slope
// TODO: I do not have much knowledge about WZ, but why almost every tile has different heights?
constexpr const unsigned short SLOPE_THRESOLD = 2;

// If an exception is thrown in thread pool FuturedTask, should we terminate or try to continue?
constexpr const bool EXCEPTION_IN_THREADPOOL_SHOULD_TERMINATE = true;

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
	bool isBlocking() const;
};

class AbstractSector {
public:
	typedef std::array<std::array<Tile, SECTOR_SIZE>, SECTOR_SIZE> tileArrayT;

	AbstractSector() = default;
	AbstractSector& operator=(AbstractSector&) = delete;
	AbstractSector& operator=(AbstractSector&&) = delete;
	AbstractSector(AbstractSector&) = delete;
	AbstractSector(AbstractSector&&) = delete;
	virtual ~AbstractSector() = default;

	virtual void setTile(Vector2i p, Tile tile) = 0;
	virtual Tile getTile(Vector2i p) const = 0;
	virtual bool checkIsEmpty() const = 0; // Actual iterating through tiles
	virtual bool isEmpty() const; // If EmptySector or just Sector
	void addPortal(unsigned int portalId);
	const std::vector<unsigned int>& getPortals() const;

	static unsigned int getIdByCoords(Vector2i p, unsigned int mapWidth);
	static unsigned int getIdByCoords(Vector2i p);
	static Vector2i getTopLeftCorner(unsigned int id); // Top-left and bottom-right
	static Vector2i getTopLeftCornerByCoords(Vector2i point); // Top-left and bottom-right
	static std::vector<unsigned int> getNeighbors(const std::vector<std::unique_ptr<AbstractSector>>& sectors, Vector2i center);

protected:
	std::vector<unsigned int> portalIds;
};

class Sector : public AbstractSector {
public:
	using AbstractSector::AbstractSector;

	void setTile(Vector2i p, Tile tile) override;
	Tile getTile(Vector2i p) const override;
	bool checkIsEmpty() const override;

private:
	tileArrayT tiles;
};

// Empty sector - optimization. Functions in this sector should always return COST_MIN.
class EmptySector : public AbstractSector {
public:
	using AbstractSector::AbstractSector;

	void setTile(Vector2i p, Tile tile) override;
	Tile getTile(Vector2i p) const override;
	bool checkIsEmpty() const override;
	bool isEmpty() const override;

private:
	std::array<std::array<Tile, 0>, 0> tiles {};
};

struct Portal {
	// Sector layout
	// Right and bottom borders are "first sector points" in each sector
	// 1 - first sector points
	// 2 - second sector points
	/*   2 2 2 2|1
		*   2       1
		*   2       1
		*   2|1 1 1 1
	*/
	
	AbstractSector* firstSector = nullptr;
	AbstractSector* secondSector = nullptr;
	std::vector<ComparableVector2i> firstSectorPoints;
	std::vector<ComparableVector2i> secondSectorPoints;
	std::vector<unsigned int> neighbors;

	Portal() = default;
	Portal(const Portal&) = delete;
	Portal& operator=(const Portal&) = delete;
	Portal(Portal&&) = default;
	Portal& operator=(Portal&&) = default;

	Portal(AbstractSector* sector1, AbstractSector* sector2, std::vector<ComparableVector2i>& firstSectorPoints, std::vector<ComparableVector2i>& secondSectorPoints);

	bool isValid() const;
	Vector2i getFirstSectorCenter() const;
	Vector2i getSecondSectorCenter() const;
};

typedef std::vector<std::unique_ptr<AbstractSector>> sectorListT;
typedef std::map<unsigned int, Portal> portalMapT;

// Generic A* algorithm. Reimplemented to better suit needs of flowfield.
// The one in astar.cpp is absolutely NOT reusable, unfortunately.
// PLEASE KEEP THIS CODE HERE CLEAN!
class AbstractAStar {
public:
	AbstractAStar() = delete;
	AbstractAStar(const AbstractAStar&) = delete;
	AbstractAStar& operator=(const AbstractAStar&) = delete;
	explicit AbstractAStar(unsigned int goal) : goal(goal) {};

	// Returns indexes of subsequent nodes in the path. Empty container means no path exists.
	std::deque<unsigned int> findPath(unsigned int startingIndex, unsigned int nodes);
	virtual bool findPathExists(unsigned int startingIndex, unsigned int nodes);
protected:
	// Implementation of following functions depends on data (tile, portal, ...)

	// Returns indexes of neighbors for given node
	virtual std::vector<unsigned int> getNeighbors(unsigned int index) = 0;

	// Returns distance between current node and considered node.
	// For grid, return "10 * tile.cost" for vertical or horizontal, and "14 * tile.cost" for diagonal. This avoids costly `sqrt(2)` for grids.
	virtual unsigned int distance(unsigned int current, unsigned int neighbor) = 0;

	// The heuristic function. Returns expected cost of moving form `start` to `goal`. Use octal for diagonal and maybe Euclidean for portals
	// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance
	virtual unsigned int heuristic(unsigned int start) = 0;

	virtual ~AbstractAStar() = default;

	unsigned int goal;
private:
	struct Node {
		unsigned int index;
		unsigned int cost;
		unsigned int heuristic;

		inline bool operator<(const Node& other) const {
			return (cost + heuristic) < (other.cost + other.heuristic);
		}
	};

	// For each node, which node it can most efficiently be reached from
	std::map<unsigned int, unsigned int> cameFrom {};

	std::deque<unsigned int> reconstructPath(unsigned int start);

	unsigned int _debugNodesVisited = 0;

	void logDebugNodesStats(unsigned int nodesTotal, int nodesInPath);
};

// This class works only in one given sector. `startingIndex` and `goal` therefore must be in range [0 ... SECTOR_SIZE^2)
class TileAStar : public AbstractAStar {
public:
	TileAStar(unsigned goal, const sectorListT& sectors);

	bool findPathExists(unsigned int startingIndex, unsigned int nodes) override;

protected:
	std::vector<unsigned int> getNeighbors(unsigned int index) override;
	unsigned int distance(unsigned int current, unsigned int neighbor) override;
	unsigned int heuristic(unsigned int start) override; // octile heuristic is preferred
private:
	Vector2i goalPoint;
	const sectorListT& sectors;
	unsigned int sectorId;

	unsigned int distanceCommon(Vector2i point1, Vector2i point2, unsigned int cost) const;
};

class PortalAStar : public AbstractAStar {
public:
	PortalAStar(const unsigned int goal, portalMapT& portals)
		: AbstractAStar(goal), portals(portals), goalPortal(portals[goal]) {
	}

protected:
	std::vector<unsigned int> getNeighbors(unsigned int index) override;
	unsigned int distance(unsigned int current, unsigned int neighbor) override;
	unsigned int heuristic(unsigned int start) override; // straight-line (any angle) heuristic is preferred
private:
	portalMapT& portals;
	const Portal& goalPortal;

	unsigned int distanceCommon(const Portal& portal1, const Portal& portal2) const;
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

class FlowfieldSector final {
public:
	typedef std::array<std::array<VectorT, SECTOR_SIZE>, SECTOR_SIZE> vectorArrayT;

	FlowfieldSector() = default;
	FlowfieldSector& operator=(FlowfieldSector&) = delete;
	FlowfieldSector& operator=(FlowfieldSector&&) = delete;
	FlowfieldSector(FlowfieldSector&) = delete;
	FlowfieldSector(FlowfieldSector&&) = default;
	~FlowfieldSector() = default;

	void setVector(Vector2i p, VectorT vector);
	VectorT getVector(Vector2i p) const;

private:
	vectorArrayT vectors;
};

struct Node {
	unsigned short predecessorCost;
	unsigned int index;

	bool operator<(const Node& other) const {
		// We want top element to have lowest cost
		return predecessorCost > other.predecessorCost;
	}
};

void initCostFields();
void costFieldReplaceWithEmpty(sectorListT& sectors);
void setupPortals();
portalMapT setupPortalsForSectors(sectorListT& sectors);

void destroyCostFields();
void destroyPortals();
void destroyFlowfieldCache();

Tile createTile(Vector2i p, PROPULSION_TYPE propulsion);

/**
	* Portal detection, by axis.
	* axisStart - starting point for one of the axis
	* otherAxis1 - value from other axis, belonging to thisSector
	* otherAxis2 - value from other axis, belonging to otherSector
	* axisEndOut - last field checked for portal existence. Used as starting point in next iteration
	* If axisStart is "x", then "otherAxis" is y.
	*/
Portal detectPortalByAxis(unsigned int axisStart, unsigned int axisEnd, unsigned int otherAxis1, unsigned int otherAxis2,
							bool isXAxis, AbstractSector& thisSector, AbstractSector& otherSector, unsigned int& axisEndOut);

void connectPortals(portalMapT& portalMap, sectorListT& sectors);
void connectPotentialNeighbor(std::pair<const unsigned int, Portal>& portalPair, unsigned int potentialNeighbor,
								TileAStar& pathfinder, portalMapT& portalMap, bool isSectorEmpty);

unsigned int pointToIndex(Vector2i p);
Vector2i getPointByFlatIndex(unsigned int index);

unsigned int straightLineDistance(Vector2i source, Vector2i destination);

std::pair<unsigned int, unsigned int> mapSourceGoalToPortals(Vector2i mapSource, Vector2i mapGoal, PROPULSION_TYPE propulsion);

/*
	* Helps to decide if we should use firstSectorPoints or secondSectorPoints as goal points
	* See Portal class for more explanation
	*/
bool isForward(Vector2i source, Vector2i firstSectorGoal, Vector2i secondSectorGoal);

std::deque<unsigned int> getFlowfieldPathFromCache(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion);
std::vector<ComparableVector2i> portalToGoals(const Portal& portal, Vector2i currentPosition);
Vector2f getMovementVector(unsigned int nextPortalId, Vector2i currentPosition, PROPULSION_TYPE propulsion);

std::vector<Vector2i> flowfieldPortalPathToCoordsPath(const std::deque<unsigned int>& path, PROPULSION_TYPE propulsion);

//////////////////////////////////////////////////////////////////////////////////////////
// +- x axis tile debug draw. Smaller values = less tiles drawn. "7" somewhat fits the default window resolution
constexpr const unsigned int DEBUG_DRAW_X_DELTA = 7;
// +- y axis tile debug draw
constexpr const unsigned int DEBUG_DRAW_Y_DELTA = 6;

void debugDrawCostField();
void debugTileDrawCost(AbstractSector* sector, Vector2i p, Vector2i screenXY);
void debugDrawPortals();
void debugDrawPortalPath();

void debugDrawFlowfields(const glm::mat4 &mvp);
void debugDrawFlowfield(const glm::mat4 &mvp);

struct ASTARREQUEST
{
	Vector2i mapSource;
	Vector2i mapGoal;
	PROPULSION_TYPE propulsion;
};

struct FLOWFIELDREQUEST
{
	std::vector<Vector2i> goals;
	std::map<unsigned int, Portal>& portals;
	const sectorListT& sectors;
	PROPULSION_TYPE propulsion;
};

static bool flowfieldEnabled = false;

void flowfieldEnable() {
	flowfieldEnabled = true;
}

bool isFlowfieldEnabled() {
	return flowfieldEnabled;
}

void flowfieldInit() {
	if (!isFlowfieldEnabled()) return;

	initCostFields();
	setupPortals();
}

void flowfieldDestroy() {
	if (!isFlowfieldEnabled()) return;

	destroyCostFields();
	destroyPortals();
	destroyFlowfieldCache();
}

std::deque<unsigned int> getFlowfieldPathFromCache(unsigned startX, unsigned startY, unsigned tX, unsigned tY, const PROPULSION_TYPE propulsion) {
	Vector2i source { map_coord(startX), map_coord(startY) };
	Vector2i goal { map_coord(tX), map_coord(tY) };
	
	unsigned int sourcePortalId, goalPortalId;
	std::tie(sourcePortalId, goalPortalId) = mapSourceGoalToPortals(source, goal, propulsion);

	return getFlowfieldPathFromCache(sourcePortalId, goalPortalId, propulsion);
}

Vector2f getMovementVector(unsigned int nextPortalId, unsigned currentX, unsigned currentY, PROPULSION_TYPE propulsion) {
	return getMovementVector(nextPortalId, { currentX, currentY }, propulsion);
}

std::vector<Vector2i> flowfieldPortalPathToCoordsPath(const std::deque<unsigned int>& path, DROID* psDroid) {
	return flowfieldPortalPathToCoordsPath(path, getPropulsionStats(psDroid)->propulsionType);
}

std::vector<ComparableVector2i> toComparableVectors(std::vector<Vector2i> values){
	std::vector<ComparableVector2i> result;

	for(auto value : values){
		result.push_back(*new ComparableVector2i(value));
	}

	return result;
}

void debugDrawFlowfields(const glm::mat4 &mvp) {
	if (!isFlowfieldEnabled()) return;

	if (COST_FIELD_DEBUG) {
		//debugDrawCostField();
	}

	if (PORTALS_DEBUG) {
		debugDrawPortals();
	}

	if (PORTAL_PATH_DEBUG) {
		debugDrawPortalPath();
	}

	if (VECTOR_FIELD_DEBUG) {
		debugDrawFlowfield(mvp);
	}
}

// If the path finding system is shutdown or not
static volatile bool ffpathQuit = false;

// threading stuff
static WZ_THREAD        *ffpathThread = nullptr;
static WZ_MUTEX         *ffpathMutex = nullptr;
static WZ_SEMAPHORE     *ffpathSemaphore = nullptr;
using aStarJob = wz::packaged_task<ASTARREQUEST()>;
static std::list<aStarJob>    aStarJobs;
static std::unordered_map<uint32_t, wz::future<ASTARREQUEST>> aStarResults;
using flowFieldJob = wz::packaged_task<FLOWFIELDREQUEST()>;
static std::list<flowFieldJob>    flowFieldJobs;
static std::unordered_map<uint32_t, wz::future<FLOWFIELDREQUEST>> flowFieldResults;

ASTARREQUEST aStarJobExecute(ASTARREQUEST job);

void calculateFlowfieldsAsync(MOVE_CONTROL * psMove, unsigned id, int startX, int startY, int tX, int tY, PROPULSION_TYPE propulsionType,
								DROID_TYPE droidType, FPATH_MOVETYPE moveType, int owner, bool acceptNearest, StructureBounds const & dstStructure) {
	Vector2i source { map_coord(startX), map_coord(startY) };
	Vector2i goal { map_coord(tX), map_coord(tY) };

	ASTARREQUEST job;
	job.mapSource = source;
	job.mapGoal = goal;
	job.propulsion = propulsionType;
printf("From (%i, %i) to (%i, %i) thru %i\n", job.mapSource.x, job.mapSource.y, job.mapGoal.x, job.mapGoal.y, propulsionType);
	aStarJob task([job]() { return aStarJobExecute(job); });

	// Add to end of list
	wzMutexLock(ffpathMutex);
	bool isFirstJob = aStarJobs.empty();
	aStarJobs.push_back(std::move(task));
	wzMutexUnlock(ffpathMutex);

	if (isFirstJob)
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
		if (aStarJobs.empty() && flowFieldJobs.empty())
		{
			wzMutexUnlock(ffpathMutex);
			wzSemaphoreWait(ffpathSemaphore);  // Go to sleep until needed.
			wzMutexLock(ffpathMutex);
			continue;
		}

		if(!aStarJobs.empty())
		{
			// Copy the first job from the queue.
			aStarJob aStarJob = std::move(aStarJobs.front());
			aStarJobs.pop_front();

			wzMutexUnlock(ffpathMutex);
			aStarJob();
			wzMutexLock(ffpathMutex);
		}

		while(!flowFieldJobs.empty()) // won't work so well with multiple threads
		{
			// Copy the first job from the queue.
			flowFieldJob flowFieldJob = std::move(flowFieldJobs.front());
			flowFieldJobs.pop_front();

			wzMutexUnlock(ffpathMutex);
			flowFieldJob();
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
std::array<sectorListT, 4> costFields;

// Portals connecting sectors
std::array<portalMapT, 4> portalArr;

// Mutex for logs and debug stuff
std::mutex logMutex;

// Mutex for portal-level A* path cache
std::mutex portalPathMutex;

// Mutex for sector-level vector field cache
std::mutex flowfieldMutex;

// Caches
typedef std::map<std::pair<unsigned int, unsigned int>, std::deque<unsigned int>> portalPathCacheT;
typedef std::map<std::vector<ComparableVector2i>, std::unique_ptr<FlowfieldSector>> flowfieldCacheT;

// Workaround because QCache is neitVector2iher copyable nor movable
std::array<std::unique_ptr<portalPathCacheT>, 4> portalPathCache {
	std::unique_ptr<portalPathCacheT>(new portalPathCacheT()),
	std::unique_ptr<portalPathCacheT>(new portalPathCacheT()),
	std::unique_ptr<portalPathCacheT>(new portalPathCacheT()),
	std::unique_ptr<portalPathCacheT>(new portalPathCacheT())
};

std::array<std::unique_ptr<flowfieldCacheT>, 4> flowfieldCache {
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT()),
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT()),
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT()),
	std::unique_ptr<flowfieldCacheT>(new flowfieldCacheT())
};

constexpr const Tile emptyTile;

unsigned int _debugTotalSectors = 0;
unsigned int _debugEmptySectors = 0;

std::deque<unsigned int> _debugPortalPath;

//////////////////////////////////////////////////////////////////////////////////////

bool Tile::isBlocking() const
{
	return cost == NOT_PASSABLE;
}

Portal::Portal(AbstractSector* sector1, AbstractSector* sector2, std::vector<ComparableVector2i>& firstSectorPoints, std::vector<ComparableVector2i>& secondSectorPoints)
	: firstSector(sector1),	secondSector(sector2), firstSectorPoints(firstSectorPoints), secondSectorPoints(secondSectorPoints)
{
	assert(firstSectorPoints.size() <= SECTOR_SIZE);
	assert(secondSectorPoints.size() <= SECTOR_SIZE);
}

bool Portal::isValid() const
{
	return !firstSectorPoints.empty();
}

Vector2i Portal::getFirstSectorCenter() const {
	return firstSectorPoints[firstSectorPoints.size() / 2];
}

Vector2i Portal::getSecondSectorCenter() const {
	return secondSectorPoints[secondSectorPoints.size() / 2];
}

unsigned int AbstractSector::getIdByCoords(Vector2i p, unsigned int mapWidth)
{
	const unsigned int xNumber = p.x / SECTOR_SIZE;
	const unsigned int yNumber = p.y / SECTOR_SIZE;
	const auto sectorsPerRow = mapWidth / SECTOR_SIZE;
	const unsigned int sectorId = yNumber * sectorsPerRow + xNumber;
	
	assert(sectorId < (mapWidth * mapHeight / (SECTOR_SIZE * SECTOR_SIZE)) && "Sector id too big");

	return sectorId;
}

unsigned int AbstractSector::getIdByCoords(Vector2i p) {
	return getIdByCoords(p, mapWidth);
}

Vector2i AbstractSector::getTopLeftCorner(unsigned int id)
{
	assert(id < (mapWidth * mapHeight / (SECTOR_SIZE * SECTOR_SIZE)) && "Sector id too big");

	const auto sectorsPerRow = mapWidth / SECTOR_SIZE;
	const unsigned int y = (id / sectorsPerRow) * SECTOR_SIZE;
	const unsigned int x = (id % sectorsPerRow) * SECTOR_SIZE;
	return Vector2i{x, y};
}

Vector2i AbstractSector::getTopLeftCornerByCoords(Vector2i point) {
	const unsigned int sectorId = AbstractSector::getIdByCoords(point);
	return AbstractSector::getTopLeftCorner(sectorId);
}

std::vector<unsigned int> AbstractSector::getNeighbors(const std::vector<std::unique_ptr<AbstractSector>>& sectors, Vector2i center) {
	assert(center.x < mapWidth);
	assert(center.y < mapHeight);
	std::vector<unsigned int> neighbors;
	const unsigned int sectorId = AbstractSector::getIdByCoords(center);

	for (int y = -1; y <= 1; y++) {
		const int realY = center.y + y;
		for (int x = -1; x <= 1; x++) {
			const int realX = center.x + x;
			if ((y == 0 && x == 0) || realY < 0 || realX < 0 || realY >= mapHeight || realX >= mapWidth) {
				// Skip self and out-of-map
				continue;
			}

			const unsigned int targetSectorId = AbstractSector::getIdByCoords({realX, realY});
			const bool isPassable = !sectors[targetSectorId]->getTile({realX, realY}).isBlocking();
			if (sectorId == targetSectorId && isPassable) {
				neighbors.push_back(realY * mapWidth + realX);
			}
		}
	}

	return neighbors;
}

bool AbstractSector::isEmpty() const
{
	return false;
}

void AbstractSector::addPortal(const unsigned int portalId)
{
	assert(std::find(portalIds.begin(), portalIds.end(), portalId) == portalIds.end() && "Portal in sector already exists");
	portalIds.push_back(portalId);
}

const std::vector<unsigned int>& AbstractSector::getPortals() const
{
	return portalIds;
}

void Sector::setTile(Vector2i p, Tile tile)
{
	assert(p.x >= 0 && p.x < mapWidth);
	assert(p.y >= 0 && p.y < mapHeight);
	this->tiles[p.x % SECTOR_SIZE][p.y % SECTOR_SIZE] = tile;
}

Tile Sector::getTile(Vector2i p) const
{
	assert(p.x >= 0 && p.x < mapWidth);
	assert(p.y >= 0 && p.y < mapHeight);
	return this->tiles[p.x % SECTOR_SIZE][p.y % SECTOR_SIZE];
}

bool Sector::checkIsEmpty() const
{
	for (auto&& row : this->tiles)
	{
		for (auto&& tile : row)
		{
			if (tile.cost != COST_MIN) return false;
		}
	}

	return true;
}

void EmptySector::setTile(Vector2i p, Tile tile)
{
	// No-op - sector is already empty
}

Tile EmptySector::getTile(Vector2i p) const
{
	return emptyTile;
}

bool EmptySector::checkIsEmpty() const
{
	return true;
}

bool EmptySector::isEmpty() const
{
	return true;
}

std::deque<unsigned int> AbstractAStar::findPath(unsigned int startingIndex, unsigned int nodes)
{
	if (findPathExists(startingIndex, nodes))
	{
		auto path = reconstructPath(startingIndex);
		logDebugNodesStats(nodes, static_cast<int>(path.size()));
		return path;
	}

	// Empty container, no path found for given goal
	return {};
}

bool AbstractAStar::findPathExists(unsigned int startingIndex, unsigned int nodes)
{
	// AKA closed set
	std::vector<bool> visited(nodes, false);

	// AKA open set
	std::set<Node> considered{Node{startingIndex, COST_MIN, heuristic(startingIndex)}};

	while (!considered.empty())
	{
		const auto&& currentIt = considered.cbegin();
		Node current = *currentIt;
		considered.erase(currentIt);
		visited[current.index] = true;

		if (DEBUG_A_STAR) {
			_debugNodesVisited++;
		}

		if (current.index == goal)
		{
			logDebugNodesStats(nodes, -1);
			return true;
		}

		for (unsigned int neighbor : getNeighbors(current.index))
		{
			if (visited[neighbor])
			{
				// Ignore the neighbor which is already evaluated.
				continue;
			}

			const unsigned int cost = current.cost + distance(current.index, neighbor);

			const auto neighborIt = std::find_if(considered.begin(), considered.end(), [=](const Node& n)
			{
				return n.index == neighbor;
			});
			if (neighborIt == considered.end() || cost < neighborIt->cost)
			{
				cameFrom[neighbor] = current.index;
				if (neighborIt != considered.end())
				{
					// Updating with new value requires deleting and re-inserting
					considered.erase(neighborIt);
				}
				considered.insert(Node{neighbor, cost, heuristic(neighbor)});
			}
		}
	}

	return false;
}

std::deque<unsigned int> AbstractAStar::reconstructPath(const unsigned int start)
{
	std::deque<unsigned int> path;
	unsigned int current = goal;

	while (current != start)
	{
		path.push_front(current);
		current = cameFrom[current];
	}

	path.push_front(start);
	return path;
}

void AbstractAStar::logDebugNodesStats(unsigned int nodesTotal, int nodesInPath) {
	if (DEBUG_A_STAR) {
		std::lock_guard<std::mutex> lock(logMutex);
		debug(LOG_FLOWFIELD, "Nodes total: %d, nodes in path: %d, nodes visited: %d\n", nodesTotal, nodesInPath, _debugNodesVisited);
	}
}

TileAStar::TileAStar(unsigned int goal, const sectorListT& sectors) : AbstractAStar(goal), sectors(sectors)
{
	assert(goal < mapWidth * mapHeight);
	goalPoint = getPointByFlatIndex(goal);
	sectorId = AbstractSector::getIdByCoords(goalPoint);
}

bool TileAStar::findPathExists(unsigned int startingIndex, unsigned int nodes)
{
	assert(startingIndex < mapWidth * mapHeight);

	unsigned int startSectorId = AbstractSector::getIdByCoords(getPointByFlatIndex(startingIndex));

	if (startSectorId != sectorId) {
		return false;
	}

	return AbstractAStar::findPathExists(startingIndex, nodes);
}

std::vector<unsigned int> TileAStar::getNeighbors(unsigned int index)
{
	assert(index < mapWidth * mapHeight);
	const Vector2i currentPoint = getPointByFlatIndex(index);

	return AbstractSector::getNeighbors(sectors, currentPoint);
}

unsigned int TileAStar::distance(unsigned int current, unsigned int neighbor)
{
	assert(current < mapWidth * mapHeight);
	assert(neighbor < mapWidth * mapHeight);
	const Vector2i currentPoint = getPointByFlatIndex(current);
	const Vector2i neighborPoint = getPointByFlatIndex(neighbor);

	return distanceCommon(currentPoint, neighborPoint, COST_MIN);
}

unsigned int TileAStar::heuristic(unsigned int start)
{
	assert(start < mapWidth * mapHeight);
	const Vector2i startPoint = getPointByFlatIndex(start);
	const unsigned int cost = sectors[sectorId]->getTile(startPoint).cost;
	return distanceCommon(startPoint, goalPoint, cost);
}

unsigned int TileAStar::distanceCommon(Vector2i point1, Vector2i point2, unsigned int cost) const
{
	const unsigned int dx = std::abs(static_cast<int>(point1.x) - static_cast<int>(point2.x));
	const unsigned int dy = abs(static_cast<int>(point1.y) - static_cast<int>(point2.y));

	// Avoid sqrt(2) for diagonal by scaling cost
	const unsigned int simpleCost = 10 * cost;
	const unsigned int diagonalCost = 14 * cost;

	return simpleCost * (dx + dy) + (diagonalCost - 2 * simpleCost) * std::min(dx, dy);
}

std::vector<unsigned int> PortalAStar::getNeighbors(unsigned int index)
{
	assert(portals.find(index) != portals.end() && "Portal does not exists");
	Portal& portal = portals[index];
	return portal.neighbors;
}

unsigned int PortalAStar::distance(unsigned int current, unsigned int neighbor)
{
	assert(portals.find(current) != portals.end() && "Portal does not exists");
	assert(portals.find(neighbor) != portals.end() && "Portal does not exists");
	Portal& currentPortal = portals[current];
	Portal& neighborPortal = portals[neighbor];
	return distanceCommon(currentPortal, neighborPortal);
}

unsigned int PortalAStar::heuristic(unsigned int start)
{
	assert(portals.find(start) != portals.end() && "Portal does not exists");
	Portal& currentPortal = portals[start];
	return distanceCommon(currentPortal, goalPortal);
}

unsigned int PortalAStar::distanceCommon(const Portal& portal1, const Portal& portal2) const
{
	return straightLineDistance(portal1.getFirstSectorCenter(), portal2.getFirstSectorCenter());
}

std::deque<unsigned int> portalWalker(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion);
void processFlowfields(ASTARREQUEST job, std::deque<unsigned int>& path);

ASTARREQUEST aStarJobExecute(ASTARREQUEST job) {

	// NOTE for us noobs!!!! This function is executed on its own thread!!!!

	if (DEBUG_BUILD) // Mutex is expensive and won't be optimized in release mode
	{
		std::lock_guard<std::mutex> lock(logMutex);
		debug(LOG_FLOWFIELD, "Path request calculation for start x: %d, y: %d, goal x: %d, y: %d, propulsion: %d\n",
				job.mapSource.x, job.mapSource.y, job.mapGoal.x, job.mapGoal.y, job.propulsion);
	}

	unsigned int sourcePortalId, goalPortalId;
	std::tie(sourcePortalId, goalPortalId) = mapSourceGoalToPortals(job.mapSource, job.mapGoal, job.propulsion);

	std::deque<unsigned int> path = portalWalker(sourcePortalId, goalPortalId, job.propulsion);

	if (DEBUG_BUILD) // Mutex is expensive and won't be optimized in release mode
	{
		std::lock_guard<std::mutex> lock(logMutex);
		_debugPortalPath = path;
	}

	processFlowfields(job, path);

	return job;
}

std::deque<unsigned int> portalWalker(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion) {
	std::unique_lock<std::mutex> lock(portalPathMutex);
	auto& localPortalPathCache = *portalPathCache[propulsionToIndex.at(propulsion)];
	std::deque<unsigned int>* pathPtr = &localPortalPathCache[{sourcePortalId, goalPortalId}];
	if (pathPtr) {
		if (DEBUG_BUILD) // Mutex is expensive and won't be optimized in release mode
		{
			std::lock_guard<std::mutex> lock(logMutex);
			debug(LOG_FLOWFIELD, "Flowfield portal cache hit\n");
		}

		return *pathPtr;
	} else {
		lock.unlock();

		if (DEBUG_BUILD) // Mutex is expensive and won't be optimized in release mode
		{
			std::lock_guard<std::mutex> lock(portalPathMutex);
			debug(LOG_FLOWFIELD, "Flowfield portal cache miss\n");
		}

		portalMapT& portals = portalArr[propulsionToIndex.at(propulsion)];
		PortalAStar portalWalker(goalPortalId, portals);
		std::deque<unsigned int> path = portalWalker.findPath(sourcePortalId, static_cast<unsigned int>(portals.size()));

		lock.lock();
		auto pathCopy = std::unique_ptr<std::deque<unsigned int>>(new std::deque<unsigned int>(path));
		localPortalPathCache.insert(std::make_pair(std::pair<unsigned int, unsigned int>(sourcePortalId, goalPortalId), *pathCopy.release()));
		return path;
	}
}

void FlowfieldSector::setVector(Vector2i p, VectorT vector) {
	vectors[p.x][p.y] = vector;
}

VectorT FlowfieldSector::getVector(Vector2i p) const {
	return vectors[p.x][p.y];
}

void processFlowfield(std::vector<ComparableVector2i> goals, portalMapT& portals, const sectorListT& sectors, PROPULSION_TYPE propulsion);
unsigned short getCostOrElse(Sector* integrationField, Vector2i coords, unsigned short elseCost);

void processFlowfields(ASTARREQUEST job, std::deque<unsigned int>& path) {
	auto& portals = portalArr[propulsionToIndex.at(job.propulsion)];
	auto& sectors = costFields[propulsionToIndex.at(job.propulsion)];
	auto& localFlowfieldCache = *flowfieldCache[propulsionToIndex.at(job.propulsion)];

	Vector2i localStartPoint = job.mapSource;

	for (unsigned int leavePortalId : path) {
		Portal& leavePortal = portals[leavePortalId];
		std::vector<ComparableVector2i> goals = portalToGoals(leavePortal, localStartPoint);
		localStartPoint = goals[0];

		if (localFlowfieldCache.count(goals) == 0) {
			processFlowfield(goals, portals, sectors, job.propulsion);
		}
	}

	// Final goal task
	// TODO: in future with better integration with Warzone, there might be multiple goals for a formation, so droids don't bump into each other
	std::vector<ComparableVector2i> finalGoals { job.mapGoal };

	if (localFlowfieldCache.count(finalGoals) == 0) {
		processFlowfield(finalGoals, portals, sectors, job.propulsion);
	}
}

void calculateIntegrationField(const std::vector<ComparableVector2i>& points, const sectorListT& sectors, AbstractSector* sector, Sector* integrationField);
void integrateFlowfieldPoints(std::priority_queue<Node>& openSet, const sectorListT& sectors, AbstractSector* sector, Sector* integrationField);
void calculateFlowfield(FlowfieldSector* flowField, Sector* integrationField);

void processFlowfield(std::vector<ComparableVector2i> goals, portalMapT& portals, const sectorListT& sectors, PROPULSION_TYPE propulsion) {
	FlowfieldSector* flowField = new FlowfieldSector();
	auto sectorId = AbstractSector::getIdByCoords(*goals.begin());
	auto& sector = sectors[sectorId];

	// NOTE: Vector field for given might have been calculated by the time this task have chance to run.
	// I don't care, since this task has proven to be short, and I want to avoid lock contention when checking cache

	Sector* integrationField = new Sector();
	calculateIntegrationField(goals, sectors, sector.get(), integrationField);

	calculateFlowfield(flowField, integrationField);

	{
		std::lock_guard<std::mutex> lock(flowfieldMutex);
		auto cache = flowfieldCache[propulsionToIndex.at(propulsion)].get();
		cache->insert(std::make_pair(goals, std::unique_ptr<FlowfieldSector>(flowField)));
	}
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

void calculateFlowfield(FlowfieldSector* flowField, Sector* integrationField) {
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
	// Assume map is already loaded. Access globals
	assert(mapWidth % SECTOR_SIZE == 0);
	assert(mapHeight % SECTOR_SIZE == 0);

	const int numSectors = (mapWidth / SECTOR_SIZE) * (mapHeight / SECTOR_SIZE);

	// Reserve and fill cost fields with empty sectors
	for (auto& sectors : costFields)
	{
		sectors.reserve(numSectors);
		for (int i = 0; i < numSectors; i++)
		{
			sectors.push_back(std::unique_ptr<Sector>(new Sector()));
		}
	}

	// Fill tiles in sectors
	for (int x = 0; x < mapWidth; x++)
	{
		for (int y = 0; y < mapHeight; y++)
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

	// Optimization: replace sectors with no cost with empty sector
	for (auto& sectors : costFields)
	{
		costFieldReplaceWithEmpty(sectors);
		_debugTotalSectors += static_cast<unsigned int>(sectors.size());
	}
}

void costFieldReplaceWithEmpty(sectorListT& sectors)
{
	for (auto& sector : sectors)
	{
		if (sector->checkIsEmpty())
		{
			sector = std::unique_ptr<EmptySector>(new EmptySector());
			_debugEmptySectors++;
		}
	}
}

void setupPortals()
{
	for (auto&& propType : propulsionToIndexUnique)
	{
		auto portalMap = setupPortalsForSectors(costFields[propType.second]);
		connectPortals(portalMap, costFields[propType.second]);
		portalArr[propType.second] = std::move(portalMap);
	}
}

portalMapT setupPortalsForSectors(sectorListT& sectors)
{
	portalMapT portals;
	const auto sectorsPerRow = mapWidth / SECTOR_SIZE;
	const auto lastRow = sectors.size() - sectorsPerRow;

	const auto portalAppender = [&](Portal& portalByAxis, AbstractSector& thisSector, AbstractSector& otherSector)
	{
		if (portalByAxis.isValid())
		{
			auto index = static_cast<unsigned int>(portals.size());
			portals[index] = std::move(portalByAxis);
			thisSector.addPortal(index);
			otherSector.addPortal(index);
		}
	};

	for (unsigned int i = 0; i < sectors.size(); i++)
	{
		const auto corner = AbstractSector::getTopLeftCorner(i);
		AbstractSector& thisSector = *sectors[i];

		// Bottom. Skip last row
		if (i < lastRow)
		{
			unsigned short failsafeCounter = 0;
			unsigned int x = corner.x;
			do
			{
				AbstractSector& otherSector = *sectors[i + sectorsPerRow];
				Portal portalByAxis = detectPortalByAxis(x, corner.x + SECTOR_SIZE, corner.y + SECTOR_SIZE - 1, corner.y + SECTOR_SIZE, true,
															thisSector, otherSector, x);
				portalAppender(portalByAxis, thisSector, otherSector);
				x++;
				failsafeCounter++; // In case of bug, prevent infinite loop
				if (!portalByAxis.isValid())
				{
					break;
				}
			} while (failsafeCounter < SECTOR_SIZE); // There could be more than one portal
		}

		// Right. Skip last column
		if (i % sectorsPerRow != sectorsPerRow - 1)
		{
			unsigned short failsafeCounter = 0;
			unsigned int y = corner.y;
			do
			{
				AbstractSector& otherSector = *sectors[i + 1];
				Portal portalByAxis = detectPortalByAxis(y, corner.y + SECTOR_SIZE, corner.x + SECTOR_SIZE - 1, corner.x + SECTOR_SIZE, false,
															thisSector, otherSector, y);
				portalAppender(portalByAxis, thisSector, otherSector);
				y++;
				failsafeCounter++; // In case of bug, prevent infinite loop
				if (!portalByAxis.isValid())
				{
					break;
				}
			} while (failsafeCounter < SECTOR_SIZE); // There could be more than one portal
		}
	}

	return portals;
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
	assert(p.x > 0 && p.x < mapWidth);
	assert(p.y > 0 && p.y < mapHeight);
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

Portal detectPortalByAxis(unsigned int axisStart, unsigned int axisEnd, unsigned otherAxis1, unsigned otherAxis2,
							bool isXAxis, AbstractSector& thisSector, AbstractSector& otherSector, unsigned int& axisEndOut)
{
	axisEndOut = 0;
	std::vector<Vector2i> firstSectorPoints;
	std::vector<Vector2i> secondSectorPoints;
	Vector2i firstSectorPoint;
	Vector2i secondSectorPoint;

	for (unsigned int axis = axisStart; axis < axisEnd; axis++)
	{
		if (isXAxis)
		{
			firstSectorPoint = Vector2i { axis, otherAxis1 };
			secondSectorPoint = Vector2i { axis, otherAxis2 };
		}
		else
		{
			firstSectorPoint = Vector2i { otherAxis1, axis };
			secondSectorPoint = Vector2i { otherAxis2, axis };
		}

		bool thisPassable = !thisSector.getTile(firstSectorPoint).isBlocking();
		bool otherPassable = !otherSector.getTile(secondSectorPoint).isBlocking();

		if (thisPassable && otherPassable)
		{
			firstSectorPoints.push_back(firstSectorPoint);
			secondSectorPoints.push_back(secondSectorPoint);
			axisEndOut = axis;
		}
		else if (!firstSectorPoints.empty())
		{
			// Not passable, but we found some points - that means we reached end of portal
			break;
		}
	}

	if (!firstSectorPoints.empty())
	{
		std::vector<ComparableVector2i> a = toComparableVectors(firstSectorPoints);
		std::vector<ComparableVector2i> b = toComparableVectors(secondSectorPoints);
		return Portal(&thisSector, &otherSector, a, b);
	}
	else
	{
		// Invalid portal
		return Portal();
	}
}

void connectPortals(portalMapT& portalMap, sectorListT& sectors)
{
	for (auto& portalWithIndex : portalMap)
	{
		Portal& portal = portalWithIndex.second;
		TileAStar firstSectorPathfinder(pointToIndex(portal.getFirstSectorCenter()), sectors);
		TileAStar secondSectorPathfinder(pointToIndex(portal.getSecondSectorCenter()), sectors);

		for (unsigned int potentialNeighbor : portal.firstSector->getPortals())
		{
			assert(portalMap.find(potentialNeighbor) != portalMap.end() && "Portal does not exists");
			connectPotentialNeighbor(portalWithIndex, potentialNeighbor, firstSectorPathfinder, portalMap, portal.firstSector->isEmpty());
		}
		for (unsigned int potentialNeighbor : portal.secondSector->getPortals())
		{
			assert(portalMap.find(potentialNeighbor) != portalMap.end() && "Portal does not exists");
			connectPotentialNeighbor(portalWithIndex, potentialNeighbor, secondSectorPathfinder, portalMap, portal.secondSector->isEmpty());
		}
	}
}

unsigned int pointToIndex(Vector2i p) {
	return p.y * mapWidth + p.x;
}

Vector2i getPointByFlatIndex(unsigned int index) {
	assert(index < mapWidth * mapHeight);
	const unsigned int y = index / mapWidth;
	const unsigned int x = index % mapWidth;
	return Vector2i { x, y };
}

void connectPotentialNeighbor(std::pair<const unsigned int, Portal>& portalWithIndex, unsigned int potentialNeighbor,
								TileAStar& pathfinder, portalMapT& portalMap, bool isSectorEmpty)
{
	bool myself = portalWithIndex.first == potentialNeighbor;
	auto& actualNeighbors = portalWithIndex.second.neighbors;
	bool alreadyProcessed = std::find(actualNeighbors.begin(), actualNeighbors.end(), potentialNeighbor) != actualNeighbors.end();
	
	if (!myself && !alreadyProcessed)
	{
		if (isSectorEmpty) {
			portalWithIndex.second.neighbors.push_back(potentialNeighbor);
			portalMap[potentialNeighbor].neighbors.push_back(portalWithIndex.first);
		}

		// We actually don't need "mapWidth * mapHeight" nodes, only SECTOR_SIZE^2, but we use absolute index for tiles
		// It's not much anyways, since it's only used for vector<bool>, which is usually compressed.
		// Worst case: 256^2 =~ 65KB in uncompressed mode, 256^2 / 8 =~ 8KB in compressed mode.
		unsigned int nodes = mapWidth * mapHeight;

		Vector2i potentialNeighborPointFirst = portalMap[potentialNeighbor].getFirstSectorCenter();
		Vector2i potentialNeighborPointSecond = portalMap[potentialNeighbor].getSecondSectorCenter();
		bool pathExists = pathfinder.findPathExists(pointToIndex(potentialNeighborPointFirst), nodes)
			|| pathfinder.findPathExists(pointToIndex(potentialNeighborPointSecond), nodes);
		if (pathExists)
		{
			portalWithIndex.second.neighbors.push_back(potentialNeighbor);
			portalMap[potentialNeighbor].neighbors.push_back(portalWithIndex.first);
		}
	}
}

unsigned int straightLineDistance(Vector2i source, Vector2i destination) {
	const unsigned int dx = abs(static_cast<int>(source.x) - static_cast<int>(destination.x));
	const unsigned int dy = abs(static_cast<int>(source.y) - static_cast<int>(destination.y));
	return 1 * sqrt(dx * dx + dy * dy);
}

std::pair<unsigned int, unsigned int> mapSourceGoalToPortals(Vector2i mapSource, Vector2i mapGoal, const PROPULSION_TYPE propulsion) {
	const unsigned int sourceSector = AbstractSector::getIdByCoords(mapSource);
	const unsigned int goalSector = AbstractSector::getIdByCoords(mapGoal);
	const auto& sectors = costFields[propulsionToIndex.at(propulsion)];
	const auto& sourcePortals = sectors[sourceSector]->getPortals();
	const auto& goalPortals = sectors[goalSector]->getPortals();

	auto& portals = portalArr[propulsionToIndex.at(propulsion)];

	// Use straight-line distance to select source portal and goal portal
	const auto lessDistance = [&](Vector2i source) {
		return [&](const unsigned int id1, const unsigned int id2) {
			Portal& p1 = portals[id1];
			Portal& p2 = portals[id2];

			const unsigned int p1Distance = straightLineDistance(source, p1.getFirstSectorCenter());
			const unsigned int p2Distance = straightLineDistance(source, p2.getFirstSectorCenter());
			return p1Distance < p2Distance;
		};
	};

	const auto sourcePortalId = *std::min_element(sourcePortals.begin(), sourcePortals.end(), lessDistance(mapSource));
	const auto goalPortalId = *std::min_element(goalPortals.begin(), goalPortals.end(), lessDistance(mapGoal));

	return { sourcePortalId , goalPortalId };
}

bool isForward(Vector2i source, Vector2i firstSectorGoal, Vector2i secondSectorGoal) {
	const Vector2i sourceTlCorner = AbstractSector::getTopLeftCornerByCoords(source);
	const Vector2i firstSectorTlCorner = AbstractSector::getTopLeftCornerByCoords(firstSectorGoal);
	const Vector2i secondSectorTlCorner = AbstractSector::getTopLeftCornerByCoords(secondSectorGoal);

	const unsigned int distFirst = straightLineDistance(sourceTlCorner, firstSectorTlCorner);
	const unsigned int distSecond = straightLineDistance(sourceTlCorner, secondSectorTlCorner);

	return distFirst < distSecond;
}

std::deque<unsigned int> getFlowfieldPathFromCache(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion) {
	std::lock_guard<std::mutex> lock(portalPathMutex);
	auto& localPortalPathCache = *portalPathCache[propulsionToIndex.at(propulsion)];

	if(localPortalPathCache.count({sourcePortalId, goalPortalId}) == 0){
		return std::deque<unsigned int> {};
	}

	return localPortalPathCache.find({sourcePortalId, goalPortalId})->second;
}

std::vector<ComparableVector2i> portalToGoals(const Portal& portal, Vector2i currentPosition) {
	const bool forward = isForward(currentPosition, portal.getFirstSectorCenter(), portal.getSecondSectorCenter());

	if (forward) {
		return portal.firstSectorPoints;
	} else {
		return portal.secondSectorPoints;
	}
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

std::vector<Vector2i> flowfieldPortalPathToCoordsPath(const std::deque<unsigned int>& path, PROPULSION_TYPE propulsion) {
	auto& portals = portalArr[propulsionToIndex.at(propulsion)];

	std::vector<Vector2i> coordsPath;
	std::transform(path.begin(), path.end(), std::back_inserter(coordsPath), [&](unsigned int portalId) {
		// TODO: does it matter, which point it is? Droid will move in the right direction. And portal-level path is only used when no flowfield is available
		auto point = portals[portalId].getFirstSectorCenter();
		return world_coord(Vector2i(point.x, point.y));
	});

	return coordsPath;
}

void debugDrawCostField()
{
	const auto& groundSectors = costFields[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)];
	if (groundSectors.empty()) return;

	const int playerXTile = map_coord(player.p.x);
	const int playerZTile = map_coord(player.p.z);

	const int xDelta = DEBUG_DRAW_X_DELTA;
	const int yDelta = DEBUG_DRAW_Y_DELTA;

	for (int y = -yDelta; y <= yDelta; y++)
	{
		for (int x = -xDelta; x <= xDelta; x++)
		{
			const int actualX = playerXTile + x;
			const int actualY = playerZTile + y;

			Vector2i p = {actualX, actualY};

			if (tileOnMap(actualX, actualY))
			{
				const unsigned int sectorId = Sector::getIdByCoords(p);
				debugTileDrawCost(groundSectors[sectorId].get(), p, {x + xDelta, y + yDelta});
			}
		}
	}
}

void debugTileDrawCost(AbstractSector* sector, Vector2i p, Vector2i screenXY)
{
	WzText costText(std::to_string(sector->getTile(p).cost), font_small);
	// HACK
	// I have completely NO IDEA how to draw stuff correctly. This code is by trial-and-error.
	// It's debug only, but it could be a bit better. Too many magic numers, and works only on initial zoom and rotation.
	const unsigned int renderX = 40 + (screenXY.x << 6);
	const unsigned int renderY = 20 + (screenXY.y << 6);
	costText.render(renderX, renderY, WZCOL_TEXT_BRIGHT);

	const bool topLeftCorner = (p.x % SECTOR_SIZE == 0) && (p.y % SECTOR_SIZE == 0);
	const bool bottomLeftCorner = (p.x % SECTOR_SIZE == 0) && (p.y % SECTOR_SIZE == SECTOR_SIZE - 1);
	const bool topRightCorner = (p.x % SECTOR_SIZE == SECTOR_SIZE - 1) && (p.y % SECTOR_SIZE == 0);
	const bool bottomRightCorner = (p.x % SECTOR_SIZE == SECTOR_SIZE - 1) && (p.y % SECTOR_SIZE == SECTOR_SIZE - 1);

	if (topLeftCorner || bottomLeftCorner || topRightCorner || bottomRightCorner)
	{
		iV_Box(renderX, renderY - 10, renderX + 60, renderY + 50, WZCOL_WHITE);
	}
}

void debugDrawPortals()
{
	const int playerXTile = map_coord(player.p.x);
	const int playerZTile = map_coord(player.p.z);

	const auto convertX = [=](const unsigned int x)
	{
		return 40 + ((x + (DEBUG_DRAW_X_DELTA - playerXTile)) << 6);
	};

	const auto convertY = [=](const unsigned int y)
	{
		return 10 + ((y + (DEBUG_DRAW_Y_DELTA - playerZTile)) << 6);
	};

	auto&& portals = portalArr[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)];

	for (auto&& portal : portals)
	{
		iV_Box(convertX(portal.second.getFirstSectorCenter().x), convertY(portal.second.getFirstSectorCenter().y),
				convertX(portal.second.getSecondSectorCenter().x + 1), convertY(portal.second.getSecondSectorCenter().y + 1), WZCOL_RED);

		// Connection with other portals
		for (unsigned int neighbor : portal.second.neighbors)
		{
			Portal& neighborPortal = portals[neighbor];
			iV_Line(convertX(portal.second.getFirstSectorCenter().x), convertY(portal.second.getFirstSectorCenter().y),
					convertX(neighborPortal.getSecondSectorCenter().x), convertY(neighborPortal.getSecondSectorCenter().y),
					WZCOL_YELLOW);
		}
	}
}

void debugDrawPortalPath() {
	const int playerXTile = map_coord(player.p.x);
	const int playerZTile = map_coord(player.p.z);

	const auto convertX = [=](const unsigned int x) {
		return 40 + ((x + (DEBUG_DRAW_X_DELTA - playerXTile)) << 6);
	};

	const auto convertY = [=](const unsigned int y) {
		return 10 + ((y + (DEBUG_DRAW_Y_DELTA - playerZTile)) << 6);
	};

	auto&& portals = portalArr[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)];

	// It is only debug. If lock happens not to be available, skip drawing
	std::unique_lock<std::mutex> lock(portalPathMutex, std::try_to_lock);
	if (lock) {
		Portal* previousPortal = nullptr;
		for (auto it = _debugPortalPath.begin(); it != _debugPortalPath.end(); it++) {
			auto& currentPortal = portals[*it];

			if (previousPortal != nullptr) {
				iV_Line(convertX(previousPortal->getSecondSectorCenter().x), convertY(previousPortal->getSecondSectorCenter().y),
						convertX(currentPortal.getFirstSectorCenter().x), convertY(currentPortal.getFirstSectorCenter().y),
						WZCOL_GREEN);
			}

			iV_Line(convertX(currentPortal.getFirstSectorCenter().x), convertY(currentPortal.getFirstSectorCenter().y),
					convertX(currentPortal.getSecondSectorCenter().x), convertY(currentPortal.getSecondSectorCenter().y),
					WZCOL_GREEN);

			previousPortal = &currentPortal;
		}
		lock.unlock();
	}
}

void debugDrawFlowfield(const glm::mat4 &mvp) {
	pie_SetRendMode(REND_OPAQUE);

	const auto playerXTile = map_coord(player.p.x);
	const auto playerZTile = map_coord(player.p.z);
	
	const auto& groundSectors = costFields[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)];

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
			}, mvp, WZCOL_TEAM2);

			// sector

			if(x % SECTOR_SIZE == 0){
				iV_PolyLine({
					{ (XA + XB) / 2, height, -ZA },
					{ (XA + XB) / 2, height, -ZB },
				}, mvp, WZCOL_WHITE);
			}

			if(z % SECTOR_SIZE == 0){
				iV_PolyLine({
					{ XA, height, -(ZA + ZB) / 2 },
					{ XB, height, -(ZA + ZB) / 2 },
				}, mvp, WZCOL_WHITE);
			}

			// cost

			const Vector3i a = { (XA + XB) / 2, height, -(ZA + ZB) / 2 };
			Vector2i b;

			pie_RotateProject(&a, mvp, &b);

			const unsigned int sectorId = Sector::getIdByCoords({x, z});
			auto sector = groundSectors[sectorId].get();
			WzText costText(std::to_string(sector->getTile({x, z}).cost), font_medium);
			costText.render(b.x, b.y, WZCOL_TEXT_BRIGHT);
	 	}
	}


	const auto convertX = [=](const unsigned int x) {
		return 60 + ((x + (DEBUG_DRAW_X_DELTA - playerXTile)) << 6);
	};

	const auto convertY = [=](const unsigned int y) {
		return 30 + ((y + (DEBUG_DRAW_Y_DELTA - playerZTile)) << 6);
	};

	// It is only debug. If lock happens not to be available, skip drawing
	std::unique_lock<std::mutex> lock(flowfieldMutex, std::try_to_lock);
	if (lock) {
		auto cache = flowfieldCache[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)].get();

		for (auto const& cacheEntry: *cache) {
			auto key = cacheEntry.first;
			int goalX = key[0].x;
			int goalY = key[0].y;
			bool onScreen = (std::abs(playerXTile - goalX) < SECTOR_SIZE * 2) && (std::abs(playerZTile - goalY) < SECTOR_SIZE * 2);

			if (onScreen) {
				Vector2i tlCorner = AbstractSector::getTopLeftCornerByCoords(key[0]);

				// Draw goals
				for (auto&& goal : key) {
					iV_Box(convertX(goal.x) - 5, convertY(goal.y) - 5,
							convertX(goal.x) + 5, convertY(goal.y) + 5,
							WZCOL_TEAM7);
				}

				// Draw vectors
				auto& sector = cacheEntry.second;
				for (int y = 0; y < SECTOR_SIZE; y++) {
					for (int x = 0; x < SECTOR_SIZE; x++) {
						auto vector = sector->getVector({x, y});
						const int absoluteX = tlCorner.x + x;
						const int absoluteY = tlCorner.y + y;

						// Vector direction
						iV_Line(convertX(absoluteX), convertY(absoluteY),
								convertX(absoluteX) + vector.x * std::pow(2, 4), convertY(absoluteY) + vector.y * std::pow(2, 4),
								WZCOL_TEAM2);

						// Vector start point
						iV_ShadowBox(convertX(absoluteX) - 2, convertY(absoluteY) - 2,
										convertX(absoluteX) + 2, convertY(absoluteY) + 2,
										0, WZCOL_TEAM7, WZCOL_TEAM7, WZCOL_TEAM7);
					}
				}
			}
		}

		lock.unlock();
	}
}
