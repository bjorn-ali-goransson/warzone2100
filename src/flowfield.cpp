#include <qelapsedtimer.h>
#include <qrunnable.h>
#include <qcache.h>

// Must be before some stuff from headers from flowfield.h, otherwise "std has no member 'mutex'"
// That simply means someone else have messed up
#include <mutex>

#include "flowfield.h"
#include "flowfield.detail.h"

#include <future>
#include <map>
#include <set>
#include <typeinfo>
#include <vector>

#include "lib/framework/debug.h"
#include "lib/ivis_opengl/pieblitfunc.h"
#include "lib/ivis_opengl/piepalette.h"
#include "lib/ivis_opengl/textdraw.h"

#include "display3d.h"
#include "map.h"

// Public API impl
namespace flowfield {
	static bool flowfieldEnabled = false;

	void enable() {
		flowfieldEnabled = true;
	}

	bool isEnabled() {
		return flowfieldEnabled;
	}

	void init() {
		if (!isEnabled()) return;

		detail::initCostFields();
		detail::setupPortals();

		QThreadPool::globalInstance()->setExpiryTimeout(detail::THREAD_POOL_EXPIRY_TIMEOUT_MS);
		detail::portalPathThreadPool.setExpiryTimeout(detail::THREAD_POOL_EXPIRY_TIMEOUT_MS);

		/*
		* By default, QThreadPool sets maxThreadCount to number of logical cores, or 1 if cannot detect it.
		* One thread is busy with all the game logic. That would allow to effectively use one less thread.
		*/
		if (DEBUG_THREAD_POOL) {
			QThreadPool::globalInstance()->setMaxThreadCount(1);
			detail::portalPathThreadPool.setMaxThreadCount(1);
		} else {			
			unsigned int maxThreads = std::max(1, QThread::idealThreadCount() - 1);
			QThreadPool::globalInstance()->setMaxThreadCount(maxThreads);

			detail::portalPathThreadPool.setMaxThreadCount(detail::PORTAL_PATH_THREAD_POOL_MAX);
		}
	}

	void destroy() {
		if (!isEnabled()) return;

		detail::destroyCostFields();
		detail::destroyPortals();
		detail::destroyFlowfieldCache();
	}

	void calculateFlowFieldsAsync(MOVE_CONTROL * psMove, unsigned id, int startX, int startY, int tX, int tY, PROPULSION_TYPE propulsionType,
								  DROID_TYPE droidType, FPATH_MOVETYPE moveType, int owner, bool acceptNearest, StructureBounds const & dstStructure) {
        Vector2i source { map_coord(startX), map_coord(startY) };
        Vector2i goal { map_coord(tX), map_coord(tY) };

		auto task = std::make_unique<detail::PathRequestTask>(source, goal, propulsionType);
		std::future<bool> pathRequestFuture = task->getFuture();
		detail::portalPathThreadPool.start(task.release());
	}

	std::deque<unsigned int> getPathFromCache(unsigned startX, unsigned startY, unsigned tX, unsigned tY, const PROPULSION_TYPE propulsion) {
        Vector2i source { map_coord(startX), map_coord(startY) };
        Vector2i goal { map_coord(tX), map_coord(tY) };
		
		unsigned int sourcePortalId, goalPortalId;
		std::tie(sourcePortalId, goalPortalId) = detail::mapSourceGoalToPortals(source, goal, propulsion);

		return detail::getPathFromCache(sourcePortalId, goalPortalId, propulsion);
	}

    Vector2f getMovementVector(unsigned int nextPortalId, unsigned currentX, unsigned currentY, PROPULSION_TYPE propulsion) {
		return detail::getMovementVector(nextPortalId, { currentX, currentY }, propulsion);
	}

	std::vector<Vector2i> portalPathToCoordsPath(const std::deque<unsigned int>& path, DROID* psDroid) {
		return detail::portalPathToCoordsPath(path, getPropulsionStats(psDroid)->propulsionType);
	}

	void debugDraw() {
		if (!isEnabled()) return;

		if (COST_FIELD_DEBUG) {
			detail::debugDrawCostField();
		}

		if (PORTALS_DEBUG) {
			detail::debugDrawPortals();
		}

		if (PORTAL_PATH_DEBUG) {
			detail::debugDrawPortalPath();
		}

		if (VECTOR_FIELD_DEBUG) {
			detail::debugDrawFlowField();
		}
	}
}

// Private impl
namespace flowfield
{
	namespace detail
	{
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
		typedef QCache<std::pair<unsigned int, unsigned int>, std::deque<unsigned int>> portalPathCacheT;
		typedef QCache<Portal::pointsT, FlowFieldSector> flowfieldCacheT;

		// Workaround because QCache is neither copyable nor movable
		std::array<std::unique_ptr<portalPathCacheT>, 4> portalPathCache {
			std::make_unique<portalPathCacheT>(PORTAL_PATH_CACHE_MAX),
			std::make_unique<portalPathCacheT>(PORTAL_PATH_CACHE_MAX),
			std::make_unique<portalPathCacheT>(PORTAL_PATH_CACHE_MAX),
			std::make_unique<portalPathCacheT>(PORTAL_PATH_CACHE_MAX)
		};

		std::array<std::unique_ptr<flowfieldCacheT>, 4> flowfieldCache {
			std::make_unique<flowfieldCacheT>(FLOWFIELD_CACHE_MAX),
			std::make_unique<flowfieldCacheT>(FLOWFIELD_CACHE_MAX),
			std::make_unique<flowfieldCacheT>(FLOWFIELD_CACHE_MAX),
			std::make_unique<flowfieldCacheT>(FLOWFIELD_CACHE_MAX)
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

		Portal::Portal(AbstractSector* sector1, AbstractSector* sector2, pointsT& firstSectorPoints, pointsT& secondSectorPoints)
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

		void PathRequestTask::runPromised() {
			if (DEBUG_BUILD) // Mutex is expensive and won't be optimized in release mode
			{
				std::lock_guard<std::mutex> lock(logMutex);
				debug(LOG_FLOWFIELD, "Path request calculation for start x: %d, y: %d, goal x: %d, y: %d, propulsion: %d\n",
					  mapSource.x, mapSource.y, mapGoal.x, mapGoal.y, propulsion);
			}

			unsigned int sourcePortalId, goalPortalId;
			std::tie(sourcePortalId, goalPortalId) = mapSourceGoalToPortals(mapSource, mapGoal, propulsion);

			std::deque<unsigned int> path = portalWalker(sourcePortalId, goalPortalId);

			if (DEBUG_BUILD) // Mutex is expensive and won't be optimized in release mode
			{
				std::lock_guard<std::mutex> lock(logMutex);
				_debugPortalPath = path;
			}

			auto flowFieldFutures = scheduleFlowFields(path);

			// Wait for all flow field task to complete
			for (auto&& future : flowFieldFutures) {
				future.wait();
			}

			setPromise(true);
		}

		std::deque<unsigned int> PathRequestTask::portalWalker(unsigned int sourcePortalId, unsigned int goalPortalId) {
			std::unique_lock<std::mutex> lock(portalPathMutex);
			auto& localPortalPathCache = *portalPathCache[propulsionToIndex.at(propulsion)];
			std::deque<unsigned int>* pathPtr = localPortalPathCache[{sourcePortalId, goalPortalId}];
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
				auto pathCopy = std::make_unique<std::deque<unsigned int>>(path);
				localPortalPathCache.insert({ sourcePortalId, goalPortalId }, pathCopy.release());
				return path;
			}
		}

		std::vector<std::future<bool>> PathRequestTask::scheduleFlowFields(std::deque<unsigned int>& path) {
			auto& portals = portalArr[propulsionToIndex.at(propulsion)];
			auto& sectors = costFields[propulsionToIndex.at(propulsion)];
			auto& localFlowFieldCache = *flowfieldCache[propulsionToIndex.at(propulsion)];
			std::vector<std::future<bool>> flowFieldFutures;

            Vector2i localStartPoint = mapSource;

			// Lock the whole loop, but hopefully this will help with lock contention that would otherwise occur
			std::lock_guard<std::mutex> lock(flowfieldMutex);
			
			for (unsigned int leavePortalId : path) {
				Portal& leavePortal = portals[leavePortalId];
				Portal::pointsT goals = portalToGoals(leavePortal, localStartPoint);
				localStartPoint = goals[0];

				if (!localFlowFieldCache.contains(goals)) {
					_debugCacheMisses++;
					
					auto task = std::make_unique<FlowfieldCalcTask>(goals, portals, sectors, propulsion);
					flowFieldFutures.push_back(task->getFuture());
					QThreadPool::globalInstance()->start(task.release());
				} else {
					_debugCacheHits++;
				}
			}

			// Final goal task
			// TODO: in future with better integration with Warzone, there might be multiple goals for a formation, so droids don't bump into each other
			Portal::pointsT finalGoals { mapGoal };

			if (!localFlowFieldCache.contains(finalGoals)) {
				_debugCacheMisses++;

				auto task = std::make_unique<FlowfieldCalcTask>(finalGoals, portals, sectors, propulsion);
				flowFieldFutures.push_back(task->getFuture());
				QThreadPool::globalInstance()->start(task.release());
			} else {
				_debugCacheHits++;
			}

			if (DEBUG_BUILD) // Mutex is expensive and won't be optimized in release mode
			{
				std::lock_guard<std::mutex> lock(logMutex);
				debug(LOG_FLOWFIELD, "Flowfield sector cache hits: %d, misses: %d\n", _debugCacheHits, _debugCacheMisses);
			}

			return flowFieldFutures;
		}

		void FlowFieldSector::setVector(Vector2i p, VectorT vector) {
			vectors[p.x][p.y] = vector;
		}

		FlowFieldSector::VectorT FlowFieldSector::getVector(Vector2i p) const {
			return vectors[p.x][p.y];
		}

		FlowfieldCalcTask::FlowfieldCalcTask(Portal::pointsT goals, portalMapT& portals, const sectorListT& sectors, PROPULSION_TYPE propulsion)
			: goals(goals), portals(portals), sectors(sectors), sectorId(AbstractSector::getIdByCoords(*goals.begin())),
			sector(*sectors[sectorId]), propulsion(propulsion), flowField() {
		}

		void FlowfieldCalcTask::runPromised() {
			// NOTE: Vector field for given might have been calculated by the time this task have chance to run.
			// I don't care, since this task has proven to be short, and I want to avoid lock contention when checking cache

			calculateIntegrationField(goals);

			calculateFlowField();

			{
				std::lock_guard<std::mutex> lock(flowfieldMutex);
				auto flowfieldMoved = std::make_unique<FlowFieldSector>(std::move(flowField));
				flowfieldCache[propulsionToIndex.at(propulsion)]->insert(goals, flowfieldMoved.release());
			}

			setPromise(true);
		}

		void FlowfieldCalcTask::calculateIntegrationField(const Portal::pointsT& points) {
			// TODO: here do checking if given tile contains a building (instead of doing that in cost field)
			// TODO: split NOT_PASSABLE into a few constants, for terrain, buildings and maybe sth else
			for (unsigned int x = 0; x < SECTOR_SIZE; x++) {
				for (unsigned int y = 0; y < SECTOR_SIZE; y++) {
					integrationField.setTile({x, y}, Tile { NOT_PASSABLE });
				}
			}

			// Thanks to priority queue, we have "water pouring effect".
			// First we go where cost is the lowest, so we don't discover better path later.
			std::priority_queue<Node> openSet;

			for (auto& point : points) {
				openSet.push({ 0, pointToIndex(point) });
			}

			while (!openSet.empty()) {
				integratePoints(openSet);
				openSet.pop();
			}
		}

		void FlowfieldCalcTask::integratePoints(std::priority_queue<Node>& openSet) {
			const Node& node = openSet.top();
            Vector2i nodePoint = getPointByFlatIndex(node.index);
			Tile nodeTile = sector.getTile(nodePoint);

			if (nodeTile.isBlocking()) {
				return;
			}

			unsigned short nodeCostFromCostField = nodeTile.cost;

			// Go to the goal, no matter what
			if (node.predecessorCost == 0) {
				nodeCostFromCostField = COST_MIN;
			}

			const unsigned short newCost = node.predecessorCost + nodeCostFromCostField;
			const unsigned short nodeOldCost = integrationField.getTile(nodePoint).cost;

			if (newCost < nodeOldCost) {
				integrationField.setTile(nodePoint, Tile { newCost });

				for (unsigned int neighbor : AbstractSector::getNeighbors(sectors, nodePoint)) {
					openSet.push({ newCost, neighbor });
				}
			}
		}

		void FlowfieldCalcTask::calculateFlowField() {
			for (int y = 0; y < SECTOR_SIZE; y++) {
				for (int x = 0; x < SECTOR_SIZE; x++) {
				    Vector2i p = {x, y};
					Tile tile = integrationField.getTile(p);
					if (tile.isBlocking() || tile.cost == COST_MIN) {
						// Skip goal and non-passable
						// TODO: probably 0.0 should be only for actual goals, not intermediate goals when crossing sectors
						flowField.setVector(p, FlowFieldSector::VectorT { 0.0f, 0.0f });
						continue;
					}

					// Use current tile cost when no cost available.
					// This will either keep the vector horizontal or vertical, or turn away from higher-cost neighbor
					// NOTICE: Flow field on sector borders might be not optimal
					const unsigned short leftCost = getCostOrElse({x - 1, y}, tile.cost);
					const unsigned short rightCost = getCostOrElse({x + 1, y}, tile.cost);

					const unsigned short topCost = getCostOrElse({x, y - 1}, tile.cost);
					const unsigned short bottomCost = getCostOrElse({x, y + 1}, tile.cost);

					FlowFieldSector::VectorT vector;
					vector.x = leftCost - rightCost;
					vector.y = topCost - bottomCost;
					vector.normalize();

					if (std::abs(vector.x) < 0.01f && std::abs(vector.y) < 0.01f) {
						// Local optima. Tilt the vector in any direction.
						vector.x = 0.1f;
						vector.y = 0.1f;
					}

					flowField.setVector(p, vector);
				}
			}
		}

		unsigned short FlowfieldCalcTask::getCostOrElse(Vector2i coords, unsigned short elseCost) {
			if (coords.x < 0 || coords.y < 0 || coords.x >= SECTOR_SIZE || coords.y >= SECTOR_SIZE) {
				// if near sector border, assume its safe to go to nearby sector
				return std::max<short>(static_cast<short>(elseCost), static_cast<short>(elseCost) - static_cast<short>(1));
			}

			const Tile& tile = integrationField.getTile(coords);
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
					sectors.push_back(std::make_unique<Sector>());
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
					sector = std::make_unique<EmptySector>();
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

			return Tile{cost};
		}

		Portal detectPortalByAxis(unsigned int axisStart, unsigned int axisEnd, unsigned otherAxis1, unsigned otherAxis2,
		                          bool isXAxis, AbstractSector& thisSector, AbstractSector& otherSector, unsigned int& axisEndOut)
		{
			axisEndOut = 0;
			Portal::pointsT firstSectorPoints;
			Portal::pointsT secondSectorPoints;
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
				return Portal(&thisSector, &otherSector, firstSectorPoints, secondSectorPoints);
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

		std::deque<unsigned int> getPathFromCache(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion) {
			std::lock_guard<std::mutex> lock(portalPathMutex);
			auto& localPortalPathCache = *portalPathCache[propulsionToIndex.at(propulsion)];
			std::deque<unsigned int>* pathPtr = localPortalPathCache[{sourcePortalId, goalPortalId}];
			if (pathPtr) {
				return *pathPtr;
			}

			return std::deque<unsigned int> {};
		}

		Portal::pointsT portalToGoals(const Portal& portal, Vector2i currentPosition) {
			const bool forward = isForward(currentPosition, portal.getFirstSectorCenter(), portal.getSecondSectorCenter());

			if (forward) {
				return portal.firstSectorPoints;
			} else {
				return portal.secondSectorPoints;
			}
		}

        Vector2f getMovementVector(unsigned int nextPortalId, Vector2i currentPosition, PROPULSION_TYPE propulsion) {
			auto&& portals = detail::portalArr[detail::propulsionToIndex.at(propulsion)];
			const detail::Portal& nextPortal = portals.at(nextPortalId);
			Portal::pointsT goals = portalToGoals(nextPortal, currentPosition);

			std::lock_guard<std::mutex> lock(flowfieldMutex);
			flowfieldCacheT& localFlowfieldCache = *flowfieldCache[propulsionToIndex.at(propulsion)];
			FlowFieldSector* sector = localFlowfieldCache[goals];

			if (sector) {
				FlowFieldSector::VectorT vector = sector->getVector(currentPosition);
				return { vector.x, vector.y };
			} else {
				// (0,0) vector considered invalid
				return { 0.0f, 0.0f };
			}
		}

		std::vector<Vector2i> portalPathToCoordsPath(const std::deque<unsigned int>& path, PROPULSION_TYPE propulsion) {
			auto& portals = portalArr[detail::propulsionToIndex.at(propulsion)];

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
						debugTileDrawCost(*groundSectors[sectorId], p, {x + xDelta, y + yDelta});
					}
				}
			}
		}

		void debugTileDrawCost(AbstractSector& sector, Vector2i p, Vector2i screenXY)
		{
			WzText costText(std::to_string(sector.getTile(p).cost), font_small);
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

		void debugDrawFlowField() {
			const int playerXTile = map_coord(player.p.x);
			const int playerZTile = map_coord(player.p.z);

			const auto convertX = [=](const unsigned int x) {
				return 60 + ((x + (DEBUG_DRAW_X_DELTA - playerXTile)) << 6);
			};

			const auto convertY = [=](const unsigned int y) {
				return 30 + ((y + (DEBUG_DRAW_Y_DELTA - playerZTile)) << 6);
			};

			// It is only debug. If lock happens not to be available, skip drawing
			std::unique_lock<std::mutex> lock(flowfieldMutex, std::try_to_lock);
			if (lock) {
				auto& cache = flowfieldCache[propulsionToIndex.at(PROPULSION_TYPE_WHEELED)];
				auto keys = cache->keys();

				for (auto&& key : keys) {
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
						auto& sector = *cache->object(key);
						for (int y = 0; y < SECTOR_SIZE; y++) {
							for (int x = 0; x < SECTOR_SIZE; x++) {
								auto vector = sector.getVector({x, y});
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
}
}

// Function needed for sector-level flowfield QCache
uint qHash(const flowfield::detail::Portal::pointsT& key) {
    uint hash = 0;

    for (auto& point : key) {
        hash ^= ::qHash(point.x) ^ ::qHash(point.y);
    }

    return hash;
}
