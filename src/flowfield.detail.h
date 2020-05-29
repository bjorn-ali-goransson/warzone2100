#include <qthreadpool.h>

#include <array>
#include <deque>
#include <queue>

#include "lib/framework/vector.h"

// Private header
namespace flowfield
{
	namespace detail {
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

		/**
		 * "-1" is never expire. Otherwise idle threads are expired after X milliseconds.
		 * It's better to keep the threads available all the time, so there is no startup delay.
		 */
		constexpr const int THREAD_POOL_EXPIRY_TIMEOUT_MS = -1;

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

		/*
		* How many parallel path finding requests can be processed at once.
		* Note that after finding portal-level path, the main task waits for it subtasks to complete, which use default pool.
		* Parallelism greater than numer of logical cores might give spikes of 100% CPU
		* but it might be not noticeable, because most of the time the task sleeps.
		*/
		const unsigned int PORTAL_PATH_THREAD_POOL_MAX = QThread::idealThreadCount();

		// Thread pool used for portal-level A* pathfinding.
		// Those tasks wait for all subtasks to complete, so give them dedicated pool, to prevent deadlock
		QThreadPool portalPathThreadPool;

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
			typedef std::vector<Vector2i> pointsT;

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
			pointsT firstSectorPoints;
			pointsT secondSectorPoints;
			std::vector<unsigned int> neighbors;

			Portal() = default;
			Portal(const Portal&) = delete;
			Portal& operator=(const Portal&) = delete;
			Portal(Portal&&) = default;
			Portal& operator=(Portal&&) = default;

			Portal(AbstractSector* sector1, AbstractSector* sector2, pointsT& firstSectorPoints, pointsT& secondSectorPoints);

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

		template<typename T>
		class FuturedTask : public QRunnable {
		public:
			void run() final {
				try {
					QElapsedTimer timer;
					timer.start();
					
					runPromised();

					if (DEBUG_BUILD) // Mutex is costly and won't be optimized in release mode
					{
						std::lock_guard<std::mutex> lock(logMutex);
						auto took = timer.elapsed();
						debug(LOG_FLOWFIELD, "FuturedTask (%s) took %d ms", typeid(*this).name(), took);
					}
				} catch (const std::exception &ex) {
					promise.set_exception(std::current_exception());
					{
						std::lock_guard<std::mutex> lock(logMutex);
						debug(LOG_ERROR, "Exception in thread pool worker: ", ex.what());
					}

					if (EXCEPTION_IN_THREADPOOL_SHOULD_TERMINATE) {
						std::terminate();
					}
				}
			}

			std::future<T> getFuture() {
				return promise.get_future();
			}
			~FuturedTask() override = default;
		protected:
			virtual void runPromised() = 0;
			void setPromise(T value) {
				promise.set_value(value);
			}
		private:
			std::promise<T> promise;
		};

		// Promise states that path request has been completed
		class PathRequestTask : public FuturedTask<bool> {
		public:
			PathRequestTask(Vector2i mapSource, Vector2i mapGoal, PROPULSION_TYPE propulsion)
				: mapSource(mapSource), mapGoal(mapGoal), propulsion(propulsion) {
			}
			void runPromised() override;
			
			~PathRequestTask() override = default;
		private:
			const Vector2i mapSource;
			const Vector2i mapGoal;
			const PROPULSION_TYPE propulsion;

			unsigned int _debugCacheHits = 0;
			unsigned int _debugCacheMisses = 0;

			std::deque<unsigned int> portalWalker(unsigned int sourcePortalId, unsigned int goalPortalId);
			std::vector<std::future<bool>> scheduleFlowFields(std::deque<unsigned int>& path);
		};

		class FlowFieldSector final {
		public:
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
			typedef std::array<std::array<VectorT, SECTOR_SIZE>, SECTOR_SIZE> vectorArrayT;

			FlowFieldSector() = default;
			FlowFieldSector& operator=(FlowFieldSector&) = delete;
			FlowFieldSector& operator=(FlowFieldSector&&) = delete;
			FlowFieldSector(FlowFieldSector&) = delete;
			FlowFieldSector(FlowFieldSector&&) = default;
			~FlowFieldSector() = default;

			void setVector(Vector2i p, VectorT vector);
			VectorT getVector(Vector2i p) const;

		private:
			vectorArrayT vectors;
		};

		// Promise states whether flow field calculation has completed for given sector
		class FlowfieldCalcTask : public FuturedTask<bool> {
		public:
			// Takes goals by copy. Need to control lifetime of goals myself (goals can be constructed ad-hoc)
			FlowfieldCalcTask(Portal::pointsT goals, portalMapT& portals, const sectorListT& sectors, PROPULSION_TYPE propulsion);
			void runPromised() override;
			~FlowfieldCalcTask() override = default;
		private:
			struct Node {
				unsigned short predecessorCost;
				unsigned int index;

				bool operator<(const Node& other) const {
					// We want top element to have lowest cost
					return predecessorCost > other.predecessorCost;
				}
			};

			Sector integrationField;
			FlowFieldSector flowField;
			
			// Constructor depends on member init order
			const Portal::pointsT goals;
			portalMapT& portals;
			const sectorListT& sectors;
			const unsigned int sectorId;
			const AbstractSector& sector;
			PROPULSION_TYPE propulsion;

			void calculateIntegrationField(const Portal::pointsT& points);
			void integratePoints(std::priority_queue<Node>& openSet);
			void calculateFlowField();
			unsigned short getCostOrElse(Vector2i coords, unsigned short elseCost);
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

		std::deque<unsigned int> getPathFromCache(unsigned int sourcePortalId, unsigned int goalPortalId, PROPULSION_TYPE propulsion);
		Portal::pointsT portalToGoals(const Portal& portal, Vector2i currentPosition);
        Vector2f getMovementVector(unsigned int nextPortalId, Vector2i currentPosition, PROPULSION_TYPE propulsion);

		std::vector<Vector2i> portalPathToCoordsPath(const std::deque<unsigned int>& path, PROPULSION_TYPE propulsion);

		//////////////////////////////////////////////////////////////////////////////////////////
		// +- x axis tile debug draw. Smaller values = less tiles drawn. "7" somewhat fits the default window resolution
		constexpr const unsigned int DEBUG_DRAW_X_DELTA = 7;
		// +- y axis tile debug draw
		constexpr const unsigned int DEBUG_DRAW_Y_DELTA = 6;

		void debugDrawCostField();
		void debugTileDrawCost(AbstractSector& sector, Vector2i p, Vector2i screenXY);
		void debugDrawPortals();
		void debugDrawPortalPath();

		void debugDrawFlowField();
}
}