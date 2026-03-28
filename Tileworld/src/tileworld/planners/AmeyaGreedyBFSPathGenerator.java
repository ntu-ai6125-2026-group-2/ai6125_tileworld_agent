package tileworld.planners;

import java.util.ArrayList;
import java.util.Collections;
import tileworld.agent.TWAgent;
import tileworld.environment.TWEnvironment;

/**
 * GreedyBFSPathGenerator
 *
 * An implementation of Greedy Best-First Search for the Tileworld grid.
 * Implements TWPathGenerator as a drop-in replacement for AstarPathGenerator.
 *
 * HOW IT DIFFERS FROM A*:
 *   A* orders nodes by f(n) = g(n) + h(n), balancing actual cost with
 *   estimated remaining cost. Greedy BFS orders nodes purely by h(n) --
 *   the estimated distance to the goal -- and completely ignores g(n),
 *   the cost already paid to reach a node.
 *
 *   This makes Greedy BFS:
 *   - FASTER: expands fewer nodes because it rushes straight toward the goal
 *   - NOT OPTIMAL: may find a longer path if a short-looking route is blocked
 *   - MORE REACTIVE: well suited to dynamic environments where a "good enough"
 *     path found quickly is more valuable than the perfect path found slowly
 *
 * STRUCTURE:
 *   Mirrors AstarPathGenerator closely (same Node class, same SortedList,
 *   same path reconstruction) so the two are directly comparable.
 *   The only algorithmic difference is in compareTo(): nodes are ordered
 *   by heuristic alone, not by heuristic + cost.
 */
public class AmeyaGreedyBFSPathGenerator implements TWPathGenerator {

    /** Nodes already fully explored */
    private ArrayList closed = new ArrayList();

    /** Nodes discovered but not yet explored, sorted by heuristic only */
    private SortedList open = new SortedList();

    /** The environment (map) being searched */
    private TWEnvironment map;

    /** Maximum search depth before giving up */
    private int maxSearchDistance;

    /** Pre-allocated node grid */
    private Node[][] nodes;

    /** No diagonal movement - cardinal only, matching A* config */
    private boolean allowDiagMovement = false;

    /** Agent reference for memory-based obstacle checking */
    private TWAgent agent;

    // -------------------------------------------------------------------------
    // CONSTRUCTOR
    // -------------------------------------------------------------------------

    public AmeyaGreedyBFSPathGenerator(TWEnvironment map, TWAgent agent,
                                   int maxSearchDistance) {
        this.map = map;
        this.agent = agent;
        this.maxSearchDistance = maxSearchDistance;

        nodes = new Node[map.getxDimension()][map.getyDimension()];
        for (int x = 0; x < map.getxDimension(); x++) {
            for (int y = 0; y < map.getyDimension(); y++) {
                nodes[x][y] = new Node(x, y);
            }
        }
    }

    // -------------------------------------------------------------------------
    // TWPathGenerator INTERFACE
    // -------------------------------------------------------------------------

    /**
     * Finds a path from (sx, sy) to (tx, ty) using Greedy Best-First Search.
     * Returns null if the target is blocked or no path exists within
     * maxSearchDistance steps.
     */
    @Override
    public TWPath findPath(int sx, int sy, int tx, int ty) {

        // If target is blocked in memory, no point searching
        if (agent.getMemory().isCellBlocked(tx, ty)) {
            return null;
        }

        // Reset state for this search
        nodes[sx][sy].cost = 0;
        nodes[sx][sy].depth = 0;
        closed.clear();
        open.clear();
        open.add(nodes[sx][sy]);
        nodes[tx][ty].parent = null;

        int maxDepth = 0;

        while ((maxDepth < maxSearchDistance) && (open.size() != 0)) {

            Node current = (Node) open.first();

            if (current == nodes[tx][ty]) {
                break;
            }

            open.remove(current);
            closed.add(current);

            // Expand neighbours
            for (int x = -1; x < 2; x++) {
                for (int y = -1; y < 2; y++) {

                    // Skip self
                    if (x == 0 && y == 0) continue;

                    // Cardinal moves only (no diagonal)
                    if (!allowDiagMovement && x != 0 && y != 0) continue;

                    int xp = current.x + x;
                    int yp = current.y + y;

                    if (isValidLocation(sx, sy, xp, yp)
                            && !agent.getMemory().isCellBlocked(xp, yp)) {

                        // g cost - still tracked for path reconstruction
                        // but NOT used for queue ordering (that's the key
                        // difference from A*)
                        double nextCost = current.cost
                                + getMovementCost(current.x, current.y, xp, yp);
                        Node neighbour = nodes[xp][yp];
                        neighbour.setVisited(true);

                        if (nextCost < neighbour.cost) {
                            if (inOpenList(neighbour))   removeFromOpen(neighbour);
                            if (inClosedList(neighbour)) removeFromClosed(neighbour);
                        }

                        if (!inOpenList(neighbour) && !inClosedList(neighbour)) {
                            neighbour.cost = nextCost;
                            // GREEDY BFS: heuristic only, no cost component
                            neighbour.heuristic = getHeuristicCost(xp, yp, tx, ty);
                            maxDepth = Math.max(maxDepth,
                                    neighbour.setParent(current));
                            addToOpen(neighbour);
                        }
                    }
                }
            }
        }

        // No path found
        if (nodes[tx][ty].parent == null) {
            return null;
        }

        // Reconstruct path from goal back to start via parent pointers
        TWPath path = new TWPath(tx, ty);
        Node target = nodes[tx][ty];
        target = target.parent; // skip goal node itself
        while (target != nodes[sx][sy]) {
            path.prependStep(target.x, target.y);
            target = target.parent;
        }
        path.prependStep(sx, sy);

        return path;
    }

    // -------------------------------------------------------------------------
    // COST FUNCTIONS
    // -------------------------------------------------------------------------

    /**
     * Movement cost between two adjacent cells.
     * Same as A* - Manhattan distance (always 1.0 for cardinal moves).
     */
    public double getMovementCost(int sx, int sy, int tx, int ty) {
        return map.getDistance(sx, sy, tx, ty);
    }

    /**
     * Heuristic: Euclidean distance to goal.
     * This is the ONLY value used for ordering in Greedy BFS.
     * Using Euclidean (same as A* reference) keeps the comparison fair.
     */
    public double getHeuristicCost(int x, int y, int tx, int ty) {
        int dx = tx - x;
        int dy = ty - y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // -------------------------------------------------------------------------
    // OPEN / CLOSED LIST HELPERS  (identical to AstarPathGenerator)
    // -------------------------------------------------------------------------

    protected boolean isValidLocation(int sx, int sy, int x, int y) {
        return map.isValidLocation(x, y) && ((sx != x) || (sy != y));
    }

    protected Node getFirstInOpen()          { return (Node) open.first(); }
    protected void addToOpen(Node node)      { open.add(node); }
    protected boolean inOpenList(Node node)  { return open.contains(node); }
    protected void removeFromOpen(Node node) { open.remove(node); }
    protected void addToClosed(Node node)    { closed.add(node); }
    protected boolean inClosedList(Node node){ return closed.contains(node); }
    protected void removeFromClosed(Node n)  { closed.remove(n); }

    // -------------------------------------------------------------------------
    // SORTED LIST  (identical to AstarPathGenerator)
    // -------------------------------------------------------------------------

    private class SortedList {
        private ArrayList list = new ArrayList();
        public Object first()            { return list.get(0); }
        public void clear()              { list.clear(); }
        public void add(Object o)        { list.add(o); Collections.sort(list); }
        public void remove(Object o)     { list.remove(o); }
        public int size()                { return list.size(); }
        public boolean contains(Object o){ return list.contains(o); }
    }

    // -------------------------------------------------------------------------
    // NODE CLASS
    // -------------------------------------------------------------------------

    /**
     * Represents a grid cell in the search.
     *
     * KEY DIFFERENCE FROM A*:
     * compareTo() orders by heuristic ONLY.
     * In AstarPathGenerator, compareTo() uses (heuristic + cost).
     * This single change is what makes this Greedy BFS instead of A*.
     */
    protected class Node implements Comparable {

        private int x, y;
        private double cost;       // g(n) - cost from start (tracked but not used for ordering)
        private Node parent;
        private double heuristic;  // h(n) - estimated cost to goal (ONLY ordering criterion)
        private int depth;
        private boolean visited;

        public Node(int x, int y) {
            this.x = x;
            this.y = y;
        }

        public int setParent(Node parent) {
            depth = parent.depth + 1;
            this.parent = parent;
            return depth;
        }

        /**
         * ORDER BY HEURISTIC ONLY.
         * This is the single algorithmic difference from A*.
         * A* uses: f = heuristic + cost
         * Greedy BFS uses: f = heuristic
         */
        @Override
        public int compareTo(Object other) {
            Node o = (Node) other;
            // Greedy BFS: compare heuristic only (ignore cost)
            if (heuristic < o.heuristic) return -1;
            if (heuristic > o.heuristic) return  1;
            return 0;
        }

        public boolean pathFinderVisited() { return visited; }
        public void setVisited(boolean b)  { visited = b; }
    }
}