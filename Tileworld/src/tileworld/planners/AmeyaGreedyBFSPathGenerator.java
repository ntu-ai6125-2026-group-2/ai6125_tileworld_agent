package tileworld.planners;

import java.util.PriorityQueue;
import tileworld.agent.TWAgent;
import tileworld.environment.TWEnvironment;

/**
 * Greedy Best-First Search path generator.
 *
 * This planner prioritizes nodes only by heuristic distance to target
 * (ignoring path cost), unlike A*.
 */
public class AmeyaGreedyBFSPathGenerator implements TWPathGenerator {

    private final TWEnvironment map;
    private final TWAgent agent;
    private final int maxSearchDistance;
    private final Node[][] nodes;

    public AmeyaGreedyBFSPathGenerator(TWEnvironment map, TWAgent agent, int maxSearchDistance) {
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

    @Override
    public TWPath findPath(int sx, int sy, int tx, int ty) {
        if (!map.isValidLocation(tx, ty) || agent.getMemory().isCellBlocked(tx, ty)) {
            return null;
        }

        resetNodes();

        PriorityQueue<Node> open = new PriorityQueue<>();
        Node start = nodes[sx][sy];
        Node goal = nodes[tx][ty];

        start.depth = 0;
        start.heuristic = heuristic(sx, sy, tx, ty);
        open.add(start);

        int exploredDepth = 0;
        Node found = null;

        while (!open.isEmpty() && exploredDepth < maxSearchDistance) {
            Node current = open.poll();
            if (current.closed) {
                continue;
            }
            current.closed = true;

            if (current == goal) {
                found = current;
                break;
            }

            exploredDepth = Math.max(exploredDepth, current.depth);

            // 4-neighbour expansion only.
            expandNeighbour(current, current.x + 1, current.y, tx, ty, open);
            expandNeighbour(current, current.x - 1, current.y, tx, ty, open);
            expandNeighbour(current, current.x, current.y + 1, tx, ty, open);
            expandNeighbour(current, current.x, current.y - 1, tx, ty, open);
        }

        if (found == null) {
            return null;
        }

        TWPath path = new TWPath(tx, ty);
        Node target = found.parent;
        while (target != null && target != start) {
            path.prependStep(target.x, target.y);
            target = target.parent;
        }

        path.prependStep(sx, sy);
        return path;
    }

    private void expandNeighbour(Node parent, int nx, int ny, int tx, int ty, PriorityQueue<Node> open) {
        if (!map.isValidLocation(nx, ny) || agent.getMemory().isCellBlocked(nx, ny)) {
            return;
        }

        Node neighbour = nodes[nx][ny];
        if (neighbour.closed) {
            return;
        }

        // First time discovered wins for GBFS tree growth.
        if (!neighbour.opened) {
            neighbour.parent = parent;
            neighbour.depth = parent.depth + 1;
            neighbour.heuristic = heuristic(nx, ny, tx, ty);
            neighbour.opened = true;
            open.add(neighbour);
        }
    }

    private void resetNodes() {
        for (int x = 0; x < map.getxDimension(); x++) {
            for (int y = 0; y < map.getyDimension(); y++) {
                Node n = nodes[x][y];
                n.parent = null;
                n.heuristic = 0;
                n.depth = 0;
                n.opened = false;
                n.closed = false;
            }
        }
    }

    private double heuristic(int x, int y, int tx, int ty) {
        int dx = tx - x;
        int dy = ty - y;
        return Math.sqrt((dx * dx) + (dy * dy));
    }

    private static class Node implements Comparable<Node> {
        private final int x;
        private final int y;
        private Node parent;
        private double heuristic;
        private int depth;
        private boolean opened;
        private boolean closed;

        private Node(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public int compareTo(Node other) {
            if (heuristic < other.heuristic) {
                return -1;
            }
            if (heuristic > other.heuristic) {
                return 1;
            }
            return 0;
        }
    }
}
