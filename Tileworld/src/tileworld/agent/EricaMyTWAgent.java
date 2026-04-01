package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;
import sim.util.Int2D;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.exceptions.CellBlockedException;
import java.util.*;

public class MyTWAgent extends TWAgentSkeleton {

    private static final double FUEL_THRESHOLD_RATIO = 0.28;
    private static final int CARRY_CAPACITY = 3;
    private static final int MEMORY_LIFESPAN = 100; 

    private final AstarPathGenerator astar;
    private final Map<Int2D, Long> lastSeenTime = new HashMap<>();
    
    private final int[][] corners;
    private int cornerIdx = 0;
    private final boolean isLargeMap;

    public MyTWAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(name, x, y, env, fuel); // Inherits agentName, x, y, env, fuel
        this.astar = new AstarPathGenerator(env, this, 2000); 
        
        // Dynamic Check: Is this a big map?
        this.isLargeMap = (env.getxDimension() >= 60 || env.getyDimension() >= 60);
        
        int maxX = env.getxDimension() - 1;
        int maxY = env.getyDimension() - 1;
        this.corners = new int[][]{{0, 0}, {maxX, 0}, {maxX, maxY}, {0, maxY}};
    }

    @Override
    protected void customCommunicate() {
        int r = Parameters.defaultSensorRange;
        long now = getEnvironment().schedule.getSteps();

        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (!getEnvironment().isValidLocation(cx, cy)) continue;
                
                Object o = getEnvironment().getObjectGrid().get(cx, cy);
                if (o instanceof TWFuelStation) {
                    this.fuelStationX = cx;
                    this.fuelStationY = cy;
                    broadcast(ArdaMessage.info(agentName, "fuel", cx, cy, getX(), getY()));
                } else if (o instanceof TWTile || o instanceof TWHole) {
                    lastSeenTime.put(new Int2D(cx, cy), now);
                    String type = (o instanceof TWTile) ? "tile" : "hole";
                    broadcast(ArdaMessage.info(agentName, type, cx, cy, getX(), getY()));
                }
            }
        }
    }

    @Override
    protected void handleTeamMessage(ArdaMessage am) {
        if (am.getEntityType().startsWith("delete")) {
            getMemory().getMemoryGrid().set(am.getX(), am.getY(), null);
            lastSeenTime.remove(new Int2D(am.getX(), am.getY()));
        } else if (am.getEntityType().equals("fuel")) {
            this.fuelStationX = am.getX(); 
            this.fuelStationY = am.getY();
        }
    }

    @Override
    protected TWThought customThink() {
        updateLocalTimestamps();
        
        double fuelLevel = getFuelLevel();
        double fuelRatio = fuelLevel / Parameters.defaultFuelLevel;

        // 1. REFUELING (Priority)
        if (fuelStationX != -1) {
            double dist = manhattan(getX(), getY(), fuelStationX, fuelStationY);
            double safetyPadding = isLargeMap ? 15 : 5;
            if (fuelLevel < (dist * 2.0 + safetyPadding) || fuelRatio < FUEL_THRESHOLD_RATIO) {
                if (getX() == fuelStationX && getY() == fuelStationY) 
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                return navigate(fuelStationX, fuelStationY);
            }
        }

        // 2. IMMEDIATE ACTIONS 
        Object here = getEnvironment().getObjectGrid().get(getX(), getY());
        if (here instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) 
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        if (here instanceof TWHole && !carriedTiles.isEmpty()) 
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);

        // 3. TARGETING 
        Int2D target = findBestTarget();
        if (target != null) return navigate(target.x, target.y);

        // 4. PATROL
        return patrol();
    }

    private Int2D findBestTarget() {
        Int2D best = null;
        double minScore = Double.MAX_VALUE;
        long now = getEnvironment().schedule.getSteps();
        
        int searchRange = isLargeMap ? 30 : 100; 

        for (int x = 0; x < getEnvironment().getxDimension(); x++) {
            for (int y = 0; y < getEnvironment().getyDimension(); y++) {
                double dist = manhattan(getX(), getY(), x, y);
                if (isLargeMap && dist > searchRange) continue;

                Object o = getMemory().getMemoryGrid().get(x, y);
                if (o == null) continue;

                Long seenAt = lastSeenTime.get(new Int2D(x, y));
                if (seenAt != null && (now - seenAt) > MEMORY_LIFESPAN) continue;

                boolean isTarget = (o instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) ||
                                   (o instanceof TWHole && !carriedTiles.isEmpty());
                if (isTarget) {
                    double score = (o instanceof TWHole) ? dist - 5 : dist; 
                    if (score < minScore) {
                        minScore = score;
                        best = new Int2D(x, y);
                    }
                }
            }
        }
        return best;
    }

    private void updateLocalTimestamps() {
        int r = Parameters.defaultSensorRange;
        long now = getEnvironment().schedule.getSteps();
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (getEnvironment().isValidLocation(cx, cy)) {
                    lastSeenTime.put(new Int2D(cx, cy), now);
                    if (getEnvironment().getObjectGrid().get(cx, cy) == null) {
                        getMemory().getMemoryGrid().set(cx, cy, null);
                    }
                }
            }
        }
    }

    @Override
    protected void act(TWThought thought) {
        try {
            int ax = getX(), ay = getY();
            switch (thought.getAction()) {
                case MOVE: move(thought.getDirection()); break;
                case REFUEL: refuel(); break;
                case PICKUP:
                    pickUpTile((TWTile) getEnvironment().getObjectGrid().get(ax, ay));
                    getMemory().getMemoryGrid().set(ax, ay, null);
                    broadcast(ArdaMessage.info(agentName, "delete_tile", ax, ay, ax, ay));
                    break;
                case PUTDOWN:
                    putTileInHole((TWHole) getEnvironment().getObjectGrid().get(ax, ay));
                    getMemory().getMemoryGrid().set(ax, ay, null);
                    broadcast(ArdaMessage.info(agentName, "delete_hole", ax, ay, ax, ay));
                    break;
            }
        } catch (CellBlockedException e) {
            // Re-evaluates next tick
        }
    }

    private void broadcast(ArdaMessage msg) {
        getEnvironment().receiveMessage(msg);
    }

    private TWThought navigate(int tx, int ty) {
        TWPath path = astar.findPath(getX(), getY(), tx, ty);
        if (path != null && path.hasNext()) return new TWThought(TWAction.MOVE, path.popNext().getDirection());
        
        int dx = Integer.compare(tx, getX()), dy = Integer.compare(ty, getY());
        TWDirection d = (dx != 0) ? (dx > 0 ? TWDirection.E : TWDirection.W) : (dy > 0 ? TWDirection.S : TWDirection.N);
        return new TWThought(TWAction.MOVE, d);
    }

    private TWThought patrol() {
        int[] corner = corners[cornerIdx];
        if (getX() == corner[0] && getY() == corner[1]) cornerIdx = (cornerIdx + 1) % corners.length;
        return navigate(corners[cornerIdx][0], corners[cornerIdx][1]);
    }

    public double manhattan(int x1, int y1, int x2, int y2) { 
        return Math.abs(x1 - x2) + Math.abs(y1 - y2); 
    }

    @Override
    public String getName() {
        return this.agentName;
    }
}
