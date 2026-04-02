package tileworld.agent;

import java.util.List;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AmeyaGreedyBFSPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;


public class AmeyaGreedyBFSAgentWithMemoryMessageSkeleton extends TWAgentSkeleton {

    private static final double FUEL_UNKNOWN_STATION_RATIO = 0.30;
    private static final double FUEL_SAFETY_BUFFER         = Parameters.defaultFuelLevel * 0.20;
    private static final double CARRY_BIAS_DISTANCE        = 5.0;

    private final AmeyaGreedyBFSPathGenerator pathGenerator;
    private final AmeyaCustomTWAgentMemory customMemory;

    private int[] pendingDeleteTile = null;
    private int[] pendingDeleteHole = null;


    public AmeyaGreedyBFSAgentWithMemoryMessageSkeleton(String name, int xpos, int ypos,
                                             TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);

        this.customMemory = new AmeyaCustomTWAgentMemory(
                this, env.schedule,
                env.getxDimension(), env.getyDimension());
        this.memory = customMemory;

        this.pathGenerator = new AmeyaGreedyBFSPathGenerator(
                env, this, env.getxDimension() * env.getyDimension());
    }


    @Override
    protected void customCommunicate() {

        if (pendingDeleteTile != null) {
            broadcastInfo(ENTITY_DELETE_TILE,
                    pendingDeleteTile[0], pendingDeleteTile[1]);
            customMemory.removeTrackedTile(pendingDeleteTile[0], pendingDeleteTile[1]);
            customMemory.removeAgentPercept(pendingDeleteTile[0], pendingDeleteTile[1]);
            pendingDeleteTile = null;
        }

        if (pendingDeleteHole != null) {
            broadcastInfo(ENTITY_DELETE_HOLE,
                    pendingDeleteHole[0], pendingDeleteHole[1]);
            customMemory.removeTrackedHole(pendingDeleteHole[0], pendingDeleteHole[1]);
            customMemory.removeAgentPercept(pendingDeleteHole[0], pendingDeleteHole[1]);
            pendingDeleteHole = null;
        }
    }


    @Override
    protected void handleTeamMessage(ArdaMessage msg) {
        if (msg.getType() != ArdaMessage.MessageType.INFO) return;

        String entity = msg.getEntityType();
        int mx = msg.getX();
        int my = msg.getY();

        if (ENTITY_DELETE_TILE.equals(entity)) {
            customMemory.removeTrackedTile(mx, my);
            customMemory.removeAgentPercept(mx, my);

        } else if (ENTITY_DELETE_HOLE.equals(entity)) {
            customMemory.removeTrackedHole(mx, my);
            customMemory.removeAgentPercept(mx, my);
        }
    }


    @Override
    protected TWThought customThink() {

        int ax = getX();
        int ay = getY();

        boolean needsFuel;
        if (fuelStationX != -1) {
            double distToStation = manhattan(ax, ay, fuelStationX, fuelStationY);
            needsFuel = fuelLevel <= (distToStation + FUEL_SAFETY_BUFFER);
        } else {
            needsFuel = fuelLevel <= (Parameters.defaultFuelLevel * FUEL_UNKNOWN_STATION_RATIO);
        }

        if (needsFuel) {
            if (fuelStationX != -1) {
                if (ax == fuelStationX && ay == fuelStationY) {
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                }
                return new TWThought(TWAction.MOVE,
                        getPathDirection(fuelStationX, fuelStationY));
            }
            return new TWThought(TWAction.MOVE,
                    getPathDirection(
                            getEnvironment().getxDimension() / 2,
                            getEnvironment().getyDimension() / 2));
        }
        TWEntity currentCell = (TWEntity) getEnvironment()
                .getObjectGrid().get(ax, ay);

        if (currentCell instanceof TWFuelStation && fuelStationX == -1) {
            fuelStationX = ax;
            fuelStationY = ay;
        }

        if (currentCell instanceof TWHole && hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        if (currentCell instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        AmeyaCustomTWAgentMemory.MemoryEntry target = selectTarget();
        if (target != null) {
            setIntention(
                    carriedTiles.size() > 0 ? ENTITY_HOLE : ENTITY_TILE,
                    target.x, target.y);
            return new TWThought(TWAction.MOVE,
                    getPathDirection(target.x, target.y));
        }

        clearIntention();
        return new TWThought(TWAction.MOVE, getExploreDirection());
    }


    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {

                case MOVE:
                    move(thought.getDirection());
                    break;

                case PICKUP:
                    TWTile tile = (TWTile) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (tile != null) {
                        int tx = tile.getX();
                        int ty = tile.getY();
                        pickUpTile(tile);
                        pendingDeleteTile = new int[]{tx, ty};
                    }
                    break;

                case PUTDOWN:
                    TWHole hole = (TWHole) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (hole != null) {
                        int hx = hole.getX();
                        int hy = hole.getY();
                        putTileInHole(hole);
                        pendingDeleteHole = new int[]{hx, hy};
                    }
                    break;

                case REFUEL:
                    refuel();
                    break;
            }
        } catch (CellBlockedException e) {
        }
    }

    private AmeyaCustomTWAgentMemory.MemoryEntry selectTarget() {
        List<AmeyaCustomTWAgentMemory.MemoryEntry> tiles = customMemory.getKnownTiles();
        List<AmeyaCustomTWAgentMemory.MemoryEntry> holes = customMemory.getKnownHoles();

        AmeyaCustomTWAgentMemory.MemoryEntry bestTile = bestEntry(tiles);
        AmeyaCustomTWAgentMemory.MemoryEntry bestHole = bestEntry(holes);

        int carried = carriedTiles.size();

        if (carried == CARRY_CAPACITY) return bestHole;

        if (carried == 2) {
            if (bestHole == null) return bestTile;
            if (bestTile == null) return bestHole;
            double distTile = manhattan(getX(), getY(), bestTile.x, bestTile.y);
            double distHole = manhattan(getX(), getY(), bestHole.x, bestHole.y);
            if (distTile + CARRY_BIAS_DISTANCE < distHole) return bestTile;
            return bestHole;
        }

        if (carried == 1) {
            if (bestTile == null) return bestHole;
            if (bestHole == null) return bestTile;
            return bestTile.utility >= bestHole.utility ? bestTile : bestHole;
        }

        return bestTile;
    }

    private AmeyaCustomTWAgentMemory.MemoryEntry bestEntry(
            List<AmeyaCustomTWAgentMemory.MemoryEntry> entries) {
        AmeyaCustomTWAgentMemory.MemoryEntry best = null;
        for (AmeyaCustomTWAgentMemory.MemoryEntry e : entries) {
            if (best == null || e.utility > best.utility) best = e;
        }
        return best;
    }


    private TWDirection getPathDirection(int tx, int ty) {
        TWPath path = pathGenerator.findPath(getX(), getY(), tx, ty);
        if (path != null && path.hasNext()) {
            TWPathStep step = path.popNext();
            if (step.getDirection() != TWDirection.Z) {
                return step.getDirection();
            }
        }
        return getExploreDirection();
    }


    private TWDirection getExploreDirection() {
        int ax = getX();
        int ay = getY();
        int maxX = getEnvironment().getxDimension() - 1;
        int maxY = getEnvironment().getyDimension() - 1;

        if (ax >= maxX) return TWDirection.W;
        if (ax <= 0)    return TWDirection.E;
        if (ay <= 0)    return TWDirection.S;
        if (ay >= maxY) return TWDirection.N;

        TWDirection[] dirs = {
            TWDirection.N, TWDirection.S,
            TWDirection.E, TWDirection.W
        };
        int start = getEnvironment().random.nextInt(4);
        for (int i = 0; i < 4; i++) {
            TWDirection d = dirs[(start + i) % 4];
            if (!getEnvironment().isCellBlocked(getX() + d.dx, getY() + d.dy)) {
                return d;
            }
        }
        return TWDirection.Z;
    }
}
