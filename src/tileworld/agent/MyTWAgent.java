package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;
import tileworld.exceptions.CellBlockedException;
import sim.util.Int2D;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class MyTWAgent extends TWAgent {
    private String name;
    private TWPath currentPath = null;
    
    // Safety buffer reduced to 22% to maximize scoring time
    private static final double FUEL_THRESHOLD_RATIO = 0.22;
    private static final int CARRY_CAPACITY = 3;

    public static int fuelStationX = -1;
    public static int fuelStationY = -1;
    public static final Map<Int2D, Long> teamMemory = new ConcurrentHashMap<>();

    public MyTWAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(x, y, env, fuel);
        this.name = name;
    }

    @Override
    public String getName() { return this.name; }

    public static void resetTeamScore() {
        fuelStationX = -1; fuelStationY = -1;
        teamMemory.clear();
    }

    private void processMessages() {
        List<Message> messages = this.getEnvironment().getMessages();
        for (Message m : messages) {
            if (m instanceof ArdaMessage) {
                ArdaMessage am = (ArdaMessage) m;
                if (am.getEntityType().equalsIgnoreCase("FuelStation")) {
                    fuelStationX = am.getX();
                    fuelStationY = am.getY();
                }
            } else if (m.getMessage().startsWith("FUEL:")) { // Listen to BenTWAgent
                String[] parts = m.getMessage().substring(5).split(",");
                fuelStationX = Integer.parseInt(parts[0]);
                fuelStationY = Integer.parseInt(parts[1]);
            }
        }
    }

    @Override
    protected TWThought think() {
        processMessages();
        updatePerception();

        double fuelRatio = getFuelLevel() / Parameters.defaultFuelLevel;
        if (fuelRatio < FUEL_THRESHOLD_RATIO && fuelStationX != -1) {
            if ((int)getX() == fuelStationX && (int)getY() == fuelStationY)
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            return navigateTo(fuelStationX, fuelStationY);
        }

        // Logic for picking up/dropping off based on capacity
        Object here = getEnvironment().getObjectGrid().get((int)getX(), (int)getY());
        if (here instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY)
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        if (here instanceof TWHole && !carriedTiles.isEmpty())
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);

        if (currentPath == null || !currentPath.hasNext()) {
            Int2D target = findBestTarget();
            if (target != null) {
                // Signal intention to prevent team clumping
                this.getEnvironment().receiveMessage(ArdaMessage.intention(this.name, "Goal", target.x, target.y, (int)getX(), (int)getY()));
                currentPath = getPathTo(target.x, target.y);
            }
        }
        return followPath();
    }

    private void updatePerception() {
        int r = Parameters.defaultSensorRange;
        for (int i = -r; i <= r; i++) {
            for (int j = -r; j <= r; j++) {
                int tx = (int)getX() + i, ty = (int)getY() + j;
                if (!getEnvironment().isInBounds(tx, ty)) continue;
                Object o = getEnvironment().getObjectGrid().get(tx, ty);
                if (o instanceof TWFuelStation) {
                    fuelStationX = tx; fuelStationY = ty;
                    this.getEnvironment().receiveMessage(new Message(this.name, null, "FUEL:" + tx + "," + ty));
                } else if (o instanceof TWHole || o instanceof TWTile) {
                    teamMemory.put(new Int2D(tx, ty), getEnvironment().schedule.getSteps());
                }
            }
        }
    }

    private Int2D findBestTarget() {
        Int2D best = null; double minSafeDist = Double.MAX_VALUE;
        Class<?> goal = (carriedTiles.size() >= CARRY_CAPACITY) ? TWHole.class : TWTile.class;
        for (Int2D pos : teamMemory.keySet()) {
            Object o = getEnvironment().getObjectGrid().get(pos.x, pos.y);
            if (o != null && o.getClass().equals(goal)) {
                double d = pos.distance(getX(), getY());
                if (d < minSafeDist) { minSafeDist = d; best = pos; }
            }
        }
        return best;
    }

    private TWThought navigateTo(int x, int y) {
        currentPath = getPathTo(x, y);
        return followPath();
    }

    private TWThought followPath() {
        if (currentPath == null || !currentPath.hasNext()) return new TWThought(TWAction.MOVE, TWDirection.Z);
        TWPathStep next = currentPath.popNext();
        return new TWThought(TWAction.MOVE, next.getDirection());
    }

    private TWPath getPathTo(int x, int y) {
        return new AstarPathGenerator(getEnvironment(), this, 2000).findPath((int)getX(), (int)getY(), x, y);
    }

    @Override
    protected void act(TWThought t) {
        try {
            switch(t.getAction()){
                case MOVE: move(t.getDirection()); break;
                case REFUEL: refuel(); break;
                case PICKUP: pickUpTile((TWTile)getEnvironment().getObjectGrid().get((int)getX(),(int)getY())); break;
                case PUTDOWN: putTileInHole((TWHole)getEnvironment().getObjectGrid().get((int)getX(),(int)getY())); break;
            }
        } catch (CellBlockedException e) {
            this.currentPath = null;
        }
    }
}