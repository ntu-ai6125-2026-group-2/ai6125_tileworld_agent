package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;
import tileworld.exceptions.CellBlockedException;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * MyTWAgent: Final Integrated Version
 * Features: Strip-based Zigzag search, Position Broadcasting, and Safe Grid Interaction.
 */
public class MyTWAgent extends TWAgent {

    private String name;
    private int fuelStationX = -1, fuelStationY = -1;
    private int stripStartRow, stripEndRow, zigzagRow, zigzagCol;
    private boolean goingRight = true;
    private boolean fuelStationBroadcasted = false;
    
    // Static team score for TileworldMain compatibility
    private static final AtomicInteger teamScore = new AtomicInteger(0);
    public static void resetTeamScore() { 
        teamScore.set(0); 
    }

    public MyTWAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        setupZigzag(env);
    }

    @Override
    public String getName() {
        return this.name;
    }

    /**
     * MANUAL DIRECTION LOGIC
     * Replaces the missing getDirectionTo method in some TWEnvironment versions
     */
    private TWDirection getDirTo(int tx, int ty) {
        if (tx > getX()) return TWDirection.E;
        if (tx < getX()) return TWDirection.W;
        if (ty > getY()) return TWDirection.S;
        if (ty < getY()) return TWDirection.N;
        return TWDirection.Z;
    }

    @Override
    public void communicate() {
        // 1. Broadcast fuel station location to all
        if (fuelStationX != -1 && !fuelStationBroadcasted) {
            this.getEnvironment().receiveMessage(new Message(this.name, "all", "f:" + fuelStationX + "," + fuelStationY));
            fuelStationBroadcasted = true;
        }

        // 2. PositionMessage Protocol: Broadcast position and tiles for coordination
        String payload = "POS:" + getX() + "," + getY() + ";TILES:" + carriedTiles.size();
        this.getEnvironment().receiveMessage(new Message(this.name, "*", payload));
    }

    @Override
    protected TWThought think() {
        this.sense();
        readMessages();

        // 1. REFUELING (High Priority)
        if (fuelStationX != -1) {
            double dist = Math.abs(getX() - fuelStationX) + Math.abs(getY() - fuelStationY);
            if (this.fuelLevel < (dist * 2 + 40)) {
                return new TWThought(TWAction.MOVE, getDirTo(fuelStationX, fuelStationY));
            }
        }

        // 2. TARGETING (Tiles if empty, Holes if full)
        TWEntity target = null;
        if (this.carriedTiles.size() >= 3) {
            target = this.memory.getNearbyHole(getX(), getY(), 15);
        } else {
            target = this.memory.getNearbyTile(getX(), getY(), 15);
        }

        if (target != null) {
            return new TWThought(TWAction.MOVE, getDirTo(target.getX(), target.getY()));
        }

        // 3. EXPLORATION (Zigzag)
        return executeZigzag();
    }

    private void readMessages() {
        for (Object o : getEnvironment().getMessages()) {
            if (!(o instanceof Message)) continue;
            Message msg = (Message) o;
            String text = msg.getMessage();
            
            if (text != null && text.startsWith("f:")) {
                try {
                    String[] p = text.substring(2).split(",");
                    this.fuelStationX = Integer.parseInt(p[0]);
                    this.fuelStationY = Integer.parseInt(p[1]);
                } catch (Exception e) {}
            }
        }
    }

    private void setupZigzag(TWEnvironment env) {
        int agentNum = Math.abs(name.hashCode()) % 6; 
        int totalRows = env.getyDimension(); 
        int stripSize = Math.max(1, totalRows / 6); 
        this.stripStartRow = agentNum * stripSize;
        this.stripEndRow = (agentNum == 5) ? totalRows - 1 : (agentNum + 1) * stripSize - 1;
        this.zigzagRow = stripStartRow;
        this.zigzagCol = 0;
    }

    private TWThought executeZigzag() {
        if (getX() == zigzagCol && getY() == zigzagRow) advanceZigzag();
        return new TWThought(TWAction.MOVE, getDirTo(zigzagCol, zigzagRow));
    }

    private void advanceZigzag() {
        int mapW = getEnvironment().getxDimension();
        if (goingRight) {
            zigzagCol++;
            if (zigzagCol >= mapW) { zigzagCol = mapW - 1; zigzagRow++; goingRight = false; }
        } else {
            zigzagCol--;
            if (zigzagCol < 0) { zigzagCol = 0; zigzagRow++; goingRight = true; }
        }
        if (zigzagRow > stripEndRow) zigzagRow = stripStartRow;
    }

    @Override
    protected void act(TWThought thought) {
        try {
            // Check for Fuel Station
            if (this.getEnvironment().inFuelStation(this)) {
                this.refuel();
                this.fuelStationX = getX();
                this.fuelStationY = getY();
            }

            // SAFE GRID INTERACTION: Use instanceof to prevent ClassCastException
            Object gridObject = getEnvironment().getObjectGrid().get(getX(), getY());

            if (gridObject instanceof TWTile) {
                TWTile tile = (TWTile) gridObject;
                if (carriedTiles.size() < 3) {
                    this.pickUpTile(tile);
                }
            } else if (gridObject instanceof TWHole) {
                TWHole hole = (TWHole) gridObject;
                if (hasTile()) {
                    this.putTileInHole(hole);
                    teamScore.incrementAndGet();
                }
            }

            // Execute Movement
            if (thought != null && thought.getAction() == TWAction.MOVE) {
                this.move(thought.getDirection());
            }
        } catch (CellBlockedException e) {
            // Path blocked, handle in next tick
        }
    }
}