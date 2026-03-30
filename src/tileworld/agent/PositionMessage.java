package tileworld.agent;

import tileworld.environment.Message;

/**
 * PositionMessage
 * A structured broadcast used to share an agent’s current position
 * and the number of tiles it is carrying.
 */
public class PositionMessage extends Message {

    private final int agentX;
    private final int agentY;
    private final int tilesCarried;

    /** Construct directly — used by ZonePatrolAgent.communicate(). */
    public PositionMessage(String from, int x, int y, int tilesCarried) {
        // Tileworld Message constructor: (String from, String to, String message)
        super(from, "*", "POS:" + x + "," + y + ";TILES:" + tilesCarried);
        this.agentX = x;
        this.agentY = y;
        this.tilesCarried = tilesCarried;
    }

    /** Private constructor used by parse(). */
    private PositionMessage(String from, String content, int x, int y, int tiles) {
        super(from, "*", content);
        this.agentX = x;
        this.agentY = y;
        this.tilesCarried = tiles;
    }

    /**
     * Attempts to parse any Message (plain or PositionMessage) into
     * a PositionMessage. Returns null if the message doesn't match
     * the protocol format "POS:x,y;TILES:n".
     */
    public static PositionMessage parse(Message m) {
        if (m == null) return null;

        // If it's already a PositionMessage, return it directly
        if (m instanceof PositionMessage) {
            return (PositionMessage) m;
        }

        // Use getMessage() instead of getContent() for Tileworld compatibility
        String content = m.getMessage();
        if (content == null) return null;

        try {
            if (!content.startsWith("POS:")) return null;

            // Strip "POS:" prefix
            String rest = content.substring(4);

            // Split on ";" to separate coordinates and tiles
            String[] parts = rest.split(";");
            if (parts.length < 2) return null;

            // Parse coordinates: "x,y"
            String[] coords = parts[0].split(",");
            if (coords.length < 2) return null;
            int x = Integer.parseInt(coords[0].trim());
            int y = Integer.parseInt(coords[1].trim());

            // Parse tiles: "TILES:n"
            if (!parts[1].startsWith("TILES:")) return null;
            int tiles = Integer.parseInt(parts[1].substring(6).trim());

            return new PositionMessage(m.getFrom(), content, x, y, tiles);

        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            return null; // malformed message — ignore
        }
    }

    public int getAgentX() { return agentX; }
    public int getAgentY() { return agentY; }
    public int getTilesCarried() { return tilesCarried; }

    /** True if this teammate can still pick up a tile. */
    public boolean canPickUp() { return tilesCarried < 3; }

    /** True if this teammate has tiles to deposit into a hole. */
    public boolean hasTiles()  { return tilesCarried > 0; }
}