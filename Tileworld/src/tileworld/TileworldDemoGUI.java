package tileworld;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import sim.display.Console;
import sim.display.Controller;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.grid.ObjectGrid2D;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.SimplePortrayal2D;
import sim.portrayal.grid.ObjectGridPortrayal2D;
import tileworld.agent.TWAgent;
import tileworld.environment.TWEnvironment;

/**
 * TileworldDemoGUI
 *
 * Extends TileworldBetterGUI with a fading path-trail overlay.
 * Each agent leaves a color-coded trail that fades over TRAIL_FADE_STEPS steps.
 * This clearly reveals each agent's movement pattern for presentation recording:
 *   - Harshdeep → rectangular perimeter loop
 *   - Balala    → boustrophedon (snake) column sweep
 *   - Erica     → 4-corner patrol
 *   - Ben       → exploration-driven scatter
 *   - Ameya     → greedy BFS clusters
 *   - Arda      → zone-based Phase 1 sweep then reactive
 *
 * Run TileworldDemoGUI.main() instead of TileworldBetterGUI.main() or TWGUI.main().
 * No existing files are modified.
 */
public class TileworldDemoGUI extends TileworldBetterGUI {

    // How many simulation steps until a trail cell fully fades out.
    // 200 steps is enough to show Harshdeep's full perimeter loop (~196 cells).
    private static final int TRAIL_FADE_STEPS = 200;

    // Trail colors must match TileworldBetterGUI.AGENT_COLORS (same order).
    private static final Color[] TRAIL_COLORS = {
        new Color(220,  55,  55),   // Red
        new Color( 30, 130, 215),   // Blue
        new Color( 40, 195,  75),   // Green
        new Color(230, 150,  20),   // Orange
        new Color(170,  55, 210),   // Purple
        new Color( 20, 190, 175),   // Teal
    };

    // Trail data
    private ObjectGrid2D                  trailGrid;
    private final ObjectGridPortrayal2D   trailPortrayal = new ObjectGridPortrayal2D();
    private final List<TWAgent>           trackedAgents  = new ArrayList<TWAgent>();

    // Shared mutable reference so TrailCellPortrayal can read the current step
    // without needing access to the SimState directly.
    private final long[] currentStep = { 0L };

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public TileworldDemoGUI(SimState state) {
        super(state);
    }

    // -------------------------------------------------------------------------
    // MASON lifecycle
    // -------------------------------------------------------------------------

    /**
     * init() is called once when the Console is created (before the user presses Play).
     * We set up the trail grid and portrayal here so resetDisplay() can reference them.
     */
    @Override
    public void init(Controller c) {
        super.init(c);  // sets up display, displayFrame, stats panel

        TWEnvironment env = (TWEnvironment) state;
        trailGrid = new ObjectGrid2D(env.getxDimension(), env.getyDimension());
        trailPortrayal.setField(trailGrid);
        trailPortrayal.setPortrayalForClass(TrailCell.class,
                new TrailCellPortrayal(currentStep));
    }

    /**
     * resetDisplay() is called by TWEnvironment.start() every time the simulation
     * is (re)started.  We override it to insert the trail layer BELOW the objects
     * and agents layers so trails appear as background highlights.
     */
    @Override
    public void resetDisplay() {
        if (display == null || trailGrid == null) {
            super.resetDisplay();
            return;
        }
        display.detatchAll();
        display.attach(trailPortrayal,      "Agent Trails");   // bottom layer
        display.attach(objectGridPortrayal, "Tileworld Objects");
        display.attach(agentGridPortrayal,  "Tileworld Agents"); // top layer
        display.setBackdrop(new Color(20, 20, 20));  // near-black: trails pop visually
    }

    /**
     * start() is called every time the user presses Play.
     * We reinitialise the trail grid (fresh on each run) and schedule the
     * trail-updater Steppable at ordering 4 — after agents step (ordering 3).
     */
    @Override
    public void start() {
        super.start();  // starts environment, sets up portrayals, refreshes stats timer

        // Reinitialise trail grid so old trails don't bleed into a new run
        TWEnvironment env = (TWEnvironment) state;
        trailGrid = new ObjectGrid2D(env.getxDimension(), env.getyDimension());
        trailPortrayal.setField(trailGrid);

        // Collect agent references from the grid (populated by TWEnvironment.start())
        trackedAgents.clear();
        for (int x = 0; x < env.getxDimension(); x++) {
            for (int y = 0; y < env.getyDimension(); y++) {
                Object o = env.getAgentGrid().get(x, y);
                if (o instanceof TWAgent) {
                    trackedAgents.add((TWAgent) o);
                }
            }
        }

        // Schedule trail updater after agents move each step
        state.schedule.scheduleRepeating(new Steppable() {
            public void step(SimState s) {
                updateTrail(s);
            }
        }, 4, 1.0);
    }

    @Override
    public void quit() {
        super.quit();
    }

    // -------------------------------------------------------------------------
    // Trail update logic
    // -------------------------------------------------------------------------

    /**
     * Called every step (ordering 4, after agents).
     * Stamps each agent's current grid cell with a TrailCell carrying the agent's
     * color and the current step number.  The portrayal uses step age to compute
     * opacity — no explicit fading loop needed.
     */
    private void updateTrail(SimState s) {
        long step = s.schedule.getSteps();
        currentStep[0] = step;
        for (int i = 0; i < trackedAgents.size(); i++) {
            TWAgent a = trackedAgents.get(i);
            int ax = a.getX();
            int ay = a.getY();
            trailGrid.set(ax, ay, new TrailCell(TRAIL_COLORS[i % TRAIL_COLORS.length], step));
        }
    }

    // -------------------------------------------------------------------------
    // Inner classes
    // -------------------------------------------------------------------------

    /**
     * Stores the agent color and the simulation step at which the cell was visited.
     * Immutable — a new TrailCell is written on each visit (old one is overwritten).
     */
    static class TrailCell {
        final Color color;
        final long  visitedStep;

        TrailCell(Color color, long visitedStep) {
            this.color       = color;
            this.visitedStep = visitedStep;
        }
    }

    /**
     * MASON portrayal for TrailCell objects.
     * Draws a semi-transparent filled rectangle whose alpha decreases linearly
     * as the cell ages, reaching 0 at TRAIL_FADE_STEPS steps after the visit.
     */
    static class TrailCellPortrayal extends SimplePortrayal2D {

        // Maximum alpha (0–255) at the moment of visit.
        // 180 is opaque enough to see clearly but still lets grid lines show.
        private static final int MAX_ALPHA = 180;

        private final long[] currentStep;

        TrailCellPortrayal(long[] currentStep) {
            this.currentStep = currentStep;
        }

        @Override
        public void draw(Object obj, Graphics2D g, DrawInfo2D info) {
            if (!(obj instanceof TrailCell)) return;

            TrailCell cell = (TrailCell) obj;
            long age = currentStep[0] - cell.visitedStep;

            // Don't draw if too old or from a future step (can happen on restart)
            if (age < 0 || age >= TRAIL_FADE_STEPS) return;

            // Linear fade: full alpha when age=0, zero alpha when age=TRAIL_FADE_STEPS
            int alpha = (int)(MAX_ALPHA * (1.0 - (double) age / TRAIL_FADE_STEPS));
            if (alpha <= 0) return;

            Color c = cell.color;
            g.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), alpha));

            Rectangle2D.Double draw = info.draw;
            int x = (int)(draw.x - draw.width  / 2.0);
            int y = (int)(draw.y - draw.height / 2.0);
            int w = Math.max(1, (int) draw.width);
            int h = Math.max(1, (int) draw.height);
            g.fillRect(x, y, w, h);
        }
    }

    // -------------------------------------------------------------------------
    // Entry point
    // -------------------------------------------------------------------------

    public static void main(String[] args) {
        TileworldDemoGUI gui = new TileworldDemoGUI(new TWEnvironment());
        Console c = new Console(gui);
        c.setVisible(true);
    }
}
