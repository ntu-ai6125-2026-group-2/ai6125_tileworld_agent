package tileworld;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import javax.swing.DefaultListCellRenderer;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.Timer;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableCellRenderer;
import sim.display.Console;
import sim.display.Controller;
import sim.engine.SimState;
import tileworld.agent.TWAgent;
import tileworld.agent.TWAgentPortrayal;
import tileworld.environment.TWEnvironment;

/**
 * TileworldBetterGUI
 *
 * Drop-in replacement for TWGUI with:
 *   1. Per-agent distinct colors (unique color per agent including sensor-range halo)
 *   2. Live "Agent Stats" panel: score, color-coded fuel bar, tiles carried (400ms refresh)
 *
 * Run TileworldBetterGUI.main() instead of TWGUI.main().
 * No existing files are modified.
 */
public class TileworldBetterGUI extends TWGUI {

    private static final Color[] AGENT_COLORS = {
        new Color(220,  55,  55),   // Red
        new Color( 30, 130, 215),   // Blue
        new Color( 40, 195,  75),   // Green
        new Color(230, 150,  20),   // Orange
        new Color(170,  55, 210),   // Purple
        new Color( 20, 190, 175),   // Teal
    };

    private final List<TWAgent> agentList = new ArrayList<TWAgent>();
    private JFrame            statsFrame;
    private DefaultTableModel tableModel;
    private Timer             statsTimer;

    public TileworldBetterGUI(SimState state) {
        super(state);
    }

    // -------------------------------------------------------------------------
    // MASON lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void start() {
        super.start();  // calls our overridden setupPortrayals() via dynamic dispatch
        if (statsTimer != null) statsTimer.restart();
    }

    @Override
    public void setupPortrayals() {
        super.setupPortrayals();
        collectAgents();
        for (int i = 0; i < agentList.size(); i++) {
            agentGridPortrayal.setPortrayalForObject(
                agentList.get(i),
                new TWAgentPortrayal(AGENT_COLORS[i % AGENT_COLORS.length],
                                     Parameters.defaultSensorRange)
            );
        }
    }

    @Override
    public void init(Controller c) {
        super.init(c);
        buildStatsPanel();
    }

    @Override
    public void quit() {
        if (statsTimer != null) statsTimer.stop();
        if (statsFrame != null) statsFrame.dispose();
        super.quit();
    }

    // -------------------------------------------------------------------------
    // Agent collection
    // -------------------------------------------------------------------------

    private void collectAgents() {
        agentList.clear();
        TWEnvironment env = (TWEnvironment) state;
        int w = env.getxDimension();
        int h = env.getyDimension();
        for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++) {
                Object o = env.getAgentGrid().get(x, y);
                if (o instanceof TWAgent) {
                    agentList.add((TWAgent) o);
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // Stats panel
    // -------------------------------------------------------------------------

    private JPanel colorKeyPanel = null;

    private void buildStatsPanel() {
        tableModel = new DefaultTableModel(
                new String[]{"", "Agent", "Class", "Score", "Fuel", "Tiles"}, 0) {
            @Override
            public boolean isCellEditable(int r, int c) { return false; }
        };

        JTable table = new JTable(tableModel);
        table.setRowHeight(26);
        table.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 12));

        table.getColumnModel().getColumn(0).setMaxWidth(22);
        table.getColumnModel().getColumn(1).setPreferredWidth(70);
        table.getColumnModel().getColumn(2).setPreferredWidth(250);
        table.getColumnModel().getColumn(3).setPreferredWidth(48);
        table.getColumnModel().getColumn(4).setPreferredWidth(140);
        table.getColumnModel().getColumn(5).setMaxWidth(48);

        table.getColumnModel().getColumn(0).setCellRenderer(new ColorDotRenderer());
        table.getColumnModel().getColumn(4).setCellRenderer(new FuelBarRenderer());

        // Color key strip
        colorKeyPanel = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 3));
        colorKeyPanel.setBackground(new Color(45, 45, 45));

        statsFrame = new JFrame("Tileworld - Live Agent Stats");
        statsFrame.setLayout(new BorderLayout());
        statsFrame.add(colorKeyPanel, BorderLayout.NORTH);
        statsFrame.add(new JScrollPane(table), BorderLayout.CENTER);
        statsFrame.setSize(630, 240);
        statsFrame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        statsFrame.setVisible(true);

        statsTimer = new Timer(400, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                refreshStats();
            }
        });
        statsTimer.start();
    }

    private void refreshStats() {
        if (colorKeyPanel != null
                && colorKeyPanel.getComponentCount() == 0
                && !agentList.isEmpty()) {
            for (int i = 0; i < agentList.size(); i++) {
                JLabel badge = new JLabel("  " + agentList.get(i).getName() + "  ");
                badge.setOpaque(true);
                badge.setBackground(AGENT_COLORS[i % AGENT_COLORS.length]);
                badge.setForeground(Color.WHITE);
                badge.setFont(badge.getFont().deriveFont(Font.BOLD, 11f));
                colorKeyPanel.add(badge);
            }
            colorKeyPanel.revalidate();
        }

        tableModel.setRowCount(0);
        for (int i = 0; i < agentList.size(); i++) {
            TWAgent a = agentList.get(i);
            tableModel.addRow(new Object[]{
                Integer.valueOf(i),
                a.getName(),
                a.getClass().getSimpleName(),
                Integer.valueOf(readScore(a)),
                Integer.valueOf((int) a.getFuelLevel()),
                Integer.valueOf(readTilesCarried(a))
            });
        }
    }

    // -------------------------------------------------------------------------
    // Reflection helpers
    // -------------------------------------------------------------------------

    private static int readScore(TWAgent a) {
        try {
            Field f = TWAgent.class.getDeclaredField("score");
            f.setAccessible(true);
            Object v = f.get(a);
            return (v instanceof Number) ? ((Number) v).intValue() : 0;
        } catch (Exception e) {
            return 0;
        }
    }

    private static int readTilesCarried(TWAgent a) {
        try {
            Field f = TWAgent.class.getDeclaredField("carriedTiles");
            f.setAccessible(true);
            Object v = f.get(a);
            if (v instanceof List)     return ((List<?>) v).size();
            if (v instanceof Object[]) return ((Object[]) v).length;
        } catch (Exception e) { /* fall through */ }
        return 0;
    }

    // -------------------------------------------------------------------------
    // Inner renderers
    // -------------------------------------------------------------------------

    private class ColorDotRenderer extends DefaultTableCellRenderer {
        @Override
        public Component getTableCellRendererComponent(
                JTable t, Object val, boolean sel, boolean foc, int row, int col) {
            JLabel l = (JLabel) super.getTableCellRendererComponent(
                    t, "", sel, foc, row, col);
            int idx = (val instanceof Number) ? ((Number) val).intValue() : row;
            l.setBackground(AGENT_COLORS[idx % AGENT_COLORS.length]);
            l.setOpaque(true);
            return l;
        }
    }

    private static class FuelBarRenderer extends JProgressBar implements TableCellRenderer {
        FuelBarRenderer() {
            super(0, Parameters.defaultFuelLevel);
            setStringPainted(true);
            setBorderPainted(false);
        }

        @Override
        public Component getTableCellRendererComponent(
                JTable t, Object val, boolean sel, boolean foc, int row, int col) {
            int fuel = (val instanceof Number) ? ((Number) val).intValue() : 0;
            setValue(fuel);
            setString(fuel + " / " + Parameters.defaultFuelLevel);
            double ratio = (double) fuel / Parameters.defaultFuelLevel;
            if (ratio > 0.4)      setForeground(new Color(40, 170, 40));
            else if (ratio > 0.2) setForeground(new Color(210, 140, 0));
            else                  setForeground(Color.RED);
            return this;
        }
    }

    // -------------------------------------------------------------------------
    // Entry point
    // -------------------------------------------------------------------------

    public static void main(String[] args) {
        TileworldBetterGUI gui = new TileworldBetterGUI(new TWEnvironment());
        Console c = new Console(gui);
        c.setVisible(true);
    }
}
