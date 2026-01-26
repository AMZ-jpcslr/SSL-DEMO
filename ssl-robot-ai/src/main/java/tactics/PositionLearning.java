package tactics;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;
import ui.FieldConfig;
import world.Ball;
import world.Robot;
import world.WorldState;

/**
 * Online learning for off-ball positioning.
 *
 * Two small linear models provide an additive bonus term:
 * - attack: prefers receiving locations that historically led to good outcomes
 * - defense: prefers defensive locations that historically prevented opponent progress/passes
 */
public final class PositionLearning {

    private PositionLearning() {}

    private static final double LR = 0.055;
    private static final double L2 = 0.002;

    // Attack features
    private static final int A_FORWARD = 0;
    private static final int A_OPEN = 1;
    private static final int A_LANE = 2;
    private static final int A_RANGE = 3;
    private static final int A_CENTRAL = 4;
    private static final int A_TEAMSPACE = 5;
    private static final int A_COUNT = 6;

    // Defense features
    private static final int D_GOALSIDE = 0;
    private static final int D_LINEHOLD = 1;
    private static final int D_LANECUT = 2;
    private static final int D_MARKDIST = 3;
    private static final int D_MOVE = 4;
    private static final int D_COUNT = 5;

    private static final double[] WA = new double[A_COUNT];
    private static final double[] WD = new double[D_COUNT];

    private static boolean loaded = false;
    private static int updatesSinceSave = 0;

    private static Path weightsPath() {
        return Paths.get("position-weights.properties");
    }

    public static synchronized void ensureLoaded() {
        if (loaded) return;

        // Defaults: mild preference in same direction as the handcrafted heuristics.
        WA[A_FORWARD] = 0.35;
        WA[A_OPEN] = 0.65;
        WA[A_LANE] = 0.75;
        WA[A_RANGE] = 0.25;
        WA[A_CENTRAL] = 0.10;
        WA[A_TEAMSPACE] = 0.20;

        WD[D_GOALSIDE] = 0.65;
        WD[D_LINEHOLD] = 0.45;
        WD[D_LANECUT] = 0.55;
        WD[D_MARKDIST] = 0.20;
        WD[D_MOVE] = -0.15;

        Path p = weightsPath();
        if (Files.exists(p)) {
            Properties props = new Properties();
            try {
                try (var in = Files.newInputStream(p)) {
                    props.load(in);
                }
                for (int i = 0; i < A_COUNT; i++) {
                    WA[i] = parse(props.getProperty("wa." + i), WA[i]);
                }
                for (int i = 0; i < D_COUNT; i++) {
                    WD[i] = parse(props.getProperty("wd." + i), WD[i]);
                }
            } catch (IOException ignore) {
            }
        }

        loaded = true;
    }

    private static double parse(String s, double fallback) {
        if (s == null) return fallback;
        try {
            return Double.parseDouble(s.trim());
        } catch (Exception e) {
            return fallback;
        }
    }

    public static synchronized double attackBonus(WorldState world, Robot self, double x, double y, int teamSign) {
        ensureLoaded();
        double[] f = attackFeatures(world, self, x, y, teamSign);
        if (f == null) return 0.0;
        // Keep it as a small bonus so heuristics still dominate.
        return dot(WA, f) * 0.55;
    }

    public static synchronized double defenseBonus(WorldState world, Robot self, double x, double y, int teamSign, double[] mark) {
        ensureLoaded();
        double[] f = defenseFeatures(world, self, x, y, teamSign, mark);
        if (f == null) return 0.0;
        return dot(WD, f) * 0.55;
    }

    public static synchronized double[] attackFeatures(WorldState world, Robot self, double x, double y, int teamSign) {
        ensureLoaded();
        if (world == null || world.ball == null || self == null) return null;
        Ball ball = world.ball;
        List<Robot> opps = (teamSign == +1) ? world.oppRobots : world.ourRobots;
        List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;

        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

        double forward = (x - ball.x) * teamSign;
        double open = nearestOpponentDistance(x, y, opps);
        double lane = laneClearance(ball.x, ball.y, x, y, opps);
        double d = Math.sqrt(ScoreGrid.dist2(x, y, ball.x, ball.y));
        double range = 1.0 - Math.abs(d - 2.0) / 2.0;
        double central = 1.0 - Math.min(1.0, Math.abs(y) / (halfW + 1e-9));
        double mateMin = nearestMateDistance(x, y, mates, self.id);

        double[] f = new double[A_COUNT];
        f[A_FORWARD] = clamp(forward / 3.5, -1.0, 1.0);
        f[A_OPEN] = clamp(open / 2.5, 0.0, 1.5);
        f[A_LANE] = clamp(lane / 1.0, 0.0, 1.5);
        f[A_RANGE] = clamp(range, -0.3, 1.0);
        f[A_CENTRAL] = clamp(central, 0.0, 1.0);
        f[A_TEAMSPACE] = clamp(mateMin / 1.4, 0.0, 1.5);
        return f;
    }

    public static synchronized double[] defenseFeatures(WorldState world, Robot self, double x, double y, int teamSign, double[] mark) {
        ensureLoaded();
        if (world == null || world.ball == null || self == null) return null;
        Ball ball = world.ball;

        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;

        double ourGoalX = (teamSign == +1) ? -halfL : halfL;

        // Goalside (prefer being closer to our goal than the ball is)
        double ballToGoal = Math.abs(ball.x - ourGoalX);
        double pointToGoal = Math.abs(x - ourGoalX);
        double goalside = clamp((ballToGoal - pointToGoal) / 2.0, -1.0, 1.0);

        // Line hold: prefer a band 2..5m from our goal depending on ball depth
        double ballFromGoal = Math.abs(ball.x - ourGoalX);
        double desired = clamp(1.9 + 0.35 * ballFromGoal, 2.0, 5.2);
        double fromGoal = Math.abs(x - ourGoalX);
        double lineHold = -Math.abs(fromGoal - desired) / 3.0;

        // Lane cut to most advanced opponent (or mark if present)
        double cut = 0.0;
        if (mark != null) {
            double[] p = ScoreGrid.closestPointOnSegment(ball.x, ball.y, mark[0], mark[1], x, y);
            double d = Math.sqrt(ScoreGrid.dist2(x, y, p[0], p[1]));
            cut = -clamp(d / 2.0, 0.0, 1.0);
        } else {
            Robot threat = null;
            double best = Double.NEGATIVE_INFINITY;
            List<Robot> opps = (teamSign == +1) ? world.oppRobots : world.ourRobots;
            if (opps != null) {
                for (Robot o : opps) {
                    if (o == null) continue;
                    double adv = o.x * teamSign;
                    if (adv > best) {
                        best = adv;
                        threat = o;
                    }
                }
            }
            if (threat != null) {
                double[] p = ScoreGrid.closestPointOnSegment(ball.x, ball.y, threat.x, threat.y, x, y);
                double d = Math.sqrt(ScoreGrid.dist2(x, y, p[0], p[1]));
                cut = -clamp(d / 2.4, 0.0, 1.0);
            }
        }

        // Mark distance preference: 0.8..1.8m from mark
        double markDist = 0.0;
        if (mark != null) {
            double dm = Math.sqrt(ScoreGrid.dist2(x, y, mark[0], mark[1]));
            double err = Math.abs(dm - 1.25);
            markDist = -clamp((err - 0.70) / 1.2, 0.0, 1.0);
        }

        // Move cost (prefer smoother)
        double move = -Math.sqrt(ScoreGrid.dist2(x, y, self.x, self.y)) / 4.0;

        double[] f = new double[D_COUNT];
        f[D_GOALSIDE] = goalside;
        f[D_LINEHOLD] = clamp(lineHold, -1.2, 0.0);
        f[D_LANECUT] = cut;
        f[D_MARKDIST] = markDist;
        f[D_MOVE] = move;
        return f;
    }

    public static synchronized void applyAttackReward(double reward, double[] features) {
        ensureLoaded();
        if (features == null || features.length != A_COUNT) return;
        double r = clamp(reward, -2.0, 2.0);
        for (int i = 0; i < A_COUNT; i++) {
            WA[i] += LR * (r * features[i] - L2 * WA[i]);
        }
        updatesSinceSave++;
        if (updatesSinceSave >= 18) {
            updatesSinceSave = 0;
            save();
        }
    }

    public static synchronized void applyDefenseReward(double reward, double[] features) {
        ensureLoaded();
        if (features == null || features.length != D_COUNT) return;
        double r = clamp(reward, -2.0, 2.0);
        for (int i = 0; i < D_COUNT; i++) {
            WD[i] += LR * (r * features[i] - L2 * WD[i]);
        }
        updatesSinceSave++;
        if (updatesSinceSave >= 18) {
            updatesSinceSave = 0;
            save();
        }
    }

    public static synchronized void save() {
        ensureLoaded();
        Properties props = new Properties();
        for (int i = 0; i < A_COUNT; i++) {
            props.setProperty("wa." + i, Double.toString(WA[i]));
        }
        for (int i = 0; i < D_COUNT; i++) {
            props.setProperty("wd." + i, Double.toString(WD[i]));
        }
        try {
            try (var out = Files.newOutputStream(weightsPath())) {
                props.store(out, "Learned off-ball positioning weights");
            }
        } catch (IOException ignore) {
        }
    }

    private static double nearestOpponentDistance(double x, double y, List<Robot> opps) {
        if (opps == null || opps.isEmpty()) return 9.0;
        double best = 9.0;
        for (Robot o : opps) {
            if (o == null) continue;
            double d = Math.sqrt(ScoreGrid.dist2(o.x, o.y, x, y));
            if (d < best) best = d;
        }
        return best;
    }

    private static double nearestMateDistance(double x, double y, List<Robot> mates, int selfId) {
        if (mates == null || mates.isEmpty()) return 9.0;
        double best = 9.0;
        for (Robot r : mates) {
            if (r == null || r.id == selfId) continue;
            double d = Math.sqrt(ScoreGrid.dist2(r.x, r.y, x, y));
            if (d < best) best = d;
        }
        return best;
    }

    private static double laneClearance(double ax, double ay, double bx, double by, List<Robot> opps) {
        if (opps == null || opps.isEmpty()) return 9.0;
        double best = 9.0;
        for (Robot o : opps) {
            if (o == null) continue;
            double d = distancePointToSegment(o.x, o.y, ax, ay, bx, by);
            if (d < best) best = d;
        }
        return best;
    }

    private static double distancePointToSegment(double px, double py, double ax, double ay, double bx, double by) {
        double abx = bx - ax;
        double aby = by - ay;
        double apx = px - ax;
        double apy = py - ay;
        double ab2 = abx * abx + aby * aby;
        if (ab2 <= 1e-9) return Math.sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
        double t = (apx * abx + apy * aby) / ab2;
        t = clamp(t, 0.0, 1.0);
        double cx = ax + t * abx;
        double cy = ay + t * aby;
        return Math.sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
    }

    private static double dot(double[] w, double[] x) {
        double s = 0.0;
        for (int i = 0; i < w.length && i < x.length; i++) s += w[i] * x[i];
        return s;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
