package ai;

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
 * Very small online-learning helper for pass selection.
 *
 * This is intentionally lightweight (no external ML libraries):
 * - Linear model score = w·features
 * - Update: w += lr * reward * features - lr*l2*w
 *
 * Rewards are provided by the simulator when a pass attempt succeeds/fails.
 */
public final class PassLearning {

    private PassLearning() {}

    // We keep weights stable with small learning rate + light L2.
    private static final double LR = 0.06;
    private static final double L2 = 0.002;

    // Feature indices
    private static final int F_FORWARD = 0;
    private static final int F_OPENNESS = 1;
    private static final int F_LANE = 2;
    private static final int F_RANGE = 3;
    private static final int F_CENTRAL = 4;
    private static final int F_COUNT = 5;

    private static final double[] W = new double[F_COUNT];

    private static boolean loaded = false;
    private static int updatesSinceSave = 0;

    private static Path weightsPath() {
        // Store next to the workspace root (process working directory).
        return Paths.get("pass-weights.properties");
    }

    public static synchronized void ensureLoaded() {
        if (loaded) return;

        // Reasonable defaults (pass-forward, open, clear lane, medium range, central).
        W[F_FORWARD] = 0.55;
        W[F_OPENNESS] = 0.85;
        W[F_LANE] = 0.95;
        W[F_RANGE] = 0.40;
        W[F_CENTRAL] = 0.25;

        Path p = weightsPath();
        if (Files.exists(p)) {
            Properties props = new Properties();
            try {
                try (var in = Files.newInputStream(p)) {
                    props.load(in);
                }
                W[F_FORWARD] = parse(props.getProperty("w.forward"), W[F_FORWARD]);
                W[F_OPENNESS] = parse(props.getProperty("w.openness"), W[F_OPENNESS]);
                W[F_LANE] = parse(props.getProperty("w.lane"), W[F_LANE]);
                W[F_RANGE] = parse(props.getProperty("w.range"), W[F_RANGE]);
                W[F_CENTRAL] = parse(props.getProperty("w.central"), W[F_CENTRAL]);
            } catch (IOException ignore) {
                // Keep defaults.
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

    public static final class ScoredPass {
        public final Robot receiver;
        public final double score;
        public final double[] features;

        private ScoredPass(Robot receiver, double score, double[] features) {
            this.receiver = receiver;
            this.score = score;
            this.features = features;
        }
    }

    /**
     * Choose the best receiver among teammates, using learned scoring.
     * The caller should still gate by basic safety checks if desired.
     */
    public static synchronized ScoredPass pickBestReceiver(Robot passer,
                                                           WorldState world,
                                                           int teamSign,
                                                           double minDist,
                                                           double maxDist) {
        ensureLoaded();
        if (passer == null || world == null || world.ball == null) return null;

        Ball ball = world.ball;
        List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        List<Robot> opps = (teamSign == +1) ? world.oppRobots : world.ourRobots;
        if (mates == null || mates.isEmpty()) return null;

        Robot best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        double[] bestF = null;

        for (Robot r : mates) {
            if (r == null) continue;
            if (r.id == passer.id) continue;

            double d = Math.sqrt(dist2(r.x, r.y, ball.x, ball.y));
            if (d < minDist || d > maxDist) continue;

            double[] f = features(ball.x, ball.y, teamSign, r, opps);
            double s = dot(W, f);
            if (s > bestScore) {
                bestScore = s;
                best = r;
                bestF = f;
            }
        }

        if (best == null) return null;
        return new ScoredPass(best, bestScore, bestF);
    }

    /**
     * Compute the same feature vector used by the learned scorer for a specific receiver.
     * Returns null if receiver cannot be found.
     */
    public static synchronized double[] featuresForReceiver(WorldState world, int teamSign, int receiverId) {
        ensureLoaded();
        if (world == null || world.ball == null) return null;
        List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        List<Robot> opps = (teamSign == +1) ? world.oppRobots : world.ourRobots;
        if (mates == null) return null;

        Robot recv = null;
        for (Robot r : mates) {
            if (r != null && r.id == receiverId) {
                recv = r;
                break;
            }
        }
        if (recv == null) return null;
        return features(world.ball.x, world.ball.y, teamSign, recv, opps);
    }

    public static synchronized void applyReward(double reward, double[] features) {
        ensureLoaded();
        if (features == null || features.length != F_COUNT) return;

        // Normalize reward to a reasonable scale.
        double r = clamp(reward, -2.0, 2.0);

        for (int i = 0; i < F_COUNT; i++) {
            // L2 regularization keeps weights bounded.
            W[i] += LR * (r * features[i] - L2 * W[i]);
        }

        updatesSinceSave++;
        if (updatesSinceSave >= 12) {
            updatesSinceSave = 0;
            save();
        }
    }

    public static synchronized void save() {
        ensureLoaded();
        Properties props = new Properties();
        props.setProperty("w.forward", Double.toString(W[F_FORWARD]));
        props.setProperty("w.openness", Double.toString(W[F_OPENNESS]));
        props.setProperty("w.lane", Double.toString(W[F_LANE]));
        props.setProperty("w.range", Double.toString(W[F_RANGE]));
        props.setProperty("w.central", Double.toString(W[F_CENTRAL]));

        try {
            try (var out = Files.newOutputStream(weightsPath())) {
                props.store(out, "Learned pass scoring weights");
            }
        } catch (IOException ignore) {
        }
    }

    private static double[] features(double ballX,
                                     double ballY,
                                     int teamSign,
                                     Robot receiver,
                                     List<Robot> opps) {
        double[] f = new double[F_COUNT];

        // 1) Forward progress (positive is good)
        double forward = (receiver.x - ballX) * teamSign;
        f[F_FORWARD] = clamp(forward / 3.5, -1.0, 1.0);

        // 2) Receiver openness: distance to nearest opponent
        double open = nearestOpponentDistance(receiver, opps);
        f[F_OPENNESS] = clamp(open / 2.5, 0.0, 1.2);

        // 3) Lane clearance: min distance of any opponent to pass segment
        double lane = laneClearance(ballX, ballY, receiver.x, receiver.y, opps);
        f[F_LANE] = clamp(lane / 1.0, 0.0, 1.5);

        // 4) Range preference: peak around 2m
        double d = Math.sqrt(dist2(receiver.x, receiver.y, ballX, ballY));
        f[F_RANGE] = clamp(1.0 - Math.abs(d - 2.0) / 2.0, -0.2, 1.0);

        // 5) Centrality: prefer passes that keep the ball away from touchlines
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
        double central = 1.0 - Math.min(1.0, Math.abs(receiver.y) / (halfW + 1e-9));
        f[F_CENTRAL] = clamp(central, 0.0, 1.0);

        return f;
    }

    private static double nearestOpponentDistance(Robot r, List<Robot> opps) {
        if (r == null || opps == null || opps.isEmpty()) return 9.0;
        double best = 9.0;
        for (Robot o : opps) {
            if (o == null) continue;
            double d = Math.sqrt(dist2(o.x, o.y, r.x, r.y));
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

    private static double dist2(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return dx * dx + dy * dy;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
