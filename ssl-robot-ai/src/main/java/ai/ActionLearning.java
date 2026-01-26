package ai;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;
import tactics.ScoreGrid;
import ui.FieldConfig;
import world.Ball;
import world.Robot;
import world.WorldState;

/**
 * Online learning for shoot-vs-pass decisions.
 *
 * We model P(shoot) = sigmoid(w·f). Update uses a simple policy-gradient/logistic rule:
 *   w += lr * reward * (action - p) * f - lr*l2*w
 * where action=1 for shoot, 0 for pass.
 */
public final class ActionLearning {

    private ActionLearning() {}

    private static final double LR = 0.06;
    private static final double L2 = 0.002;

    // Features
    private static final int F_IN_SHOOT_ZONE = 0;
    private static final int F_GOAL_LANE = 1;
    private static final int F_DIST_TO_GOAL = 2;
    private static final int F_BALL_X_ATTACK = 3;
    private static final int F_BEST_PASS_SCORE = 4;
    private static final int F_SAFE_PASS_COUNT = 5;
    private static final int F_COUNT = 6;

    private static final double[] W = new double[F_COUNT];
    private static boolean loaded = false;
    private static int updatesSinceSave = 0;

    private static Path weightsPath() {
        return Paths.get("action-weights.properties");
    }

    public static synchronized void ensureLoaded() {
        if (loaded) return;

        // Conservative defaults: shoot more in good zones and with clear lane.
        W[F_IN_SHOOT_ZONE] = 0.85;
        W[F_GOAL_LANE] = 0.90;
        W[F_DIST_TO_GOAL] = -0.35;
        W[F_BALL_X_ATTACK] = 0.20;
        W[F_BEST_PASS_SCORE] = -0.55;
        W[F_SAFE_PASS_COUNT] = -0.25;

        Path p = weightsPath();
        if (Files.exists(p)) {
            Properties props = new Properties();
            try {
                try (var in = Files.newInputStream(p)) {
                    props.load(in);
                }
                W[F_IN_SHOOT_ZONE] = parse(props.getProperty("w.inShootZone"), W[F_IN_SHOOT_ZONE]);
                W[F_GOAL_LANE] = parse(props.getProperty("w.goalLane"), W[F_GOAL_LANE]);
                W[F_DIST_TO_GOAL] = parse(props.getProperty("w.distToGoal"), W[F_DIST_TO_GOAL]);
                W[F_BALL_X_ATTACK] = parse(props.getProperty("w.ballXAttack"), W[F_BALL_X_ATTACK]);
                W[F_BEST_PASS_SCORE] = parse(props.getProperty("w.bestPassScore"), W[F_BEST_PASS_SCORE]);
                W[F_SAFE_PASS_COUNT] = parse(props.getProperty("w.safePassCount"), W[F_SAFE_PASS_COUNT]);
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

    public static synchronized double[] features(WorldState world, int teamSign, int passerId) {
        ensureLoaded();
        if (world == null || world.ball == null) return null;

        Ball ball = world.ball;
        Robot passer = findById((teamSign == +1) ? world.ourRobots : world.oppRobots, passerId);
        if (passer == null) return null;

        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double ballXAttack = ball.x * teamSign;

        // Shoot zone heuristic (same as existing: last 30% of the field)
        double shootZoneX = halfL * 0.30;
        boolean inShootZone = ballXAttack > shootZoneX;

        double goalX = (teamSign == +1) ? halfL : -halfL;
        boolean goalLaneSafe = passLaneLooksSafe(ball.x, ball.y, goalX, 0.0,
                (teamSign == +1) ? world.oppRobots : world.ourRobots);

        double distToGoal = Math.sqrt(dist2(ball.x, ball.y, goalX, 0.0));

        // Estimate how good the best pass is (learned receiver score)
        PassLearning.ScoredPass sp = PassLearning.pickBestReceiver(passer, world, teamSign, 0.75, 4.2);
        double bestPassScore = (sp == null) ? -1.0 : sp.score;

        int safeCount = countSafePasses(passer, world, teamSign);

        double[] f = new double[F_COUNT];
        f[F_IN_SHOOT_ZONE] = inShootZone ? 1.0 : 0.0;
        f[F_GOAL_LANE] = goalLaneSafe ? 1.0 : 0.0;
        f[F_DIST_TO_GOAL] = clamp(distToGoal / (halfL * 2.0), 0.0, 1.2);
        f[F_BALL_X_ATTACK] = clamp(ballXAttack / halfL, -1.0, 1.0);
        f[F_BEST_PASS_SCORE] = clamp(bestPassScore / 4.0, -1.0, 1.0);
        f[F_SAFE_PASS_COUNT] = clamp(safeCount / 4.0, 0.0, 1.2);
        return f;
    }

    public static synchronized boolean chooseShoot(double[] features, double epsilon) {
        ensureLoaded();
        if (features == null || features.length != F_COUNT) return false;

        double z = dot(W, features);
        double p = sigmoid(z);

        // Add a small epsilon for exploration.
        double u = Math.random();
        if (u < epsilon) {
            return Math.random() < 0.5;
        }
        return Math.random() < p;
    }

    public static synchronized void applyReward(boolean actionShoot, double reward, double[] features) {
        ensureLoaded();
        if (features == null || features.length != F_COUNT) return;

        double r = clamp(reward, -2.0, 2.0);
        double z = dot(W, features);
        double p = sigmoid(z);
        double a = actionShoot ? 1.0 : 0.0;

        double g = (a - p);
        for (int i = 0; i < F_COUNT; i++) {
            W[i] += LR * (r * g * features[i] - L2 * W[i]);
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
        props.setProperty("w.inShootZone", Double.toString(W[F_IN_SHOOT_ZONE]));
        props.setProperty("w.goalLane", Double.toString(W[F_GOAL_LANE]));
        props.setProperty("w.distToGoal", Double.toString(W[F_DIST_TO_GOAL]));
        props.setProperty("w.ballXAttack", Double.toString(W[F_BALL_X_ATTACK]));
        props.setProperty("w.bestPassScore", Double.toString(W[F_BEST_PASS_SCORE]));
        props.setProperty("w.safePassCount", Double.toString(W[F_SAFE_PASS_COUNT]));
        try {
            try (var out = Files.newOutputStream(weightsPath())) {
                props.store(out, "Learned shoot vs pass weights");
            }
        } catch (IOException ignore) {
        }
    }

    private static int countSafePasses(Robot passer, WorldState world, int teamSign) {
        List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        List<Robot> opps = (teamSign == +1) ? world.oppRobots : world.ourRobots;
        if (mates == null || passer == null || world == null || world.ball == null) return 0;

        Ball ball = world.ball;
        int count = 0;
        for (Robot r : mates) {
            if (r == null || r.id == passer.id) continue;
            double d = Math.sqrt(dist2(r.x, r.y, ball.x, ball.y));
            if (d < 0.75 || d > 4.2) continue;
            boolean lane = passLaneLooksSafe(ball.x, ball.y, r.x, r.y, opps);
            if (!lane) continue;
            count++;
        }
        return count;
    }

    // Keep consistent with PasserAttackerBehavior lane check style (using ScoreGrid helper).
    private static boolean passLaneLooksSafe(double ax, double ay, double bx, double by, List<Robot> opps) {
        return !ScoreGrid.segmentBlockedByOpponents(ax, ay, bx, by, opps, 0.30);
    }

    private static Robot findById(List<Robot> robots, int id) {
        if (robots == null) return null;
        for (Robot r : robots) {
            if (r != null && r.id == id) return r;
        }
        return null;
    }

    private static double dot(double[] w, double[] x) {
        double s = 0.0;
        for (int i = 0; i < w.length && i < x.length; i++) s += w[i] * x[i];
        return s;
    }

    private static double sigmoid(double z) {
        if (z >= 0) {
            double ez = Math.exp(-z);
            return 1.0 / (1.0 + ez);
        }
        double ez = Math.exp(z);
        return ez / (1.0 + ez);
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
