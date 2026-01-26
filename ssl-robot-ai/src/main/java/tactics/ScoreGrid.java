package tactics;

import java.util.List;
import ui.FieldConfig;
import world.Robot;
import world.WorldState;

/**
 * Samples the field on a fixed grid and returns the best-scoring point.
 *
 * This is intentionally lightweight: no path planning, just a position evaluation function.
 */
public final class ScoreGrid {

    private ScoreGrid() {}

    /**
     * Find the best point on a grid.
     *
     * @param step grid spacing in meters
     */
    public static GridPoint findBest(WorldState world,
                                     Robot self,
                                     int teamSign,
                                     double step,
                                     PositionScorer scorer) {
        if (world == null || self == null || world.ball == null || scorer == null) {
            return new GridPoint(self != null ? self.x : 0.0, self != null ? self.y : 0.0, Double.NEGATIVE_INFINITY);
        }

        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
        double margin = FieldConfig.ROBOT_RADIUS_M + 0.06;

        double bestX = self.x;
        double bestY = self.y;
        double best = Double.NEGATIVE_INFINITY;

        // Soft-focus search region: sample whole field, but give the scorer a chance to penalize far points.
        for (double x = -halfL + margin; x <= halfL - margin; x += step) {
            for (double y = -halfW + margin; y <= halfW - margin; y += step) {
                double s = scorer.score(world, self, x, y, teamSign);
                if (s > best) {
                    best = s;
                    bestX = x;
                    bestY = y;
                }
            }
        }

        return new GridPoint(bestX, bestY, best);
    }

    // --- helper utilities used by scorers ---

    public static double dist2(double ax, double ay, double bx, double by) {
        double dx = ax - bx;
        double dy = ay - by;
        return dx * dx + dy * dy;
    }

    public static Robot closestRobot(List<Robot> robots, double x, double y) {
        if (robots == null || robots.isEmpty()) return null;
        Robot best = null;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (Robot r : robots) {
            if (r == null) continue;
            double d2 = dist2(r.x, r.y, x, y);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
            }
        }
        return best;
    }

    public static boolean segmentBlockedByOpponents(double ax, double ay,
                                                    double bx, double by,
                                                    List<Robot> opps,
                                                    double dangerRadius) {
        if (opps == null) return false;
        double danger2 = dangerRadius * dangerRadius;
        for (Robot o : opps) {
            if (o == null) continue;
            double[] p = closestPointOnSegment(ax, ay, bx, by, o.x, o.y);
            double dx = o.x - p[0];
            double dy = o.y - p[1];
            if (dx * dx + dy * dy < danger2) return true;
        }
        return false;
    }

    public static double[] closestPointOnSegment(double ax, double ay,
                                                 double bx, double by,
                                                 double px, double py) {
        double abx = bx - ax;
        double aby = by - ay;
        double apx = px - ax;
        double apy = py - ay;

        double ab2 = abx * abx + aby * aby;
        if (ab2 <= 1e-9) {
            return new double[] { ax, ay };
        }

        double t = (apx * abx + apy * aby) / ab2;
        t = Math.max(0.0, Math.min(1.0, t));
        return new double[] { ax + t * abx, ay + t * aby };
    }

    /** Predict ball position after t seconds using current velocity (no acceleration). */
    public static double[] predictBallPos(WorldState world, double tSec) {
        if (world == null || world.ball == null) return new double[] { 0.0, 0.0 };
        double x = world.ball.x + world.ball.vx * tSec;
        double y = world.ball.y + world.ball.vy * tSec;
        return new double[] { x, y };
    }

    /**
     * Rough "can opponent intercept pass" check. We assume robots can travel at maxSpeed in straight line.
     * If any opponent can reach within captureRadius of the pass segment earlier than the ball arrives,
     * treat it as interceptable.
     */
    public static boolean passInterceptable(double ax, double ay,
                                            double bx, double by,
                                            List<Robot> opps,
                                            double ballSpeed,
                                            double oppMaxSpeed,
                                            double captureRadius) {
        if (opps == null || opps.isEmpty()) return false;
        double dx = bx - ax;
        double dy = by - ay;
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 1e-6) return false;
        double travelTime = dist / Math.max(0.1, ballSpeed);

        for (Robot o : opps) {
            if (o == null) continue;
            // Best intercept point is closest point on segment.
            double[] p = closestPointOnSegment(ax, ay, bx, by, o.x, o.y);
            double od = Math.sqrt(dist2(o.x, o.y, p[0], p[1]));

            // Estimate time for opponent to reach capture radius of that point.
            double need = Math.max(0.0, od - captureRadius);
            double tOpp = need / Math.max(0.1, oppMaxSpeed);

            // Estimate when ball arrives to that point (ratio along segment).
            double along = Math.sqrt(dist2(ax, ay, p[0], p[1]));
            double tBall = along / Math.max(0.1, ballSpeed);

            if (tOpp < tBall && tBall <= travelTime + 1e-6) {
                return true;
            }
        }
        return false;
    }
}
