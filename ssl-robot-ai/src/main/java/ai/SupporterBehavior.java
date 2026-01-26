package ai;

import ui.FieldConfig;
import world.Ball;
import world.Robot;
import world.WorldState;

/**
 * Supporter (receiver) behavior to make passing work.
 *
 * Idea:
 * - If we are "attacking" (ball is on opponent half), try to get open and become a good pass target.
 * - Otherwise, do nothing (caller should run DefenderBehavior in that situation).
 */
public class SupporterBehavior implements Behavior {

    private final int teamSign; // +1: blue attacks +x, -1: red attacks -x

    public SupporterBehavior(int teamSign) {
        this.teamSign = (teamSign >= 0) ? +1 : -1;
    }

    @Override
    public RobotCommand decide(Robot self, WorldState world) {
        RobotCommand cmd = new RobotCommand();
        cmd.robotId = self.id;

        if (world == null || world.ball == null) {
            return cmd;
        }

        Ball ball = world.ball;

        // Assign a stable "lane" based on id so multiple supporters spread out.
        // Values: -1, +1, -2, +2, ...
        int slot = slotFromId(self.id);
    // Wider lanes so we use the full field width.
    double laneY = slot * 1.35; // meters

        // Compute a "support" point a bit ahead of the ball (toward opponent goal)
        // and slightly offset in y so we don't stack on the attacker.
    double ahead = 1.35; // meters in front of the ball

        double targetX = ball.x + ahead * teamSign;
        // Stay reasonably aligned with current ball.y but stabilize around our lane.
        double targetY = 0.5 * ball.y + 0.5 * laneY;

        // If the pass lane (ball -> target) is blocked, slide laterally to create a lane.
        // Keep it deterministic (based on id) to avoid jitter.
        if (!passLaneLooksSafe(ball.x, ball.y, targetX, targetY, world.oppRobots)) {
            double slide = ((self.id % 2) == 0) ? 0.7 : -0.7;
            // Try a few lateral offsets, alternating further out.
            double[] tries = new double[] { slide, -slide, 1.2 * slide, -1.2 * slide };
            boolean found = false;
            for (double off : tries) {
                double ty = targetY + off;
                if (passLaneLooksSafe(ball.x, ball.y, targetX, ty, world.oppRobots)) {
                    targetY = ty;
                    found = true;
                    break;
                }
            }
            if (!found) {
                // Even if not fully open, at least spread away from the congested line.
                targetY += slide;
            }
        }

    // Don't park too close to the ball while supporting.
    double minBallDist = 0.9;
        double dx = targetX - ball.x;
        double dy = targetY - ball.y;
        double d = Math.sqrt(dx * dx + dy * dy);
        if (d < minBallDist && d > 1e-6) {
            double scale = minBallDist / d;
            targetX = ball.x + dx * scale;
            targetY = ball.y + dy * scale;
        }

        // Keep support target within play area
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
        double margin = FieldConfig.ROBOT_RADIUS_M + 0.05;
        targetX = clamp(targetX, -halfL + margin, halfL - margin);
        targetY = clamp(targetY, -halfW + margin, halfW - margin);

        moveTo(cmd, self, targetX, targetY, 1.3);
        cmd.kick = false;
        return cmd;
    }

    // Very rough: check if any opponent is close to the segment ball->support point.
    private static boolean passLaneLooksSafe(double ax, double ay, double bx, double by, java.util.List<Robot> opps) {
        if (opps == null) return true;
        double segLen = Math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        if (segLen < 1e-6) return false;

        double danger = 0.30; // meters
        double danger2 = danger * danger;

        for (Robot o : opps) {
            double[] p = closestPointOnSegment(ax, ay, bx, by, o.x, o.y);
            double dx = o.x - p[0];
            double dy = o.y - p[1];
            double d2 = dx * dx + dy * dy;
            if (d2 < danger2) return false;
        }
        return true;
    }

    private static double[] closestPointOnSegment(double ax, double ay, double bx, double by, double px, double py) {
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

    private static int slotFromId(int id) {
        // Make id -> 0.. (avoid negative)
        int k = Math.abs(id);
        int i = (k % 4); // 0..3
        int mag = (i / 2) + 1; // 1,1,2,2
        int sign = (i % 2 == 0) ? -1 : +1;
        return sign * mag;
    }

    private static void moveTo(RobotCommand cmd, Robot self, double targetX, double targetY, double speed) {
        double dx = targetX - self.x;
        double dy = targetY - self.y;
        double dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < 0.05) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            return;
        }

        cmd.vx = (dx / dist) * speed;
        cmd.vy = (dy / dist) * speed;
        cmd.omega = 0;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
