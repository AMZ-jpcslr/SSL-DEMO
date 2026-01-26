package ai;

import ui.FieldConfig;
import world.Ball;
import world.Robot;
import world.WorldState;

/**
 * Very simple defender:
 * - Default: stay on the line between ball and our goal center (x = -FIELD_LENGTH/2, y = 0)
 * - If an opponent is the "likely receiver" (closest to the ball), move to cut the pass line
 *   (ball -> receiver) by going to the closest point on that segment.
 * - If the ball is in our half and close enough, try to clear (kick) to +x.
 */
public class DefenderBehavior implements Behavior {

    // +1: defend left goal (blue team), -1: defend right goal (red team)
    private final int defendGoalSign;

    public DefenderBehavior() {
        this(+1);
    }

    public DefenderBehavior(int defendGoalSign) {
        this.defendGoalSign = (defendGoalSign >= 0) ? +1 : -1;
    }

    @Override
    public RobotCommand decide(Robot self, WorldState world) {
        RobotCommand cmd = new RobotCommand();
        cmd.robotId = self.id;

        if (world == null || world.ball == null) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            cmd.kick = false;
            return cmd;
        }

        Ball ball = world.ball;

        // --- Targets ---
    // Goal center: blue defends left (-x), red defends right (+x)
    double goalX = (defendGoalSign == +1)
        ? -FieldConfig.FIELD_LENGTH_M / 2.0
        :  FieldConfig.FIELD_LENGTH_M / 2.0;
        double goalY = 0.0;

        // Find a likely receiver: opponent closest to the ball.
        Robot receiver = findClosestRobot(world.oppRobots, ball.x, ball.y);

        // Default: block line from ball to our goal center.
        double targetX = (ball.x + goalX) * 0.5;
        double targetY = (ball.y + goalY) * 0.5;

        // If there is a receiver, try to cut the pass line ball->receiver.
        if (receiver != null) {
            double[] p = closestPointOnSegment(ball.x, ball.y, receiver.x, receiver.y, self.x, self.y);
            targetX = p[0];
            targetY = p[1];

            // Encourage staying in our half
            targetX = Math.min(targetX, 0.0);
        }

        // --- Clear if ball is dangerous (in our half) and close ---
        double dxBall = ball.x - self.x;
        double dyBall = ball.y - self.y;
        double distBall = Math.sqrt(dxBall * dxBall + dyBall * dyBall);
        double controlRange = FieldConfig.ROBOT_RADIUS_M + 0.05;

    boolean ballInOurHalf = (defendGoalSign == +1) ? (ball.x < 0.0) : (ball.x > 0.0);
        if (ballInOurHalf && distBall <= controlRange) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            cmd.kick = true;

            // Clear away from our goal: blue clears toward +x, red clears toward -x
            cmd.kickVx = 5.0 * defendGoalSign;
            cmd.kickVy = 0.0;
            return cmd;
        }

        // --- Move to target ---
        moveTo(cmd, self, targetX, targetY, 1.2);
        cmd.kick = false;
        return cmd;
    }

    private static Robot findClosestRobot(java.util.List<Robot> robots, double x, double y) {
        if (robots == null || robots.isEmpty()) return null;
        Robot best = null;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (Robot r : robots) {
            double dx = r.x - x;
            double dy = r.y - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
            }
        }
        return best;
    }

    /**
     * Closest point to (px,py) on segment AB.
     */
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
        t = clamp(t, 0.0, 1.0);

        return new double[] { ax + t * abx, ay + t * aby };
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
