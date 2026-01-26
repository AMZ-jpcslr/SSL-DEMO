package ai;

import java.util.List;
import tactics.GridPoint;
import tactics.ScoreGrid;
import tactics.TacticalScorers;
import ui.FieldConfig;
import world.Ball;
import world.Robot;
import world.WorldState;

/**
 * Attacker that can pass.
 *
 * Heuristic:
 * - If close to ball:
 *   - If there is a teammate in front (toward opponent goal) and pass line is not blocked, pass (kick).
 *   - Else shoot (kick) toward opponent goal.
 * - If not close: move to ball.
 *
 * NOTE: Kick direction is expressed via RobotCommand.kickVx/kickVy.
 */
public class PasserAttackerBehavior implements Behavior {

    private final int teamSign; // +1 blue attacks +x, -1 red attacks -x

    public PasserAttackerBehavior(int teamSign) {
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

        // Distance to ball
        double dx = ball.x - self.x;
        double dy = ball.y - self.y;
        double dist = Math.sqrt(dx * dx + dy * dy);

        double controlRange = FieldConfig.ROBOT_RADIUS_M + 0.06;
        boolean hasBall = dist <= controlRange;

        if (!hasBall) {
            // Move to ball (with gentle collision-avoidance so we don't ram into robots)
            moveToWithAvoid(cmd, self, ball.x, ball.y, 1.4, world);
            cmd.kick = false;
            return cmd;
        }

        // We have the ball: pass only if the lane is clearly open.
        // If the lane is blocked, keep/carry (dribble) until it opens instead of forcing a bad pass.
        Robot mate = bestPassTarget(self, world.ourRobots);
        boolean canPass = mate != null
            && passLaneLooksSafe(ball.x, ball.y, mate.x, mate.y, world.oppRobots)
            && mateIsReasonablyOpen(mate, world.oppRobots);

        // Learned pass scoring: among feasible short passes, prefer the one that is
        // (a) more progressive, (b) more open, and (c) has a clearer lane.
        PassLearning.ScoredPass learnedShort = PassLearning.pickBestReceiver(self, world, teamSign, 1.05, 5.2);
        boolean canLearnedShort = learnedShort != null
            && learnedShort.receiver != null
            && passLaneLooksSafe(ball.x, ball.y, learnedShort.receiver.x, learnedShort.receiver.y, world.oppRobots)
            && mateIsReasonablyOpen(learnedShort.receiver, world.oppRobots);

    // (2) Prefer a "requested" pass: choose a high-score point for ourselves, then pass to the teammate
    // closest to that point if it creates a clean lane. This couples the off-ball score map with the passer.
    Robot requestedMate = null;
    {
        GridPoint best = ScoreGrid.findBest(world, self, teamSign, 0.55, TacticalScorers.attackOffBall());
        requestedMate = closestMateToPoint(self, world.ourRobots, best.x, best.y);
    }
    boolean canRequestedPass = requestedMate != null
        && passLaneLooksSafe(ball.x, ball.y, requestedMate.x, requestedMate.y, world.oppRobots)
        && mateIsReasonablyOpen(requestedMate, world.oppRobots);

    // Long ball / switch-of-play: if short options are blocked, try a farther teammate (often wide)
    // with a higher speed kick.
    Robot longMate = bestLongPassTarget(self, world.ourRobots, world.oppRobots, teamSign);
    boolean canLongPass = longMate != null
        && passLaneLooksSafe(ball.x, ball.y, longMate.x, longMate.y, world.oppRobots)
        && mateIsReasonablyOpenLong(longMate, world.oppRobots);

        // If we're already in a good shooting zone, bias toward shooting.
        // This helps avoid over-passing (especially when passes are always considered "safe").
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double xTowardGoal = (ball.x * teamSign); // + means closer to opponent goal
        double shootZoneX = halfL * 0.30; // start shooting more from roughly the final 30%
        boolean inShootZone = xTowardGoal > shootZoneX;

        // Also consider shot, but don't take a shot if it's obviously blocked or can't reach the goal.
        double goalX = (teamSign == +1) ? (FieldConfig.FIELD_LENGTH_M / 2.0) : (-FieldConfig.FIELD_LENGTH_M / 2.0);
        double goalHalfW = FieldConfig.GOAL_WIDTH_M / 2.0;
        Double shotY = pickBestShotY(ball.x, ball.y, goalX, goalHalfW, world.oppRobots);
        // Only shoot when the ball can realistically reach the goal under our simple friction model.
        double plannedShotSpeed = inShootZone ? 5.6 : 5.0;
        boolean inShotReach = canReachGoal(ball.x, ball.y, goalX, shotY, plannedShotSpeed);
        boolean canShoot = (shotY != null) && inShotReach;

        // Learned shoot-vs-pass decision (keeps modern pass-first bias, but learns when shooting pays off).
        double[] actionFeats = ActionLearning.features(world, teamSign, self.id);
        boolean preferShoot = ActionLearning.chooseShoot(actionFeats, inShootZone ? 0.05 : 0.07);

        // Shoot if the learned policy prefers it and we have a clear lane.
        if (canShoot && preferShoot) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            cmd.kick = true;
            cmd.passTargetId = -1;
            cmd.shotIntent = true;
            cmd.kickVx = 0;
            cmd.kickVy = 0;
            setKickToward(cmd, ball.x, ball.y, goalX, shotY, plannedShotSpeed);
            return cmd;
        }

        // Prefer a learned short pass if available (modern, pass-first style), unless we're in the shoot zone.
        if (!inShootZone && canLearnedShort) {
            Robot r = learnedShort.receiver;
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            cmd.kick = true;
            cmd.passTargetId = r.id;
            cmd.shotIntent = false;
            cmd.kickVx = 0;
            cmd.kickVy = 0;
            setKickToward(cmd, ball.x, ball.y, r.x, r.y, 4.1);
            return cmd;
        }

        // Requested pass is useful, but don't spam it: require the target to be more forward/open.
        if (canRequestedPass && requestedMate != null) {
            double forward = (requestedMate.x - ball.x) * teamSign;
            if (forward > 0.35) {
                cmd.vx = 0;
                cmd.vy = 0;
                cmd.omega = 0;
                cmd.kick = true;
                cmd.passTargetId = requestedMate.id;
                cmd.shotIntent = false;
                cmd.kickVx = 0;
                cmd.kickVy = 0;
                setKickToward(cmd, ball.x, ball.y, requestedMate.x, requestedMate.y, 4.2);
                return cmd;
            }
        }

        // If forward options are not available, prefer a safe reset/back pass instead of solo dribbling.
        // This directly addresses the "one attacker charges alone" issue.
        if (!canPass && !canRequestedPass && !canLongPass) {
            Robot backMate = bestBackPassTarget(self, world.ourRobots, world.oppRobots, teamSign);
            if (backMate != null
                    && passLaneLooksSafe(ball.x, ball.y, backMate.x, backMate.y, world.oppRobots)
                    && mateIsReasonablyOpenLong(backMate, world.oppRobots)) {
                cmd.vx = 0;
                cmd.vy = 0;
                cmd.omega = 0;
                cmd.kick = true;
                cmd.passTargetId = backMate.id;
                cmd.shotIntent = false;
                cmd.kickVx = 0;
                cmd.kickVy = 0;
                setKickToward(cmd, ball.x, ball.y, backMate.x, backMate.y, 3.8);
                return cmd;
            }
        }

        if (canPass && mate != null) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            cmd.kick = true;
            cmd.passTargetId = mate.id;
            cmd.shotIntent = false;
            cmd.kickVx = 0;
            cmd.kickVy = 0;
            setKickToward(cmd, ball.x, ball.y, mate.x, mate.y, 4.0);
            return cmd;
        }

        if (canLongPass && longMate != null) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            cmd.kick = true;
            cmd.passTargetId = longMate.id;
            cmd.shotIntent = false;
            cmd.kickVx = 0;
            cmd.kickVy = 0;
            // Longer, faster kick.
            setKickToward(cmd, ball.x, ball.y, longMate.x, longMate.y, 6.4);
            return cmd;
        }

        // Otherwise, allow occasional learned exploration shots when lane is open.
        if (canShoot && Math.random() < 0.05) {
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = 0;
            cmd.kick = true;
            cmd.passTargetId = -1;
            cmd.shotIntent = true;
            cmd.kickVx = 0;
            cmd.kickVy = 0;
            setKickToward(cmd, ball.x, ball.y, goalX, shotY, plannedShotSpeed);
            return cmd;
        }

    // Neither pass nor shot is suitable: keep the ball.
        // "Dribble" by moving while staying close enough to keep possession.
        // Default carry direction: toward opponent goal, with a small lateral component to search for an open lane.
    double carryX = ball.x + 1.05 * teamSign;
        double carryY = ball.y + pickCarryOffsetY(self, world);
        carryX = clamp(carryX, -FieldConfig.FIELD_LENGTH_M / 2.0 + 0.5, FieldConfig.FIELD_LENGTH_M / 2.0 - 0.5);
        carryY = clamp(carryY, -FieldConfig.FIELD_WIDTH_M / 2.0 + 0.5, FieldConfig.FIELD_WIDTH_M / 2.0 - 0.5);

        moveToWithAvoid(cmd, self, carryX, carryY, 1.0, world);
        cmd.kick = false;
        cmd.passTargetId = -1;
        cmd.shotIntent = false;
        cmd.kickVx = 0;
        cmd.kickVy = 0;
        return cmd;
    }

    /**
     * Pick a back-pass/reset target: a teammate behind the ball (in attack direction),
     * reasonably far (avoid tiny taps), and preferably central.
     */
    private static Robot bestBackPassTarget(Robot self, List<Robot> mates, List<Robot> opps, int teamSign) {
        if (mates == null) return null;
        Robot best = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (Robot r : mates) {
            if (r == null || r.id == self.id) continue;

            double dx = r.x - self.x;
            double dy = r.y - self.y;
            double d = Math.sqrt(dx * dx + dy * dy);
            if (d < 0.8) continue;

            // Must be behind the ball/holder relative to attack direction.
            double behind = (self.x - r.x) * teamSign;
            if (behind < 0.35) continue;

            // Prefer central receivers for resets.
            double central = 1.0 - Math.min(1.0, Math.abs(r.y) / (FieldConfig.FIELD_WIDTH_M / 2.0));

            // Prefer if the receiver is not immediately under pressure.
            boolean open = mateIsReasonablyOpenLong(r, opps);
            double openBonus = open ? 0.7 : 0.0;

            // Prefer moderate distance (too far = slow reset).
            double distPenalty = -0.25 * d;

            double score = 1.2 * central + openBonus + distPenalty;
            if (score > bestScore) {
                bestScore = score;
                best = r;
            }
        }
        return best;
    }

    private static Robot bestPassTarget(Robot self, List<Robot> mates) {
        if (mates == null) return null;
        Robot best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (Robot r : mates) {
            if (r == null || r.id == self.id) continue;
            // Prefer closer teammates and those not too close (avoid tiny passes)
            double dx = r.x - self.x;
            double dy = r.y - self.y;
            double d = Math.sqrt(dx * dx + dy * dy);
            if (d < 0.5) continue;
            double score = -d;
            if (score > bestScore) {
                bestScore = score;
                best = r;
            }
        }
        return best;
    }

    private static Robot closestMateToPoint(Robot self, List<Robot> mates, double x, double y) {
        if (mates == null) return null;
        Robot best = null;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (Robot r : mates) {
            if (r == null || r.id == self.id) continue;
            double d2 = (r.x - x) * (r.x - x) + (r.y - y) * (r.y - y);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
            }
        }
        // Don't choose a "requested" mate if it's basically on top of us.
        if (best != null) {
            double d2Self = (best.x - self.x) * (best.x - self.x) + (best.y - self.y) * (best.y - self.y);
            if (d2Self < (0.55 * 0.55)) return null;
        }
        return best;
    }

    // Very rough: check if any opponent is close to the segment ball->mate.
    private static boolean passLaneLooksSafe(double ax, double ay, double bx, double by, List<Robot> opps) {
        if (opps == null) return true;
        double segLen = Math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        if (segLen < 1e-6) return false;

    double danger = 0.28; // meters (a bit conservative so we don't force passes)
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

    /**
     * Pick a shot target Y within the goal mouth that has the best clearance.
     * Returns null if every mouth line is too threatened.
     */
    private static Double pickBestShotY(double ballX, double ballY, double goalX, double goalHalfW, List<Robot> opps) {
        // Avoid aiming exactly at the posts.
        double margin = 0.06;
        double lo = -goalHalfW + margin;
        double hi = goalHalfW - margin;
        if (hi < lo) {
            lo = -goalHalfW;
            hi = goalHalfW;
        }

        // Sample a few lines inside the mouth.
        double[] ys = new double[] {
                0.0,
                lo * 0.55,
                hi * 0.55,
                lo,
                hi
        };

        // Must beat this clearance to be considered a "valid" shot lane.
        double danger = 0.28;

        Double bestY = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (double y : ys) {
            double cy = clamp(y, lo, hi);

            // Compute clearance to the segment ball->(goalX,cy)
            double clearance = 9.0;
            if (opps != null) {
                for (Robot o : opps) {
                    if (o == null) continue;
                    double[] p = closestPointOnSegment(ballX, ballY, goalX, cy, o.x, o.y);
                    double dx = o.x - p[0];
                    double dy = o.y - p[1];
                    double d = Math.sqrt(dx * dx + dy * dy);
                    if (d < clearance) clearance = d;
                }
            }

            if (clearance < danger) continue;

            // Prefer higher clearance, then prefer closer to center.
            double score = clearance * 2.0 - Math.abs(cy) * 0.25;
            if (score > bestScore) {
                bestScore = score;
                bestY = cy;
            }
        }

        return bestY;
    }

    private static boolean canReachGoal(double ballX, double ballY, double goalX, Double goalY, double kickSpeed) {
        if (goalY == null) return false;
        // Mirror the simulator's model: 60 FPS, velocity *= 0.98 each tick.
        double dt = 1.0 / 60.0;
        double damping = 0.98;

        // Total travel distance for geometric decay: v0*dt * (1/(1-damping))
        double maxTravel = (kickSpeed * dt) / (1.0 - damping);

        // Keep a small safety factor for collisions/imperfections.
        maxTravel *= 0.92;

        double dx = goalX - ballX;
        double dy = goalY - ballY;
        double dist = Math.sqrt(dx * dx + dy * dy);
        return dist <= maxTravel;
    }

    private static boolean mateIsReasonablyOpen(Robot mate, List<Robot> opps) {
        if (mate == null) return false;
        if (opps == null) return true;
        double openR = 0.45; // meters
        double openR2 = openR * openR;
        for (Robot o : opps) {
            double dx = o.x - mate.x;
            double dy = o.y - mate.y;
            if (dx * dx + dy * dy < openR2) return false;
        }
        return true;
    }

    private static boolean mateIsReasonablyOpenLong(Robot mate, List<Robot> opps) {
        if (mate == null) return false;
        if (opps == null) return true;
        // Long passes can tolerate defenders being slightly closer.
        double openR = 0.35; // meters
        double openR2 = openR * openR;
        for (Robot o : opps) {
            double dx = o.x - mate.x;
            double dy = o.y - mate.y;
            if (dx * dx + dy * dy < openR2) return false;
        }
        return true;
    }

    private static Robot bestLongPassTarget(Robot self, List<Robot> mates, List<Robot> opps, int teamSign) {
        if (mates == null) return null;
        Robot best = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (Robot r : mates) {
            if (r == null || r.id == self.id) continue;
            double dx = r.x - self.x;
            double dy = r.y - self.y;
            double d = Math.sqrt(dx * dx + dy * dy);

            // We only consider "long" options.
            if (d < 2.6) continue;

            // Prefer forward teammates and wide positioning (switch of play).
            double forward = dx * teamSign; // positive if ahead in attack direction
            if (forward < -0.5) continue;

            double width = Math.min(Math.abs(r.y), FieldConfig.FIELD_WIDTH_M / 2.0);

            // Penalize if an opponent is very close to the target.
            double nearestOppD = Double.POSITIVE_INFINITY;
            if (opps != null) {
                for (Robot o : opps) {
                    if (o == null) continue;
                    double odx = o.x - r.x;
                    double ody = o.y - r.y;
                    double od = Math.sqrt(odx * odx + ody * ody);
                    if (od < nearestOppD) nearestOppD = od;
                }
            }

            double score = 0.9 * forward + 0.6 * width - 0.7 * d + 0.35 * nearestOppD;
            if (score > bestScore) {
                bestScore = score;
                best = r;
            }
        }
        return best;
    }

    /**
     * Pick a small sideways offset to "search" for a pass lane.
     * Uses id parity for determinism (so we don't jitter).
     */
    private static double pickCarryOffsetY(Robot self, WorldState world) {
        double base = ((self.id % 2) == 0) ? 0.6 : -0.6;
        // If we're near a sideline already, bias back inward
        if (world != null && world.ball != null) {
            double y = world.ball.y;
            double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
            if (y > halfW - 0.8) base = -0.6;
            if (y < -halfW + 0.8) base = 0.6;
        }
        return base;
    }

    private static void moveToWithAvoid(RobotCommand cmd, Robot self, double targetX, double targetY, double speed, WorldState world) {
        // Base desired velocity toward target
        double dx = targetX - self.x;
        double dy = targetY - self.y;
        double dist = Math.sqrt(dx * dx + dy * dy);

        double vx = 0;
        double vy = 0;
        if (dist >= 0.05) {
            vx = (dx / dist) * speed;
            vy = (dy / dist) * speed;
        }

        // Repulsion from other robots (both teams) to keep personal space
        Vec2 repel = new Vec2(0, 0);
        double keep = FieldConfig.ROBOT_RADIUS_M * 2.4; // desired min distance
        double keep2 = keep * keep;
        double influence = keep * 2.0;
        double influence2 = influence * influence;

        if (world != null) {
            repel = repel.add(repelFromList(self, world.ourRobots, keep2, influence2));
            repel = repel.add(repelFromList(self, world.oppRobots, keep2, influence2));
        }

        // Blend (repulsion is capped so it remains a gentle bias)
        double repelGain = 1.2;
    vx += repel.x * repelGain;
    vy += repel.y * repelGain;

        double vMag = Math.sqrt(vx * vx + vy * vy);
        if (vMag > speed && vMag > 1e-9) {
            vx = vx / vMag * speed;
            vy = vy / vMag * speed;
        }

        cmd.vx = vx;
        cmd.vy = vy;
        cmd.omega = 0;
    }

    private static Vec2 repelFromList(Robot self, List<Robot> robots, double keep2, double influence2) {
        double rx = 0.0;
        double ry = 0.0;
        if (robots != null) {
            for (Robot r : robots) {
                if (r == null || r.id == self.id) continue;
                double dx = self.x - r.x;
                double dy = self.y - r.y;
                double d2 = dx * dx + dy * dy;
                if (d2 < 1e-9) continue;
                if (d2 > influence2) continue;
                // Stronger when closer; only push hard inside keep distance
                double w = (d2 < keep2) ? 1.0 : (influence2 - d2) / (influence2 - keep2);
                double d = Math.sqrt(d2);
                rx += (dx / d) * w;
                ry += (dy / d) * w;
            }
        }
        return new Vec2(rx, ry);
    }

    private static final class Vec2 {
        final double x;
        final double y;
        Vec2(double x, double y) {
            this.x = x;
            this.y = y;
        }
        Vec2 add(Vec2 o) {
            return new Vec2(this.x + o.x, this.y + o.y);
        }
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
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

    private static void setKickToward(RobotCommand cmd,
                                     double fromX, double fromY,
                                     double toX, double toY,
                                     double speed) {
        double dx = toX - fromX;
        double dy = toY - fromY;
        double d = Math.sqrt(dx * dx + dy * dy);
        if (d <= 1e-6) {
            cmd.kickVx = 0;
            cmd.kickVy = 0;
            return;
        }
        cmd.kickVx = (dx / d) * speed;
        cmd.kickVy = (dy / d) * speed;
    }
}
