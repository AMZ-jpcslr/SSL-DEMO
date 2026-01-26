package tactics;

import sim.Main;
import ui.FieldConfig;
import world.Ball;
import world.Robot;

/**
 * A small collection of heuristic scorers.
 *
 * The intent is NOT perfect soccer, but a flexible framework where you can
 * add/weight terms and immediately see different team shapes.
 */
public final class TacticalScorers {

    private TacticalScorers() {}

    /**
     * Attacking off-ball scoring:
     * - Prefer being ahead of the ball (in attack direction)
     * - Prefer width (use the field)
     * - Prefer being open from opponents
     * - Prefer pass line from ball to point being safe
     * - Mild penalty for going too far from current position (keeps motion smoother)
     */
    public static PositionScorer attackOffBall() {
        return (world, self, x, y, teamSign) -> {
            Ball ball = world.ball;

            double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
            double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

            // Ball motion context
            double ballSpeed = Math.sqrt(ball.vx * ball.vx + ball.vy * ball.vy);
            double[] ballFuture = ScoreGrid.predictBallPos(world, 0.45);

            // --- Requested scoring breakdown (10 points total) ---
            // (1) Enemy not nearby (open space): 2 points
            Robot nearestOpp = ScoreGrid.closestRobot(world.oppRobots, x, y);
            double oppD = (nearestOpp == null) ? 9.0 : Math.sqrt(ScoreGrid.dist2(nearestOpp.x, nearestOpp.y, x, y));
            double open2 = (oppD >= 1.0) ? 2.0 : clamp(oppD / 1.0, 0.0, 1.0) * 2.0;

            // (2) Not too close to teammates: 1 point
            double mateMin = 9.0;
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null || r.id == self.id) continue;
                    double d = Math.sqrt(ScoreGrid.dist2(r.x, r.y, x, y));
                    if (d < mateMin) mateMin = d;
                }
            }
            // Full 1pt if >=1.05m, else scaled down.
            double mate1 = (mateMin >= 1.05) ? 1.0 : clamp(mateMin / 1.05, 0.0, 1.0) * 1.0;

            // (3) Pass-course options: +1 point per available option (including the ball holder position).
            // We count how many distinct teammates can pass to (x,y) without opponent blocking.
            int passOptions = 0;
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null) continue;
                    if (r.id == self.id) continue;
                    // If the line r -> (x,y) is not blocked, it's a valid option.
                    boolean blocked = ScoreGrid.segmentBlockedByOpponents(r.x, r.y, x, y, world.oppRobots, 0.30);
                    if (!blocked) passOptions++;
                }
            }
            // Cap to avoid overweighting in small teams.
            double passPts = Math.min(4, passOptions) * 1.0;

            // Extra: penalize locations where likely passes are easily interceptable (time-to-intercept).
            // This is motion-aware via assumed ball speed (if currently slow, interceptions are easier).
            double assumedBallSpeed = Math.max(1.2, ballSpeed * 0.9 + 1.2);
            boolean interceptable = ScoreGrid.passInterceptable(ball.x, ball.y, x, y,
                    world.oppRobots, assumedBallSpeed, 1.55, 0.18);
            double interceptPenalty = interceptable ? -1.15 : 0.0;

            // (4) Shootability: 2 points if we can shoot (x,y)->goal without strong block
            double theirGoalX = (teamSign == +1) ? halfL : -halfL;
            double theirGoalY = 0.0;
            boolean shootBlocked = ScoreGrid.segmentBlockedByOpponents(x, y, theirGoalX, theirGoalY, world.oppRobots, 0.35);
            double shoot2 = shootBlocked ? 0.0 : 2.0;

            // Small shaping terms (not part of the 10-point breakdown) to avoid degeneracy:
            // - Keep some minimum distance from the ball
            // - Encourage having both forward and backward support existing by not forcing everyone ahead
            double ballD = Math.sqrt(ScoreGrid.dist2(x, y, ball.x, ball.y));
            double minBallD = 0.85;
            double nearBallPenalty = (ballD < minBallD) ? -(minBallD - ballD) * 1.2 : 0.0;

            // Anticipation: be available where the ball is going, not only where it is now.
            // Only mild so it doesn't drag everyone forward on a fast clearance.
            double futureD = Math.sqrt(ScoreGrid.dist2(x, y, ballFuture[0], ballFuture[1]));
            double anticipateBonus = (ballSpeed > 0.25) ? clamp(1.6 - futureD, -2.0, 2.0) * 0.35 : 0.0;

        // Directional shaping: when the ball is moving forward in attack direction,
        // reward being in front of the *moving* ball a bit more (prevents drifting back).
        double ballVAttack = (ball.vx * teamSign);
        double xAttack = x * teamSign;
        double ballXAttack = ball.x * teamSign;
        double aheadOfBall = xAttack - ballXAttack; // positive => ahead
        double forwardFlow = (ballSpeed > 0.25 && ballVAttack > 0.20)
            ? clamp(aheadOfBall, -1.5, 2.5) * 0.18
            : 0.0;

            // Prefer not being extremely close to our own goal when attacking.
            double ourGoalX = (teamSign == +1) ? -halfL : halfL;
            double goalDist = Math.abs(x - ourGoalX);
            double goalPenalty = (goalDist < 1.1) ? -(1.1 - goalDist) * 0.8 : 0.0;

            double score10 = open2 + mate1 + passPts + shoot2;
            return score10 + nearBallPenalty + goalPenalty + interceptPenalty + anticipateBonus + forwardFlow;
        };
    }

    /**
     * Defensive off-ball scoring used when our team is actually attacking (so defenders should step up).
     *
     * This is intentionally "offense-score-like" (so they occupy good, passable, open positions),
     * but with additional constraints:
     * - don't drift behind our ball / too close to our goal
     * - don't crowd the ball winner
     */
    public static PositionScorer defendWhileAttacking() {
        return (world, self, x, y, teamSign) -> {
            Ball ball = world.ball;
            double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
            double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

            double ballSpeed = Math.sqrt(ball.vx * ball.vx + ball.vy * ball.vy);
            double[] ballFuture = ScoreGrid.predictBallPos(world, 0.35);

            // Start from the same "10pt" rubric as attack.
            Robot nearestOpp = ScoreGrid.closestRobot(world.oppRobots, x, y);
            double oppD = (nearestOpp == null) ? 9.0 : Math.sqrt(ScoreGrid.dist2(nearestOpp.x, nearestOpp.y, x, y));
            double open2 = (oppD >= 1.0) ? 2.0 : clamp(oppD / 1.0, 0.0, 1.0) * 2.0;

            double mateMin = 9.0;
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null || r.id == self.id) continue;
                    double d = Math.sqrt(ScoreGrid.dist2(r.x, r.y, x, y));
                    if (d < mateMin) mateMin = d;
                }
            }
            double mate1 = (mateMin >= 1.05) ? 1.0 : clamp(mateMin / 1.05, 0.0, 1.0);

            int passOptions = 0;
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null) continue;
                    if (r.id == self.id) continue;
                    boolean blocked = ScoreGrid.segmentBlockedByOpponents(r.x, r.y, x, y, world.oppRobots, 0.30);
                    if (!blocked) passOptions++;
                }
            }
            double passPts = Math.min(3, passOptions) * 1.0;

            double theirGoalX = (teamSign == +1) ? halfL : -halfL;
            boolean shootBlocked = ScoreGrid.segmentBlockedByOpponents(x, y, theirGoalX, 0.0, world.oppRobots, 0.35);
            double shoot2 = shootBlocked ? 0.0 : 2.0;

            double score10 = open2 + mate1 + passPts + shoot2;

            // Defensive-line constraint while attacking: stay "behind" the ball a bit, but not too deep.
            // We want defenders to cross midfield when ball is advanced, but still provide cover.
            double xAttack = (x * teamSign);
            double ballXAttack = (ball.x * teamSign);
            double behindBall = ballXAttack - xAttack; // positive => we are behind the ball
            // Prefer being moderately behind the ball (cover distance), not too far and not too close.
            // Band target: ~1.4..3.2m behind, with a peak around 2.2m.
            double desired = 2.2;
            double slack = 0.9; // within desired±slack is OK
            double err = Math.abs(behindBall - desired);
            double behindPenalty = -(Math.max(0.0, err - slack)) * 0.95;

            // Actively penalize being way too far behind ("left behind" effect).
            double tooFarPenalty = (behindBall > 4.2) ? -(behindBall - 4.2) * 1.15 : 0.0;

            // Also penalize being ahead of the ball as a defender.
            double aheadPenalty = (behindBall < -0.2) ? -(-0.2 - behindBall) * 1.35 : 0.0;

            // Don't hang inside our own third when we're attacking.
            double ourGoalX = (teamSign == +1) ? -halfL : halfL;
            double fromOurGoal = Math.abs(x - ourGoalX);
            double deepPenalty = (fromOurGoal < 2.8) ? -(2.8 - fromOurGoal) * 0.9 : 0.0;

            // Still avoid being right on top of the ball.
            double ballD = Math.sqrt(ScoreGrid.dist2(x, y, ball.x, ball.y));
            double nearBallPenalty = (ballD < 1.05) ? -(1.05 - ballD) * 1.4 : 0.0;

            // If ball is moving toward our half quickly, keep the safety a bit more conservative.
            // (prevents rest-defender from stepping up right as we lose possession).
            double ballVAttack = (ball.vx * teamSign);
            double transitionPenalty = (ballSpeed > 0.35 && ballVAttack < -0.25) ? -0.9 : 0.0;

            // Cover the ball's projected lane too (if it's about to roll into a channel).
            double futureD = Math.sqrt(ScoreGrid.dist2(x, y, ballFuture[0], ballFuture[1]));
            double coverFuture = (ballSpeed > 0.25) ? clamp(2.2 - futureD, -2.0, 2.0) * 0.25 : 0.0;

            // Avoid extreme wings for defenders while attacking (keeps a compact rest-defense).
            double yNorm = Math.abs(y) / halfW;
            double wingPenalty = (yNorm > 0.70) ? -(yNorm - 0.70) * 0.8 : 0.0;

            // When the ball is contested, ties in scoring + deconfliction can push the rest-defender
            // toward a deep corner (our side + touchline). This creates the "DF stuck in corner" bug.
            // Add an explicit penalty for camping in our deep corners.
            double fromOurGoalLine = Math.abs(x - ourGoalX);
            boolean inDeepThird = (fromOurGoalLine < 2.2);
            boolean nearTouch = (Math.abs(y) > halfW * 0.82);
            double cornerPenalty = (inDeepThird && nearTouch) ? -3.5 : 0.0;

            // Soft preference against being too close to the goal line even if not near touchline.
            double goalLinePenalty = (fromOurGoalLine < 1.0) ? -(1.0 - fromOurGoalLine) * 1.6 : 0.0;

            return score10
                    + behindPenalty + tooFarPenalty + aheadPenalty
                    + deepPenalty + nearBallPenalty + wingPenalty
            + cornerPenalty + goalLinePenalty
            + transitionPenalty + coverFuture;
        };
    }

    /**
     * Wide defenders joining attack (MF-like) while still being part of "rest-defense".
     *
     * Goal:
     * - when our attack is advanced, wide defenders should step into midfield (often across x=0)
     * - keep enough width to be a safe outlet
     * - still reward open / spaced / passable points, so they don't stand in traffic
     */
    public static PositionScorer wideDefenderJoinAttack() {
        return (world, self, x, y, teamSign) -> {
            Ball ball = world.ball;
            double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
            double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

            // Base: same 10pt rubric core
            Robot nearestOpp = ScoreGrid.closestRobot(world.oppRobots, x, y);
            double oppD = (nearestOpp == null) ? 9.0 : Math.sqrt(ScoreGrid.dist2(nearestOpp.x, nearestOpp.y, x, y));
            double open2 = (oppD >= 1.0) ? 2.0 : clamp(oppD / 1.0, 0.0, 1.0) * 2.0;

            double mateMin = 9.0;
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null || r.id == self.id) continue;
                    double d = Math.sqrt(ScoreGrid.dist2(r.x, r.y, x, y));
                    if (d < mateMin) mateMin = d;
                }
            }
            double mate1 = (mateMin >= 1.05) ? 1.0 : clamp(mateMin / 1.05, 0.0, 1.0);

            int passOptions = 0;
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null) continue;
                    if (r.id == self.id) continue;
                    boolean blocked = ScoreGrid.segmentBlockedByOpponents(r.x, r.y, x, y, world.oppRobots, 0.30);
                    if (!blocked) passOptions++;
                }
            }
            double passPts = Math.min(3, passOptions) * 1.0;

            double theirGoalX = (teamSign == +1) ? halfL : -halfL;
            boolean shootBlocked = ScoreGrid.segmentBlockedByOpponents(x, y, theirGoalX, 0.0, world.oppRobots, 0.35);
            double shoot2 = shootBlocked ? 0.0 : 2.0;

            double base = open2 + mate1 + passPts + shoot2;

            // --- Join-midfield push ---
            // Encourage stepping to just behind the ball, but NOT deep.
            // If ball is advanced past midfield, push to midfield too.
            double ballXAttack = (ball.x * teamSign);
            double xAttack = (x * teamSign);

            // Desired x position in attack direction:
            // - when ball is near midfield: be slightly behind it
            // - when ball is far advanced: be around midfield/attacking half line
            double desiredXAttack;
            if (ballXAttack < 0.30) {
                desiredXAttack = ballXAttack - 0.55; // still not camping in our third
            } else {
                desiredXAttack = Math.max(0.20, ballXAttack - 1.05);
            }
            double xErr = Math.abs(xAttack - desiredXAttack);
            double xHold = -xErr * 0.85;

            // Explicit reward for crossing midfield when the ball is already advanced
            double crossMidReward = (ballXAttack > 0.45 && xAttack > 0.0) ? 1.2 : 0.0;

            // Width: wide defender should keep width but not hug the wall.
            double yAbs = Math.abs(y);
            double idealY = clamp(halfW * 0.48, 0.75, 2.20);
            double yErr = Math.abs(yAbs - idealY);
            double widthScore = -yErr * 0.55;
            double wallPenalty = (yAbs > halfW * 0.90) ? -(yAbs - halfW * 0.90) * 1.4 : 0.0;

            // Still avoid being right on top of the ball.
            double ballD = Math.sqrt(ScoreGrid.dist2(x, y, ball.x, ball.y));
            double nearBallPenalty = (ballD < 1.10) ? -(1.10 - ballD) * 1.4 : 0.0;

            // Don't drift too close to our goal.
            double ourGoalX = (teamSign == +1) ? -halfL : halfL;
            double fromOurGoal = Math.abs(x - ourGoalX);
            double deepPenalty = (fromOurGoal < 3.1) ? -(3.1 - fromOurGoal) * 1.1 : 0.0;

            return base + xHold + crossMidReward + widthScore + wallPenalty + nearBallPenalty + deepPenalty;
        };
    }

    /**
     * Defensive off-ball scoring:
     * - Prefer positions that cut the ball -> dangerous opponent receiver line
     * - Prefer staying goal-side / between ball and our goal
     * - Prefer being close enough to influence but not clustering around ball
     */
    public static PositionScorer defendOffBall() {
        return (world, self, x, y, teamSign) -> {
            Ball ball = world.ball;
            double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
            double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

            // Assigned mark info (set per-frame in sim.Main when defending)
            double[] mark = Main.getMarkTargetForRobot(self.id);
            boolean hasMark = (mark != null);
            double markX = hasMark ? mark[0] : 0.0;
            double markY = hasMark ? mark[1] : 0.0;

            // Ball motion context
            double ballSpeed = Math.sqrt(ball.vx * ball.vx + ball.vy * ball.vy);
            double[] ballFuture = ScoreGrid.predictBallPos(world, 0.35);

            // When we have a mark, remove offense-like spacing and passability terms.
            // Marking should not be weakened by "stay open" / "stay spaced" heuristics.
            double base = 0.0;
            if (!hasMark) {
                Robot nearestOpp = ScoreGrid.closestRobot(world.oppRobots, x, y);
                double oppD = (nearestOpp == null) ? 9.0 : Math.sqrt(ScoreGrid.dist2(nearestOpp.x, nearestOpp.y, x, y));
                double open2 = (oppD >= 1.0) ? 2.0 : clamp(oppD / 1.0, 0.0, 1.0) * 2.0;

                double mateMin = 9.0;
                if (world.ourRobots != null) {
                    for (Robot r : world.ourRobots) {
                        if (r == null || r.id == self.id) continue;
                        double d = Math.sqrt(ScoreGrid.dist2(r.x, r.y, x, y));
                        if (d < mateMin) mateMin = d;
                    }
                }
                double mate1 = (mateMin >= 1.05) ? 1.0 : clamp(mateMin / 1.05, 0.0, 1.0);

                int passOptions = 0;
                if (world.ourRobots != null) {
                    for (Robot r : world.ourRobots) {
                        if (r == null) continue;
                        if (r.id == self.id) continue;
                        boolean blocked = ScoreGrid.segmentBlockedByOpponents(r.x, r.y, x, y, world.oppRobots, 0.30);
                        if (!blocked) passOptions++;
                    }
                }
                double passPts = Math.min(2, passOptions) * 1.0;

                double theirGoalX = (teamSign == +1) ? halfL : -halfL;
                boolean shootBlocked = ScoreGrid.segmentBlockedByOpponents(x, y, theirGoalX, 0.0, world.oppRobots, 0.35);
                double shoot2 = shootBlocked ? 0.0 : 1.0;

                base = open2 + mate1 + passPts + shoot2;
            }

            // ---- Shape / anti-ball-chasing terms (new) ----
            // (A) Role anchoring: prefer staying near our initial "lane" (y) to avoid everybody collapsing.
            // This is intentionally soft: just enough to keep the 3 defenders from converging.
            double laneErr = Math.abs(y - self.y);
            double laneHold = -laneErr * 0.28;

            // (B) Home-line anchoring: resist large x excursions relative to our current x when ball is fast.
            // On fast transitions, moving too much creates gaps.
            double ballVAttack = (ball.vx * teamSign);
            double speedHold = 0.0;
            if (ballSpeed > 0.45) {
                double xMove = Math.abs(x - self.x);
                speedHold = -xMove * 0.22;
            }

            // Goalside / line hold
            double ourGoalX = (teamSign == +1) ? -halfL : halfL;
            double ballToGoal = Math.abs(ball.x - ourGoalX);
            double pointToGoal = Math.abs(x - ourGoalX);
            double goalside = (ballToGoal - pointToGoal);
            double goalsideScore = clamp(goalside, -2.0, 2.0);

            // Defensive line targets: when defending, keep a line roughly 2.0..3.3m from our goal
            // but move up when ball moves up (prevents "always stay deep").
            double ballFromGoal = Math.abs(ball.x - ourGoalX);
            double desiredLineFromGoal = clamp(1.9 + 0.35 * ballFromGoal, 2.0, 5.2);
            double fromGoal = Math.abs(x - ourGoalX);
            double lineHold = -Math.abs(fromGoal - desiredLineFromGoal) * 0.55;

            // Pass-lane cutting: prefer being near the line from ball to the most advanced opponent.
            Robot threat = null;
            if (world.oppRobots != null && !world.oppRobots.isEmpty()) {
                double best = Double.NEGATIVE_INFINITY;
                for (Robot o : world.oppRobots) {
                    if (o == null) continue;
                    double adv = (o.x * teamSign);
                    if (adv > best) {
                        best = adv;
                        threat = o;
                    }
                }
            }
            double lineCut = 0.0;
            if (threat != null) {
                double[] p = ScoreGrid.closestPointOnSegment(ball.x, ball.y, threat.x, threat.y, x, y);
                double d = Math.sqrt(ScoreGrid.dist2(x, y, p[0], p[1]));
                lineCut = -clamp(d, 0.0, 2.0);
            }

            // Also consider the future ball position when the ball is rolling (reduces late reactions).
            double futureCut = 0.0;
            if (threat != null && ballSpeed > 0.25) {
                double[] p = ScoreGrid.closestPointOnSegment(ballFuture[0], ballFuture[1], threat.x, threat.y, x, y);
                double d = Math.sqrt(ScoreGrid.dist2(x, y, p[0], p[1]));
                futureCut = -clamp(d, 0.0, 2.4) * 0.45;
            }

            // Don't crowd the ball (disabled while marking).
            double ballD = Math.sqrt(ScoreGrid.dist2(x, y, ball.x, ball.y));
            double ballBandPenalty = hasMark ? 0.0 : ((ballD < 0.90) ? -(0.90 - ballD) * 1.8 : 0.0);

            // (C) Hard anti-chase: if the candidate point is very close to the ball, but the ball is fast
            // or moving toward our goal, strongly discourage running straight to the ball.
            double chaseDiscourage = 0.0;
            boolean dangerousTransition = (ballSpeed > 0.35 && ballVAttack < -0.20);
            if (ballD < 1.20 && (ballSpeed > 0.45 || dangerousTransition)) {
                chaseDiscourage = -(1.20 - ballD) * 2.2;
            }

            // When the ball is fast, defenders should prioritize keeping structure (less ball-chasing).
            double speedStructure = (ballSpeed > 0.45) ? 0.25 : 0.0;

            // Smoothness
            double moveCost = Math.sqrt(ScoreGrid.dist2(x, y, self.x, self.y));
            double movePenalty = -0.25 * moveCost;

            // Keep some width but avoid extreme corners.
            double yNorm = Math.abs(y) / halfW;
            double widthHold = -(Math.max(0.0, yNorm - 0.88)) * 0.7;

            // --- Man-mark shaping (new) ---
            // If we have an assigned mark, stay somewhat close to them while keeping goal-side.
            // We don't force "stand on top"; instead we bias the score so different defenders
            // naturally cover different opponents and stop collapsing.
            double markBias = 0.0;
            double markLaneCut = 0.0;
            double markGoalSide = 0.0;
            double markLaneSeparate = 0.0;
            if (hasMark) {
                double dMark = Math.sqrt(ScoreGrid.dist2(x, y, markX, markY));
                // Prefer being 0.8..1.8m from the mark (close enough to contest, not colliding)
                double desired = 1.25;
                double err = Math.abs(dMark - desired);
                // Keep this relatively soft; too strong causes everyone to converge on similar lane-cut points.
                markBias = -(Math.max(0.0, err - 0.70)) * 0.55;

                // Be between mark and our goal (goal-side). Penalize being "behind" the mark.
                double ourGoalX2 = (teamSign == +1) ? -halfL : halfL;
                double markToGoal = Math.abs(markX - ourGoalX2);
                double pointToGoal2 = Math.abs(x - ourGoalX2);
                // If pointToGoal2 > markToGoal => we're farther from goal than the mark => not goal-side
                double notGoalSide = pointToGoal2 - markToGoal;
                markGoalSide = (notGoalSide > 0.0) ? -clamp(notGoalSide, 0.0, 2.0) * 1.05 : 0.0;

                // Cut "ball -> mark" passing lane a bit.
                double[] lp = ScoreGrid.closestPointOnSegment(ball.x, ball.y, markX, markY, x, y);
                double dl = Math.sqrt(ScoreGrid.dist2(x, y, lp[0], lp[1]));
                markLaneCut = -clamp(dl, 0.0, 2.2) * 0.45;

                // Encourage defenders to cover slightly different offsets around the mark-lane.
                // This uses current lane sign as a tie-breaker (keeps our 3 DF spread).
                double laneSign = (self.y >= 0.0) ? 1.0 : -1.0;
                double yLane = y - markY;
                double desiredLane = laneSign * 0.55;
                markLaneSeparate = -Math.abs(yLane - desiredLane) * 0.18;
            }

            return base
                    + 1.10 * goalsideScore
                    + 0.95 * lineHold
                    + 0.85 * lineCut
            + futureCut
            + laneHold
            + speedHold
                    + widthHold
            + (1.0 + speedStructure) * movePenalty
            + ballBandPenalty
            + chaseDiscourage
            + markBias
            + markGoalSide
            + markLaneCut
            + markLaneSeparate;
        };
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
