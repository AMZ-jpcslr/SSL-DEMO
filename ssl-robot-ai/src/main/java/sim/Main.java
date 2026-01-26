// sim/Main.java
package sim;

import ai.Behavior;
import ai.DefenderBehavior;
import ai.PasserAttackerBehavior;
import ai.RobotCommand;
import ai.SimpleStriker;
import ai.SupporterBehavior;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.swing.*;
import tactics.GridPoint;
import tactics.PositionLearning;
import tactics.PositionScorer;
import tactics.ScoreGrid;
import tactics.TacticalScorers;
import ui.FieldConfig;
import ui.FieldPanel;
import world.Ball;
import world.Robot;
import world.WorldState;

public class Main {
    // Debug hooks for UI overlay (FieldPanel uses reflection to read these).
    // { blueX, blueY, redX, redY }
    private static final double[] DEBUG_TARGETS = new double[] { 0.0, 0.0, 0.0, 0.0 };

    // Store intended off-ball targets per robot each frame, so we can deconflict against
    // teammates' planned destinations (not just their current positions).
    // Index by robotId directly (ids are small: 0..5 and 10..15 by default).
    private static final double[] PLANNED_TX = new double[32];
    private static final double[] PLANNED_TY = new double[32];
    private static final boolean[] PLANNED_HAS = new boolean[32];

    // Per-frame marking assignment (defense only). Index by robotId.
    // If HAS=false, robot has no active mark.
    private static final double[] MARK_TX = new double[32];
    private static final double[] MARK_TY = new double[32];
    private static final boolean[] MARK_HAS = new boolean[32];

    // Online learning hook for pass selection.
    // We record the most recent tagged pass attempt and reward it based on whether
    // the intended receiver actually gets possession.
    private static long LAST_PASS_AT_NANOS = 0L;
    private static int LAST_PASS_TEAM = 0;         // +1 blue, -1 red
    private static int LAST_PASS_FROM_ID = -1;
    private static int LAST_PASS_TO_ID = -1;
    private static double LAST_PASS_START_X = 0.0;
    private static double[] LAST_PASS_FEATURES = null;

    // Online learning hook for shoot-vs-pass.
    private static long LAST_ACTION_AT_NANOS = 0L;
    private static int LAST_ACTION_TEAM = 0; // +1 blue, -1 red
    private static int LAST_ACTION_FROM_ID = -1;
    private static int LAST_ACTION_PASS_TO_ID = -1; // only when action was pass
    private static boolean LAST_ACTION_SHOOT = false;
    private static double LAST_ACTION_START_X = 0.0;
    private static double[] LAST_ACTION_FEATURES = null;

    // Simple scoring + goal reward signals.
    private static int SCORE_BLUE = 0;
    private static int SCORE_RED = 0;
    private static int GOAL_PENDING_TEAM = 0;

    // Per-frame tactical context: when the ball-winner is attempting a pass, encourage teammates
    // to spread wide and avoid clustering for short triangles.
    private static boolean TEAM_PASSING_BLUE = false;
    private static boolean TEAM_PASSING_RED = false;

    // Per-frame tactical context: when our team is likely to regain the ball soon (and not contested),
    // prepare by spreading for the next pass/move.
    private static boolean TEAM_REGAIN_SOON_BLUE = false;
    private static boolean TEAM_REGAIN_SOON_RED = false;

    // Online learning hook for off-ball positioning: store last chosen features per robot.
    private static final double[][] LAST_ATTACK_POS_FEATURES = new double[32][];
    private static final long[] LAST_ATTACK_POS_AT_NANOS = new long[32];
    private static final double[][] LAST_DEF_POS_FEATURES = new double[32][];
    private static final long[] LAST_DEF_POS_AT_NANOS = new long[32];

    private static void recordPassAttempt(int fromId,
                                          int teamSign,
                                          int toId,
                                          double ballX,
                                          double[] features) {
        LAST_PASS_AT_NANOS = System.nanoTime();
        LAST_PASS_TEAM = teamSign;
        LAST_PASS_FROM_ID = fromId;
        LAST_PASS_TO_ID = toId;
        LAST_PASS_START_X = ballX;
        LAST_PASS_FEATURES = features;
    }

    private static void recordActionAttempt(int fromId,
                                            int teamSign,
                                            boolean shoot,
                                            int passToId,
                                            double ballX,
                                            double[] features) {
        LAST_ACTION_AT_NANOS = System.nanoTime();
        LAST_ACTION_TEAM = teamSign;
        LAST_ACTION_FROM_ID = fromId;
        LAST_ACTION_SHOOT = shoot;
        LAST_ACTION_PASS_TO_ID = passToId;
        LAST_ACTION_START_X = ballX;
        LAST_ACTION_FEATURES = features;
    }

    private static void clearLastAction() {
        LAST_ACTION_AT_NANOS = 0L;
        LAST_ACTION_TEAM = 0;
        LAST_ACTION_FROM_ID = -1;
        LAST_ACTION_PASS_TO_ID = -1;
        LAST_ACTION_SHOOT = false;
        LAST_ACTION_FEATURES = null;
    }

    private static void maybeTimeoutLastAction(int ballOwnerTeam, double ballX) {
        if (LAST_ACTION_TEAM == 0 || LAST_ACTION_FEATURES == null) return;
        long now = System.nanoTime();
        long window = (long) (1.75e9);
        if (now - LAST_ACTION_AT_NANOS <= window) return;

        // If the opponent got the ball, this should have been handled elsewhere.
        if (ballOwnerTeam != 0 && ballOwnerTeam != LAST_ACTION_TEAM) {
            ai.ActionLearning.applyReward(LAST_ACTION_SHOOT, -1.0, LAST_ACTION_FEATURES);
            clearLastAction();
            return;
        }

        // Otherwise, score by forward progress.
        double prog = (ballX - LAST_ACTION_START_X) * LAST_ACTION_TEAM;
        double reward = clamp(prog * 0.10, -0.6, 0.6);
        ai.ActionLearning.applyReward(LAST_ACTION_SHOOT, reward, LAST_ACTION_FEATURES);
        clearLastAction();
    }

    private static void applyTeamAttackPositionReward(WorldState world, int teamSign, double reward) {
        if (world == null) return;
        long now = System.nanoTime();
        long window = (long) (2.0e9);
        java.util.List<Robot> team = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        if (team == null) return;
        for (Robot r : team) {
            if (r == null) continue;
            int id = r.id;
            if (id < 0 || id >= LAST_ATTACK_POS_FEATURES.length) continue;
            if (LAST_ATTACK_POS_FEATURES[id] == null) continue;
            if (now - LAST_ATTACK_POS_AT_NANOS[id] > window) continue;
            PositionLearning.applyAttackReward(reward, LAST_ATTACK_POS_FEATURES[id]);
        }
    }

    private static void applyTeamDefensePositionReward(WorldState world, int teamSign, double reward) {
        if (world == null) return;
        long now = System.nanoTime();
        long window = (long) (2.0e9);
        java.util.List<Robot> team = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        if (team == null) return;
        for (Robot r : team) {
            if (r == null) continue;
            int id = r.id;
            if (id < 0 || id >= LAST_DEF_POS_FEATURES.length) continue;
            if (LAST_DEF_POS_FEATURES[id] == null) continue;
            if (now - LAST_DEF_POS_AT_NANOS[id] > window) continue;
            PositionLearning.applyDefenseReward(reward, LAST_DEF_POS_FEATURES[id]);
        }
    }

    private static void maybeRewardLastPass(WorldState world, int newOwnerId, int newOwnerTeam, double ballX) {
        if (LAST_PASS_TO_ID < 0 || LAST_PASS_TEAM == 0) return;
        long now = System.nanoTime();
        long window = (long) (1.25e9);
        if (now - LAST_PASS_AT_NANOS > window) {
            // Timed out without a clear successful reception.
            if (LAST_PASS_FEATURES != null) {
                ai.PassLearning.applyReward(-1.0, LAST_PASS_FEATURES);
            }
            if (LAST_ACTION_TEAM == LAST_PASS_TEAM && !LAST_ACTION_SHOOT && LAST_ACTION_FEATURES != null) {
                ai.ActionLearning.applyReward(false, -0.8, LAST_ACTION_FEATURES);
                clearLastAction();
            }
            LAST_PASS_TO_ID = -1;
            LAST_PASS_TEAM = 0;
            LAST_PASS_FROM_ID = -1;
            LAST_PASS_FEATURES = null;
            return;
        }

        // If the opponent gained possession soon after the pass, it's a failure.
        if (newOwnerTeam != 0 && newOwnerTeam != LAST_PASS_TEAM) {
            if (LAST_PASS_FEATURES != null) {
                ai.PassLearning.applyReward(-1.0, LAST_PASS_FEATURES);
            }
            if (LAST_ACTION_TEAM == LAST_PASS_TEAM && !LAST_ACTION_SHOOT && LAST_ACTION_FEATURES != null) {
                ai.ActionLearning.applyReward(false, -1.0, LAST_ACTION_FEATURES);
                clearLastAction();
            }
            LAST_PASS_TO_ID = -1;
            LAST_PASS_TEAM = 0;
            LAST_PASS_FROM_ID = -1;
            LAST_PASS_FEATURES = null;
            return;
        }

        // If the intended receiver got it, reward by success + forward progress.
        if (newOwnerTeam == LAST_PASS_TEAM && newOwnerId == LAST_PASS_TO_ID) {
            double prog = (ballX - LAST_PASS_START_X) * LAST_PASS_TEAM;
            double reward = 1.0 + clamp(prog * 0.20, -1.0, 1.0);
            if (LAST_PASS_FEATURES != null) {
                ai.PassLearning.applyReward(reward, LAST_PASS_FEATURES);
            }
            // Also reward the action choice (pass) that led to this outcome.
            if (LAST_ACTION_TEAM == LAST_PASS_TEAM && !LAST_ACTION_SHOOT && LAST_ACTION_FEATURES != null
                    && LAST_ACTION_FROM_ID == LAST_PASS_FROM_ID && LAST_ACTION_PASS_TO_ID == LAST_PASS_TO_ID) {
                ai.ActionLearning.applyReward(false, reward, LAST_ACTION_FEATURES);
                clearLastAction();
            }

            // Reward the team's attacking off-ball positions lightly on successful receptions.
            applyTeamAttackPositionReward(world, LAST_PASS_TEAM, 0.22 + clamp(prog * 0.05, -0.25, 0.35));

            LAST_PASS_TO_ID = -1;
            LAST_PASS_TEAM = 0;
            LAST_PASS_FROM_ID = -1;
            LAST_PASS_FEATURES = null;
        }
    }

    private static void onGoal(WorldState world, int scoringTeam) {
        if (scoringTeam == +1) SCORE_BLUE++;
        if (scoringTeam == -1) SCORE_RED++;
        System.out.println("GOAL! scoringTeam=" + scoringTeam + "  score BLUE=" + SCORE_BLUE + " RED=" + SCORE_RED);

        // Reward/penalize the most recent action.
        if (LAST_ACTION_TEAM != 0 && LAST_ACTION_FEATURES != null) {
            double r = (LAST_ACTION_TEAM == scoringTeam) ? 2.0 : -2.0;
            ai.ActionLearning.applyReward(LAST_ACTION_SHOOT, r, LAST_ACTION_FEATURES);
            clearLastAction();
        }

        // Reward team shapes.
        applyTeamAttackPositionReward(world, scoringTeam, +0.6);
        applyTeamDefensePositionReward(world, scoringTeam, +0.3);
        applyTeamAttackPositionReward(world, -scoringTeam, -0.4);
        applyTeamDefensePositionReward(world, -scoringTeam, -0.8);
    }

    /** Return current mark target for a robot id, or null if none (per-frame). */
    public static double[] getMarkTargetForRobot(int robotId) {
        if (robotId < 0 || robotId >= MARK_HAS.length) return null;
        if (!MARK_HAS[robotId]) return null;
        return new double[] { MARK_TX[robotId], MARK_TY[robotId] };
    }

    public static double[] getDebugTargets() {
        return DEBUG_TARGETS;
    }

    /** Returns current score as {blue, red}. Used by FieldPanel via reflection. */
    public static int[] getScore() {
        return new int[] { SCORE_BLUE, SCORE_RED };
    }

    /**
     * Returns a snapshot of planned per-robot targets for the current frame.
     * Index is robotId (0..31). If a robot has no planned target, the entry is null.
     * Each non-null entry is {x, y} in field meters.
     */
    public static double[][] getPlannedTargets() {
        double[][] out = new double[PLANNED_HAS.length][];
        for (int i = 0; i < PLANNED_HAS.length; i++) {
            if (!PLANNED_HAS[i]) continue;
            out[i] = new double[] { PLANNED_TX[i], PLANNED_TY[i] };
        }
        return out;
    }

    /**
     * Returns a snapshot of per-robot mark targets for the current frame.
     * Index is robotId (0..31). If no mark, the entry is null.
     * Each non-null entry is {x, y} in field meters.
     */
    public static double[][] getMarkTargets() {
        double[][] out = new double[MARK_HAS.length][];
        for (int i = 0; i < MARK_HAS.length; i++) {
            if (!MARK_HAS[i]) continue;
            out[i] = new double[] { MARK_TX[i], MARK_TY[i] };
        }
        return out;
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {

            // ----- WorldState を準備 -----
            WorldState world = new WorldState();

            // ボールの初期位置（センター）
            world.ball = new Ball(0.0, 0.0);

            // ★★ ここでロボットの位置を決める ★★

            // --- 6v6: GK + (3-2) formation ---
            // Convention:
            // - Blue (ourRobots) defends left goal (-x), attacks +x
            // - Red  (oppRobots) defends right goal (+x), attacks -x
            resetFormation(world);

            // ----- Window を開く -----
            JFrame frame = new JFrame("SSL Field View");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            FieldPanel panel = new FieldPanel(world);
            frame.add(panel);

            frame.setSize(1200, 800);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            // ----- Simple simulation loop (60 FPS) -----
            // Always AI control for all robots.
            final double dt = 1.0 / 60.0;
            final AtomicBoolean running = new AtomicBoolean(true);

            // Roles (for when runAllOurRobots = true)
            final Behavior attacker = new PasserAttackerBehavior(+1);
            final Behavior defender = new DefenderBehavior();
            final Behavior supporter = new SupporterBehavior(+1);

            // Opponent roles (red team)
            // - Attacker: reuse SimpleStriker but run in mirrored coordinates (so it also "attacks" +x)
            // - Defender: defend the right goal (+x)
            final Behavior oppAttacker = new PasserAttackerBehavior(+1); // used in mirrored frame
            // NOTE: Opponent is run in mirrored coordinates, where it attacks toward +x.
            // So the opponent behaviors should be configured the same way as our team (+1).
            final Behavior oppDefender = new DefenderBehavior(+1);
            final Behavior oppSupporter = new SupporterBehavior(+1);

            // Unused now (we run full AI), but keep for experimentation.
            final Behavior singleRobotAttacker = new SimpleStriker();

            // --- Role stabilization (avoid rapid attacker switching) ---
            // We keep the current "attackerId" for a short time before allowing a switch.
            final int[] ourAttackerId = new int[] { 0 };
            final int[] oppAttackerId = new int[] { 10 };
            final long[] lastSwitchNanosOur = new long[] { System.nanoTime() };
            final long[] lastSwitchNanosOpp = new long[] { System.nanoTime() };
            final long holdNanos = (long) (0.6e9); // 0.6s

            // --- Simple possession / dribble model ---
            // When a robot gets close to a (slow) ball, it can "hold" it (carry) so contested touches
            // don't endlessly bounce the ball away. This is intentionally simple and tunable.
            final int[] ballOwnerId = new int[] { -1 };   // -1: free
            final int[] ballOwnerTeam = new int[] { 0 };  // +1: blue, -1: red

            // If a robot loses the ball and an opponent takes it, prevent the loser from re-taking
            // for a short time to avoid endless corner oscillations.
            final java.util.Map<Integer, Long> pickupBanUntilNanos = new java.util.HashMap<>();
            final int[] recentlyLostOwnerId = new int[] { -1 };
            final int[] recentlyLostOwnerTeam = new int[] { 0 };
            final long[] recentlyLostAtNanos = new long[] { -1L };

            // Goalkeeper special: allow GK to carry the ball for a short time, during which
            // the opponent cannot take the ball from the GK.
            final long gkHoldNanos = (long) (2.0e9); // 2.0 seconds
            final long[] gkHoldUntilNanos = new long[] { 0L };
            final int[] gkHoldOwnerId = new int[] { -1 };
            final int[] gkHoldOwnerTeam = new int[] { 0 };

            // Rare deadlock breaker: if the ball is stuck in a close contest for too long,
            // randomly award possession to one of the contesting robots.
            final long[] stuckSinceNanos = new long[] { -1 };
            final double[] lastBallX = new double[] { world.ball.x };
            final double[] lastBallY = new double[] { world.ball.y };

            // Track robot motion between frames to reduce false "stuck" detections.
            final double[] lastRobotX = new double[32];
            final double[] lastRobotY = new double[32];
            final boolean[] lastRobotHas = new boolean[32];

            frame.addKeyListener(new KeyAdapter() {
                @Override
                public void keyPressed(KeyEvent e) {
                    // Space: start/stop
                    if (e.getKeyCode() == KeyEvent.VK_SPACE) {
                        running.set(!running.get());
                        System.out.println("running = " + running.get());
                        return;
                    }

                    // R: reset positions + ball
                    if (e.getKeyCode() == KeyEvent.VK_R) {
                        resetFormation(world);
                        world.ball.x = 0.0;
                        world.ball.y = 0.0;
                        world.ball.vx = 0.0;
                        world.ball.vy = 0.0;
                        ballOwnerId[0] = -1;
                        ballOwnerTeam[0] = 0;
                        gkHoldUntilNanos[0] = 0L;
                        gkHoldOwnerId[0] = -1;
                        gkHoldOwnerTeam[0] = 0;
                        stuckSinceNanos[0] = -1;
                        lastBallX[0] = world.ball.x;
                        lastBallY[0] = world.ball.y;
                        java.util.Arrays.fill(lastRobotHas, false);
                        System.out.println("RESET");
                    }

                    // 1: place ball near Blue GK (for symmetric GK-catch testing)
                    if (e.getKeyCode() == KeyEvent.VK_1) {
                        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
                        world.ball.x = -halfL + 0.75;
                        world.ball.y = 0.0;
                        world.ball.vx = 0.0;
                        world.ball.vy = 0.0;
                        ballOwnerId[0] = -1;
                        ballOwnerTeam[0] = 0;
                        gkHoldUntilNanos[0] = 0L;
                        gkHoldOwnerId[0] = -1;
                        gkHoldOwnerTeam[0] = 0;
                        stuckSinceNanos[0] = -1;
                        lastBallX[0] = world.ball.x;
                        lastBallY[0] = world.ball.y;
                        System.out.println("BALL -> near BLUE GK");
                        return;
                    }

                    // 2: place ball near Red GK (for symmetric GK-catch testing)
                    if (e.getKeyCode() == KeyEvent.VK_2) {
                        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
                        world.ball.x = halfL - 0.75;
                        world.ball.y = 0.0;
                        world.ball.vx = 0.0;
                        world.ball.vy = 0.0;
                        ballOwnerId[0] = -1;
                        ballOwnerTeam[0] = 0;
                        gkHoldUntilNanos[0] = 0L;
                        gkHoldOwnerId[0] = -1;
                        gkHoldOwnerTeam[0] = 0;
                        stuckSinceNanos[0] = -1;
                        lastBallX[0] = world.ball.x;
                        lastBallY[0] = world.ball.y;
                        System.out.println("BALL -> near RED GK");
                        return;
                    }
                }
            });
            frame.setFocusable(true);
            frame.requestFocusInWindow();

            int delayMs = (int) Math.round(dt * 1000.0);
            new Timer(delayMs, e -> {
                if (!running.get()) {
                    panel.repaint();
                    return;
                }

                // Reset planned target cache for this frame.
                java.util.Arrays.fill(PLANNED_HAS, false);
                java.util.Arrays.fill(MARK_HAS, false);

                // Reset per-frame pass-intent flags.
                TEAM_PASSING_BLUE = false;
                TEAM_PASSING_RED = false;
                TEAM_REGAIN_SOON_BLUE = false;
                TEAM_REGAIN_SOON_RED = false;

                // Pre-compute marking assignments (defense only).
                // Blue is defending when ball.x <= 0; Red is defending when ball.x >= 0.
                if (!isAttackingWithTeamSign(world, +1)) {
                    assignMarks(world, +1, ballOwnerId, ballOwnerTeam);
                }
                if (!isAttackingWithTeamSign(world, -1)) {
                    assignMarks(world, -1, ballOwnerId, ballOwnerTeam);
                }

                // 1) Decide + 2) Apply
                // --- Our team (blue) ---
                Robot ourClosest = findClosestRobot(world.ourRobots, world.ball);
                Robot oppClosestToBall = findClosestRobot(world.oppRobots, world.ball);
                long now = System.nanoTime();
                if (shouldSwitchAttacker(now, lastSwitchNanosOur[0], holdNanos,
                        ourAttackerId[0], ourClosest, world.ball, world.ourRobots)) {
                    ourAttackerId[0] = ourClosest.id;
                    lastSwitchNanosOur[0] = now;
                }

                // Precompute ball-winner command first so other robots can react (spread) in the same frame.
                RobotCommand ourWinnerCmd = null;
                int ourWinnerId = (ourClosest == null) ? -1 : ourClosest.id;
                if (ourClosest != null && !isGoalkeeper(ourClosest)) {
                    ourWinnerCmd = attacker.decide(ourClosest, world);
                    TEAM_PASSING_BLUE = (ourWinnerCmd != null && ourWinnerCmd.kick && ourWinnerCmd.passTargetId >= 0 && !ourWinnerCmd.shotIntent);
                }

                // If we are likely to regain the ball soon (and it's not a close contest), spread early.
                // Exclude true contests (opponent close enough to challenge).
                if (world.ball != null && ourClosest != null && !isGoalkeeper(ourClosest) && ballOwnerTeam[0] != +1) {
                    double dOur = Math.sqrt(dist2(ourClosest.x, ourClosest.y, world.ball.x, world.ball.y));
                    double dOpp = (oppClosestToBall == null) ? 9.0 : Math.sqrt(dist2(oppClosestToBall.x, oppClosestToBall.y, world.ball.x, world.ball.y));
                    // "Likely regain" when we clearly arrive first and the opponent isn't close enough to contest.
                    boolean opponentClose = dOpp <= 0.75;
                    boolean weArriveSoon = dOur <= 0.75;
                    boolean clearLead = (dOur + 0.18) < dOpp;
                    TEAM_REGAIN_SOON_BLUE = weArriveSoon && clearLead && !opponentClose;
                }

                for (Robot r : world.ourRobots) {
                    // GK: stay defender always
                    boolean isGK = isGoalkeeper(r);
                    boolean isBallWinner = (ourClosest != null && r.id == ourClosest.id);
                    boolean isBackup = isSecondClosestRobot(world.ourRobots, world.ball, r);

                    Behavior b;
                    if (isGK) {
                        b = new DefenderBehavior(+1);
                    } else if (isBallWinner) {
                        b = attacker;
                    } else {
                        // Tactical off-ball roles: create passing lanes (attack) or mark/cut lanes (defense).
                        b = selectTacticalOffBallBehaviorForOur(r, world, supporter, defender);
                    }

                    RobotCommand cmd;
                    if (isBallWinner && ourWinnerCmd != null && r.id == ourWinnerId) {
                        cmd = ourWinnerCmd;
                    } else {
                        cmd = b.decide(r, world);
                    }
                    // Extra tactical positioning beyond the base Behavior
                    if (isGK) {
                        cmd = applyGoalkeeperConstraints(cmd, r, world, +1);
                    } else if (!isBallWinner) {
                        cmd = applyTacticalOffBallAdjustment(cmd, r, world, +1);
                    }
                    if (!isGK) {
                        if (!isBallWinner && isBackup) {
                            cmd = applyBackupSupport(cmd, r, world, +1);
                        }
                        if (!isBallWinner) {
                            applyDeconflictBias(cmd, r, world, +1);
                        }
                    }
                    applyCommand(world, r, cmd, dt, +1, ballOwnerId, ballOwnerTeam);
                }

                // --- Opponent team (red) ---
                WorldState mWorld = mirrorWorld(world);
                Robot oppClosestM = findClosestRobot(mWorld.ourRobots, mWorld.ball);
                Robot blueClosestToBallM = findClosestRobot(mWorld.oppRobots, mWorld.ball);
                if (shouldSwitchAttacker(now, lastSwitchNanosOpp[0], holdNanos,
                        oppAttackerId[0], oppClosestM, mWorld.ball, mWorld.ourRobots)) {
                    oppAttackerId[0] = oppClosestM.id;
                    lastSwitchNanosOpp[0] = now;
                }

                // Precompute opponent ball-winner (in mirrored frame) first.
                RobotCommand oppWinnerCmdM = null;
                int oppWinnerId = (oppClosestM == null) ? -1 : oppClosestM.id;
                if (oppClosestM != null && !isGoalkeeper(oppClosestM)) {
                    oppWinnerCmdM = oppAttacker.decide(oppClosestM, mWorld);
                    TEAM_PASSING_RED = (oppWinnerCmdM != null && oppWinnerCmdM.kick && oppWinnerCmdM.passTargetId >= 0 && !oppWinnerCmdM.shotIntent);
                }

                // Regain-soon for Red team in mirrored frame.
                if (mWorld.ball != null && oppClosestM != null && !isGoalkeeper(oppClosestM) && ballOwnerTeam[0] != -1) {
                    double dRed = Math.sqrt(dist2(oppClosestM.x, oppClosestM.y, mWorld.ball.x, mWorld.ball.y));
                    double dBlue = (blueClosestToBallM == null) ? 9.0 : Math.sqrt(dist2(blueClosestToBallM.x, blueClosestToBallM.y, mWorld.ball.x, mWorld.ball.y));
                    boolean opponentClose = dBlue <= 0.75;
                    boolean weArriveSoon = dRed <= 0.75;
                    boolean clearLead = (dRed + 0.18) < dBlue;
                    TEAM_REGAIN_SOON_RED = weArriveSoon && clearLead && !opponentClose;
                }

                for (Robot r : world.oppRobots) {
                    boolean isGK = isGoalkeeper(r);

                    Robot mr = mirrorRobot(r);
                    Behavior b;
                    boolean isBallWinnerM = (oppClosestM != null && r.id == oppClosestM.id);
                    boolean isBackupM = isSecondClosestRobot(mWorld.ourRobots, mWorld.ball, mr);
                    if (isGK) {
                        // In mirrored frame, GK defends the left goal (same as our team).
                        b = new DefenderBehavior(+1);
                    } else if (isBallWinnerM) {
                        b = oppAttacker;
                    } else {
                        b = selectTacticalOffBallBehaviorForOur(mr, mWorld, oppSupporter, oppDefender);
                    }

                    RobotCommand cmd = b.decide(mr, mWorld);
                    if (isBallWinnerM && oppWinnerCmdM != null && mr.id == oppWinnerId) {
                        cmd = oppWinnerCmdM;
                    }
                    if (isGK) {
                        cmd = applyGoalkeeperConstraints(cmd, mr, mWorld, +1);
                    } else if (!isBallWinnerM) {
                        cmd = applyTacticalOffBallAdjustment(cmd, mr, mWorld, +1);
                    }
                    if (!isGK) {
                        if (!isBallWinnerM && isBackupM) {
                            cmd = applyBackupSupport(cmd, mr, mWorld, +1);
                        }
                        if (!isBallWinnerM) {
                            applyDeconflictBias(cmd, mr, mWorld, +1);
                        }
                    }
                    applyCommand(world, r, unmirrorCommand(cmd), dt, -1, ballOwnerId, ballOwnerTeam);
                }

                // 3) Integrate ball motion
                integrateBall(world, dt);

                // If a goal was detected during ball integration, reset state cleanly here.
                if (GOAL_PENDING_TEAM != 0) {
                    resetFormation(world);
                    world.ball.x = 0.0;
                    world.ball.y = 0.0;
                    world.ball.vx = 0.0;
                    world.ball.vy = 0.0;
                    ballOwnerId[0] = -1;
                    ballOwnerTeam[0] = 0;
                    gkHoldUntilNanos[0] = 0L;
                    gkHoldOwnerId[0] = -1;
                    gkHoldOwnerTeam[0] = 0;
                    stuckSinceNanos[0] = -1;
                    lastBallX[0] = world.ball.x;
                    lastBallY[0] = world.ball.y;
                    java.util.Arrays.fill(lastRobotHas, false);
                    GOAL_PENDING_TEAM = 0;
                    panel.repaint();
                    return;
                }

                // 3.5) Ball-robot collision for all robots (both teams)
                // If someone currently "owns" the ball, exclude that robot so the carry model isn't undone.
                resolveBallRobotCollisions(world, ballOwnerId[0], ballOwnerTeam[0]);

                // 3.6) Robot-robot collision (prevent robot overlap)
                resolveRobotRobotCollisions(world);

                // 3.7) Possession / dribble: attach ball to an owner when controllable
                // Run after collisions so robots can actually "reach" the ball.
                updatePossessionAndDribble(world, dt, ballOwnerId, ballOwnerTeam,
                    pickupBanUntilNanos, recentlyLostOwnerId, recentlyLostOwnerTeam, recentlyLostAtNanos,
                    gkHoldNanos, gkHoldUntilNanos, gkHoldOwnerId, gkHoldOwnerTeam);

                // Resolve timeouts for shoot-vs-pass learning (progress-based fallback).
                maybeTimeoutLastAction(ballOwnerTeam[0], world.ball.x);

                // 3.75) Break rare stuck contests by randomly assigning possession
                maybeBreakStuckContest(world, ballOwnerId, ballOwnerTeam, stuckSinceNanos, lastBallX, lastBallY,
                    lastRobotX, lastRobotY, lastRobotHas,
                    gkHoldUntilNanos, gkHoldOwnerId);

                // 4) Render
                panel.repaint();
            }).start();
        });
    }

    /**
     * Off-ball tactical selection (for the "our" side in the passed world frame).
     * - If attacking: spread into pass lanes (SupporterBehavior)
     * - If defending: cut pass lanes / mark receivers (DefenderBehavior)
     */
    private static Behavior selectTacticalOffBallBehaviorForOur(Robot self,
                                                                WorldState world,
                                                                Behavior supporter,
                                                                Behavior defender) {
        if (self == null || world == null || world.ball == null) return defender;
        boolean attacking = isAttackingWithTeamSign(world, +1);
        // When attacking, we want almost everyone to behave like an off-ball receiver/runner.
        // The actual positioning (rest-defense vs push-up) is handled by applyTacticalOffBallAdjustment
        // via the scorer selection.
        return attacking ? supporter : defender;
    }

    /**
     * Goalkeeper should not join midfield contests: keep it near its goal and inside a small box.
     * This is applied as a post-processing constraint on cmd.
     */
    private static RobotCommand applyGoalkeeperConstraints(RobotCommand cmd,
                                                           Robot self,
                                                           WorldState world,
                                                           int teamSign) {
        if (cmd == null) cmd = new RobotCommand();
        if (self == null || world == null || world.ball == null) return cmd;

        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;

        // Allowed GK box (defensive area)
        double boxDepth = 1.15; // meters from goal line into field
        double boxHalfW = 1.05; // meters

        double ownGoalX = -halfL * teamSign;
        double fieldDirFromGoal = teamSign; // direction from our goal into the field

        // GK home X: slightly in front of goal line (inside field)
        double homeX = ownGoalX + fieldDirFromGoal * 0.35;
        double homeY = 0.0;

        // Clamp range inside the GK box
        double edgeX = ownGoalX + fieldDirFromGoal * FieldConfig.ROBOT_RADIUS_M;
        double deepX = ownGoalX + fieldDirFromGoal * boxDepth;
        double minX = Math.min(edgeX, deepX);
        double maxX = Math.max(edgeX, deepX);
        double minY = -boxHalfW;
        double maxY = boxHalfW;

        boolean ballInGKBox = isBallInGKBox(world.ball, teamSign);

        double targetX;
        double targetY;

        if (ballInGKBox) {
            // If the ball is in our box, go to the ball (so the GK can actually take possession).
            targetX = clamp(world.ball.x, minX, maxX);
            targetY = clamp(world.ball.y, minY, maxY);
        } else {
            // Otherwise, stay between the ball and our goal, inside the box.
            double towardGoalX = (world.ball.x + ownGoalX) * 0.5;
            double towardGoalY = world.ball.y * 0.7;
            targetX = clamp(towardGoalX, minX, maxX);
            targetY = clamp(towardGoalY, minY, maxY);

            // If ball is far away (deep in opponent half), just go home.
            if ((world.ball.x * teamSign) > 0.5) {
                targetX = homeX;
                targetY = homeY;
            }
        }

        double dx = targetX - self.x;
        double dy = targetY - self.y;
        double d = Math.sqrt(dx * dx + dy * dy);
        if (d > 1e-6) {
            double speed = ballInGKBox ? 1.7 : 1.4;
            cmd.vx = (dx / d) * speed;
            cmd.vy = (dy / d) * speed;
        } else {
            cmd.vx = 0;
            cmd.vy = 0;
        }
        cmd.omega = 0;
        // GK only kicks if it's already in the kick range (handled in DefenderBehavior); don't force it.
        return cmd;
    }

    private static boolean isBallInGKBox(Ball ball, int teamSign) {
        if (ball == null) return false;
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double boxDepth = 1.15;
        double boxHalfW = 1.05;

        double ownGoalX = -halfL * teamSign;
        double fieldDirFromGoal = teamSign;

        double xFromGoal = (ball.x - ownGoalX) * fieldDirFromGoal;
        if (xFromGoal < 0.0 || xFromGoal > boxDepth + 0.05) return false;
        return Math.abs(ball.y) <= (boxHalfW + 0.05);
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
        t = Math.max(0.0, Math.min(1.0, t));
        return new double[] { ax + t * abx, ay + t * aby };
    }

    /**
     * Tactical off-ball adjustment:
     * - When attacking: move to a "pass-receive" point that opens a lane away from nearest defender.
     * - When defending: mark the most threatening receiver (opponent closest to ball, excluding their ball-winner).
     */
    private static RobotCommand applyTacticalOffBallAdjustment(RobotCommand cmd,
                                                               Robot self,
                                                               WorldState world,
                                                               int teamSign) {
        if (cmd == null) cmd = new RobotCommand();
        if (self == null || world == null || world.ball == null) return cmd;
        cmd.robotId = self.id;

        Ball ball = world.ball;
        boolean attacking = isAttackingWithTeamSign(world, teamSign);

    // If the ball is already in the opponent half, we want the *side defenders* to join the attack
    // and behave like the front roles (receive / create lanes).
    // NOTE: we still keep exactly one safety as rest-defense (typically the center defender).
    boolean ballInOppHalf = (teamSign == +1) ? (ball.x > 0.0) : (ball.x < 0.0);

        // When attacking (but not yet deep), we still need "rest-defense": keep exactly ONE robot as the safety.
        // Option A: choose the deepest (closest to our goal) non-GK robot as the rest-defender.
        // This works for both teams because red runs in the mirrored frame.
        int restDefId;
        if (!attacking) {
            restDefId = -1;
        } else if (ballInOppHalf) {
            // Force the rest-defender to be the "center defender" so BOTH side defenders join the attack.
            restDefId = findCenterRestDefenderId(world, teamSign);
        } else {
            // Normal case: choose the deepest robot as safety.
            restDefId = findRestDefenderId(world, teamSign);
        }
        boolean isRestDefender = (self.id == restDefId);

        if (attacking) {
            // --- ATTACK: score grid points and move to the best receiving location ---
            // Wide defenders are treated as temporary midfielders: they also pick receiving points.
            // Central defender uses a rest-defense / high-line scorer.
            double step = 0.45; // grid spacing (tunable)
            boolean isSideDefender = (Math.abs(self.y) > 0.55);

            // Everyone except the designated rest-defender should play high: receive, shoot, create lanes.
            // When the ball is in opponent half, side defenders should explicitly join as MF-like runners.
            PositionScorer scorer;
            if (isRestDefender) {
                scorer = TacticalScorers.defendWhileAttacking();
            } else if (ballInOppHalf && isSideDefender) {
                scorer = TacticalScorers.wideDefenderJoinAttack();
            } else {
                scorer = TacticalScorers.attackOffBall();
            }

            // Add a small learned bonus on top of the heuristic scorer.
            boolean teamTryingToPass = isTeamPassingNow(self.id);
            boolean teamRegainSoon = isTeamRegainSoonNow(self.id);
            PositionScorer learnedScorer = (w, s, x, y, ts) -> {
                double base = scorer.score(w, s, x, y, ts) + PositionLearning.attackBonus(w, s, x, y, ts);
                if (teamTryingToPass && !isRestDefender) {
                    base += passSpreadBonus(w, s, x, y, ts);
                }
                if (teamRegainSoon && !isRestDefender) {
                    base += preRegainSpreadBonus(w, s, x, y, ts);
                }
                return base;
            };

            GridPoint best = ScoreGrid.findBest(world, self, teamSign, step, learnedScorer);

            double targetX = best.x;
            double targetY = best.y;

            // Extra team-level deconfliction: if many robots choose the same best grid point,
            // add a small repulsion away from teammates' intended targets.
            double[] off = computeTargetDeconflictOffset(self, world, targetX, targetY, teamSign);
            targetX += off[0];
            targetY += off[1];

            // Publish our final planned target for later robots in this frame.
            recordPlannedTarget(self, targetX, targetY);

            // Store attack positioning features for later reward.
            if (self.id >= 0 && self.id < LAST_ATTACK_POS_FEATURES.length) {
                LAST_ATTACK_POS_FEATURES[self.id] = PositionLearning.attackFeatures(world, self, targetX, targetY, teamSign);
                LAST_ATTACK_POS_AT_NANOS[self.id] = System.nanoTime();
            }

            // Publish a representative target for debug overlay (FINAL target after deconfliction).
            if (teamSign == +1) {
                DEBUG_TARGETS[0] = targetX;
                DEBUG_TARGETS[1] = targetY;
            } else {
                DEBUG_TARGETS[2] = targetX;
                DEBUG_TARGETS[3] = targetY;
            }

            // Replace movement: go to best spot.
            double mx = targetX - self.x;
            double my = targetY - self.y;
            double md = Math.sqrt(mx * mx + my * my);
            if (md > 1e-6) {
                double speed = 1.35;
                cmd.vx = (mx / md) * speed;
                cmd.vy = (my / md) * speed;
            } else {
                cmd.vx = 0;
                cmd.vy = 0;
            }
            cmd.omega = 0;
            cmd.kick = false;
            return cmd;
        }

        // --- DEFENSE: score grid points to cut lanes / stay goal-side ---
        {
            double step = 0.55; // slightly coarser when defending
            PositionScorer base = TacticalScorers.defendOffBall();
            PositionScorer learned = (w, s, x, y, ts) -> {
                double[] mark = Main.getMarkTargetForRobot(s.id);
                double v = base.score(w, s, x, y, ts) + PositionLearning.defenseBonus(w, s, x, y, ts, mark);
                // If we are about to regain the ball and this robot isn't assigned to man-mark,
                // start spreading early to prepare for the next pass/move.
                if (mark == null && isTeamRegainSoonNow(s.id)) {
                    v += preRegainSpreadBonus(w, s, x, y, ts);
                }
                return v;
            };
            GridPoint best = ScoreGrid.findBest(world, self, teamSign, step, learned);

            // Publish a representative target for debug overlay.
            if (teamSign == +1) {
                DEBUG_TARGETS[0] = best.x;
                DEBUG_TARGETS[1] = best.y;
            } else {
                DEBUG_TARGETS[2] = best.x;
                DEBUG_TARGETS[3] = best.y;
            }
            double targetX = best.x;
            double targetY = best.y;

            // If we have a man-mark assignment, do NOT apply spacing/deconfliction.
            // Marking should be decisive and is allowed to ignore attack-like distance shaping.
            if (Main.getMarkTargetForRobot(self.id) == null) {
                double[] off = computeTargetDeconflictOffset(self, world, targetX, targetY, teamSign);
                targetX += off[0];
                targetY += off[1];
            }

            recordPlannedTarget(self, targetX, targetY);

            // Store defense positioning features for later reward.
            if (self.id >= 0 && self.id < LAST_DEF_POS_FEATURES.length) {
                double[] mark = Main.getMarkTargetForRobot(self.id);
                LAST_DEF_POS_FEATURES[self.id] = PositionLearning.defenseFeatures(world, self, targetX, targetY, teamSign, mark);
                LAST_DEF_POS_AT_NANOS[self.id] = System.nanoTime();
            }

            double dx = targetX - self.x;
            double dy = targetY - self.y;
            double d = Math.sqrt(dx * dx + dy * dy);
            if (d > 1e-6) {
                double speed = 1.25;
                cmd.vx = (dx / d) * speed;
                cmd.vy = (dy / d) * speed;
            } else {
                cmd.vx = 0;
                cmd.vy = 0;
            }
            cmd.omega = 0;
            cmd.kick = false;
        }

        return cmd;
    }

    private static boolean isTeamPassingNow(int robotId) {
        // In this sim, blue ids are 0..5, red ids are 10..15 by default.
        // Keep logic simple and robust enough for current setup.
        if (robotId >= 10) return TEAM_PASSING_RED;
        return TEAM_PASSING_BLUE;
    }

    private static boolean isTeamRegainSoonNow(int robotId) {
        if (robotId >= 10) return TEAM_REGAIN_SOON_RED;
        return TEAM_REGAIN_SOON_BLUE;
    }

    private static double passSpreadBonus(WorldState world, Robot self, double x, double y, int teamSign) {
        if (world == null || world.ball == null || self == null) return 0.0;

        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

        // 1) Use width: encourage wide positions (but not hard-corner camping).
        double width01 = Math.min(1.0, Math.abs(y) / (halfW + 1e-9));
        double widthBonus = 0.85 * width01;

        // 2) Avoid clustering near the ball (prevents tiny triangles around the passer).
        double db = Math.sqrt(dist2(x, y, world.ball.x, world.ball.y));
        double ballPenalty = 0.0;
        if (db < 1.10) {
            ballPenalty = -1.35 * (1.10 - db) / 1.10;
        }

        // 3) Encourage spacing from teammates.
        java.util.List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        double nearest = 9.0;
        if (mates != null) {
            for (Robot r : mates) {
                if (r == null) continue;
                if (r.id == self.id) continue;
                double d = Math.sqrt(dist2(x, y, r.x, r.y));
                if (d < nearest) nearest = d;
            }
        }
        // Prefer >= ~1.4m separation; small penalty if closer.
        double spacing = clamp((nearest - 1.4) / 1.2, -1.0, 1.0);
        double spacingBonus = 0.65 * spacing;

        // 4) Small forward bias so "wide" doesn't mean drifting backwards.
        double forward = (x - world.ball.x) * teamSign;
        double forwardBonus = 0.20 * clamp(forward / 2.5, -1.0, 1.0);

        return widthBonus + spacingBonus + ballPenalty + forwardBonus;
    }

    private static double preRegainSpreadBonus(WorldState world, Robot self, double x, double y, int teamSign) {
        if (world == null || world.ball == null || self == null) return 0.0;

        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

        // Lighter version of passSpreadBonus: spread and get ready, but don't abandon structure.
        double width01 = Math.min(1.0, Math.abs(y) / (halfW + 1e-9));
        double widthBonus = 0.55 * width01;

        double db = Math.sqrt(dist2(x, y, world.ball.x, world.ball.y));
        double ballPenalty = 0.0;
        if (db < 1.05) {
            ballPenalty = -0.90 * (1.05 - db) / 1.05;
        }

        java.util.List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        double nearest = 9.0;
        if (mates != null) {
            for (Robot r : mates) {
                if (r == null) continue;
                if (r.id == self.id) continue;
                double d = Math.sqrt(dist2(x, y, r.x, r.y));
                if (d < nearest) nearest = d;
            }
        }
        double spacing = clamp((nearest - 1.35) / 1.1, -1.0, 1.0);
        double spacingBonus = 0.45 * spacing;

        double forward = (x - world.ball.x) * teamSign;
        double forwardBonus = 0.12 * clamp(forward / 2.8, -1.0, 1.0);

        return widthBonus + spacingBonus + ballPenalty + forwardBonus;
    }

    /**
     * Pick exactly one rest-defender while attacking: the deepest (closest to our own goal) non-GK robot.
     * Excludes the current ball-winner so we don't accidentally force the attacker to "stay".
     */
    private static int findRestDefenderId(WorldState world, int teamSign) {
        if (world == null || world.ball == null) return -1;

        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double ourGoalX = (teamSign == +1) ? -halfL : halfL;

        // Identify ball winner (closest to ball) to exclude.
        Robot closest = findClosestRobot((teamSign == +1) ? world.ourRobots : world.oppRobots, world.ball);
        int ballWinnerId = (closest == null) ? Integer.MIN_VALUE : closest.id;

        java.util.List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        if (mates == null || mates.isEmpty()) return -1;

        Robot best = null;
        double bestDist = Double.POSITIVE_INFINITY;
        for (Robot r : mates) {
            if (r == null) continue;
            if (isGoalkeeper(r)) continue;
            if (r.id == ballWinnerId) continue;

            double d = Math.abs(r.x - ourGoalX);
            if (d < bestDist) {
                bestDist = d;
                best = r;
            }
        }
        return (best == null) ? -1 : best.id;
    }

    /**
     * Choose the "center defender" as the single rest-defender (smallest |y| among field players).
     * This is used when we want BOTH side defenders to join attack.
     */
    private static int findCenterRestDefenderId(WorldState world, int teamSign) {
        if (world == null || world.ball == null) return -1;

        // Exclude the ball-winner (closest to ball) so the attacker isn't forced into rest-defense.
        Robot closest = findClosestRobot((teamSign == +1) ? world.ourRobots : world.oppRobots, world.ball);
        int ballWinnerId = (closest == null) ? Integer.MIN_VALUE : closest.id;

        java.util.List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        if (mates == null || mates.isEmpty()) return -1;

        Robot best = null;
        double bestAbsY = Double.POSITIVE_INFINITY;
        for (Robot r : mates) {
            if (r == null) continue;
            if (isGoalkeeper(r)) continue;
            if (r.id == ballWinnerId) continue;

            double ay = Math.abs(r.y);
            if (ay < bestAbsY) {
                bestAbsY = ay;
                best = r;
            }
        }
        return (best == null) ? -1 : best.id;
    }

    /**
     * If multiple teammates are trying to go to almost the same (x,y), push them apart.
     * This is a post-process on the selected grid target, so it works for any scorer.
     */
    private static double[] computeTargetDeconflictOffset(Robot self,
                                                          WorldState world,
                                                          double targetX,
                                                          double targetY,
                                                          int teamSign) {
        if (self == null || world == null) return new double[] { 0.0, 0.0 };

        double pushX = 0.0;
        double pushY = 0.0;

        // Stronger deconfliction for defenders to avoid the 3 defenders collapsing to the same lane-cut point.
        double radius = 0.95; // meters: if planned targets are within this, push apart
        double gain = 0.42;   // overall strength
        boolean isDefenderLike = (Math.abs(self.y) <= 1.2); // GK excluded earlier; center-ish => often defender
        if (isDefenderLike) {
            radius = 1.15;
            gain = 0.55;
        }

        // Repel from teammates' PLANNED targets (only those already processed this frame).
        for (int i = 0; i < PLANNED_HAS.length; i++) {
            if (!PLANNED_HAS[i]) continue;
            if (i == self.id) continue;

            double rx = PLANNED_TX[i];
            double ry = PLANNED_TY[i];
            double dx = targetX - rx;
            double dy = targetY - ry;
            double d2 = dx * dx + dy * dy;
            if (d2 < 1e-9) {
                // Identical point: add a deterministic tiny push so robots don't stack.
                double ang = (self.id * 1.7 + i * 0.9);
                pushX += Math.cos(ang);
                pushY += Math.sin(ang);
                continue;
            }

            double d = Math.sqrt(d2);
            if (d > radius) continue;

            double strength = (radius - d) / radius;
            strength = strength * strength; // stronger repulsion when very close
            pushX += (dx / d) * strength;
            pushY += (dy / d) * strength;
        }

        // Limit the offset to keep targets stable.
        double maxOff = 0.70;
        double mag = Math.sqrt(pushX * pushX + pushY * pushY);
        if (mag > 1e-6) {
            pushX = (pushX / mag) * Math.min(maxOff, mag * gain);
            pushY = (pushY / mag) * Math.min(maxOff, mag * gain);
        }

        // Clamp inside field.
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
        double margin = FieldConfig.ROBOT_RADIUS_M + 0.05;
        double nx = clamp(targetX + pushX, -halfL + margin, halfL - margin);
        double ny = clamp(targetY + pushY, -halfW + margin, halfW - margin);
        return new double[] { nx - targetX, ny - targetY };
    }

    private static void recordPlannedTarget(Robot self, double x, double y) {
        if (self == null) return;
        int id = self.id;
        if (id < 0 || id >= PLANNED_HAS.length) return;
        PLANNED_HAS[id] = true;
        PLANNED_TX[id] = x;
        PLANNED_TY[id] = y;
    }

    /**
     * Assign simple, stable man-marks for the defending team.
     *
     * Contract:
     * - Writes MARK_HAS/MARK_TX/MARK_TY for defender-like robots only.
     * - Skips GK and the current ball-winner (closest to ball), so the challenger can pressure.
     * - Picks distinct opponents by greedy matching (good enough for 3 defenders).
     */
    private static void assignMarks(WorldState world, int teamSign, int[] ballOwnerId, int[] ballOwnerTeam) {
        if (world == null || world.ball == null) return;

        int ownerId = (ballOwnerId != null && ballOwnerId.length > 0) ? ballOwnerId[0] : -1;
        int ownerTeam = (ballOwnerTeam != null && ballOwnerTeam.length > 0) ? ballOwnerTeam[0] : 0;

        java.util.List<Robot> defenders = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        java.util.List<Robot> opponents = (teamSign == +1) ? world.oppRobots : world.ourRobots;
        if (defenders == null || defenders.isEmpty()) return;
        if (opponents == null || opponents.isEmpty()) return;

        // Ball winner on the defending team (the one who should pressure).
        Robot ballWinner = findClosestRobot(defenders, world.ball);
        int ballWinnerId = (ballWinner == null) ? Integer.MIN_VALUE : ballWinner.id;

        // Mark candidates: normally ignore opponent GK.
        java.util.ArrayList<Robot> oppCandidates = new java.util.ArrayList<>();
        for (Robot o : opponents) {
            if (o == null) continue;
            oppCandidates.add(o);
        }
        if (oppCandidates.isEmpty()) return;

        boolean[] oppTaken = new boolean[32];

        // Restrict marking assignments to the back line defenders only.
        // (Our default 6v6 formation uses 3 defenders with ids 1..3 for blue and 11..13 for red.)
        // This avoids attackers/supporters also taking marks and collapsing.
        java.util.ArrayList<Robot> defList = new java.util.ArrayList<>();
        for (Robot d : defenders) {
            if (d == null) continue;
            if (isGoalkeeper(d)) continue;
            if (d.id == ballWinnerId) continue;
            if (teamSign == +1) {
                if (d.id < 1 || d.id > 3) continue;
            } else {
                if (d.id < 11 || d.id > 13) continue;
            }
            defList.add(d);
        }
        defList.sort((a, b) -> Double.compare(Math.abs(a.y), Math.abs(b.y)));

        // --- Priority marks ---
        // 1) Mark opponent ball holder (highest priority)
        // 2) Mark likely receiver (next priority)
        int oppTeamSign = -teamSign;
        Robot oppHolder = null;
        if (ownerId != -1 && ownerTeam == oppTeamSign) {
            oppHolder = findRobotById(world, ownerId, ownerTeam);
        }

        // Likely receiver: closest non-GK opponent to the ball, excluding the holder.
        Robot likelyReceiver = null;
        double bestRecvD2 = Double.POSITIVE_INFINITY;
        for (Robot o : opponents) {
            if (o == null) continue;
            if (isGoalkeeper(o)) continue;
            if (oppHolder != null && o.id == oppHolder.id) continue;
            double d2 = dist2(o.x, o.y, world.ball.x, world.ball.y);
            if (d2 < bestRecvD2) {
                bestRecvD2 = d2;
                likelyReceiver = o;
            }
        }

        java.util.HashSet<Integer> usedDef = new java.util.HashSet<>();

        if (oppHolder != null) {
            Robot bestDef = null;
            double bestD2 = Double.POSITIVE_INFINITY;
            for (Robot d : defList) {
                if (d == null) continue;
                double d2 = dist2(d.x, d.y, oppHolder.x, oppHolder.y);
                if (d2 < bestD2) {
                    bestD2 = d2;
                    bestDef = d;
                }
            }
            if (bestDef != null) {
                int id = bestDef.id;
                if (id >= 0 && id < MARK_HAS.length) {
                    MARK_HAS[id] = true;
                    MARK_TX[id] = oppHolder.x;
                    MARK_TY[id] = oppHolder.y;
                }
                if (oppHolder.id >= 0 && oppHolder.id < oppTaken.length) {
                    oppTaken[oppHolder.id] = true;
                }
                usedDef.add(bestDef.id);
            }
        }

        if (likelyReceiver != null) {
            Robot bestDef = null;
            double bestD2 = Double.POSITIVE_INFINITY;
            for (Robot d : defList) {
                if (d == null) continue;
                if (usedDef.contains(d.id)) continue;
                double d2 = dist2(d.x, d.y, likelyReceiver.x, likelyReceiver.y);
                if (d2 < bestD2) {
                    bestD2 = d2;
                    bestDef = d;
                }
            }
            if (bestDef != null) {
                int id = bestDef.id;
                if (id >= 0 && id < MARK_HAS.length) {
                    MARK_HAS[id] = true;
                    MARK_TX[id] = likelyReceiver.x;
                    MARK_TY[id] = likelyReceiver.y;
                }
                if (likelyReceiver.id >= 0 && likelyReceiver.id < oppTaken.length) {
                    oppTaken[likelyReceiver.id] = true;
                }
                usedDef.add(bestDef.id);
            }
        }

        for (Robot d : defList) {
            if (d == null) continue;
            if (usedDef.contains(d.id)) continue;

            Robot best = null;
            double bestCost = Double.POSITIVE_INFINITY;

            for (Robot o : oppCandidates) {
                if (o == null) continue;
                if (o.id >= 0 && o.id < oppTaken.length && oppTaken[o.id]) continue;

                // Skip opponent GK unless it is the ball holder.
                if (isGoalkeeper(o) && (oppHolder == null || o.id != oppHolder.id)) continue;

                double cost = markCost(world, d, o, teamSign);
                if (cost < bestCost) {
                    bestCost = cost;
                    best = o;
                }
            }

            if (best != null) {
                int id = d.id;
                if (id >= 0 && id < MARK_HAS.length) {
                    MARK_HAS[id] = true;
                    MARK_TX[id] = best.x;
                    MARK_TY[id] = best.y;
                }
                if (best.id >= 0 && best.id < oppTaken.length) {
                    oppTaken[best.id] = true;
                }
            }
        }
    }

    /**
     * Lower cost is better.
     * We want each defender to pick a threatening opponent, but keep lane stability.
     */
    private static double markCost(WorldState world, Robot defender, Robot opp, int teamSign) {
        // Threat: opponents more advanced toward OUR goal are more dangerous.
        // In our convention, teamSign is the direction this team attacks.
        // So "toward our goal" for the opponent is along (-teamSign).
        double advToOurGoal = opp.x * (-teamSign);

        // We are defending, so an opponent being more forward (adv large) is dangerous.
        // Convert to threat by rewarding adv, i.e., reduce cost.
    double threatBonus = clamp(advToOurGoal, -6.0, 6.0) * 0.75;

        // If opponent is closer to the ball, they are more likely to receive soon.
        double dBall = Math.sqrt(dist2(opp.x, opp.y, world.ball.x, world.ball.y));
        double receiveBonus = clamp(2.2 - dBall, -2.2, 2.2) * 0.60;

        // Lane stability: match by y to avoid all defenders picking the same central man.
    double yCost = Math.abs(opp.y - defender.y) * 1.05;

        // Note: Do NOT penalize long mark distances here.
        // When marking, we want priorities (ball holder / likely receiver) to dominate.
        double distCost = 0.0;

        // Prefer to mark someone who is in front of (or level with) us in attack direction.
        // Marking a player behind you often pulls you out of line.
        double ahead = (opp.x - defender.x) * teamSign;
        double behindPenalty = (ahead < -0.2) ? (-ahead) * 0.65 : 0.0;

        // Total cost
        double cost = 0.0;
        cost += yCost + distCost + behindPenalty;
        cost -= threatBonus;
        cost -= receiveBonus;
        return cost;
    }

    private static void resetFormation(WorldState world) {
        if (world == null) return;
        world.ourRobots.clear();
        world.oppRobots.clear();

        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

        // --- Blue: GK + 3-2 ---
        // GK
        world.ourRobots.add(new Robot(0, -halfL + 0.35, 0.0, 0.0));
        // 3 defenders
        world.ourRobots.add(new Robot(1, -halfL + 1.35, -1.0, 0.0));
        world.ourRobots.add(new Robot(2, -halfL + 1.35, 0.0, 0.0));
        world.ourRobots.add(new Robot(3, -halfL + 1.35, 1.0, 0.0));
        // 2 attackers
        world.ourRobots.add(new Robot(4, -0.6, -0.9, 0.0));
        world.ourRobots.add(new Robot(5, -0.6, 0.9, 0.0));

        // --- Red: GK + 3-2 ---
        world.oppRobots.add(new Robot(10, halfL - 0.35, 0.0, Math.PI));
        world.oppRobots.add(new Robot(11, halfL - 1.35, -1.0, Math.PI));
        world.oppRobots.add(new Robot(12, halfL - 1.35, 0.0, Math.PI));
        world.oppRobots.add(new Robot(13, halfL - 1.35, 1.0, Math.PI));
        world.oppRobots.add(new Robot(14, 0.6, -0.9, Math.PI));
        world.oppRobots.add(new Robot(15, 0.6, 0.9, Math.PI));

        // Keep inside field just in case
        for (Robot r : world.ourRobots) keepInsideField(r);
        for (Robot r : world.oppRobots) keepInsideField(r);
    }

    private static boolean isGoalkeeper(Robot r) {
        if (r == null) return false;
        return r.id == 0 || r.id == 10;
    }

    private static void maybeBreakStuckContest(WorldState world,
                                               int[] ballOwnerId,
                                               int[] ballOwnerTeam,
                                               long[] stuckSinceNanos,
                                               double[] lastBallX,
                                               double[] lastBallY,
                                               double[] lastRobotX,
                                               double[] lastRobotY,
                                               boolean[] lastRobotHas,
                                               long[] gkHoldUntilNanos,
                                               int[] gkHoldOwnerId) {
        if (world == null || world.ball == null) return;
        if (ballOwnerId == null || ballOwnerId.length == 0) return;
        if (ballOwnerTeam == null || ballOwnerTeam.length == 0) return;
        if (stuckSinceNanos == null || stuckSinceNanos.length == 0) return;
        if (lastBallX == null || lastBallX.length == 0) return;
        if (lastBallY == null || lastBallY.length == 0) return;
        if (lastRobotX == null || lastRobotY == null || lastRobotHas == null) return;
        if (gkHoldUntilNanos == null || gkHoldUntilNanos.length == 0) return;
        if (gkHoldOwnerId == null || gkHoldOwnerId.length == 0) return;

        // Stuck detection: if the ball position barely changes for some time, assume a deadlock.
        // This catches "hard contests" where robots keep pushing but the ball doesn't go anywhere.
        double dx = world.ball.x - lastBallX[0];
        double dy = world.ball.y - lastBallY[0];
        lastBallX[0] = world.ball.x;
        lastBallY[0] = world.ball.y;

        double move2 = dx * dx + dy * dy;
        // In corners, friction + wall constraints can make the ball appear almost static even though
        // robots are pushing. Use a slightly looser threshold to detect that case.
        boolean ballBarelyMoved = move2 < (0.02 * 0.02); // < 2cm per frame

        // If the ball moved meaningfully, reset stuck timer immediately.
        if (!ballBarelyMoved) {
            stuckSinceNanos[0] = -1;
            return;
        }

        // Track robot motion (kept for diagnostics / future tuning; not required for triggering).
        double sumMove2 = 0.0;
        double maxMove2 = 0.0;
        int moveCount = 0;

        java.util.List<Robot> all = new java.util.ArrayList<>();
        all.addAll(world.ourRobots);
        all.addAll(world.oppRobots);

        for (Robot r : all) {
            if (r == null) continue;
            int id = r.id;
            if (id < 0 || id >= lastRobotHas.length) continue;

            if (lastRobotHas[id]) {
                double rdx = r.x - lastRobotX[id];
                double rdy = r.y - lastRobotY[id];
                double rm2 = rdx * rdx + rdy * rdy;
                sumMove2 += rm2;
                if (rm2 > maxMove2) maxMove2 = rm2;
                moveCount++;
            }
            lastRobotX[id] = r.x;
            lastRobotY[id] = r.y;
            lastRobotHas[id] = true;
        }

        // If we don't have enough history yet, don't trigger stuck logic.
        if (moveCount < 4) {
            stuckSinceNanos[0] = -1;
            return;
        }

        // Additional gates to reduce false positives:
        // - require at least one nearby robot (otherwise a stationary ball in open space is fine)
        // - respect GK protected hold window (never override)
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
        boolean ballNearWall = (Math.abs(world.ball.x) > (halfL - 0.10)) || (Math.abs(world.ball.y) > (halfW - 0.10));

        Robot blueC = findClosestRobot(world.ourRobots, world.ball);
        Robot redC = findClosestRobot(world.oppRobots, world.ball);
        if (blueC == null || redC == null) {
            stuckSinceNanos[0] = -1;
            return;
        }
        double blueD2 = dist2(blueC.x, blueC.y, world.ball.x, world.ball.y);
        double redD2 = dist2(redC.x, redC.y, world.ball.x, world.ball.y);
        long now = System.nanoTime();

        // If GK is currently in its protected hold window, never override possession.
        if (ballOwnerId[0] != -1 && gkHoldOwnerId[0] == ballOwnerId[0] && now < gkHoldUntilNanos[0]) {
            stuckSinceNanos[0] = -1;
            return;
        }

        boolean someoneNearBall = (blueD2 < (0.65 * 0.65)) || (redD2 < (0.65 * 0.65)) || (ballOwnerId[0] != -1);
        if (!someoneNearBall) {
            stuckSinceNanos[0] = -1;
            return;
        }

        if (stuckSinceNanos[0] == -1) {
            stuckSinceNanos[0] = now;
            return;
        }

        double stuckSec = (now - stuckSinceNanos[0]) / 1e9;

        // Require a short persistence window.
        if (stuckSec < 0.65) return;

        // True contest: both teams are right on the ball.
        // Loosen slightly so "almost-contact" corner fights are detected too.
        boolean closeContest = blueD2 < (0.33 * 0.33) && redD2 < (0.33 * 0.33);

        // If we're not in a close contest, treat it as a pin/stall and nudge the ball into play.
        if (!closeContest) {
            // Clear ownership so the ball can actually move away.
            ballOwnerId[0] = -1;
            ballOwnerTeam[0] = 0;

            // Nudge direction: toward field center, plus a small component away from the nearest wall.
            double toCenterX = -world.ball.x;
            double toCenterY = -world.ball.y;
            double cd = Math.sqrt(toCenterX * toCenterX + toCenterY * toCenterY);
            if (cd > 1e-6) {
                toCenterX /= cd;
                toCenterY /= cd;
            }

            double nx = 0.0;
            double ny = 0.0;
            double margin = FieldConfig.ROBOT_RADIUS_M + 0.06;
            if (world.ball.x < -halfL + margin) nx = 1.0;
            else if (world.ball.x > halfL - margin) nx = -1.0;
            if (world.ball.y < -halfW + margin) ny = 1.0;
            else if (world.ball.y > halfW - margin) ny = -1.0;

            double kickX = toCenterX;
            double kickY = toCenterY;
            if (nx != 0.0 || ny != 0.0) {
                double nm = Math.sqrt(nx * nx + ny * ny);
                nx /= nm;
                ny /= nm;
                kickX = kickX * 0.70 + nx * 0.30;
                kickY = kickY * 0.70 + ny * 0.30;
            }
            double km = Math.sqrt(kickX * kickX + kickY * kickY);
            if (km > 1e-9) {
                kickX /= km;
                kickY /= km;
            }

            double speed = ballNearWall ? 1.9 : 1.5;
            world.ball.vx = kickX * speed;
            world.ball.vy = kickY * speed;

            // Also move the ball slightly away from the boundary to break immediate re-stick.
            world.ball.x = clamp(world.ball.x + kickX * 0.10, -halfL + margin, halfL - margin);
            world.ball.y = clamp(world.ball.y + kickY * 0.10, -halfW + margin, halfW - margin);

            stuckSinceNanos[0] = -1;
            return;
        }

        // Award possession deterministically:
        // - If someone owns the ball, give it to the closest robot from the NON-owning team.
        // - If no owner, give it to the closest robot overall.
        Robot winner;
        int team;
        if (ballOwnerTeam[0] == +1) {
            winner = redC;
            team = -1;
        } else if (ballOwnerTeam[0] == -1) {
            winner = blueC;
            team = +1;
        } else {
            // No owner: pick closest overall.
            winner = (blueD2 <= redD2) ? blueC : redC;
            team = (winner == blueC) ? +1 : -1;
        }
        ballOwnerId[0] = winner.id;
        ballOwnerTeam[0] = team;

        // Snap ball slightly in front to break symmetry.
        double fx = Math.cos(winner.orientation);
        double fy = Math.sin(winner.orientation);
        double carryOffset = FieldConfig.ROBOT_RADIUS_M + 0.012;
        world.ball.x = winner.x + fx * carryOffset;
        world.ball.y = winner.y + fy * carryOffset;
        world.ball.vx *= 0.2;
        world.ball.vy *= 0.2;

        // Extra nudge away from walls/corners to avoid instantly re-sticking.
        halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
        double margin = FieldConfig.ROBOT_RADIUS_M + 0.06;
        double nx = 0.0;
        double ny = 0.0;

        if (world.ball.x < -halfL + margin) nx = 1.0;
        else if (world.ball.x > halfL - margin) nx = -1.0;
        if (world.ball.y < -halfW + margin) ny = 1.0;
        else if (world.ball.y > halfW - margin) ny = -1.0;

        if (nx != 0.0 || ny != 0.0) {
            double mag = Math.sqrt(nx * nx + ny * ny);
            nx /= mag;
            ny /= mag;

            world.ball.x = clamp(world.ball.x + nx * 0.10, -halfL + margin, halfL - margin);
            world.ball.y = clamp(world.ball.y + ny * 0.10, -halfW + margin, halfW - margin);
            world.ball.vx += nx * 0.35;
            world.ball.vy += ny * 0.35;
        }

        stuckSinceNanos[0] = -1;
    }

    private static Behavior selectBehaviorForOppRobot(Robot r, Behavior attacker, Behavior defender) {
        // For now:
        // - id 10: attacker
        // - others: defender
        return (r.id == 10) ? attacker : defender;
    }

    private static Behavior selectBehaviorForOppRobot(Robot r,
                                                      Behavior attacker,
                                                      Behavior defender,
                                                      Behavior supporter,
                                                      WorldState world) {
        if (r.id == 10) return attacker;
        boolean attacking = isAttackingWithTeamSign(world, -1);
        return attacking ? supporter : defender;
    }

    /**
     * Mirror robot into a coordinate frame where the opponent attacks toward +x.
     * Current convention: Blue attacks toward +x, Red attacks toward -x.
     * Mirroring: (x,y) -> (-x,-y)
     */
    private static Robot mirrorRobot(Robot r) {
        return new Robot(r.id, -r.x, -r.y, r.orientation + Math.PI);
    }

    private static WorldState mirrorWorld(WorldState world) {
        WorldState m = new WorldState();
        if (world.ball != null) {
            m.ball = new Ball(-world.ball.x, -world.ball.y);
            m.ball.vx = -world.ball.vx;
            m.ball.vy = -world.ball.vy;
        }

        // Swap teams (so behaviors can still look at world.ourRobots / world.oppRobots)
        for (Robot r : world.oppRobots) {
            m.ourRobots.add(mirrorRobot(r));
        }
        for (Robot r : world.ourRobots) {
            m.oppRobots.add(mirrorRobot(r));
        }
        return m;
    }

    private static RobotCommand unmirrorCommand(RobotCommand cmd) {
        if (cmd == null) return null;
        RobotCommand u = new RobotCommand();
        u.robotId = cmd.robotId;
        // Inverse transform for velocities: (vx,vy) -> (-vx,-vy)
        u.vx = -cmd.vx;
        u.vy = -cmd.vy;
        u.omega = cmd.omega;
        u.kick = cmd.kick;
        u.kickVx = -cmd.kickVx;
        u.kickVy = -cmd.kickVy;
        return u;
    }

    private static Behavior selectBehaviorForOurRobot(Robot r, Behavior attacker, Behavior defender) {
        // Backward-compat overload
        return (r.id == 0) ? attacker : defender;
    }

    private static Behavior selectBehaviorForOurRobot(Robot r,
                                                      Behavior attacker,
                                                      Behavior defender,
                                                      Behavior supporter,
                                                      WorldState world) {
        if (r.id == 0) return attacker;
        boolean attacking = isAttackingWithTeamSign(world, +1);
        return attacking ? supporter : defender;
    }

    private static boolean isAttackingWithTeamSign(WorldState world, int teamSign) {
        if (world == null || world.ball == null) return false;
        // teamSign +1: blue attacks toward +x -> opponent half is x > 0
        // teamSign -1: red attacks toward -x -> opponent half is x < 0
        return (teamSign == +1) ? (world.ball.x > 0.0) : (world.ball.x < 0.0);
    }

    private static boolean shouldSwitchAttacker(long now,
                                                long lastSwitch,
                                                long holdNanos,
                                                int currentAttackerId,
                                                Robot closest,
                                                Ball ball,
                                                java.util.List<Robot> teamRobots) {
        if (closest == null || ball == null) return false;
        if (now - lastSwitch < holdNanos) return false;
        if (closest.id == currentAttackerId) return false;

        // Find current attacker robot (if missing, allow switch).
        Robot current = null;
        if (teamRobots != null) {
            for (Robot r : teamRobots) {
                if (r.id == currentAttackerId) {
                    current = r;
                    break;
                }
            }
        }
        if (current == null) return true;

        // Distances (squared) to ball
        double candD2 = dist2(closest.x, closest.y, ball.x, ball.y);
        double curD2 = dist2(current.x, current.y, ball.x, ball.y);

        // Hysteresis margin to prevent flip-flops. Candidate must be sufficiently closer.
        // - Base margin: 25% closer (or 0.25m absolute if distances are tiny)
        double minAbs = 0.25; // meters
        double minAbs2 = minAbs * minAbs;
        boolean clearlyCloser = candD2 < curD2 * 0.75 || (curD2 - candD2) > minAbs2;
        if (!clearlyCloser) return false;

        // Near center line, require an even stronger advantage to switch.
        double centerDeadband = 0.15; // meters
        if (Math.abs(ball.x) < centerDeadband) {
            boolean veryClearlyCloser = candD2 < curD2 * 0.55 || (curD2 - candD2) > (0.35 * 0.35);
            return veryClearlyCloser;
        }

        return true;
    }

    private static double dist2(double ax, double ay, double bx, double by) {
        double dx = bx - ax;
        double dy = by - ay;
        return dx * dx + dy * dy;
    }

    private static Robot findClosestRobot(java.util.List<Robot> robots, Ball ball) {
        if (robots == null || ball == null || robots.isEmpty()) return null;
        Robot best = null;
        double bestD2 = Double.POSITIVE_INFINITY;
        for (Robot r : robots) {
            double dx = ball.x - r.x;
            double dy = ball.y - r.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
            }
        }
        return best;
    }

    private static boolean isSecondClosestRobot(java.util.List<Robot> robots, Ball ball, Robot candidate) {
        if (robots == null || ball == null || candidate == null || robots.size() < 2) return false;
        Robot best = null;
        Robot second = null;
        double bestD2 = Double.POSITIVE_INFINITY;
        double secondD2 = Double.POSITIVE_INFINITY;
        for (Robot r : robots) {
            double d2 = dist2(r.x, r.y, ball.x, ball.y);
            if (d2 < bestD2) {
                second = best;
                secondD2 = bestD2;
                best = r;
                bestD2 = d2;
            } else if (d2 < secondD2) {
                second = r;
                secondD2 = d2;
            }
        }
        return second != null && second.id == candidate.id;
    }

    private static RobotCommand applyBackupSupport(RobotCommand base, Robot self, WorldState world, int teamSign) {
        if (base == null) base = new RobotCommand();
        if (self == null || world == null || world.ball == null) return base;

        Ball ball = world.ball;

        // --- "Behind defender" backup positioning ---
        // Find the nearest opponent to the ball (likely blocker/defender).
        // Then position ourselves just behind that opponent relative to the ball->goal direction,
        // plus a small lateral offset to create a clean passing lane.
        Robot nearestOpp = findClosestRobot(world.oppRobots, ball);

        double goalX = (teamSign == +1)
                ? (FieldConfig.FIELD_LENGTH_M / 2.0)
                : (-FieldConfig.FIELD_LENGTH_M / 2.0);
        double goalY = 0.0;

        double dirX = goalX - ball.x;
        double dirY = goalY - ball.y;
        double dirD = Math.sqrt(dirX * dirX + dirY * dirY);
        if (dirD < 1e-6) {
            dirX = teamSign;
            dirY = 0.0;
            dirD = 1.0;
        }
        dirX /= dirD;
        dirY /= dirD;

        double latX = -dirY;
        double latY = dirX;
        double lateral = ((self.id % 2) == 0) ? 0.45 : -0.45;

        double targetX;
        double targetY;
        if (nearestOpp != null) {
            double behindDef = 0.55;
            targetX = nearestOpp.x - dirX * behindDef + latX * lateral;
            targetY = nearestOpp.y - dirY * behindDef + latY * lateral;
        } else {
            // Fallback: behind the ball (relative to attack direction)
            double behind = 0.8;
            targetX = ball.x - behind * teamSign;
            targetY = ball.y + (((self.id % 2) == 0) ? 0.7 : -0.7);
        }

        // Clamp inside field
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
        double margin = FieldConfig.ROBOT_RADIUS_M + 0.05;
        targetX = clamp(targetX, -halfL + margin, halfL - margin);
        targetY = clamp(targetY, -halfW + margin, halfW - margin);

        // Overwrite movement to go to backup point.
        double dx = targetX - self.x;
        double dy = targetY - self.y;
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > 1e-6) {
            double speed = 1.2;
            base.vx = (dx / dist) * speed;
            base.vy = (dy / dist) * speed;
        } else {
            base.vx = 0;
            base.vy = 0;
        }
        base.omega = 0;
        base.kick = false;
        return base;
    }

    private static void applyDeconflictBias(RobotCommand cmd, Robot self, WorldState world, int teamSign) {
        if (cmd == null || self == null || world == null || world.ball == null) return;

        // If command is basically "go to ball" (very similar direction to ball), spread robots laterally.
        double dx = world.ball.x - self.x;
        double dy = world.ball.y - self.y;
        double d = Math.sqrt(dx * dx + dy * dy);
        if (d < 1e-6) return;

        double vx = cmd.vx;
        double vy = cmd.vy;
        double v = Math.sqrt(vx * vx + vy * vy);
        if (v < 0.2) return;

        // Are we moving roughly toward the ball? (cosine similarity)
        double cos = (vx * dx + vy * dy) / (v * d);
        if (cos < 0.85) return;

        // Add a small bias perpendicular to ball direction.
        double latX = -dy / d;
        double latY = dx / d;
        double sign = ((Math.abs(self.id) % 2) == 0) ? 1.0 : -1.0;
        double bias = 0.18; // m/s

        cmd.vx += latX * bias * sign;
        cmd.vy += latY * bias * sign;
    }

    private static Robot findOurRobot(WorldState world, int robotId) {
        for (Robot r : world.ourRobots) {
            if (r.id == robotId) {
                return r;
            }
        }
        return null;
    }

    private static void applyCommand(WorldState world,
                                     Robot self,
                                     RobotCommand cmd,
                                     double dt,
                                     int teamSign,
                                     int[] ballOwnerId,
                                     int[] ballOwnerTeam) {
        if (cmd == null) return;

        // Clamp speeds a bit so debug is easier.
        double maxV = 2.0;       // m/s
        double maxOmega = 6.0;   // rad/s

        double vx = clamp(cmd.vx, -maxV, maxV);
        double vy = clamp(cmd.vy, -maxV, maxV);
        double omega = clamp(cmd.omega, -maxOmega, maxOmega);

        // --- If we currently own the ball, avoid endless "dribble to corner" and wall pinning ---
        boolean isOwner = (ballOwnerId != null && ballOwnerId.length > 0 && ballOwnerTeam != null && ballOwnerTeam.length > 0)
                && (ballOwnerId[0] == self.id) && (ballOwnerTeam[0] == teamSign);

        if (isOwner && world != null && world.ball != null) {
            double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
            double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
            double wallBand = 0.35; // meters from wall considered "near wall"

            boolean nearWall = (Math.abs(self.x) > (halfL - wallBand)) || (Math.abs(self.y) > (halfW - wallBand));
            if (nearWall) {
                // Slow down near walls so we don't glue ourselves into the corner.
                vx *= 0.55;
                vy *= 0.55;

                // If opponent is close while we're near the wall, auto-clear or auto-pass out.
                Robot nearestOpp = findClosestRobot(world.oppRobots, world.ball);
                double oppD2 = (nearestOpp == null) ? Double.POSITIVE_INFINITY
                        : dist2(nearestOpp.x, nearestOpp.y, world.ball.x, world.ball.y);
                boolean pressure = oppD2 < (0.55 * 0.55);

                // Also trigger if the ball is very close to the boundary.
                boolean ballNearWall = (Math.abs(world.ball.x) > (halfL - 0.08)) || (Math.abs(world.ball.y) > (halfW - 0.08));

                if (pressure || ballNearWall) {
                    cmd.kick = true;

                    // Kick direction: push back toward field center, with a small forward component.
                    double toCenterX = -world.ball.x;
                    double toCenterY = -world.ball.y;
                    double d = Math.sqrt(toCenterX * toCenterX + toCenterY * toCenterY);
                    if (d < 1e-6) {
                        toCenterX = 0.0;
                        toCenterY = 0.0;
                        d = 1.0;
                    }
                    toCenterX /= d;
                    toCenterY /= d;

                    double forwardX = teamSign;
                    double forwardY = 0.0;

                    double kx = (toCenterX * 0.8 + forwardX * 0.2);
                    double ky = (toCenterY * 0.8 + forwardY * 0.2);
                    double kd = Math.sqrt(kx * kx + ky * ky);
                    if (kd > 1e-9) {
                        kx /= kd;
                        ky /= kd;
                    }
                    cmd.kickVx = kx * 4.2;
                    cmd.kickVy = ky * 4.2;
                }
            }
        }

        // --- Priority / right-of-way near ball (NO hard distance keeping) ---
        // When ball is free or being contested, only the designated "ball winner" should fully attack.
        // Other robots should yield near the ball so it becomes reachable.
        // This is a speed/steering rule (not a distance constraint), so robots can still approach if needed.
        if (world != null && world.ball != null) {
            // Allow direct ball contests: the closest robot of EACH team should be able to collide and fight.
            // So we only apply "yield" to robots that are NOT one of the two closest robots to the ball.
            Robot ourClosest = findClosestRobot(world.ourRobots, world.ball);
            Robot oppClosest = findClosestRobot(world.oppRobots, world.ball);

            boolean isTeamClosest = (teamSign == +1)
                    ? (ourClosest != null && ourClosest.id == self.id)
                    : (oppClosest != null && oppClosest.id == self.id);
            boolean isContestant = isTeamClosest;

            double ballD2 = dist2(self.x, self.y, world.ball.x, world.ball.y);
            double nearBall2 = (0.55 * 0.55);

            boolean gkPriority = isGoalkeeper(self) && isBallInGKBox(world.ball, teamSign);
            if (!isContestant && !gkPriority && ballD2 < nearBall2) {
                // Non-contestants yield to keep the contest area clear.
                vx *= 0.35;
                vy *= 0.35;
                double awayX = self.x - world.ball.x;
                double awayY = self.y - world.ball.y;
                double d = Math.sqrt(awayX * awayX + awayY * awayY);
                if (d > 1e-6) {
                    vx += (awayX / d) * 0.25;
                    vy += (awayY / d) * 0.25;
                }
            }

            // If there is a real contest (both teams have a close robot near the ball),
            // bias the contestants to push toward the ball so they can actually make contact.
            if (ourClosest != null && oppClosest != null) {
                double ourD2 = dist2(ourClosest.x, ourClosest.y, world.ball.x, world.ball.y);
                double oppD2 = dist2(oppClosest.x, oppClosest.y, world.ball.x, world.ball.y);
                double contest2 = (0.65 * 0.65);
                boolean isCloseContest = (ourD2 < contest2) && (oppD2 < contest2);

                if (isCloseContest && isContestant) {
                    double toBallX = world.ball.x - self.x;
                    double toBallY = world.ball.y - self.y;
                    double d = Math.sqrt(toBallX * toBallX + toBallY * toBallY);
                    if (d > 1e-6) {
                        // Push toward the ball direction. Keep small so we still respect Behavior steering.
                        double push = 0.45;
                        vx += (toBallX / d) * push;
                        vy += (toBallY / d) * push;
                    }
                }
            }
        }

        self.x += vx * dt;
        self.y += vy * dt;
        self.orientation += omega * dt;

        // Very simple kick: if close enough to ball, add ball velocity toward opponent goal.
        if (cmd.kick && world.ball != null) {
            // GK special: do NOT auto-kick on first touch.
            // DefenderBehavior clears when the ball is close in our half, which makes the GK
            // always punt the ball forward before possession logic can attach it.
            // If the ball is controllable, suppress the kick and let updatePossessionAndDribble
            // grant GK possession (catch/hold) instead.
            if (self != null && isGoalkeeper(self)) {
                double bs2 = world.ball.vx * world.ball.vx + world.ball.vy * world.ball.vy;
                double attachBallSpeed = 0.85;
                boolean controllable = bs2 <= (attachBallSpeed * attachBallSpeed);
                boolean inGkBox = isBallInGKBox(world.ball, teamSign);
                if (controllable && inGkBox) {
                    keepInsideField(self);
                    return;
                }
            }

            double dx = world.ball.x - self.x;
            double dy = world.ball.y - self.y;
            double dist = Math.sqrt(dx * dx + dy * dy);

            double kickRange = FieldConfig.ROBOT_RADIUS_M + 0.03; // 3cm margin
            if (dist <= kickRange) {
                // If this kick is tagged as a pass and we were the owner, record it for learning.
                // Note: features are recomputed here (cheap) to keep sim/Main independent of behavior internals.
                boolean isOwnerNow = (ballOwnerId != null && ballOwnerId.length > 0 && ballOwnerTeam != null && ballOwnerTeam.length > 0)
                        && (ballOwnerId[0] == self.id) && (ballOwnerTeam[0] == teamSign);
                if (isOwnerNow && cmd.passTargetId >= 0) {
                    double[] feats = ai.PassLearning.featuresForReceiver(world, teamSign, cmd.passTargetId);
                    recordPassAttempt(self.id, teamSign, cmd.passTargetId, world.ball.x, feats);
                }

                // Record the shoot-vs-pass action (for learning). We tag only deliberate passes (passTargetId>=0)
                // and deliberate shots (shotIntent=true).
                if (isOwnerNow && (cmd.passTargetId >= 0 || cmd.shotIntent)) {
                    double[] actionFeats = ai.ActionLearning.features(world, teamSign, self.id);
                    recordActionAttempt(self.id, teamSign, cmd.shotIntent, cmd.passTargetId, world.ball.x, actionFeats);
                }

                // Preferred: use explicit kick vector if provided by behavior.
                double kx = cmd.kickVx;
                double ky = cmd.kickVy;

                // Fallback: Blue attacks +x, Red attacks -x
                if (Math.abs(kx) < 1e-9 && Math.abs(ky) < 1e-9) {
                    kx = 4.0 * teamSign;
                    ky = 0.0;
                }

                world.ball.vx = kx;
                world.ball.vy = ky;

                // Release ownership immediately on kick so the ball can leave.
                if (ballOwnerId != null && ballOwnerId.length > 0 && ballOwnerTeam != null && ballOwnerTeam.length > 0) {
                    if (ballOwnerId[0] == self.id && ballOwnerTeam[0] == teamSign) {
                        ballOwnerId[0] = -1;
                        ballOwnerTeam[0] = 0;
                    }
                }
            }
        }

        keepInsideField(self);
    }

    private static void updatePossessionAndDribble(WorldState world,
                                                   double dt,
                                                   int[] ballOwnerId,
                                                   int[] ballOwnerTeam,
                                                   java.util.Map<Integer, Long> pickupBanUntilNanos,
                                                   int[] recentlyLostOwnerId,
                                                   int[] recentlyLostOwnerTeam,
                                                   long[] recentlyLostAtNanos,
                                                   long gkHoldNanos,
                                                   long[] gkHoldUntilNanos,
                                                   int[] gkHoldOwnerId,
                                                   int[] gkHoldOwnerTeam) {
        if (world == null || world.ball == null) return;
        if (ballOwnerId == null || ballOwnerId.length == 0) return;
        if (ballOwnerTeam == null || ballOwnerTeam.length == 0) return;
        if (pickupBanUntilNanos == null) return;
        if (recentlyLostOwnerId == null || recentlyLostOwnerId.length == 0) return;
        if (recentlyLostOwnerTeam == null || recentlyLostOwnerTeam.length == 0) return;
        if (recentlyLostAtNanos == null || recentlyLostAtNanos.length == 0) return;
        if (gkHoldUntilNanos == null || gkHoldUntilNanos.length == 0) return;
        if (gkHoldOwnerId == null || gkHoldOwnerId.length == 0) return;
        if (gkHoldOwnerTeam == null || gkHoldOwnerTeam.length == 0) return;

        Ball ball = world.ball;
        long now = System.nanoTime();

        // --- parameters (tunable) ---
        final double controlDist = FieldConfig.ROBOT_RADIUS_M + 0.035; // needs to be close
        final double controlDist2 = controlDist * controlDist;
        final double attachBallSpeed = 0.85; // m/s, ball must be relatively slow to be trapped
        final double detachDist = FieldConfig.ROBOT_RADIUS_M + 0.10; // if owner moves away, release
        final double detachDist2 = detachDist * detachDist;
        final double carryOffset = FieldConfig.ROBOT_RADIUS_M + 0.012; // ball in front of robot

        // If there is an owner, verify it is still valid.
        if (ballOwnerId[0] != -1) {
            Robot owner = findRobotById(world, ballOwnerId[0], ballOwnerTeam[0]);
            if (owner == null) {
                // Owner disappeared: treat as lost possession.
                if (gkHoldOwnerId[0] == ballOwnerId[0]) {
                    gkHoldUntilNanos[0] = 0L;
                    gkHoldOwnerId[0] = -1;
                    gkHoldOwnerTeam[0] = 0;
                }
                recentlyLostOwnerId[0] = ballOwnerId[0];
                recentlyLostOwnerTeam[0] = ballOwnerTeam[0];
                recentlyLostAtNanos[0] = now;
                ballOwnerId[0] = -1;
                ballOwnerTeam[0] = 0;
            } else {
                boolean ownerIsGK = isGoalkeeper(owner);

                // GK special: start/maintain a protected hold window.
                if (ownerIsGK) {
                    if (gkHoldOwnerId[0] != owner.id || gkHoldOwnerTeam[0] != ballOwnerTeam[0] || gkHoldUntilNanos[0] <= 0L) {
                        gkHoldOwnerId[0] = owner.id;
                        gkHoldOwnerTeam[0] = ballOwnerTeam[0];
                        gkHoldUntilNanos[0] = now + Math.max(0L, gkHoldNanos);
                    }

                    // Force a pass shortly before the hold window ends.
                    // This prevents GK from wandering while holding and ensures fast distribution.
                    long timeLeft = gkHoldUntilNanos[0] - now;
                    long passLead = (long) (0.35e9); // pass when <= 0.35s left
                    if (timeLeft <= passLead) {
                        Robot recv = pickBestGkPassReceiver(world, ballOwnerTeam[0], owner.id, ball);
                        if (recv != null) {
                            double px = recv.x - ball.x;
                            double py = recv.y - ball.y;
                            double pd = Math.sqrt(px * px + py * py);
                            if (pd > 1e-6) {
                                px /= pd;
                                py /= pd;
                            }
                            double kickSpeed = 4.1;
                            ball.vx = px * kickSpeed;
                            ball.vy = py * kickSpeed;
                        } else {
                            // Fallback: clear toward opponent half.
                            ball.vx = 4.4 * ballOwnerTeam[0];
                            ball.vy = 0.0;
                        }

                        // Release ownership so the ball can travel.
                        ballOwnerId[0] = -1;
                        ballOwnerTeam[0] = 0;
                        gkHoldUntilNanos[0] = 0L;
                        gkHoldOwnerId[0] = -1;
                        gkHoldOwnerTeam[0] = 0;
                        return;
                    }
                } else {
                    // Not GK: clear GK hold state if it was stale.
                    if (gkHoldOwnerId[0] == owner.id) {
                        gkHoldUntilNanos[0] = 0L;
                        gkHoldOwnerId[0] = -1;
                        gkHoldOwnerTeam[0] = 0;
                    }
                }

                double d2 = dist2(owner.x, owner.y, ball.x, ball.y);
                if (d2 > detachDist2) {
                    // Owner drifted away: treat as lost possession.
                    if (gkHoldOwnerId[0] == ballOwnerId[0]) {
                        gkHoldUntilNanos[0] = 0L;
                        gkHoldOwnerId[0] = -1;
                        gkHoldOwnerTeam[0] = 0;
                    }
                    recentlyLostOwnerId[0] = ballOwnerId[0];
                    recentlyLostOwnerTeam[0] = ballOwnerTeam[0];
                    recentlyLostAtNanos[0] = now;
                    ballOwnerId[0] = -1;
                    ballOwnerTeam[0] = 0;
                } else {
                    // Failed steal cooldown: if a non-GK robot from the opposing team gets close enough
                    // to "try" taking the ball but does not become owner, ban it briefly from re-trying.
                    // This reduces oscillations and ambiguous contests.
                    boolean ownerGKProtected = ownerIsGK && (gkHoldOwnerId[0] == owner.id) && (now < gkHoldUntilNanos[0]);
                    if (!ownerGKProtected) {
                        final long failBanNanos = (long) (0.8e9); // 0.8s
                        double bs2 = ball.vx * ball.vx + ball.vy * ball.vy;
                        if (bs2 <= attachBallSpeed * attachBallSpeed) {
                            java.util.List<Robot> stealers = (ballOwnerTeam[0] == +1) ? world.oppRobots : world.ourRobots;
                            Robot closestStealer = null;
                            double closestD2 = Double.POSITIVE_INFINITY;
                            for (Robot s : stealers) {
                                if (s == null) continue;
                                if (isGoalkeeper(s)) continue;
                                double sd2 = dist2(s.x, s.y, ball.x, ball.y);
                                if (sd2 < closestD2) {
                                    closestD2 = sd2;
                                    closestStealer = s;
                                }
                            }

                            if (closestStealer != null && closestD2 <= controlDist2) {
                                Long until = pickupBanUntilNanos.get(closestStealer.id);
                                if (until == null || now >= until) {
                                    pickupBanUntilNanos.put(closestStealer.id, now + failBanNanos);
                                }
                            }
                        }
                    }

                    // Carry the ball: keep it in front of the robot and follow robot motion.
                    double fx = Math.cos(owner.orientation);
                    double fy = Math.sin(owner.orientation);
                    double bx = owner.x + fx * carryOffset;
                    double by = owner.y + fy * carryOffset;

                    // Wall slide: if the carried ball would go outside, slide it along the wall a bit
                    // (and bias slightly away from the wall) to avoid pinning / deadlocks at edges.
                    double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
                    double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;
                    double margin = 0.02; // minimal safety margin

                    double clampedX = clamp(bx, -halfL + margin, halfL - margin);
                    double clampedY = clamp(by, -halfW + margin, halfW - margin);

                    boolean hitWallX = Math.abs(bx - clampedX) > 1e-9;
                    boolean hitWallY = Math.abs(by - clampedY) > 1e-9;
                    if (hitWallX || hitWallY) {
                        // Slide direction = perpendicular to robot forward.
                        double latX = -fy;
                        double latY = fx;
                        double slide = ((Math.abs(owner.id) % 2) == 0) ? 0.03 : -0.03;

                        // If pinned on X wall, allow sliding mostly in Y; if pinned on Y wall, slide mostly in X.
                        if (hitWallX) {
                            clampedY = clamp(clampedY + latY * slide, -halfW + margin, halfW - margin);
                        }
                        if (hitWallY) {
                            clampedX = clamp(clampedX + latX * slide, -halfL + margin, halfL - margin);
                        }
                    }

                    ball.x = clampedX;
                    ball.y = clampedY;

                    // Ball velocity follows the owner (light damping so it doesn't explode).
                    // We approximate owner velocity by looking at how much position changed is hard here,
                    // so keep it small and rely on kicks to impart speed.
                    ball.vx *= 0.4;
                    ball.vy *= 0.4;
                    return;
                }
            }
        }

        // No owner: try to attach if a robot is close and the ball is slow / controllable.
        double ballSpeed2 = ball.vx * ball.vx + ball.vy * ball.vy;
        if (ballSpeed2 > attachBallSpeed * attachBallSpeed) return;

        Robot best = null;
        int bestTeam = 0;
        double bestD2 = Double.POSITIVE_INFINITY;

        // Helper: skip robots that are temporarily banned from picking up the ball.
        java.util.function.IntPredicate isBanned = (rid) -> {
            Long until = pickupBanUntilNanos.get(rid);
            if (until == null) return false;
            if (now >= until) {
                pickupBanUntilNanos.remove(rid);
                return false;
            }
            return true;
        };

        for (Robot r : world.ourRobots) {
            if (r == null) continue;
            if (isBanned.test(r.id)) continue;
            double d2 = dist2(r.x, r.y, ball.x, ball.y);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
                bestTeam = +1;
            }
        }
        for (Robot r : world.oppRobots) {
            if (r == null) continue;
            if (isBanned.test(r.id)) continue;
            double d2 = dist2(r.x, r.y, ball.x, ball.y);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
                bestTeam = -1;
            }
        }

        if (best != null && bestD2 <= controlDist2) {
            // If an opponent picked up shortly after someone lost it, ban the loser for ~1s.
            // This prevents immediate re-steals that often cause corner deadlocks.
            final long banNanos = (long) 1e9; // 1 second
            final long stealWindowNanos = (long) 2e9; // consider it a "steal" if within 2s
            int lostId = recentlyLostOwnerId[0];
            int lostTeam = recentlyLostOwnerTeam[0];
            long lostAt = recentlyLostAtNanos[0];
            if (lostId != -1 && lostAt > 0
                    && (now - lostAt) <= stealWindowNanos
                    && lostTeam != 0
                    && bestTeam != lostTeam
                    && best.id != lostId) {
                pickupBanUntilNanos.put(lostId, now + banNanos);
            }

            ballOwnerId[0] = best.id;
            ballOwnerTeam[0] = bestTeam;

            // If we just resolved a tagged pass attempt, reward it.
            maybeRewardLastPass(world, ballOwnerId[0], ballOwnerTeam[0], ball.x);

            // If this was a steal / turnover, update team-level positioning rewards.
            // (lostTeam is set when an owner detached due to steal/lose; it is 0 otherwise.)
            if (lostTeam != 0 && lostTeam != bestTeam) {
                // The losing team gets a small negative (attack shape didn’t protect the ball).
                applyTeamAttackPositionReward(world, lostTeam, -0.25);
                // The winning team gets a small positive (defensive shape / pressure worked).
                applyTeamDefensePositionReward(world, bestTeam, +0.25);

                // Also penalize the last action if it likely caused a turnover.
                if (LAST_ACTION_TEAM == lostTeam && LAST_ACTION_FEATURES != null) {
                    ai.ActionLearning.applyReward(LAST_ACTION_SHOOT, -1.0, LAST_ACTION_FEATURES);
                    clearLastAction();
                }
            }

            // Clear "recent loss" bookkeeping once the ball is claimed again.
            recentlyLostOwnerId[0] = -1;
            recentlyLostOwnerTeam[0] = 0;
            recentlyLostAtNanos[0] = -1L;

            // If GK picked up, start its hold window now.
            if (isGoalkeeper(best)) {
                gkHoldOwnerId[0] = best.id;
                gkHoldOwnerTeam[0] = bestTeam;
                gkHoldUntilNanos[0] = now + Math.max(0L, gkHoldNanos);
            }

            // Snap ball to front immediately
            double fx = Math.cos(best.orientation);
            double fy = Math.sin(best.orientation);
            ball.x = best.x + fx * carryOffset;
            ball.y = best.y + fy * carryOffset;
            ball.vx *= 0.2;
            ball.vy *= 0.2;
        }
    }

    private static Robot pickBestGkPassReceiver(WorldState world, int teamSign, int excludeId, Ball ball) {
        if (world == null || ball == null) return null;
        java.util.List<Robot> mates = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        java.util.List<Robot> opps = (teamSign == +1) ? world.oppRobots : world.ourRobots;
        if (mates == null || mates.isEmpty()) return null;

        Robot best = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (Robot r : mates) {
            if (r == null) continue;
            if (r.id == excludeId) continue;
            if (isGoalkeeper(r)) continue;

            double dBall = Math.sqrt(dist2(r.x, r.y, ball.x, ball.y));
            if (dBall < 0.65) continue; // too close to pass

            // Prefer receivers that are more open (farther from nearest opponent).
            double nearestOpp = 9.0;
            if (opps != null) {
                for (Robot o : opps) {
                    if (o == null) continue;
                    double d = Math.sqrt(dist2(o.x, o.y, r.x, r.y));
                    if (d < nearestOpp) nearestOpp = d;
                }
            }

            // Prefer forward options (toward opponent goal) but not extremely deep.
            double forward = (r.x * teamSign);
            double forwardScore = clamp(forward, -3.0, 6.0) * 0.35;

            // Prefer medium range passes.
            double rangeScore = -Math.abs(dBall - 2.0) * 0.35;

            double openScore = clamp(nearestOpp, 0.0, 3.0) * 0.55;
            double score = openScore + forwardScore + rangeScore;

            if (score > bestScore) {
                bestScore = score;
                best = r;
            }
        }

        return best;
    }

    private static Robot findRobotById(WorldState world, int robotId, int teamSign) {
        if (world == null) return null;
        java.util.List<Robot> list = (teamSign == +1) ? world.ourRobots : world.oppRobots;
        if (list == null) return null;
        for (Robot r : list) {
            if (r != null && r.id == robotId) return r;
        }
        return null;
    }

    private static Vec2 computeAvoidanceVelocity(WorldState world, Robot self) {
        if (world == null || self == null) return new Vec2(0, 0);

        double keep = FieldConfig.ROBOT_RADIUS_M * 2.0 + 0.10; // desired personal space
        double keep2 = keep * keep;
        double influence = keep * 2.0;
        double influence2 = influence * influence;

        double ax = 0.0;
        double ay = 0.0;

        // Yield away from all robots (both teams)
        ax += avoidanceFromList(self, world.ourRobots, keep2, influence2).x;
        ay += avoidanceFromList(self, world.ourRobots, keep2, influence2).y;
        ax += avoidanceFromList(self, world.oppRobots, keep2, influence2).x;
        ay += avoidanceFromList(self, world.oppRobots, keep2, influence2).y;

        // Limit avoidance magnitude so it remains a gentle bias
        double maxAvoid = 0.9; // m/s
        double mag = Math.sqrt(ax * ax + ay * ay);
        if (mag > maxAvoid && mag > 1e-9) {
            ax = ax / mag * maxAvoid;
            ay = ay / mag * maxAvoid;
        }

        return new Vec2(ax, ay);
    }

    private static Vec2 avoidanceFromList(Robot self, java.util.List<Robot> robots, double keep2, double influence2) {
        if (robots == null) return new Vec2(0, 0);
        double rx = 0.0;
        double ry = 0.0;
        for (Robot r : robots) {
            if (r == null || r.id == self.id) continue;
            double dx = self.x - r.x;
            double dy = self.y - r.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < 1e-9) continue;
            if (d2 > influence2) continue;

            // Weight grows as we get closer. Strong push when inside keep distance.
            double w = (d2 < keep2) ? 1.0 : (influence2 - d2) / (influence2 - keep2);
            double d = Math.sqrt(d2);
            rx += (dx / d) * w;
            ry += (dy / d) * w;
        }
        return new Vec2(rx, ry);
    }

    private static Vec2 findBallCarrier(WorldState world) {
        if (world == null || world.ball == null) return null;
        Ball ball = world.ball;
        double control = FieldConfig.ROBOT_RADIUS_M + 0.06;
        double control2 = control * control;

        Robot best = null;
        double bestD2 = Double.POSITIVE_INFINITY;

        for (Robot r : world.ourRobots) {
            double d2 = dist2(r.x, r.y, ball.x, ball.y);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
            }
        }
        for (Robot r : world.oppRobots) {
            double d2 = dist2(r.x, r.y, ball.x, ball.y);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = r;
            }
        }

        if (best != null && bestD2 <= control2) {
            return new Vec2(best.x, best.y, best.id);
        }
        return null;
    }

    private static final class Vec2 {
        final double x;
        final double y;
        final int id;
        Vec2(double x, double y) {
            this(x, y, -1);
        }
        Vec2(double x, double y, int id) {
            this.x = x;
            this.y = y;
            this.id = id;
        }
    }

    private static void resolveBallRobotCollisions(WorldState world) {
        resolveBallRobotCollisions(world, -1, 0);
    }

    private static void resolveBallRobotCollisions(WorldState world, int ballOwnerId, int ballOwnerTeam) {
        if (world.ball == null) return;

        // Slightly relax separation so the ball is not constantly pushed out before a kick can happen.
        // This helps contested-ball situations where both robots "touch" the ball.
        double minDist = FieldConfig.ROBOT_RADIUS_M + 0.016; // robot radius + (slightly smaller) ball radius
        double minDist2 = minDist * minDist;

        // Our team
        for (Robot r : world.ourRobots) {
            if (ballOwnerId != -1 && ballOwnerTeam == +1 && r.id == ballOwnerId) continue;
            pushBallOutOfRobot(world.ball, r, minDist, minDist2);
        }
        // Opp team
        for (Robot r : world.oppRobots) {
            if (ballOwnerId != -1 && ballOwnerTeam == -1 && r.id == ballOwnerId) continue;
            pushBallOutOfRobot(world.ball, r, minDist, minDist2);
        }
    }

    private static void resolveRobotRobotCollisions(WorldState world) {
        if (world == null) return;

    // Only prevent overlap (no extra distance keeping).
    double minDist = FieldConfig.ROBOT_RADIUS_M * 2.0;
        double minDist2 = minDist * minDist;

        java.util.List<Robot> all = new java.util.ArrayList<>();
        all.addAll(world.ourRobots);
        all.addAll(world.oppRobots);

        // Iterate a couple of times to reduce multi-overlap cases.
        for (int iter = 0; iter < 2; iter++) {
            for (int i = 0; i < all.size(); i++) {
                Robot a = all.get(i);
                for (int j = i + 1; j < all.size(); j++) {
                    Robot b = all.get(j);
                    pushRobotsApart(a, b, minDist, minDist2);
                }
            }
        }
    }

    private static void pushRobotsApart(Robot a, Robot b, double minDist, double minDist2) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double d2 = dx * dx + dy * dy;
        if (d2 >= minDist2) return;

        double d = Math.sqrt(Math.max(d2, 1e-9));
        double nx = dx / d;
        double ny = dy / d;

        double overlap = (minDist - d);
        double push = overlap * 0.5;

        a.x -= nx * push;
        a.y -= ny * push;
        b.x += nx * push;
        b.y += ny * push;

        keepInsideField(a);
        keepInsideField(b);
    }

    private static void pushBallOutOfRobot(Ball ball, Robot robot, double minDist, double minDist2) {
        double dx = ball.x - robot.x;
        double dy = ball.y - robot.y;
        double d2 = dx * dx + dy * dy;
        if (d2 >= minDist2) return;

        double d = Math.sqrt(Math.max(d2, 1e-9));
        double nx = dx / d;
        double ny = dy / d;

        // Place ball on boundary (simple separation)
        ball.x = robot.x + nx * minDist;
        ball.y = robot.y + ny * minDist;

        // Dampen ball velocity along collision normal (crude)
        double vn = ball.vx * nx + ball.vy * ny;
        if (vn < 0) {
            // reflect and damp
            ball.vx -= (1.6 * vn) * nx;
            ball.vy -= (1.6 * vn) * ny;
        }
    }

    private static void integrateBall(WorldState world, double dt) {
        if (world.ball == null) return;

        world.ball.x += world.ball.vx * dt;
        world.ball.y += world.ball.vy * dt;

        // Simple friction
        double damping = 0.98;
        world.ball.vx *= damping;
        world.ball.vy *= damping;

        // Simple wall bounce within field boundaries
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

        // Goal detection: if the ball crosses the goal line within the goal mouth, count a goal.
        double goalHalfW = FieldConfig.GOAL_WIDTH_M / 2.0;
        if (world.ball.x < -halfL && Math.abs(world.ball.y) <= goalHalfW) {
            onGoal(world, -1);
            GOAL_PENDING_TEAM = -1;
            // Put the ball at center immediately; main loop will clear ownership and reset formation.
            world.ball.x = 0.0;
            world.ball.y = 0.0;
            world.ball.vx = 0.0;
            world.ball.vy = 0.0;
            return;
        }
        if (world.ball.x > halfL && Math.abs(world.ball.y) <= goalHalfW) {
            onGoal(world, +1);
            GOAL_PENDING_TEAM = +1;
            world.ball.x = 0.0;
            world.ball.y = 0.0;
            world.ball.vx = 0.0;
            world.ball.vy = 0.0;
            return;
        }

        if (world.ball.x < -halfL) {
            world.ball.x = -halfL;
            world.ball.vx = -world.ball.vx;
        } else if (world.ball.x > halfL) {
            world.ball.x = halfL;
            world.ball.vx = -world.ball.vx;
        }

        if (world.ball.y < -halfW) {
            world.ball.y = -halfW;
            world.ball.vy = -world.ball.vy;
        } else if (world.ball.y > halfW) {
            world.ball.y = halfW;
            world.ball.vy = -world.ball.vy;
        }
    }

    private static void keepInsideField(Robot r) {
        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

        // Keep centers inside play area (roughly)
        double margin = FieldConfig.ROBOT_RADIUS_M;
        r.x = clamp(r.x, -halfL + margin, halfL - margin);
        r.y = clamp(r.y, -halfW + margin, halfW - margin);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
