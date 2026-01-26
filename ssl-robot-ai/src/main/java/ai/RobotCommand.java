package ai;

public class RobotCommand {
    public int robotId;
    public double vx, vy;
    public double omega;
    public boolean kick;

    // Optional: when kick is intended as a shot on goal.
    // Used by the simulator for learning/reward signals. Defaults to false.
    public boolean shotIntent = false;

    /**
     * Optional kick direction in field coordinates.
     * If (kickVx,kickVy) is (0,0), sim may fall back to a default kick direction.
     */
    public double kickVx, kickVy;

    // Optional: when kick is intended as a pass, set this to the teammate id.
    // The simulator can use it for debugging/learning. -1 means "not a pass".
    public int passTargetId = -1;
}
