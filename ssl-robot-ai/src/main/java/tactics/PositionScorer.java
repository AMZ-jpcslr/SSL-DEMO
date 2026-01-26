package tactics;

import world.Robot;
import world.WorldState;

/**
 * Scores a candidate position for a specific robot.
 * Higher score means the robot would like to be there.
 */
@FunctionalInterface
public interface PositionScorer {
    double score(WorldState world, Robot self, double x, double y, int teamSign);
}
