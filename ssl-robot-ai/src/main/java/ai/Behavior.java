package ai;

import world.Robot;
import world.WorldState;

public interface Behavior {

    RobotCommand decide(Robot self, WorldState world);
}
