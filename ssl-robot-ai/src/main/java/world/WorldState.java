package world;

import java.util.ArrayList;
import java.util.List;

public class WorldState {
    public List<Robot> ourRobots = new ArrayList<>();
    public List<Robot> oppRobots = new ArrayList<>();
    public Ball ball;

    public double fieldLength;
    public double fieldWidth;
}