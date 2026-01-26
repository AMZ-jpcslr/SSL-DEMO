import java.util.List;

public class Ball{
    public double x, y;
    public double vx, vy;
}

public class Robot{
    public int id;
    public double x, y;
    public double orientation; 
}

public class WorldState{
    public List<Robot> ourRobots;
    public List<Robot> oppRobots;
    public Ball ball;
}

public class Robotcommand{
    public int robotId;
    public double vx, vy;
    public double omega;
    public boolean kick;
}

public interface Behavior{
    Robotcommand decide(WorldState state, Robot self);
}