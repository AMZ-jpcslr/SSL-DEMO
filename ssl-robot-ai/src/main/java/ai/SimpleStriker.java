package ai;

import world.Ball;
import world.Robot;
import world.WorldState;

public class SimpleStriker implements Behavior {

    // Default: treat this striker as blue-side logic (attacks toward +x).
    // For the red attacker we run it in a mirrored world frame, so +x still means "toward opponent goal".
    private static final double KICK_SPEED = 5.0;

    @Override
    public RobotCommand decide(Robot self, WorldState world) {
        RobotCommand command = new RobotCommand();
        command.robotId = self.id;

        Ball ball = world.ball;

        // Simple logic: Move towards the ball
        double dx = ball.x - self.x;
        double dy = ball.y - self.y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        if (distance > 0.1) { // If not too close to the ball
            command.vx = (dx / distance) * 1.0; // Move at 1 m/s towards the ball
            command.vy = (dy / distance) * 1.0;
            command.omega = 0; // No rotation
            command.kick = false; // Don't kick yet
        } else {
            // If close to the ball, prepare to kick
            command.vx = 0;
            command.vy = 0;
            command.omega = 0;
            command.kick = true; // Kick the ball

            // Kick toward opponent goal center (+x)
            command.kickVx = KICK_SPEED;
            command.kickVy = 0;
        }

        return command;
    }
}