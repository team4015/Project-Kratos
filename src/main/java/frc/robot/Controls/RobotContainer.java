package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Command.Driver;
import frc.robot.Subsystem.Drivetrain;

public class RobotContainer {
    private static final int JOYSTICK_PORT = 0;
    private final Joystick controller = new Joystick(JOYSTICK_PORT);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Driver driver = new Driver(drivetrain, controller, 1, 2);

    public RobotContainer(){
        setConfigurations();
    }

    private void setConfigurations(){
        drivetrain.setDefaultCommand(driver);
    }

    public void disable(){
        drivetrain.stop();
    }
}
