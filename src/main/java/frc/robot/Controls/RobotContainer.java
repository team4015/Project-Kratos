package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Command.Driver;
import frc.robot.Subsystem.Drivetrain;

public class RobotContainer {
    private final Joystick controller = new Joystick(0);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Driver driver = new Driver(drivetrain, controller);

    public RobotContainer(){
        setConfigurations();
    }

    private void setConfigurations(){
        drivetrain.setDefaultCommand(driver);
    }
    
}
