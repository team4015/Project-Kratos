package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Command.Driver;
import frc.robot.Subsystem.Drivetrain;

public class RobotContainer {
    private final XboxController controller = new XboxController(0);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Driver driver = new Driver(drivetrain, controller);

    public RobotContainer(){
        setConfigurations();
    }

    private void setConfigurations(){
        drivetrain.setDefaultCommand(driver);
    }
    
}
