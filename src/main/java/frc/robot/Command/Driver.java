package frc.robot.Command;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Drivetrain;

public class Driver extends Command{
    private final Drivetrain drivetrain;
    private final XboxController controller;

    public Driver(Drivetrain drivetrain, XboxController controller){
        this.drivetrain = drivetrain;
        this.controller = controller;

        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        double speed = -controller.getLeftY();
        double turn = controller.getLeftX();

        drivetrain.drive(speed, turn);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
