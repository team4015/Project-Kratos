package frc.robot.Command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Drivetrain;

public class Driver extends Command{
    private final Drivetrain drivetrain;
    private final Joystick controller;

    public Driver(Drivetrain drivetrain, Joystick controller){
        this.drivetrain = drivetrain;
        this.controller = controller;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.resetEncoder();
    }

    @Override
    public void execute(){
        double speed = -controller.getY();
        double turn = controller.getX();

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
