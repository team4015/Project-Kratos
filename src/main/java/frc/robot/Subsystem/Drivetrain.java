package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private static final int LeftMotorChannel = 0;
    private static final int RightMotorChannel = 1;

    private VictorSP rightmotor;
    private VictorSP leftmotor;

    private DifferentialDrive drive;

    public Drivetrain(){
        rightmotor = new VictorSP(RightMotorChannel);
        leftmotor = new VictorSP(LeftMotorChannel);

        rightmotor.setInverted(true);
        leftmotor.setInverted(false);

        drive = new DifferentialDrive(leftmotor, rightmotor);
        drive.setSafetyEnabled(true);
        drive.setExpiration(0.1); 
    }

    public void drive(double speed, double turn){
        speed = applyDeadband(speed, 0.02);
        turn = applyDeadband(turn, 0.02);
        drive.arcadeDrive(speed, turn);
    }

    private double applyDeadband(double value, double deadband){
        if(Math.abs(value) < deadband){
            return 0.0;
        }
        return value;
    }

    public void stop(){
        drive.arcadeDrive(0, 0);
    }
}
