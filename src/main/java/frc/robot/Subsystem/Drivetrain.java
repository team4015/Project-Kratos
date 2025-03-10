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

        rightmotor.setInverted(false);
        leftmotor.setInverted(false);

        drive = new DifferentialDrive(leftmotor, rightmotor);
    }

    public void drive(double speed, double turn){
        drive.arcadeDrive(speed, turn);
    }

    public void stop(){
        drive.arcadeDrive(0, 0);
    }

}
