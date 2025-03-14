package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private static final int LeftMotorChannel = 0;
    private static final int RightMotorChannel = 1;

    private static final int LeftEncoderChannelA = 0; //subject to change
    private static final int LeftEncoderChannelB = 1; //subject to change
    private static final int RightEncoderChannelA = 2; //subject to change
    private static final int RightEncoderChannelB = 3; //subject to change


    private VictorSP rightmotor;
    private VictorSP leftmotor;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private DifferentialDrive drive;

    public Drivetrain(){
        rightmotor = new VictorSP(RightMotorChannel);
        leftmotor = new VictorSP(LeftMotorChannel);

        rightmotor.setInverted(true);
        leftmotor.setInverted(false);

        drive = new DifferentialDrive(leftmotor, rightmotor);

        drive.setSafetyEnabled(true);
        drive.setExpiration(0.1); 

        leftEncoder = new Encoder(LeftEncoderChannelA, LeftEncoderChannelB);
        rightEncoder = new Encoder(RightEncoderChannelA, RightEncoderChannelB);

        leftEncoder.setDistancePerPulse(1.0/2048.0); //Configure these numbers 
        rightEncoder.setDistancePerPulse(1.0/2048.0); //Configure these numbers

        leftEncoder.reset();
        rightEncoder.reset();
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

    public void resetEncoder(){
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getLeftEncoderDistance(){
        return leftEncoder.getDistance();
    }

    public double getRightEncoderDistance(){
        return rightEncoder.getDistance();
    }

    public double getLeftEncoderRate(){
        return leftEncoder.getRate();
    }

    public double getRightEncoderRate(){
        return rightEncoder.getRate();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Encoder Distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("Right Encoder Distance", getRightEncoderDistance());
        SmartDashboard.putNumber("Left Encoder Rate", getLeftEncoderRate());
        SmartDashboard.putNumber("Right Encoder Rate", getRightEncoderRate());
    }
}
