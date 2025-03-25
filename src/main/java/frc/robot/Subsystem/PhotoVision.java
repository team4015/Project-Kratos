package frc.robot.Subsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class PhotoVision extends TimedRobot {
    private static final int LeftMotorChannel = 0;
    private static final int RightMotorChannel = 1;

    private ADXRS450_Gyro gyro;
    private PhotonCamera camera;
    private DifferentialDrive drive;

    private PIDController driveController;
    private PIDController turnController;

    VictorSP leftMotor;
    VictorSP rightMotor;

    private double targetYaw = 0;
    private double targetDistance = 1.0;

    public PhotoVision(){
        leftMotor = new VictorSP(LeftMotorChannel);
        rightMotor = new VictorSP(RightMotorChannel);

        rightMotor.setInverted(true);
        leftMotor.setInverted(false);

        SendableRegistry.addChild(drive, leftMotor);
        SendableRegistry.addChild(drive, rightMotor);

        drive.setSafetyEnabled(true);
        drive.setExpiration(0.1); 
    }

    @Override
    public void robotInit(){
        gyro.calibrate();
        gyro.reset();
    }

    @Override
    public void teleopPeriodic(){
        var result = camera.getLatestResult();

        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();

            double yaw = target.getYaw();
            double distance = target.getBestCameraToTarget().getTranslation().getX();

            double forwardSpeed = driveController.calculate(distance, targetDistance);
            double rotationSpeed = turnController.calculate(gyro.getAngle(), targetYaw);

            drive.arcadeDrive(forwardSpeed, rotationSpeed);

            System.out.println("Yaw: " + yaw + ", Distance: " + distance);
            System.out.println("Foward Speed: " + forwardSpeed + ", Rotation Speed: " + rotationSpeed);

        }

        else{
            drive.arcadeDrive(0, 0);
            System.out.println("No AprilTag detected");
        }

    }
    
}
