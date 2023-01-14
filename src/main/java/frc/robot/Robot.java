// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.AprilTags.AprilSetup;
import frc.Field.RoboField;
import org.photonvision.RobotPoseEstimator;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot
{
    AHRS gyro = new AHRS();
    private final WPI_TalonSRX[] wpi_talonSRXES = new WPI_TalonSRX[]{new WPI_TalonSRX(1),new WPI_TalonSRX(3),new WPI_TalonSRX(2),new WPI_TalonSRX(4)};
    private final MotorControllerGroup leftDrive = new MotorControllerGroup(wpi_talonSRXES[0],wpi_talonSRXES[1]);
    private final MotorControllerGroup rightDrive = new MotorControllerGroup(wpi_talonSRXES[2],wpi_talonSRXES[3]);
    private static double rotateVal;
     private XboxController m_controller = new XboxController(0);
    private final Timer timer = new Timer();
    private static RobotPoseEstimator poseEstimator;

    private static Pose3d robotPose3d;
    private static Pose2d robotPose2d;





    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        RoboField.fieldSetup();
        m_controller = new XboxController(0);
        rightDrive.setInverted(true);
        poseEstimator = AprilSetup.getRobotPoseEstimator();
        poseEstimator.update();
        robotPose3d = poseEstimator.getReferencePose();
    }
    
    
    /** This method is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit()
    {
        timer.reset();
        timer.start();
        
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
      robotPose2d = RoboField.fieldUpdate(AprilSetup.getEstimatedGlobalPose(robotPose3d.toPose2d()).getFirst()); // position of robot on the field

    }
    
    
    /** This method is called once each time the robot enters teleoperated mode. */
    @Override
    public void teleopInit() {
        leftDrive.set(0);
        rightDrive.set(0);
    }
    @Override
    public void disabledPeriodic(){}
    
    @Override
    public void robotPeriodic(){


    }

    /** This method is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic()
    {
        if(m_controller.getAButtonPressed()){
            leftDrive.stopMotor();
            rightDrive.stopMotor();
        }
        
        
        double forward = m_controller.getLeftTriggerAxis()*.6;
        double reverse = m_controller.getRightTriggerAxis()*.6;
        rotateVal = m_controller.getRightX()*.6;
        
        leftDrive.set(forward+rotateVal);
        rightDrive.set(forward-rotateVal);
        
        leftDrive.set(-1.0 * reverse - rotateVal);
        rightDrive.set(-1.0 * reverse + rotateVal);
        
        
    }
    
    
    
    
    //TODO: Setup photonvision sim testing
    @Override
    public void testInit() {
    
    }
    
    @Override
    public void simulationInit() {
    
    
    
    }
    
    @Override
    public void simulationPeriodic() {

    }
    
    @Override
    public void testPeriodic() {}
}
