package frc.robot.subsystems;

// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
// Swerve specific imports
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveUtils;
// Position imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Constants
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;

import java.io.IOException;
// import com.pathplanner.lib.util.ReplanningConfig;
import java.util.Map;

import org.json.simple.parser.ParseException;
/* The DriveTrain class handles the drive subsystem of the robot.
 * Configured for REV Robotics Swerve Modules
 * 
 */
public class DriveTrain extends SubsystemBase {
    // Creates swerve modules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  /* Gyro sensor, mainly used for field relativity
   * NavX Gyro requires 3rd party vendor libraries
   * 
   */
  private final AHRS m_navx = new AHRS(NavXComType.kMXP_SPI);
  
  // Slew rate variables
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  // Slew rate controlls lateral acceleration of robot
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  //private final int x;
  private static RobotConfig config;
  // public static RobotConfig config; 
  // RobotConfig config;

  /* Odemetry tracks the robot pose utilizing the gyro
   * mainly used for field relativity
   * 
   */
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_navx.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  // Creates new Drive Subsystem
  public DriveTrain() {
    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    // RobotConfig config;
    
    try{
      // RobotConfig config = new RobotConfig.fromGUISettings();
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      // new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.3, 0.0, 0.0), // Translation PID constants P between 5 and 6
                    new PIDConstants(0.35, 0.0, 0.0) // Rotation PID constants
            ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        //   return alliance.get() == DriverStation.Alliance.Red;
        // }
        return true;
      },
      this // Reference to this subsystem to set requirements
    );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      throw new RuntimeException("Failed to configure AutoBuilder", e);
    }
}
    

public void updatePidValues() {
  // double translationP = SmartDashboard.getNumber("Translation P", 5.0);
  // double translationI = SmartDashboard.getNumber("Translation I", 0.0);
  // double translationD = SmartDashboard.getNumber("Translation D", 0.0);
  // double rotationP = SmartDashboard.getNumber("Rotation P", 5.0);
  // double rotationI = SmartDashboard.getNumber("Rotation I", 0.0);
  // double rotationD = SmartDashboard.getNumber("Rotation D", 0.0);
  
  // // Set new PID constants
  // PIDConstants translationPID = new PIDConstants(translationP, translationI, translationD);
  // PIDConstants rotationPID = new PIDConstants(rotationP, rotationI, rotationD);
  
} 

  // Updates odometry periodically
  public void periodic() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_navx.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
      updatePidValues();
  }

//   // Returns estimated robot pose
   public Pose2d getPose() {
     return m_odometry.getPoseMeters();
   }

//   public Rotation2d getRotation2d() {
//     return Rotation2d.fromDegrees(getHeading());
//   }

//   // Resets odometry to a specific pose
  public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(
         Rotation2d.fromDegrees(m_navx.getAngle()),
         new SwerveModulePosition[] {
             m_frontLeft.getPosition(),
             m_frontRight.getPosition(),
             m_rearLeft.getPosition(),
             m_rearRight.getPosition()
         },
         pose);
}

  /* Drive method for swerve
   * Uses two joysticks
   * 
   * xSpeed         = speed of robot in x direction
   * ySpeed         = speed of robot in y direction
   * rot              = angular rate of the robot
   * fieldRelative  = if x and y speeds are relative to the field
   * rateLimit      = enables smoother controlling 
   * 
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0;
      }
      
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { 
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Speed conversion into correct units
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    /*
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_navx.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    */

    ChassisSpeeds chassisSpeeds;
        if (fieldRelative = true) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading()));
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        }
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(moduleStates);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,true, true);
  }

  /* Drive command, stop function
   * While held, wheels stay in a X formation, and speed is set to 0
   * 
   */
  public void setX() {
    //check
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /* Sets swerve module states
   * utilizes desiredstates to set robot to correct state
   *
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  // Resets drive encoders to read a position of 0
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  // Zeros robot heading
  public void zeroHeading() {
    m_navx.reset();
  }

  /* Returns the heading of the robot
   * Returns the heading from -180 to 180 degrees
   * 
   */
  public double getHeading() {
    return Math.IEEEremainder(-m_navx.getAngle(), 360.0);
  }

  /* Returns turn rate of the Robot
   * Uses gyro to determine rate
   * 
   */
  public double getTurnRate() {
    return m_navx.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setLimit1(){
    m_frontLeft.setlimit1();
    m_rearLeft.setlimit1();
    m_rearRight.setlimit1();
    m_frontRight.setlimit1();
  }

  public void unsettling(){
    m_frontLeft.unsettling();
    m_rearLeft.unsettling();
    m_rearRight.unsettling();
    m_frontRight.unsettling();
  }

  public Object getTrajectoryConfig() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTrajectoryConfig'");
  }
}