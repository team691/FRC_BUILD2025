package frc.robot.Constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
/* Constants class holds static variables that are referrenced in other classes
 * This class should not have any functions or other purposes
 * 
 */
public final class Constants {
    public static final double X_REEF_ALIGNMENT_P = 3.3;
    public static final double Y_REEF_ALIGNMENT_P = 3.3;
    public static final double ROT_REEF_ALIGNMENT_P = 0.058;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;



  // Drive constants pertain to the handling and motors of the drivetrain
  public static final class DriveConstants {    
    // Driving parameters
    public static final double kMaxSpeedMetersPerSecond = 4.0; //set to 4.0
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    //public double speedmult = 6;

    // Slew rates
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = .7; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = .8; // percent per second (1 = 100%)
    // previously 1.8 and 2.0

    // Swerve drivetrain physical distances
    public static final double kTrackWidth = Units.inchesToMeters(28);
    public static final double kWheelBase = Units.inchesToMeters(28);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d  (-kWheelBase / 2, -kTrackWidth / 2)); //All of these are inside the object :]

    // Angular offset variables (radians)
    public static final double kFrontLeftChassisAngularOffset = -Math.PI/2; 
    public static final double kFrontRightChassisAngularOffset = 0; // 0
    public static final double kBackLeftChassisAngularOffset = Math.PI; // Math.PI
    public static final double kBackRightChassisAngularOffset = Math.PI/2; // Math.PI/2

    // SPARK MAX motor controller CAN Ids 
    // Driving motors
    public static final int kFrontLeftDrivingCanId = 4;//8;
    public static final int kRearLeftDrivingCanId = 6;//2;
    public static final int kFrontRightDrivingCanId = 2;//6;
    public static final int kRearRightDrivingCanId = 8;//4;
    // Turning motors
    public static final int kFrontLeftTurningCanId = 3;//7;
    public static final int kRearLeftTurningCanId = 5;//1;
    public static final int kFrontRightTurningCanId = 1;//5;
    public static final int kRearRightTurningCanId = 7;//3;

    // Is gyro reversed?
    public static final boolean kGyroReversed = true;
  }
  public static final class ButtonBoardConstants{
    public static final int ReefID = 2;
    public static final int BlueID = 3;

    public static final class Blue {
      public static final int ClimbF = 1;
      public static final int ClimbR = 2;
      public static final int Outtake = 3;
      public static final int Align = 4;
    }

    public static final class Reef {
      public static final int FarLeftRed = 1;
      public static final int CloseLeftWhite = 2;
      public static final int CloseLeftRed = 3;
      public static final int CloseWhite = 4;
      public static final int CloseRed = 5;
      public static final int CloseRightWhite = 6;
      public static final int CloseRightRed = 7;
      public static final int FarRightWhite = 8;
      public static final int FarRightRed = 9;
      public static final int FarWhite = 10;
      public static final int FarRed = 11;
      public static final int FarLeftWhite = 12;
    }
  }

  public static final class ElevatorConstants {
    public static final double LOW_POSITION = 1.0;  // example rotations
    public static final double MID_POSITION = 2.5;
    public static final double HIGH_POSITION = 4.0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 74.75/1000; //Top right wheel is a millimeter off
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04; //0.04 
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1.0; //1
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1.0;
    public static final double kTurningMaxOutput = 1.0;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 42; // amps
    public static final int kTurningMotorCurrentLimit = 18; // amps

    public static final double kRadiusFromModule = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
    public static final int kDriverControllerPort3 = 2;
    public static final double kDriveDeadband = 0.2;
  }

  public static final class LightConstants {
    public static final double brightness = 1.0;
    public static final double speed = 0.5;
    public static final int numLed = 20;
    public static final double sparking = 0.7;
    public static final double cooling = 0.3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ShooterConstants{
    public static final int ShooterID = 9;
    public static final int PassThroughID = 10;

    public static final double ShooterPower = -0.6;
    public static final double PassThroughPower = 0.85;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676.0;
  }
}
