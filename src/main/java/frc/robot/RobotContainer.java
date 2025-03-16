package frc.robot;

import java.util.Objects;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.events.EventTrigger;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lights;

// import frc.robot.subsystems.Level2;
// import frc.robot.subsystems.LevelOne;
// import frc.robot.subsystems.Sonar;
//import frc.robot.subsystems.Lights;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.commands.basicLime;
import com.pathplanner.lib.events.EventTrigger;
// import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.AutoAlign;
import frc.robot.enums.RobotMode;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.BeamBreakers;
// import frc.robot.subsystems.Sonar;
import frc.robot.subsystems.Climber;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */


public class RobotContainer {
  // The robot's subsystems
    private static final RobotMode JAVA_SIM_MODE = RobotMode.SIM;
    
    private final Climber m_climber = new Climber();
    private Lights m_lights = new Lights();
    // public final BeamBreakers m_beam = new BeamBreakers();
    // private final LevelOne m_levelone = new LevelOne();

    private final AutoAlign m_align = new AutoAlign();
    

    private final SwerveDriveSimulation driveSimulation;

    private final Field2d field = new Field2d();
    // The driver's controller
    private final Controller controller = new Controller();
    
    // Initialize Sendable Chooser
    private final SendableChooser<Command> m_chooser;
      
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      // m_chooser = AutoBuilder.buildAutoChooser();
      boolean isCompetition = false;
      m_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
          ? stream.filter(auto -> auto.getName().startsWith("comp"))
          : stream
      );
  
      SmartDashboard.putData("Auto Chooser", m_chooser);
      // Configure the button bindings
      //   new EventTrigger("L2").and(new Trigger(m_level2::intake)).onTrue(Commands.print("shoot note"));
      // Add PathPlanner autonomous
      //  new EventTrigger("align").and(new Trigger(m_align::execute)).onTrue(Commands.print("auto align"));
      
        
        // Ignore controller warnings
        DriverStation.silenceJoystickConnectionWarning(true);
        // System.out.println("Value" + ReturnValueFromMap(-MathUtil.applyDeadband(-m_joystick1.getY(), OIConstants.kDriveDeadband)) * setSpeed());
        // Configure default commands
       
        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                powerDistribution = LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kCTRE);

                /* REV Chassis */
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.CTRE_ON_CANIVORE,
                        new GyroIOPigeon2(TunerConstants.DrivetrainConstants),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));

                aprilTagVision = new AprilTagVision(new AprilTagVisionIOReal(camerasProperties), camerasProperties);
            }

            case SIM -> {
                SimulatedArena.overrideSimulationTimings(
                        Seconds.of(Robot.defaultPeriodSecs), DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD);
                this.driveSimulation = new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default()
                                .withRobotMass(DriveTrainConstants.ROBOT_MASS)
                                .withBumperSize(DriveTrainConstants.BUMPER_LENGTH, DriveTrainConstants.BUMPER_WIDTH)
                                .withTrackLengthTrackWidth(
                                        DriveTrainConstants.TRACK_LENGTH, DriveTrainConstants.TRACK_WIDTH)
                                .withSwerveModule(new SwerveModuleSimulationConfig(
                                        DriveTrainConstants.DRIVE_MOTOR_MODEL,
                                        DriveTrainConstants.STEER_MOTOR_MODEL,
                                        DriveTrainConstants.DRIVE_GEAR_RATIO,
                                        DriveTrainConstants.STEER_GEAR_RATIO,
                                        DriveTrainConstants.DRIVE_FRICTION_VOLTAGE,
                                        DriveTrainConstants.STEER_FRICTION_VOLTAGE,
                                        DriveTrainConstants.WHEEL_RADIUS,
                                        DriveTrainConstants.STEER_INERTIA,
                                        DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION))
                                .withGyro(DriveTrainConstants.gyroSimulationFactory),
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                powerDistribution = LoggedPowerDistribution.getInstance();
                // Sim robot, instantiate physics sim IO implementations
                final ModuleIOSim frontLeft = new ModuleIOSim(driveSimulation.getModules()[0]),
                        frontRight = new ModuleIOSim(driveSimulation.getModules()[1]),
                        backLeft = new ModuleIOSim(driveSimulation.getModules()[2]),
                        backRight = new ModuleIOSim(driveSimulation.getModules()[3]);
                final GyroIOSim gyroIOSim = new GyroIOSim(driveSimulation.getGyroSimulation());
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        gyroIOSim,
                        (canBusInputs) -> {},
                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight);

                aprilTagVision = new AprilTagVision(
                        new ApriltagVisionIOSim(
                                camerasProperties,
                                VisionConstants.fieldLayout,
                                driveSimulation::getSimulatedDriveTrainPose),
                        camerasProperties);

                SimulatedArena.getInstance().resetFieldForAuto();
            }

            default -> {
                this.driveSimulation = null;

                powerDistribution = LoggedPowerDistribution.getInstance();
                // Replayed robot, disable IO implementations
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        (canBusInputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {});

                aprilTagVision = new AprilTagVision((inputs) -> {}, camerasProperties);
            }
        }
      //SmartDashboard.putData(m_chooser);
    }


/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //m_robotDrive.updatePidValues();
    return m_align;
  }
}