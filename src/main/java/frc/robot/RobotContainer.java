package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Configs;
import frc.robot.enums.RobotMode;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */


public class RobotContainer {
  // The robot's subsystems
    private static final RobotMode JAVA_SIM_MODE = RobotMode.SIM;
  
    private Lights m_lights = new Lights();
    public final LoggedPowerDistribution powerDistribution;

    // The driver's controller
    public final Controller controller = new Controller();
    
    // Initialize Sendable Chooser
    private final SendableChooser<Command> m_chooser;
    private final SwerveDriveSimulation driveSimulation;

    private final Field2d field = new Field2d();
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      boolean isCompetition = false;
      m_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
          ? stream.filter(auto -> auto.getName().startsWith("comp"))
          : stream
      );

      NamedCommands.registerCommand("Shoot", Shooter.getInstance().shootTest(0.5));
      
      switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                powerDistribution = LoggedPowerDistribution.getInstance(1, PowerDistribution.ModuleType.kRev);

                /* CTRE Chassis: */

            }

            case SIM -> {
                SimulatedArena.overrideSimulationTimings(
                        Seconds.of(0.02), Configs.DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD);
                this.driveSimulation = new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default()
                                .withRobotMass(Configs.DriveTrainConstants.ROBOT_MASS)
                                .withBumperSize(Configs.DriveTrainConstants.BUMPER_LENGTH, Configs.DriveTrainConstants.BUMPER_WIDTH)
                                .withTrackLengthTrackWidth(
                                        Configs.DriveTrainConstants.TRACK_LENGTH, Configs.DriveTrainConstants.TRACK_WIDTH)
                                .withSwerveModule(new SwerveModuleSimulationConfig(
                                        Configs.DriveTrainConstants.DRIVE_MOTOR_MODEL,
                                        Configs.DriveTrainConstants.STEER_MOTOR_MODEL,
                                        Configs.DriveTrainConstants.DRIVE_GEAR_RATIO,
                                        Configs.DriveTrainConstants.STEER_GEAR_RATIO,
                                        Configs.DriveTrainConstants.DRIVE_FRICTION_VOLTAGE,
                                        Configs.DriveTrainConstants.STEER_FRICTION_VOLTAGE,
                                        Configs.DriveTrainConstants.WHEEL_RADIUS,
                                        Configs.DriveTrainConstants.STEER_INERTIA,
                                        Configs.DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION))
                                .withGyro(Configs.DriveTrainConstants.gyroSimulationFactory),
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                powerDistribution = LoggedPowerDistribution.getInstance();
                // Sim robot, instantiate physics sim IO implementations
                SimulatedArena.getInstance().resetFieldForAuto();
            }

            default -> {
              driveSimulation = null;
              powerDistribution = LoggedPowerDistribution.getInstance();
            }
        }

        SmartDashboard.putData("Auto Chooser", m_chooser);

      // Add PathPlanner autonomous
      new EventTrigger("Shoot").whileTrue(Shooter.getInstance().shootTest(-0.50));
      
      // Ignore controller warnings
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void resetSimulationField() {
      if (!Robot.isSimulation()) return;

      DriveTrain.getInstance().resetOdometry(new Pose2d(3, 3, new Rotation2d()));
      SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
      if (!Robot.isSimulation()) return;

      SimulatedArena.getInstance().simulationPeriodic();
      Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
      Logger.recordOutput(
              "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
      Logger.recordOutput(
              "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
  /* 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}