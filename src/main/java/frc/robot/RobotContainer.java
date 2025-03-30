package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lights;
//import frc.robot.utils.AlertsManager;

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
import frc.robot.Constants.Configs;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.enums.RobotMode;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.BeamBreakers;
// import frc.robot.subsystems.Sonar;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
    // public final BeamBreakers m_beam = new BeamBreakers();
    // private final Shooter m_levelone = new LevelOne();

    private final AutoAlign m_align = new AutoAlign();
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
      // m_chooser = AutoBuilder.buildAutoChooser();
      boolean isCompetition = false;
      m_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
          ? stream.filter(auto -> auto.getName().startsWith("comp"))
          : stream
      );

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

      //driveSimulation = new SwerveDriveSimulation(null, null);
      SmartDashboard.putData("Auto Chooser", m_chooser);
      // Add PathPlanner autonomous
      //  new EventTrigger("align").and(new Trigger(m_align::execute)).onTrue(Commands.print("auto align"));
      
      // Ignore controller warnings
      DriverStation.silenceJoystickConnectionWarning(true);
       
      //SmartDashboard.putData(m_chooser);
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
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_align;
  }
}