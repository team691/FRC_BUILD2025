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
import frc.robot.commands.AutoAlign;
import frc.robot.enums.RobotMode;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.BeamBreakers;
// import frc.robot.subsystems.Sonar;

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
       
      //SmartDashboard.putData(m_chooser);
    }


/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_align;
  }
}