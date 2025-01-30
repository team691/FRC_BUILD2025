package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Sonar;
//import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.commands.basicLime;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.AutoAlign;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Sonar;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */


public class RobotContainer {
  // The robot's subsystems
  public final DriveTrain m_robotDrive = new DriveTrain();
  //private final Climber m_climber = new Climber();
  //private final Lights m_lights = new Lights();
  private final Limelight m_lime = new Limelight();
  private final Sonar m_sonar = new Sonar(0);
  private static boolean sonarOn = true;
  
    // The driver's controller
    Joystick m_joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
    //XboxController m_operator = new XboxController(OIConstants.kDriverControllerPort3);
    public static void changeSonar() {
        if (sonarOn == true) {
            sonarOn = false;
        }
        else {
            sonarOn = true;
        }
  }

  // Initialize Sendable Chooser
  private final SendableChooser<Command> m_chooser;


  // TEST STAGE: Register PathFinder Commands
    // values will be between 0 and 1 in this map
    double[] PowerMap =
    {
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0.1,0.1,0.1,0.15,0.15,
        0.15,0.15,0.15,0.15,0.2,0.2,0.2,0.2,0.2,0.2,
        0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.3,0.3,
        0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.4,0.4,0.4,
        0.4,0.4,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,
        0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,
        0.6,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,
        0.7,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,
        0.9,0.9,0.9,0.9,0.9,1,1,1,1,1,1
    };
   double ReturnValueFromMap(double index)
   {
        return index < 0 ? -PowerMap[(int)(-(index*100))] : PowerMap[(int)(index*100)];
   }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // m_chooser = AutoBuilder.buildAutoChooser();
    boolean isCompetition = true;
    m_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Auto Chooser", m_chooser);
    // Configure the button bindings
    configureButtonBindings();
    // new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
   // Add PathPlanner autonomous
   
    // SmartDashboard.putData("Auto Chooser", m_chooser);
    // SmartDashboard.putNumber("Translation P", 0.0);
    // SmartDashboard.putNumber("Translation I", 0.0);
    // SmartDashboard.putNumber("Translation D", 0.0);
    // SmartDashboard.putNumber("Rotation P", 0.0);
    // SmartDashboard.putNumber("Rotation I", 0.0);
     SmartDashboard.putNumber("Rotation D", 0.0);
    
    // Ignore controller warnings
    DriverStation.silenceJoystickConnectionWarning(true);
    // System.out.println("Value" + ReturnValueFromMap(-MathUtil.applyDeadband(-m_joystick1.getY(), OIConstants.kDriveDeadband)) * setSpeed());
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                // getY() between 1 and -1 down is 1 up is -1
                // getX() between -1 and 1 where right is 1
                // get z is between 1 and -1 spin right is 1
                //max speed value robot can be set to drive is 3, scaling speed is capped around 3
                
                ReturnValueFromMap(-MathUtil.applyDeadband(m_joystick1.getY(), OIConstants.kDriveDeadband)) * setSpeed() , //m_operator.getRawAxis(3)
                ReturnValueFromMap(-MathUtil.applyDeadband(m_joystick1.getX(), OIConstants.kDriveDeadband)) * setSpeed() , // * m_sonar.getSpeed(sonarOn)
                (-MathUtil.applyDeadband(m_joystick2.getZ(), OIConstants.kDriveDeadband)) * 3.5,
                true, true),
            m_robotDrive));
   
    //SmartDashboard.putData(m_chooser);
  }


  // Button mapping and config, pass to JoystickButton
  private void configureButtonBindings() {
        
    new JoystickButton(m_joystick2, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


            // This button for the DRIVER will zero the gyro's angle
    new JoystickButton(m_joystick1, 3)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));


        // Light function for OPERATOR lights amp motor

        new JoystickButton(m_joystick1, 6)
        .toggleOnTrue(Commands.startEnd(
            // Start Action: Begin aligning
            () -> new AutoAlign(m_robotDrive, m_lime).schedule(),
            // End Action: Stop aligning
            () -> new AutoAlign(m_robotDrive, m_lime).cancel(),
            // Subsystems required by the AutoAlignCommand
            m_robotDrive,
            m_lime
        ));

        new JoystickButton(m_joystick1, 5)
        .toggleOnTrue(Commands.startEnd(
            // Start Action: Begin aligning
            () -> changeSonar(),
            // End Action: Stop aligning
            () -> changeSonar(),
            // Subsystems required by the AutoAlignCommand
            m_sonar
        ));
    
  }
    /* Not sure what this does
    new JoystickButton(m_joystick1, 1)
        .onTrue(new RunCommand(
            () -> m_robotDrive.setLimit1()))
        .onFalse(new RunCommand(
            () -> m_robotDrive.unsettling()));
    } */


    //SPEED CMD
    public double setSpeed() {
        if (m_joystick1.getRawButton(1) == true) {
            return 2.0;
        }
        else {
            return 9.0;
        }
    }


/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //m_robotDrive.updatePidValues();
    return m_chooser.getSelected();
  }
}