package frc.robot;

import java.lang.reflect.Array;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoAlign;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Controller extends SubsystemBase{

    Joystick m_joystick1 = new Joystick(0);
    Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
    Limelight m_lime = new Limelight(DriveTrain.getInstance());
    boolean shouldRunBelt = true;
    boolean isPressed = false;
    boolean isBeltOn = false;

    double m_joystick1_degrees = m_joystick1.getDirectionDegrees();
    double m_joystick2_degrees = m_joystick2.getDirectionDegrees();
    double m_joystick1_magnitude = m_joystick1.getMagnitude();
    double m_joystick2_magnitude = m_joystick2.getMagnitude();

    double[] m_joystick1_pos = {m_joystick1.getX(), m_joystick1.getY(), m_joystick1.getZ()};
    double[] m_joystick2_pos = {m_joystick2.getX(), m_joystick2.getY(), m_joystick2.getZ()};

    // values will be between 0 and 1 in this map
    private double[] PowerMap =
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
    private double ReturnValueFromMap(double index) {
        return index < 0 ? -PowerMap[(int)(-(index*100))] : PowerMap[(int)(index*100)];
    }
    private double setSpeed() {
        if (m_joystick1.getRawButton(1) == true) {
            return 2.0; // 9.0
        }
        else {
            return 8.0; // 2.0
        }
    }
    public Controller (){
        DriveTrain.getInstance().setDefaultCommand(new RunCommand(
              () -> DriveTrain.getInstance().drive(
                  
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick1.getY(), OIConstants.kDriveDeadband)) * setSpeed() , //m_operator.getRawAxis(3)
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick1.getX(), OIConstants.kDriveDeadband)) * setSpeed() , // * m_sonar.getSpeed(sonarOn)
                  (-MathUtil.applyDeadband(m_joystick2.getZ(), OIConstants.kDriveDeadband)) * 3.25,
                  true, true),
              DriveTrain.getInstance()));
        //buttonBoard.SetupButtons();
        configureButtonBindings();

        Shuffleboard.getTab("Joysticks").add("Joystick1 Degrees", m_joystick1_degrees);
        Shuffleboard.getTab("Joysticks").add("Joystick2 Degrees", m_joystick2_degrees);
        Shuffleboard.getTab("Joysticks").add("Joystick1 Magnitude", m_joystick1_magnitude);
        Shuffleboard.getTab("Joysticks").add("Joystick1 Magnitude", m_joystick2_magnitude);
        Shuffleboard.getTab("Joysticks").add("Joystick1 Position", m_joystick1_pos);
        Shuffleboard.getTab("Joysticks").add("Joystick2 Position", m_joystick2_pos);
    }

    //configures all buttons
    private void configureButtonBindings(){
        new JoystickButton(m_joystick2, 12)
            .whileTrue(new RunCommand(
                () -> DriveTrain.getInstance().setX(),
                DriveTrain.getInstance()));

        // This button for the DRIVER will zero the gyro's angle
        new JoystickButton(m_joystick1, 12)
            .whileTrue(new RunCommand(
                () -> DriveTrain.getInstance().zeroHeading(),
                DriveTrain.getInstance()));

        new JoystickButton(m_joystick1, 3)
            .whileTrue(Shooter.getInstance().shootTest(Constants.ShooterConstants.ShooterPower))
            .whileFalse(Shooter.getInstance().stopShoot());

        new JoystickButton(m_joystick1, 5)
            .whileTrue(Shooter.getInstance().shootTest(-Constants.ShooterConstants.ShooterPower))
            .whileFalse(Shooter.getInstance().stopShoot());

        new JoystickButton(m_joystick2, 6)
            .onTrue(Climber.getInstance().actuator(0.0))
            .onFalse(Climber.getInstance().actuator(1.0));

        new JoystickButton(m_joystick2, 3)
          .whileTrue(Climber.getInstance().climb());

        new JoystickButton(m_joystick2, 4)
          .whileTrue(Climber.getInstance().lower());

        new JoystickButton(m_joystick2, 5)
            .whileTrue(Climber.getInstance().stop());

        new JoystickButton(m_joystick1, m_joystick1.getPOV(90))
            .onTrue(new AutoAlign(true, DriveTrain.getInstance()).withTimeout(3));
    }
    // m_joystick1.povRight().onTrue(new AlignToReefTagRelative(true, drivebase).withTimeout(3));
	// m_joystick1.povLeft().onTrue(new AlignToReefTagRelative(false, drivebase).withTimeout(3));

    @Override
    public void periodic(){
        if(shouldRunBelt && !isPressed && m_joystick1.getRawButton(4)){
            isPressed = true;
            if(!isBeltOn){
                Shooter.getInstance().passThrough().execute();
                isBeltOn = true;
            }
            else {
                isBeltOn = false;
                Shooter.getInstance().stopPass().execute();
            }
        }
        else if (shouldRunBelt && !m_joystick1.getRawButton(4)){
            isPressed = false;
        }
    }
    public void TurnOffBelt() {
        shouldRunBelt = false;
        isBeltOn = false;
        isPressed = false;
    }
    public void TurnOnBelt(){
        shouldRunBelt = true;
    }
}
