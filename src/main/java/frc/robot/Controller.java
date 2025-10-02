package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Test;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.AlignToReefTagRelative;

public class Controller extends SubsystemBase{

    Joystick m_joystick1 = new Joystick(0);
    Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
    // Limelight m_lime = new Limelight(DriveTrain.getInstance());

    boolean shouldRunBelt = true;
    boolean isPressed = false;
    boolean isBeltOn = false;
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

        new JoystickButton(m_joystick1, 4)
            .whileTrue(Shooter.getInstance().shootTest(-Constants.ShooterConstants.ShooterPower))
            .whileFalse(Shooter.getInstance().stopShoot());

        new JoystickButton(m_joystick1, 10)
            // .whileTrue(new AlignToReefTagRelative(true, DriveTrain.getInstance()));
            .whileTrue(AutoAlign.getInstance());
        //     // .whileFalse(new AlignToReefTagRelative(false, DriveTrain.getInstance()));

        new JoystickButton(m_joystick1, 11)
            .whileTrue(Test.getInstance());

        new JoystickButton(m_joystick1, 7)
            .onTrue(Elevator.getInstance().goLow());

        new JoystickButton(m_joystick1, 8)
            .onTrue(Elevator.getInstance().goMid());

        new JoystickButton(m_joystick1, 9)
            .onTrue(Elevator.getInstance().goHigh());

        new JoystickButton(m_joystick2, 6)
            .onTrue(Climber.getInstance().actuator(0.0))
            .onFalse(Climber.getInstance().actuator(1.0));

        new JoystickButton(m_joystick2, 3)
          .whileTrue(Climber.getInstance().climb());

        new JoystickButton(m_joystick2, 4)
          .whileTrue(Climber.getInstance().lower());

        new JoystickButton(m_joystick2, 5)
            .whileTrue(Climber.getInstance().stop());  
    }
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
