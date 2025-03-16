package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class Controller {
    Joystick m_joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
    //XboxController m_operator = new XboxController(OIConstants.kDriverControllerPort3);

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
    private double ReturnValueFromMap(double index)
    {
        return index < 0 ? -PowerMap[(int)(-(index*100))] : PowerMap[(int)(index*100)];
    }
    private double setSpeed() {
        if (m_joystick1.getRawButton(1) == true) {
            return 9.0; // 9.0
        }
        else {
            return 2.0; // 2.0
        }
    }
    public Controller (){
        DriveTrain.getInstance().setDefaultCommand(new RunCommand(
              () -> DriveTrain.getInstance().drive(
                  
                  ReturnValueFromMap(-MathUtil.applyDeadband(m_joystick1.getY(), OIConstants.kDriveDeadband)) * setSpeed() , //m_operator.getRawAxis(3)
                  ReturnValueFromMap(-MathUtil.applyDeadband(m_joystick1.getX(), OIConstants.kDriveDeadband)) * setSpeed() , // * m_sonar.getSpeed(sonarOn)
                  (-MathUtil.applyDeadband(m_joystick2.getZ(), OIConstants.kDriveDeadband)) * 3.5,
                  true, true),
              DriveTrain.getInstance()));

        configureButtonBindings();
    }

    //configures all buttons
    private void configureButtonBindings(){
        new JoystickButton(m_joystick2, Button.kR1.value)
            .whileTrue(new RunCommand(
                () -> DriveTrain.getInstance().setX(),
                DriveTrain.getInstance()));

        // This button for the DRIVER will zero the gyro's angle
        new JoystickButton(m_joystick1, 12)
            .whileTrue(new RunCommand(
                () -> DriveTrain.getInstance().zeroHeading(),
                DriveTrain.getInstance()));

        new JoystickButton(m_joystick1, 3)
          .whileTrue(new RunCommand(
            () -> m_climber.climb(),
            m_climber)).whileFalse(new RunCommand(() -> m_climber.stop(),m_climber));

        new JoystickButton(m_joystick1, 4)
          .whileTrue(new RunCommand(
            () -> m_climber.lower(),
            m_climber));

        // new JoystickButton(m_joystick1, 5)
        //   .whileTrue(new RunCommand(
        //     () -> m_intake.intake(),
        //     m_intake));

        // new JoystickButton(m_joystick1, 6)
        //   .whileTrue(new RunCommand(
        //     () -> m_intake.shoot(),
        //     m_intake));

        // new JoystickButton(m_joystick1, 4)
        //     .whileTrue(new RunCommand(
        //         () -> m_intake.intake(0.6), // Move whole intake mechanism up
        //         m_robotDrive));
        
        // new JoystickButton(m_joystick1, 6)
        //     .whileTrue(new RunCommand(
        //         () -> m_intake.outtake(-0.5), // Move whole intake mechanism down
        //         m_robotDrive));
    
        // Light function for OPERATOR lights amp motor
  
        //   new JoystickButton(m_joystick1, 5)
        //   .toggleOnTrue(Commands.startEnd(
        //       // Start Action: Begin aligning
        //       () -> changeSonar(),
        //       // End Action: Stop aligning
        //       () -> changeSonar(),
        //       // Subsystems required by the AutoAlignCommand
        //       m_sonar
        //   ));
  
          // new JoystickButton(m_joystick2)
          // .whileTrue(
          //     new RunCommand(
          //         () -> m_alage.intake()
          //     ),
          //     m_joystick2
          // )
          // new JoystickButton(m_joystick1, 3)
          // .whileTrue(new RunCommand(
          //     () -> m_alage.uplift(),
          //     m_alage));
          // This line is causing the error
        // Bind "intake" to button 1 on joystick 2 (change button number as needed)
            // new JoystickButton(m_joystick2, 1)
            //     .whileTrue(new RunCommand(m_algae::intake, m_algae));

            // new JoystickButton(m_joystick2, 2)
            //     .whileTrue(new RunCommand(m_algae::outTake, m_algae));
        
            // // Bind "uplift" to button 2 on joystick 2
            // new JoystickButton(m_joystick2, 3)
            //     .whileTrue(new RunCommand(m_algae::uplift, m_algae));
        
            // // Bind "lower" to button 3 on joystick 2
            // new JoystickButton(m_joystick2, 4)
            //     .whileTrue(new RunCommand(m_algae::lower, m_algae));
        
            // // Bind "stop" to button 4 on joystick 2
            // new JoystickButton(m_joystick2, 5)
            //     .onTrue(new RunCommand(m_algae::stop, m_algae));

            // new JoystickButton(m_joystick1, 4)
            // .whileTrue(new RunCommand(
            //     () -> m_fourbar.setPosition(5),
            //     m_fourbar));

            // new JoystickButton(m_joystick1, 7)
            // .whileTrue(new RunCommand(
            //     () -> m_levelone.outake(),
            //     m_levelone));

            // new JoystickButton(m_joystick1, 8)
            // .whileTrue(new RunCommand(
            //     () -> m_levelone.intake(),
            //     m_levelone));
    
        // Other existing bindings remain unchanged...      
          
    }
}
