package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class ButtonBoard {
    Joystick Reef = new Joystick(Constants.ButtonBoardConstants.ReefID);
    Joystick Blue = new Joystick(3);//OIConstants.kDriverControllerPort);//Constants.ButtonBoardConstants.BlueID);

    public void SetupButtons(){
        new JoystickButton(Blue, Constants.ButtonBoardConstants.Blue.ClimbF)
            .whileTrue(new RunCommand(
                () -> Climber.getInstance().climb(), Climber.getInstance()
                ));
        
        new JoystickButton(Blue, Constants.ButtonBoardConstants.Blue.Outtake)
            .whileTrue(Shooter.getInstance().shootTest())
            .whileFalse(Shooter.getInstance().stopShoot());
        
        new JoystickButton(Blue, Constants.ButtonBoardConstants.Blue.Align)
            .whileTrue(Shooter.getInstance().passTest())
            .whileFalse(Shooter.getInstance().stopPass());
        
/*      
        new JoystickButton(Blue, Constants.ButtonBoardConstants.Blue.Align)
            .whileTrue(new RunCommand(null, null));
        new JoystickButton(Blue, Constants.ButtonBoardConstants.Blue.ClimbR)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.FarLeftRed)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.CloseLeftWhite)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.CloseLeftRed)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.CloseWhite)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.CloseRed)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.CloseRightWhite)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.CloseRightRed)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.FarRightWhite)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.FarRightRed)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.FarWhite)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.FarRed)
            .whileTrue(new RunCommand(
                null, null
                ));
        new JoystickButton(Reef, Constants.ButtonBoardConstants.Reef.FarLeftWhite)
            .whileTrue(new RunCommand(
                null, null
                )); */
    }
}