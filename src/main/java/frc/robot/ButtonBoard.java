package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.Climber;

public class ButtonBoard {
    Joystick Reef = new Joystick(Constants.ButtonBoardConstants.ReefID);
    Joystick Blue = new Joystick(3);

    public void SetupButtons(){
        new JoystickButton(Blue, Constants.ButtonBoardConstants.Blue.ClimbF)
            .whileTrue(new RunCommand(
                () -> Climber.getInstance().climb(), Climber.getInstance()
                ));
    
    }
}