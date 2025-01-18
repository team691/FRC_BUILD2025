package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;
//import com.ctre.phoenix6.*;
//import com.ctre.phoenix.led.FireAnimation;
//import frc.robot.Constants.LightConstants;


public class Lights extends SubsystemBase {

    // Initialize CANdle
    private final CANdle candle = new CANdle (0);

    // Light RGBs
    public void ledRed () {
        candle.setLEDs(255, 0, 0);
      }
    
    
      public void ledWhite () {
        candle.setLEDs(255, 255, 255);
      }
    
    
      public void ledNo () {
        candle.setLEDs(0, 0, 0);
      }
    
    
      public void ledBlue () {
        candle.setLEDs(0,0,255);
      }
    
    
      public void ledYellow () {
        candle.setLEDs(255, 255, 0);
      }
    
    
      public void ledPurple () {
        candle.setLEDs (127, 0, 255);
      }
     
      public void ledGreen () {
        candle.setLEDs(0,255,0);
      }

      /*
      public void climberTheme() {
        FireAnimation fireAnimation = new FireAnimation(LightConstants.brightness, LightConstants.speed, LightConstants.numLed, LightConstants.sparking, LightConstants.cooling);

        candle.animate(fireAnimation);
      }
      */
 
}