package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;


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
 
}