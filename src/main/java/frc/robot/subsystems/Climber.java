package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    // Define motors and other variables
    private SparkMax Climb;

    private static final double MOTOR_POWER_CLIMB = 1.0; // Default motor power level

    public Climber() {
        // instantiate said variables
        // SET DEVICE ID TO PROPER ID ONCE SUBSYSTEM IS MADE
        Climb = new SparkMax(11, MotorType.kBrushless);
    }

    public void climb() {
        Climb.set(MOTOR_POWER_CLIMB);
    }

    public void lower() {
        Climb.set(-1 * MOTOR_POWER_CLIMB);
    }
    public void stop(){
        Climb.set (0);
    }

}
