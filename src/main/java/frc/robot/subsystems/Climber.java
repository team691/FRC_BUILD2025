package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    // Define motors and other variables
    private final TalonFX tfmotor = new TalonFX(0);


    private static final double MOTOR_POWER_CLIMB = 0.2; // Default motor power level

    public Climber() {
        // instantiate said variables
        // SET DEVICE ID TO PROPER ID ONCE SUBSYSTEM IS MADE
    }

    public void climb() {
        tfmotor.set(MOTOR_POWER_CLIMB);
    }

    public void lower() {
        tfmotor.set(-1 * MOTOR_POWER_CLIMB);
    }
    public void stop(){
        tfmotor.set (0);
    }

}
