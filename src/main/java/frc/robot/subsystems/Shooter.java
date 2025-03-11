package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Timer;
import java.util.TimerTask;

public class Shooter extends SubsystemBase {
    // Define the SPARK MAX motor controllers with their CAN IDs
    private final SparkMax motorOuttake;
    private final SparkMax motorIntake;

    private static final double MOTOR_POWER = 1.0; // Default motor power level
    private static final int DEFAULT_DURATION_MS = 2000; // Default duration in milliseconds

    public Shooter() {
        motorOuttake = new SparkMax(1, MotorType.kBrushless);
        motorIntake = new SparkMax(2, MotorType.kBrushless);
    }

    // Intake method with a timer
    public void intake() {
        motorIntake.set(MOTOR_POWER); // Start motor
        stopMotorAfterDelay(motorIntake, DEFAULT_DURATION_MS); // Stop motor after delay
    }

    // Shoot method with a timer
    public void shoot() {
        motorOuttake.set(MOTOR_POWER); // Start motor
        stopMotorAfterDelay(motorOuttake, DEFAULT_DURATION_MS); // Stop motor after delay
    }

    // Helper method to stop a motor after a certain delay
    private void stopMotorAfterDelay(SparkMax motor, int delayMs) {
        Timer timer = new Timer();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                motor.set(0); // Stop the motor
            }
        }, delayMs);
    }
}