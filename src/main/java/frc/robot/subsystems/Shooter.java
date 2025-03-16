package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Timer;
import java.util.TimerTask;


//Ideal for this is that on our reef board a button click will just align the robot, and one more button, probably on the joystick would shoot it
// Intake would be automatic from beam breakers -> this would be written in commands thingy

public class Shooter extends SubsystemBase {
    // Define the SPARK MAX motor controllers with their CAN IDs
    private final SparkMax Shooter;
    private final SparkMax PassThrough;

    private static final int DEFAULT_DURATION_MS = 2000; // Default duration in milliseconds

    public Shooter() {
        Shooter = new SparkMax(Constants.ShooterConstants.ShooterID, MotorType.kBrushless);
        PassThrough = new SparkMax(Constants.ShooterConstants.PassThroughID, MotorType.kBrushless);
    }

    // Intake method with a timer
    public Command passTest() {
        return run(() -> {
            PassThrough.set(Constants.ShooterConstants.PassThroughPower); // Start motor
        });
    }

    // Shoot method with a timer
    public Command shootTest() {
        return run(() -> {
            Shooter.set(Constants.ShooterConstants.ShooterPower); // Start motor
        });
    }
    public Command stopShoot() {
        return run(() -> {
            Shooter.set(0); // Start motor
        });
    }
    public Command stopPass() {
        return run(() -> {
            PassThrough.set(0); // Start motor
        });
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