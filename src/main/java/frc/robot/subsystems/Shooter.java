package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.enums.ShooterStates;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Timer;
import java.util.TimerTask;

public class Shooter extends SubsystemBase {
    private static Shooter m_Shooter = new Shooter();
    public static Shooter getInstance() {
        return m_Shooter;
    }

    public boolean isLaunched = false;
    // Define the SPARK MAX motor controllers with their CAN IDs
    private final TalonFX Shooter;
    private final TalonFX PassThrough;
    public ShooterStates states = ShooterStates.Reset; 

    private Shooter() {
        Shooter = new TalonFX(3);
        PassThrough = new TalonFX(2);
    }
    public boolean Score(){
        switch(states){
            case Reset:
                Shooter.set(0);
                PassThrough.set(0);
                states = ShooterStates.MoveForward;
                isLaunched = false;
                break;
            case MoveForward:
                stopMotorAfterDelay(PassThrough, 0.1, 1500);
                stopMotorAfterDelay(Shooter, 0.1, 1500);
                states = ShooterStates.Reverse;
                break;
            case Reverse:
                stopMotorAfterDelay(Shooter, -0.1, 1500);   
                states = ShooterStates.Stop;
                break;
            case Stop:
                Shooter.set(0);
                PassThrough.set(0);
                return true;
        }
        return false;
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
            Shooter.set(Constants.ShooterConstants.ShooterPower); 
            states = ShooterStates.Reset;
            isLaunched = true;// Start motor
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
    private void stopMotorAfterDelay(TalonFX motor, double speed, int delayMs) {
        Timer timer = new Timer();
        motor.set(speed);
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                motor.set(0); // Stop the motor
            }
        }, delayMs);
    }
}