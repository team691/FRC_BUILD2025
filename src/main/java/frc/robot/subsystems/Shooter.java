package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.enums.ShooterStates;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.TimerTask;

public class Shooter extends SubsystemBase {
    private static Shooter m_Shooter = new Shooter();
    public static Shooter getInstance() {
        return m_Shooter;
    }
    public boolean shouldPassThrough = false;
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
                Shooter.setPosition(0);
                PassThrough.setPosition(0);
                Timer.delay(1);
                states = ShooterStates.MoveForward;
                isLaunched = false;
                break;
            case MoveForward:
                PassThrough.set(0.5);
                double a = PassThrough.getPosition().getValueAsDouble();
                System.out.println(a);
                if(a > 95){
                    Shooter.set(0);
                    PassThrough.set(0);
                    states = ShooterStates.Stop;
                }
                break;
            case Stop:
                System.out.println("stop");
                Shooter.set(0);
                PassThrough.set(0);
                return true;
        }
        return false;
    }
    // Intake method with a timer
    public Command passThrough() {
        return run(() -> {
            //System.out.println("asdfasdf");
            PassThrough.set(Constants.ShooterConstants.PassThroughPower); // Start motor
        });
    }

    // Shoot method with a timer
    public Command shootTest(double speed) {
        return run(() -> {
            Shooter.set(speed); 
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
}