package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.enums.ShooterStates;

import com.ctre.phoenix6.hardware.TalonFX;

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
                states = ShooterStates.MoveForward;
                isLaunched = false;
                break;
            case MoveForward:
                PassThrough.set(0.85);
                double a = PassThrough.getPosition().getValueAsDouble();
                if(a > 120){
                    Shooter.set(0);
                    PassThrough.set(0);
                    states = ShooterStates.Stop;
                }
                break;
            case Stop:
                Shooter.set(0);
                PassThrough.set(0);
                return true;
        }
        return false;
    }
    // Intake method with a timer
    public Command passThrough() {
        return run(() -> {
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