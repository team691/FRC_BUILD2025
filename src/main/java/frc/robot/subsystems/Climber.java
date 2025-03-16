package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {
    private static final Climber m_climb = new Climber();
    public static Climber getInstance() {return m_climb;}

    private final TalonFX tfmotor = new TalonFX(0);
    private static final double MOTOR_POWER_CLIMB = 0.2; // Default motor power level

    private Climber() {
        tfmotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command climb() {
        return run(() -> {
            tfmotor.set(MOTOR_POWER_CLIMB);
        });
    }
    public Command lower() {
        return run(() -> {
            tfmotor.set(-1 * MOTOR_POWER_CLIMB);
        });
    }
    public Command stop(){
        return run(() -> {
            tfmotor.set(0);
        });
    }

}
