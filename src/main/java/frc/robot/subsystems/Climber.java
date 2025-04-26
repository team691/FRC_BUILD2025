package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;

public class Climber extends SubsystemBase {
    private static final Climber m_climb = new Climber();
    public static Climber getInstance() {return m_climb;}

    public final TalonFX tfmotor = new TalonFX(1);
    private static final double MOTOR_POWER_CLIMB = 0.4; // Default motor power level
    private final Servo servo = new Servo(0);

    private Climber() {
        tfmotor.setNeutralMode(NeutralModeValue.Brake);
        servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
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

    public Command actuator(double value) {
        return run(() -> {
            servo.set(value);
        });
    }

}
