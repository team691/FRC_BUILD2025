package frc.robot.subsystems;
import frc.robot.enums.ElevatorStates;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// May need to setup Trapezoid Profile/PID for fluid control

public class Elevator extends SubsystemBase {
    private static Elevator m_elevator = new Elevator();
    private static double ELEVSPEED = 0.4;

    public static ElevatorStates states = ElevatorStates.Up;

    public static Elevator getInstance() {
        return m_elevator;
    }

    private final TalonFX elevMotor1;
    private final TalonFX elevMotor2;

    private Elevator() { // TODO: change IDS when they are properly set
        elevMotor1 = new TalonFX(1); 
        elevMotor2 = new TalonFX(0);

        elevMotor1.getConfigurator().apply(new TalonFXConfiguration());
        elevMotor2.getConfigurator().apply(new TalonFXConfiguration());

        // Setting Current Limits so Motor can lost longer (they lwk kinda expensive)

        var currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit = 80;
        currentConfig.StatorCurrentLimitEnable = true;

        elevMotor1.getConfigurator().refresh(currentConfig);
        elevMotor1.getConfigurator().apply(currentConfig);
        
        elevMotor2.getConfigurator().refresh(currentConfig);
        elevMotor2.getConfigurator().apply(currentConfig);
    }

    // TODO: Setup Enums for different scoring levels
    public void ElevStates() {
        switch(states) {
            case Up:
                elevMotor1.set(ELEVSPEED);
                elevMotor2.set(ELEVSPEED);
                states = ElevatorStates.Stop;
                break;

            case Stop:
                elevMotor1.stopMotor();
                elevMotor2.stopMotor();
                states = ElevatorStates.Down;
                break;

            case Down:
                elevMotor1.set(-ELEVSPEED);
                elevMotor2.set(-ELEVSPEED);
                states = ElevatorStates.Stop;
                break;
        }
    }
}
