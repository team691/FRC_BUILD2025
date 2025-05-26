package frc.robot.subsystems;
import frc.robot.enums.ElevatorStates;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// May need to setup Trapezoid Profile/PID for fluid control

public class Elevator extends SubsystemBase {
    private static Elevator m_elevator = new Elevator();
    private static double ELEVSPEED = 0.4;

    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);

    public static ElevatorStates states = ElevatorStates.Stop;

    public static Elevator getInstance() {
        if (m_elevator == null) {
            return m_elevator;
        }

        return m_elevator;
    }

    public ElevatorStates setState(ElevatorStates newState) {
        states = newState;
        return states;
    }

    public ElevatorStates getState() {
        return states;
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

        elevMotor2.setControl(new Follower(elevMotor1.getDeviceID(), false));
    }

    // TODO: Setup Enums for different scoring levels
    public void periodic() {
        switch(states) {
            case Low:
                elevMotor1.setControl(positionRequest.withPosition(ElevatorConstants.LOW_POSITION));
                break;
            case Mid:
                elevMotor1.setControl(positionRequest.withPosition(ElevatorConstants.MID_POSITION));
                break;
            case High:
                elevMotor1.setControl(positionRequest.withPosition(ElevatorConstants.HIGH_POSITION));
                break;
            case ManualUp:
                elevMotor1.set(ELEVSPEED);
                break;
            case ManualDown:
                elevMotor1.set(-ELEVSPEED);
                break;
            case Stop:
                elevMotor1.stopMotor();
                break;
            }
    }

    public Command goLow() {
        return (run(() -> {
            setState(ElevatorStates.Low);
        }));
    }

    public Command goMid() {
        return (run(() -> {
            setState(ElevatorStates.Mid);
        }));
    }

    public Command goHigh() {
        return (run(() -> {
            setState(ElevatorStates.High);
        }));
    }
}
