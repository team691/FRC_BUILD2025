package frc.robot.subsystems;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;


public class Sonar extends SubsystemBase {
    private static final Sonar m_Sonar = new Sonar(4);
    public static Sonar getInstance() {
        return m_Sonar;
    } 
    private final CANrange canRangeFinder;
    private double distanceCentimeters = 100;
    private double conversionFactor = 100; // Conversion factor from meters to centimeters


    public Sonar(int deviceID) {
        canRangeFinder = new CANrange(deviceID);
    }

    @Override
    public void periodic() {
        // Get the StatusSignal value
        StatusSignal<Distance> distance = canRangeFinder.getDistance();
        distanceCentimeters = distance.getValueAsDouble() * conversionFactor;
        StatusSignal<Double> sonar_ambient_signal = canRangeFinder.getAmbientSignal();
        StatusSignal<Boolean> sonar_is_detected = canRangeFinder.getIsDetected();
        StatusSignal<Voltage> sonar_supply_voltage = canRangeFinder.getSupplyVoltage();
        // TODO: get array of FOV position from sonar

        Shuffleboard.getTab("Sensors").add("Sonar Distance", distanceCentimeters);
        Shuffleboard.getTab("Sensors").add("Sonar Ambient Signal", sonar_ambient_signal);
        Shuffleboard.getTab("Sensors").add("Sonar Boolean Detection", sonar_is_detected);
        Shuffleboard.getTab("Sensors").add("Sonar Supply Voltage", sonar_supply_voltage);
        // Shuffleboard.getTab("Sensors").add("Sonar lah", )
        // Shuffleboard.getTab("Sensors")
        // System.out.println(distanceCentimeters);
    }

    public boolean checkSonar() {
        if (distanceCentimeters < 8) {
            return true;
        }
        else {
            return false;
        }
    }
}