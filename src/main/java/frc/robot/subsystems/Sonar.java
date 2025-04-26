package frc.robot.subsystems;


import edu.wpi.first.units.measure.Distance;
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
        System.out.println(distanceCentimeters);
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