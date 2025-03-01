package frc.robot.subsystems;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;


import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class Sonar extends SubsystemBase {
    private final CANrange canRangeFinder;
    private final List<Double> distances;
    private final double conversionFactor = 100; // Conversion factor from meters to centimeters
    private final Timer timer;
    private double median;


    public Sonar(int deviceID) {
        canRangeFinder = new CANrange(deviceID);
        distances = new ArrayList<>();
        timer = new Timer();
        timer.start();
    }


    @Override
    public void periodic() {
        // Get the StatusSignal value
        StatusSignal<Distance> distance = canRangeFinder.getDistance();
        Double distanceCentimeters = distance.getValueAsDouble() * conversionFactor;
        // Add the distance to the list if it's valid
        StatusSignal<Boolean> detected = canRangeFinder.getIsDetected();
        boolean ifDetected = detected.getValue();
        if (ifDetected) {
            distances.add(distanceCentimeters); // Convert to inches and add to the list
        }


        // Check if a second has passed
        if (timer.hasElapsed(0.01)) {
            // Calculate the median
           
            median = calculateMedian(distances);


            // Print the median distance
            System.out.println("Median Distance: " + median + " centimeters");


            // Clear the list and reset the timer
            distances.clear();
            timer.reset();
        }
    }


    private double calculateMedian(List<Double> distances) {
        if (distances.isEmpty()) {
            return 1000000;
        }
        Collections.sort(distances);
        int middle = distances.size() / 2;
        if (distances.size() % 2 == 0) {
            return (distances.get(middle - 1) + distances.get(middle)) / 2.0;
        } else {
            return distances.get(middle);
        }
    }
    public double getSpeed(boolean sonarOn) {
        double distanceToSpeed = 1;
        if (sonarOn == false) {
            return 1;
        }
        if (median <= 7) {
            return 0;
        }
        else if (median < 400) {
            distanceToSpeed *= median/500;
            return distanceToSpeed;
        }
        else {
            return 1;
        }
    }
}
