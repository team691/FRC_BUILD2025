package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    // Define the SPARK MAX motor controller with its CAN ID
    private SparkMax motorOuttake;
    private SparkMax motorIntake;
    
    //private SparkClosedLoopController pidController1;
    //private SparkClosedLoopController pidController2;
    // public RelativeEncoder encoderMoveDown;
    // public RelativeEncoder encoderMoveBar;

    public Intake() {
        motorOuttake = new SparkMax(1, MotorType.kBrushless);
        motorIntake = new SparkMax(2, MotorType.kBrushless);
    }

    public void intake(double speed) { // set speed 1
        // Stop the motor when the robot is disabled
        motorIntake.set(speed);
    }

    public void outtake(double speed) { // set speed -1
        motorOuttake.set(speed);  
    }

    // public int commandNumber = 1;
    // public void cycleGautham() {
    //     if (commandNumber > 3) {
    //         commandNumber = 1;
    //     }
    //     if (commandNumber == 1) {
    //         bigBoiGautham(0);
    //         intake(0.3);
    //     }
    //     else if (commandNumber == 2) {
    //         bigBoiGautham(0.5);
    //         outtake(0.3);
    //     }
    //     else {
    //         bigBoiGautham(1);
    //     }
    //     commandNumber++;
    // }
}