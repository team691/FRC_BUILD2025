package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Level2 extends SubsystemBase {
    // Define the SPARK MAX motor controller with its CAN ID
    // private stprivate SparkMax motorWheel1;
    private SparkMax motorIntake;

    public Level2() {
        motorIntake = new SparkMax(2, MotorType.kBrushless);
    }

    public void intake(double speed) { // set speed 1
        // Stop the motor when the robot is disabled 
        motorIntake.set(speed);
    }

    public void outtakeL2(double speed) { // set speed -1
        motorIntake.set(-speed);
    }
}