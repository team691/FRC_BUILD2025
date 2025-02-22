package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    // Define motors and other variables
    private SparkMax Climb;

    public Climber() {
        // instantiate said variables
        // SET DEVICE ID TO PROPER ID ONCE SUBSYSTEM IS MADE
        Climb = new SparkMax(0, MotorType.kBrushless);
    }

    public void setPower(double speed) {
        Climb.set(speed);
    }

    // Add extra methods as necessary and implement button bindings

}
