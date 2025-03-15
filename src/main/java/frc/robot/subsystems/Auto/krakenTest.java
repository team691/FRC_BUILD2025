// package frc.robot.subsystems;
// // import edu.wpi.first.wpilibj;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class krakenTest extends SubsystemBase {
//     private final TalonFX tfmotor = new TalonFX(0);

//     public void test() {
//         tfmotor.set(0.1);
//     }
// }


package frc.robot.subsystems.Auto;
// import com.revrobotics.spark.CanSparkMax;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class krakenTest extends SubsystemBase {
    private SparkMax testMotor;
    public krakenTest() {
        testMotor = new SparkMax(11, MotorType.kBrushless);
    }
    public void test() {
        testMotor.set(0.1);
    }
}