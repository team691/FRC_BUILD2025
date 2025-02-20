package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LevelOne extends SubsystemBase {
    private SparkMax levelOne;

    public void initialize() {
        levelOne = new SparkMax(6, MotorType.kBrushless);
    }

    public void outake() {
        levelOne.set(0.7);
    }

    public void intake() {
        levelOne.set(-0.7);
    }
}
