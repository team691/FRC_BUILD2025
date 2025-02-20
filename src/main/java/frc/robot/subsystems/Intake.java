package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    // Define the SPARK MAX motor controller with its CAN ID
    // private stprivate SparkMax motorWheel1;
    private SparkMax motorMoveDown;
    private SparkMax motorMoveBar;
    private SparkMax motorIntake;
    
    //private SparkClosedLoopController pidController1;
    //private SparkClosedLoopController pidController2;
    public RelativeEncoder encoderMoveDown;
    public RelativeEncoder encoderMoveBar;

    public Intake() {
        motorMoveDown = new SparkMax(0, MotorType.kBrushless);
        motorMoveBar = new SparkMax(1, MotorType.kBrushless);
        motorIntake = new SparkMax(2, MotorType.kBrushless);
        // motorLift1 = new SparkMax(3, MotorType.kBrushless);
        //pidController1 = motorLift1.getClosedLoopController();
        //pidController2 = motorLift2.getClosedLoopController();
        encoderMoveDown = motorMoveDown.getEncoder();
        encoderMoveBar = motorMoveBar.getEncoder();
    }

    // ALL POSITION VALUES TBD
    public void bigBoiGautham(double position) { // Move Whole intake Mechanism Coral Intake: 0, L1: 0.5, L2 Transfer: 1
        // Set motor to 50% power
        encoderMoveDown.setPosition(position);
    }

    // ALL POSITION VALUES TBD
    public void littleMatthew(double position) { // Move Flywheel intake Mechanism: Coral Intake: 0, Algae Intake: 0.5, L2 Transfer: 1
        encoderMoveBar.setPosition(position);
    }


    public void intake(double speed) { // set speed 1
        // Stop the motor when the robot is disabled
        motorIntake.set(speed);
    }

    public void outtake(double speed) { // set speed -1
        motorIntake.set(speed);  
    }

    public void algaeIntake(double speed) { // set speed -1
        motorIntake.set(speed);
    }

    public void algaeOuttake(double speed) { // set speed -1
        motorIntake.set(speed);
    }
    public int commandNumber = 1;
    public void cycleGautham() {
        if (commandNumber > 3) {
            commandNumber = 1;
        }
        if (commandNumber == 1) {
            bigBoiGautham(0);
        }
        else if (commandNumber == 2) {
            bigBoiGautham(0.5);
        }
        else {
            bigBoiGautham(1);
        }
        commandNumber++;
    }
}