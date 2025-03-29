package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakers extends SubsystemBase {
    private static BeamBreakers beamBreakers = new BeamBreakers();
    public static BeamBreakers getInstance()  {
        return beamBreakers;
    }
    private DigitalInput beamBreaker;

    private BeamBreakers() { 
        // Connect to DIO port 0
        beamBreaker = new DigitalInput(0);
    }

    public boolean checkBeam() {
        return beamBreaker.get();
    }
}