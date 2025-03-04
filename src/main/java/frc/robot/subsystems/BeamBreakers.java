// package frc.robot.subsystems;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class BeamBreakers extends SubsystemBase {
//     private DigitalInput beamBreaker;

//     public BeamBreakers() { //constructor for beam breaker
//         beamBreaker = new DigitalInput(0); // Connect to DIO port 0
//     }

//     public String checkBeam() {
//         if (beamBreaker.get()) {//check if beam is broken
//             // System.out.println("Beam is broken");
//             return "Beam is unbroken";
//         } else { 
//             // System.out.println("Beam is unbroken");
//             return "Beam is broken";
//         }
//     }
// }