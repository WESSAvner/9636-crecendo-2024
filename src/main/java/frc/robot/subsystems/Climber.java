package frc.robot.subsystems;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Climber extends SubsystemBase {
    private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);
   
    public void m_robotClimber() {
        m_solenoid.toggle();
    }
}
