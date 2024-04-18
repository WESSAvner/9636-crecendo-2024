package frc.robot.subsystems;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Climber extends SubsystemBase {

    private final Solenoid m_solenoid;
    private final Boolean up = true;
    
    public Climber(Solenoid climberSolenoid) {
        m_solenoid = climberSolenoid;
    }

    public void m_robotClimber() {
        m_solenoid.toggle();
    }

    public void raiseClimber() {
        m_solenoid.set(up);
    }

    public void lowerClimber() {
        m_solenoid.set(!up);
    }
}
