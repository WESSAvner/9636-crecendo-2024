package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.AutoCloseable;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends SubsystemBase {

    private final CANSparkMax m_intakeMotorLead = new CANSparkMax(12, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_intakeMotorFollow = new CANSparkMax(11, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);



    public void m_robotIntake(double speed) {
        m_intakeMotorLead.set(speed);
        m_intakeMotorFollow.set(-speed);
    }
        
    public void m_robotIntakeOut(double speed) {
        m_intakeMotorLead.set(-speed);
        m_intakeMotorFollow.set(speed);
    }      

    public void m_robotIntakeStop() {
        m_intakeMotorLead.set(0);
        m_intakeMotorFollow.set(0);
    }
}