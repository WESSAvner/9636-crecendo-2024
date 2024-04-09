package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;


public class Arm extends SubsystemBase{
    // private double armAngle;
    private final CANSparkMax m_armMotorRight;
    private final CANSparkMax m_armMotorLeft;
    // private final AbsoluteEncoder angEncoder;
    // private final PIDController PIDController;
    // private final SimpleMotorFeedforward feedforwardController;

    public Arm() {
        m_armMotorLeft = new CANSparkMax(8, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        m_armMotorRight = new CANSparkMax(5, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        m_armMotorRight.setInverted(true);
        // angEncoder = m_armMotorRight.getAbsoluteEncoder();
        // angEncoder.setZeroOffset(0);
        // angEncoder.setInverted(false);
        // PIDController = new PIDController(1, .1, 0.1);
        // feedforwardController = new SimpleMotorFeedforward(0, .3, 0);
    }

    // public void setArmAngle(double angle){
    //     armAngle = angle;
    // }

    public void setArmSpeed(double speed) {
        m_armMotorLeft.set(speed);
        m_armMotorRight.set(speed);
    }

    @Override
    public void periodic() {
        // m_armMotorRight.setVoltage(PIDController.calculate(angEncoder.getPosition(), armAngle));
    }
}
