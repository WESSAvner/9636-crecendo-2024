package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;


public class Arm extends SubsystemBase{
    private double armAngle;
    private final CANSparkMax m_armMotor;
    private final AbsoluteEncoder angEncoder;
    private final PIDController controller;

    public Arm() {
        m_armMotor = new CANSparkMax(11, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        angEncoder = m_armMotor.getAbsoluteEncoder();
        controller = new PIDController(0.5, 0, 0);
    }

    public void setArmAngle(double angle){
        armAngle = angle;
    }

    public Command setArmAngleCmd(double angle){
        return this.runOnce(
          () -> setArmAngle(angle)
        );
    }

    @Override
    public void periodic() {
        m_armMotor.setVoltage(controller.calculate(angEncoder.getPosition(), armAngle));
    }
}
