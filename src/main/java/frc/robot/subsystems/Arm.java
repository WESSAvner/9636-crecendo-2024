package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;


public class Arm extends SubsystemBase{
    private double armTargetAngle;
    private final CANSparkMax m_armMotorRight;
    private final CANSparkMax m_armMotorLeft;
    private final AbsoluteEncoder angEncoder;
    private final PIDController PIDController;

    public Arm() {
        m_armMotorLeft = new CANSparkMax(8, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        m_armMotorRight = new CANSparkMax(5, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        m_armMotorRight.setInverted(true);
        angEncoder = m_armMotorRight.getAbsoluteEncoder();
        angEncoder.setZeroOffset(ArmConstants.kArmAngleOffset);
        angEncoder.setInverted(false);
        // HIGHER P CAUSES MORE OCCELATION. TOO LOW CAUSES NO POWER/NOT ENOUGH
        // D SLOWS DOWN THE CRAZYNESS OF THE P 
        // I PUSHES THE P IF IT IS TOO LOW
        // D OVERREACTS TO NOISE THAT THE P WILL NOT MESS WITH
        PIDController = new PIDController(
            ArmConstants.kArmP,
            ArmConstants.kArmI,
            ArmConstants.kArmD
        );
    }

    public void setArmAngle(double angle){
        armTargetAngle = angle;
    }

    public Boolean isAtTraget(){
        return armTargetAngle - angEncoder.getPosition() < ArmConstants.kArmThreshold;
    }

    public void setArmSpeed(double speed) {
        m_armMotorLeft.set(speed);
        m_armMotorRight.set(speed);
    }

    
    public void periodic() {
        m_armMotorRight.setVoltage(
            PIDController.calculate(angEncoder.getPosition(), armTargetAngle)
            );
    }
}
