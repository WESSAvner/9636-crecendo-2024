package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class ArmUp extends Command {
    public final Arm m_arm;

    public ArmUp(Arm robotArm) {

        m_arm = robotArm;

    }

    @Override
    public void initialize() {

        m_arm.setArmAngle(90);
    }

    @Override
    public void execute() {
        m_arm.periodic();

    }

    @Override
    public boolean isFinished() {
      
      return m_arm.isAtTraget();
    }
  
    @Override
    public void end(boolean interrupted) {
  
    }
}
