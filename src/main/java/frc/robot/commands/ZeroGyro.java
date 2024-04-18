package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class ZeroGyro extends Command {
    public static SwerveSubsystem m_swerveDrive;


    public ZeroGyro(SwerveSubsystem swerveSubsystem) {


        m_swerveDrive = swerveSubsystem;


    }


    @Override
    public void initialize() {


        m_swerveDrive.zeroGyro();
    }


    @Override
    public void execute() {




    }


    @Override
    public boolean isFinished() {
     
      return true;
    }
 
    @Override
    public void end(boolean interrupted) {
    }
  }