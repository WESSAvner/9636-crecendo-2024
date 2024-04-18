package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ClimbToggle extends Command {
    public static Climber m_ClimbToggle;


    public ClimbToggle(Climber ClimbToggleCommand) {


        m_ClimbToggle = ClimbToggleCommand;


    }


    @Override
    public void initialize() {


        m_ClimbToggle.raiseClimber();
    }


    @Override
    public void execute() {




    }


    @Override
    public boolean isFinished() {
     
      return false;
    }
 
    @Override
    public void end(boolean interrupted) {
      m_ClimbToggle.lowerClimber();
    }
  }