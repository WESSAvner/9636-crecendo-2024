package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ClimbToggle extends Command {
    public static Climber m_ClimbToggle;


    public ClimbToggle(Climber ClimbToggleCommand) {


        m_ClimbToggle = ClimbToggleCommand;


    }


    //@Override
    public void initialize2() {


        m_ClimbToggle.m_robotClimber();
    }


    //@Override
    public void execute2() {




    }


    //@Override
    public boolean isFinished2() {
     
      return false;
    }
 
    //@Override
    public void end2(boolean interrupted) {
   
    }
  }