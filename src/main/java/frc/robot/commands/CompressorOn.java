package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Compressor;



public class CompressorOn extends Command {
    public final Compressor m_compressor;

    public CompressorOn(Compressor robotCompressor) {

        m_compressor = robotCompressor;

    }

    @Override
    public void initialize() {

        m_compressor.enableAnalog(60, 120);;
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
        m_compressor.disable();
    }
}
