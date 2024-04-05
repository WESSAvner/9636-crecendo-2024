package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class MobilityAuton extends SequentialCommandGroup {
    private DriveSubsystem drivetrain;
    public MobilityAuton() {
        drivetrain = RobotContainer.m_robotDrive;
        addCommands(
            new WaitCommand(2),
            new InstantCommand(() -> drivetrain.drive(1, 0, 0, true, true))
                .withTimeout(5)
        );
    }
}
