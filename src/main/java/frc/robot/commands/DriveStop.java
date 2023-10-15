package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStop extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;

    /**
     * Sets the drive speed to 0
     * 
     * @param driveSubsystem subsystem for driving the robot
     */
    public DriveStop(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(0, 0, 0, false, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}