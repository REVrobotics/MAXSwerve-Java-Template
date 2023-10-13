package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class WheelsX extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;

    /**
     * Locks the wheels into a x position
     * 
     * @param DriveSubsystem subsystem for driving
     */
    public WheelsX(DriveSubsystem subsystem) {
        m_driveSubsystem = subsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_driveSubsystem.setX();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}