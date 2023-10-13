package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotSpeed;
    private final BooleanSupplier m_fieldRelative;
    private final BooleanSupplier m_rateLimit;
    private final BooleanSupplier m_slow;

    /**
     * Command for driving the robot
     * 
     * 
     * @param driveSubsystem subsystm for driving the robot
     * @param xSpeed         speed to move on the x-axis
     * @param ySpeed         speed to move on the y-axis
     * @param rotSpeed       rotational speed, positive is counter-clockwise
     * @param fieldRelative  whether commands are relative to the field or the
     *                       robot, true is relative to the field
     * @param rateLimit      whether to enable rate limiting
     * @param slow           when true, slows speed to 1/4
     */
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            DoubleSupplier rotSpeed, BooleanSupplier fieldRelative, BooleanSupplier rateLimit, BooleanSupplier slow) {
        m_driveSubsystem = driveSubsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_fieldRelative = fieldRelative;
        m_rateLimit = rateLimit;
        m_slow = slow;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(), m_rotSpeed.getAsDouble(),
                m_fieldRelative.getAsBoolean(), m_rateLimit.getAsBoolean(), m_slow.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0.0, 0.0, 0.0, m_fieldRelative.getAsBoolean(), m_rateLimit.getAsBoolean(),
                m_slow.getAsBoolean());
    }
}