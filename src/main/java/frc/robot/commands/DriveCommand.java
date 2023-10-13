package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase{
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotSpeed;
    private final BooleanSupplier m_fieldRelative;
    private final BooleanSupplier m_rateLimit;
    private final BooleanSupplier m_slow;

    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed, BooleanSupplier fieldRelative, BooleanSupplier rateLimit, BooleanSupplier slow){
        m_driveSubsystem = subsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_fieldRelative = fieldRelative;
        m_rateLimit = rateLimit;
        m_slow = slow;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute(){
        m_driveSubsystem.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(), m_rotSpeed.getAsDouble(), m_fieldRelative.getAsBoolean(), m_rateLimit.getAsBoolean(), m_slow.getAsBoolean());
    }
   

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_driveSubsystem.drive(0.0, 0.0, 0.0, m_fieldRelative.getAsBoolean(), m_rateLimit.getAsBoolean(), m_slow.getAsBoolean());
    }
}