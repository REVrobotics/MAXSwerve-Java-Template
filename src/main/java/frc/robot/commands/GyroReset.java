package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class GyroReset extends CommandBase {

    /**
     * Locks the wheels into a x position
     * 
     * @param DriveSubsystem subsystem for driving
     */
    public GyroReset() {
    }

    @Override
    public void initialize() {
        Constants.Sensors.gyro.reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}