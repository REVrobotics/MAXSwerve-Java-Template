package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class GyroReset extends CommandBase {

    /**
     * Resets the gyro angle to zero
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