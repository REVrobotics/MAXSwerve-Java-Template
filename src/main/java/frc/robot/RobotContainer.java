// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GyroReset;
import frc.robot.commands.WheelsX;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FilteredButton;
import frc.robot.utils.FilteredController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        public final static DriveSubsystem m_robotDrive = new DriveSubsystem();

        // The driver's controller

        FilteredController m_driverController = new FilteredController(
                        OIConstants.kDriverControllerPort);
        FilteredButton m_buttons = new FilteredButton(OIConstants.kButtonPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive, m_driverController::getXLeft,
                                m_driverController::getYLeft, m_driverController::getXRight,
                                m_buttons::getTopSwitch,
                                Constants.DriveConstants.kRateLimitsEnabled,
                                m_driverController::getLeftTriggerActive));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                new Trigger(m_buttons::getOneA).or(
                                m_driverController::getXButton).onTrue(new WheelsX(m_robotDrive));
                new Trigger(m_buttons::getOneC).onTrue(new GyroReset());
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return Robot.m_autoChooser.getSelected();
        }
}
