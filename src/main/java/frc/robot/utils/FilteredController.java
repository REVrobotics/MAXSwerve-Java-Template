// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

/** 
The filtered controller class automatically filters controllers, eliminating the need for excessive logic 
*/
public class FilteredController {
    private XboxController controller;

//constructor
    public FilteredController(int port) {
        this.controller = new XboxController(port);
    }

    /**
     * Gets the filtered X input for the given stick.
     * 
     * @param deadzone
     * @return double
     */
    public double getXLeft(double deadzone) {
        return MathUtil.applyDeadband(controller.getLeftX(), deadzone);
    }

    /**
     * Gets the filtered X input for the given stick.
     * 
     * @param deadzone
     * @return double
     */
    public double getXRight(double deadzone) {
        return MathUtil.applyDeadband(controller.getRightX(), deadzone);
    }

    /**
     * Gets the filtered Y input for the given stick.
     * 
     * @param deadzone
     * @return double
     */
    public double getYLeft(double deadzone) {
        return MathUtil.applyDeadband(controller.getLeftY(), deadzone) * -1;
    }

    /**
     * Gets the filtered Y input for the given stick.
     * 
     * @param deadzone
     * @return double
     */
    public double getYRight(double deadzone) {
        return MathUtil.applyDeadband(controller.getRightY(), deadzone) * -1;
    }

    /**
     * Gets the filtered X input for the given stick.
     * 
     * @return double
     */
    public double getXLeft() {
        return MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);
    }

    /**
     * Gets the filtered X input for the given stick.
     * 
     * @return double
     */
    public double getXRight() {
        return MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);
    }

    /**
     * Gets the filtered Y input for the given stick.
     * 
     * @return double
     */
    public double getYLeft() {
        return MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband) * -1;
    }

    /**
     * Gets the filtered Y input for the given stick.
     * 
     * @return double
     */
    public double getYRight() {
        return MathUtil.applyDeadband(controller.getRightY(), OIConstants.kDriveDeadband) * -1;
    }

    /**
     * Returns if the right trigger is active or not within a deadzone
     * 
     * @param deadzone
     * @return boolean
     */
    public boolean getRightTriggerActive(double deadzone) {
        if (controller.getRightTriggerAxis() > deadzone) {
            return true;
        } else {
            return false;
        }

    }

    /**
     * Returns if the left trigger is active or not within a deadzone
     * 
     * @param deadzone
     * @return boolean
     */
    public boolean getLeftTriggerActive(double deadzone) {
        if (controller.getLeftTriggerAxis() > deadzone) {
            return true;
        } else {
            return false;
        }

    }

    /**
     * Returns whether or not the right trigger is pressed or not with a hard-coded
     * deadzone
     * 
     * @return boolean
     */
    public boolean getRightTriggerActive() {
        if (controller.getRightTriggerAxis() > OIConstants.kDriveDeadband) {
            return true;
        } else {
            return false;
        }

    }

    /**
     * Returns whether or not the left trigger is pressed or not with a hard-coded
     * deadzone
     * 
     * @return boolean
     */
    public boolean getLeftTriggerActive() {
        if (controller.getLeftTriggerAxis() > OIConstants.kDriveDeadband) {
            return true;
        } else {
            return false;
        }

    }

    /**
     * Returns if the A button is pressed or not
     * 
     * @return boolean
     */
    public boolean getAButton() {
        if (controller.getAButton()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns if the B button is pressed or not
     * 
     * @return boolean
     */
    public boolean getBButton() {
        if (controller.getBButton()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns if the Y button is pressed or not
     * 
     * @return boolean
     */
    public boolean getYButton() {
        if (controller.getYButton()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns if the X button is pressed or not
     * 
     * @return boolean
     */
    public boolean getXButton() {
        if (controller.getXButton()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns if any POVButton is pressed or not
     * 
     * @return boolean
     */
    public boolean getPOVPressed() {
        return controller.getPOV() != -1;
    }

    /**
     * Returns depending on which POVButton is pressed
     * 
     * @return int
     */
    public int getPOVButton() {
        int POVButton;
        if (controller.getPOV() != -1) {
            switch (controller.getPOV()) {
                case 0:
                    POVButton = 8;
                    break;

                case 45:
                    POVButton = 9;
                    break;

                case 90:
                    POVButton = 6;
                    break;

                case 135:
                    POVButton = 3;
                    break;

                case 180:
                    POVButton = 2;
                    break;

                case 225:
                    POVButton = 1;
                    break;

                case 270:
                    POVButton = 4;
                    break;

                case 315:
                    POVButton = 7;
                    break;

                case 360:
                    POVButton = 8;
                    break;
                default:
                    POVButton = 0;
            }
            return POVButton;
        } else {
            return 0;
        }
    }

    /**
     * Returns whether the left bumper is pressed or not
     * 
     * @return boolean
     */
    public boolean getLeftBumper() {
        return controller.getLeftBumper();
    }

    /**
     * Returns whether the right bumper is pressed or not
     * 
     * @return boolean
     */
    public boolean getRightBumper() {
        return controller.getRightBumper();
    }

    public boolean getLeftStickPressed() {
        return controller.getLeftStickButton();
    }

    public boolean getRightStickPressed() {
        return controller.getRightStickButton();
    }

    public boolean getBackButton() {
        return controller.getBackButton();
    }

    public boolean getStartButton() {
        return controller.getStartButton();
    }
}