package frc.robot.libs;

import edu.wpi.first.wpilibj.XboxController;

// TODO: why are we creating a new class for this? BAD NAME Easily confused with XboxController and how is it different.  Make it clear why this is different from the original. Is the documentation correct?
/**
 * Represents an Xbox controller with modified input values.
 *
 * This class is a custom representation of an Xbox controller. It's designed to handle input from the controller's buttons and joysticks, 
 * and it modifies these input values to suit the specific needs of our application.
 *
 * For example, the raw input values from the controller might range from -1 to 1, but this class could modify these values to range from 0 to 1, 
 * or apply a non-linear transformation to the input values to change the controller's sensitivity.
 *
 * This class could also map the controller's buttons to specific actions in the application, and it could handle things like button debouncing 
 * (ignoring multiple presses of a button in quick succession) or button combos (pressing multiple buttons at the same time to trigger a specific action).
 */ 
public class XboxCotroller extends XboxController {
    /**
     * Represents an Xbox controller.
     * This class extends the WPILib Joystick class.
     * It provides additional functionality specific to Xbox controllers.
     *
     * @param port The port number of the Xbox controller.
     */
    public XboxCotroller(int port) {
        super(port);
    }

    double deadband = .07;

    /**
     * Returns the value of the left X-axis joystick, with deadband applied.
     * If the absolute value of the joystick value is greater than the deadband,
     * the value is squared and returned. If the joystick value is within the deadband,
     * 0 is returned.
     *
     * @return The processed value of the left X-axis joystick.
     */
    @Override
    public double getLeftX() {
        if (Math.abs(super.getLeftX()) > deadband) {
            if (super.getLeftX() > 0)
                return Math.pow(super.getLeftX(), 2); // TODO: in general this seems like a poor way to handle this.  Probably better to just put the code in the Driver
            else
                return -Math.pow(super.getLeftX(), 2);

        } else {
            return 0;
        }
    }

    /**
     * Returns the value of the right X-axis joystick input, with deadband applied.
     * If the absolute value of the input is greater than the deadband threshold,
     * the input is squared and returned. Otherwise, 0 is returned.
     *
     * @return The processed value of the right X-axis joystick input.
     */
    @Override
    public double getRightX() {
        if (Math.abs(super.getRightX()) > deadband) {
            if (super.getRightX() > 0)
                return Math.pow(super.getRightX(), 2);
            else
                return -Math.pow(super.getRightX(), 2);
        } else {
            return 0;
        }
    }

    /**
     * Returns the value of the left Y-axis of the Xbox controller.
     * Applies a deadband to filter out small movements.
     * If the value is greater than the deadband, it applies a square function to amplify the input.
     * If the value is less than the deadband, it returns 0.
     *
     * @return The processed value of the left Y-axis.
     */
    @Override
    public double getLeftY() {
        if (Math.abs(super.getLeftY()) > deadband) {
            if (super.getLeftY() > 0)
                return Math.pow(super.getLeftY(), 2);
            else
                return -Math.pow(super.getLeftY(), 2);

        } else {
            return 0;
        }
    }

    /**
     * Returns the value of the right Y-axis on the Xbox controller.
     * Applies a deadband to filter out small movements.
     * If the value is greater than the deadband, it squares the value for sensitivity adjustment.
     * If the value is negative, it returns the negated squared value.
     * If the value is within the deadband, it returns 0.
     *
     * @return The adjusted value of the right Y-axis.
     */
    @Override
    public double getRightY() {
        if (Math.abs(super.getRightY()) > deadband) {
            if (super.getRightY() > 0)
                return Math.pow(super.getRightY(), 2);
            else
                return -Math.pow(super.getRightY(), 2);
        } else {
            return 0;
        }
    }
}
