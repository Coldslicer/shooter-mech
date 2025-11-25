package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int PORT = 0;

	/* ======================== Private variables ======================== */
	// Input objects
	private final PS4Controller controller;


	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		controller = new PS4Controller(PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Gets whether the shooter magnitude should be increased.
	 * @return whether the shooter magnitude should be increased
	 */
	public boolean isIncreaseMagnitudeButtonPressed() {
		return controller.getR1ButtonPressed();
	}

	/**
	 * Gets whether the shooter magnitude should be reduced.
	 * @return whether the shooter magnitude should be reduced
	 */
	public boolean isDecreaseMagnitudeButtonPressed() {
		return controller.getL1ButtonPressed();
	}

	/**
	 * Gets whether the shooter magnitude should be reversed.
	 * @return whether the shooter magnitude should be reversed
	 */
	public boolean isReverseDirectionButtonPressed() {
		return controller.getCircleButtonPressed();
	}

	/**
	 * Gets whether the controller should be used to control the shooter.
	 * @return whether the controller should be used to control the shooter
	 */
	public boolean toggleControllerButtonPressed() {
		return controller.getTriangleButtonPressed();
	}

	/* ======================== Private methods ======================== */
}
