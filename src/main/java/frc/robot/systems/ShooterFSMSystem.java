package frc.robot.systems;

import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkClosedLoopController;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.motors.SparkMaxWrapper;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

enum FSMState {
	ShuffleboardMode,
	ControllerMode
}

public class ShooterFSMSystem extends FSMSystem<FSMState> {
	/* ======================== Constants ======================== */

	private static final AngularVelocity MAX_VEL = RPM.of(100);
	public static final FSMState DEFAULT_STATE = FSMState.ShuffleboardMode;

	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	public static record MotorSettings(int id, boolean inverted) { }

	public static final MotorSettings[] SETTINGS = {
		new MotorSettings(1, false),
		new MotorSettings(2, true),
		new MotorSettings(3, false)
	};

	private final SparkMax[] motors;
	private final SparkClosedLoopController controller;
	private float mult = 0;

	private final GenericEntry shuffleboardValue = Shuffleboard
		.getTab("Shooter")
		.add("Requested Ouput", 0)
		.withWidget(BuiltInWidgets.kTextView)
		.getEntry();

	private final GenericEntry shuffleboardInputEnabled = Shuffleboard
		.getTab("Shooter")
		.add("Use Shuffleboard Input", true)
		.withWidget(BuiltInWidgets.kToggleButton)
		.getEntry();

	private final GenericEntry shuffleboardSetpoint = Shuffleboard
		.getTab("Shooter")
		.add("True Setpoint", 0)
		.withWidget(BuiltInWidgets.kTextView)
		.getEntry();

	private final GenericEntry shuffleboardRequestedOutput = Shuffleboard
		.getTab("Shooter")
		.add("Requested Setpoint", 0)
		.withWidget(BuiltInWidgets.kTextView)
		.getEntry();



	/**
	 * Default constructor for the ShooterFSMSystem.
	 */
	public ShooterFSMSystem() {
		List<SparkMax> motorsList = new ArrayList<>(SETTINGS.length);
		int prevCanId = -1;
		for (MotorSettings settings : SETTINGS) {
			var motor = new SparkMaxWrapper(settings.id, MotorType.kBrushless);
			var config = new SparkMaxConfig()
				.inverted(settings.inverted);
			if (prevCanId != 0) {
				config = config.follow(prevCanId);
			}
			motor.configure(
				config,
				ResetMode.kResetSafeParameters,
				PersistMode.kNoPersistParameters
			);
			motorsList.add(motor);
			prevCanId = settings.id;
		}
		motors = motorsList.toArray(new SparkMax[SETTINGS.length]);
		controller = motors[0].getClosedLoopController();
	}

	@Override
	public void reset() {
		for (var motor : motors) {
			motor.stopMotor();
		}
		setCurrentState(DEFAULT_STATE);
	}

	@Override
	public void update(TeleopInput input) {
		mult = switch (getCurrentState()) {
			case ShuffleboardMode -> (float) clamp(shuffleboardValue.getDouble(mult), 1., 0.);
			case ControllerMode ->
				(float) clamp((
					mult
					+ (input.isIncreaseMagnitudeButtonPressed() ? 1 : 0)
					- (input.isDecreaseMagnitudeButtonPressed() ? 1 : 0)
				)
				* (input.isReverseDirectionButtonPressed() ? 1 : -1),
				1., 0.);

		};
		shuffleboardValue.setDouble(mult);
		controller.setReference(MAX_VEL.times(mult).in(RPM), ControlType.kMAXMotionVelocityControl);
		shuffleboardRequestedOutput.setDouble(mult);
		shuffleboardSetpoint.setDouble(motors[0].get());
	}

	@Override
	public boolean updateAutonomous(AutoFSMState autoState) {
		return true;
	}

	@Override
	protected FSMState nextState(TeleopInput input) {
		if (input.toggleControllerButtonPressed()) {
			var nextState = getCurrentState() == FSMState.ControllerMode
				? FSMState.ShuffleboardMode
				: FSMState.ControllerMode;
			shuffleboardInputEnabled.setBoolean(getCurrentState() == FSMState.ShuffleboardMode);
			return nextState;
		} else {
			return shuffleboardInputEnabled.getBoolean(false)
				? FSMState.ShuffleboardMode : FSMState.ControllerMode;
		}
	}

	private double clamp(double val, double b1, double b2) {
		double min = Math.min(b1, b2);
		double max = Math.max(b1, b2);
		return Math.min(Math.max(val, min), max);
	}
}
