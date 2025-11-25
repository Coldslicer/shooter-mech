package frc.robot.systems;

import java.util.ArrayList;
import java.util.List;



// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

enum FSMState {
	ShuffleboardMode,
	ControllerMode
}

public class ShooterFSMSystem extends FSMSystem<FSMState> {
	/* ======================== Constants ======================== */
	public static final FSMState DEFAULT_STATE = FSMState.ControllerMode;
	public static final double SETPOINT_DIFF = 0.1;

	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	public static record MotorSettings(int id, boolean inverted) { }

	public static final MotorSettings[] SETTINGS = {
		new MotorSettings(8, false),
		new MotorSettings(7, false),
		new MotorSettings(20, true)
	};

	private final SparkMax[] motors;
	private double mult = 0;

	// private final GenericEntry shuffleboardValue = Shuffleboard
	// 	.getTab("Shooter")
	// 	.add("Requested Ouput", 0)
	// 	.withWidget(BuiltInWidgets.kTextView)
	// 	.getEntry();

	// private final GenericEntry shuffleboardState = Shuffleboard
	// 	.getTab("Shooter")
	// 	.add("State", FSMState.ShuffleboardMode.name())
	// 	.withWidget(BuiltInWidgets.kTextView)
	// 	.getEntry();

	// private final GenericEntry shuffleboardInputEnabled = Shuffleboard
	// 	.getTab("Shooter")
	// 	.add("Use Shuffleboard Input", true)
	// 	.withWidget(BuiltInWidgets.kToggleButton)
	// 	.getEntry();

	// private final GenericEntry shuffleboardSetpoint = Shuffleboard
	// 	.getTab("Shooter")
	// 	.add("True Setpoint", 0)
	// 	.withWidget(BuiltInWidgets.kTextView)
	// 	.getEntry();

	// private final GenericEntry shuffleboardRequestedOutput = Shuffleboard
	// 	.getTab("Shooter")
	// 	.add("Requested Setpoint", 0)
	// 	.withWidget(BuiltInWidgets.kTextView)
	// 	.getEntry();



	/**
	 * Default constructor for the ShooterFSMSystem.
	 */
	public ShooterFSMSystem() {
		List<SparkMax> motorsList = new ArrayList<>(SETTINGS.length);
		int leader = SETTINGS[0].id;
		for (MotorSettings settings : SETTINGS) {
			var motor = new SparkMax(settings.id, MotorType.kBrushless);
			var config = new SparkMaxConfig()
				.inverted(settings.inverted);
			if (settings.id != leader) {
				config = config.follow(leader);
			}
			motor.configure(
				config,
				ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters
			);
			motorsList.add(motor);
		}
		motors = motorsList.toArray(new SparkMax[SETTINGS.length]);
	}

	@Override
	public void reset() {
		motors[0].stopMotor();
		// for (var motor : motors) {
		// 	motor.stopMotor();
		// }
		mult = 0;
		setCurrentState(DEFAULT_STATE);
	}

	@Override
	public void update(TeleopInput input) {
		if (input.isIncreaseMagnitudeButtonPressed()) {
			mult += SETPOINT_DIFF;
		}
		if (input.isDecreaseMagnitudeButtonPressed()) {
			mult -= SETPOINT_DIFF;
		}
		mult = MathUtil.clamp(mult, -1, 1);
		// mult = switch (getCurrentState()) {
		// 	case ShuffleboardMode -> mult;
		// 	//(float) clamp(shuffleboardValue.getDouble(mult), 1., 0.);
		// 	case ControllerMode ->
		// 		(float) MathUtil.clamp((
		// 			mult
		// 			+ (input.isIncreaseMagnitudeButtonPressed() ? SETPOINT_DIFF : 0)
		// 			- (input.isDecreaseMagnitudeButtonPressed() ? SETPOINT_DIFF : 0)
		// 		),
		// 		// * (input.isReverseDirectionButtonPressed() ? 1 : -1),
		// 		-1, 1);
		// };
		motors[0].set(mult);
		// shuffleboardRequestedOutput.setDouble(mult);
		// System.out.println(mult);
		// System.out.println(input.isIncreaseMagnitudeButtonPressed());
		// System.out.println(input.isDecreaseMagnitudeButtonPressed());
		// shuffleboardSetpoint.setDouble(motors[0].get());
		// shuffleboardState.setString(getCurrentState().name());
		setCurrentState(nextState(input));
	}

	@Override
	public boolean updateAutonomous(AutoFSMState autoState) {
		return true;
	}

	@Override
	protected FSMState nextState(TeleopInput input) {
		return FSMState.ControllerMode;
		// if (input.toggleControllerButtonPressed()) {
		// 	var nextState = getCurrentState() == FSMState.ControllerMode
		// 		? FSMState.ShuffleboardMode
		// 		: FSMState.ControllerMode;
		// 	shuffleboardInputEnabled.setBoolean(getCurrentState() == FSMState.ShuffleboardMode);
		// 	return nextState;
		// } else {
		// 	return shuffleboardInputEnabled.getBoolean(false)
		// 		? FSMState.ShuffleboardMode : FSMState.ControllerMode;
		// }
	}
}
