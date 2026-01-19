package robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Constants.Subsystems.TurretConstants;
import robot.utilities.Compliance;

/**
 * A subsystem which has the ability to change both yaw and pitch in order to
 * adjust the direction fuel will exit the shooter.
 */
public class Turret extends SubsystemBase {
	/**
	 * The yaw motor allows the mechanism to pivot left and right.
	 */
	private final SparkMax m_yawMotor;
	/**
	 * The pitch motor allows the mechanism to pivot up and down.
	 */
	private final SparkMax m_pitchMotor;
	/**
	 * The through bore encoder on the yaw axis allows us to check the yaw angle.
	 */
	private final SparkAbsoluteEncoder m_yawEncoder;
	/**
	 * The through bore encoder on the pitch axis allows us to check the pitch
	 * angle.
	 */
	private final SparkAbsoluteEncoder m_pitchEncoder;
	/**
	 * The yaw PID controller is used to aim the turret in a specific yaw direction
	 * in a closed feedback loop.
	 */
	private final PIDController m_yawPid;
	/**
	 * The pitch PID controller is used to aim the turret in a specific pitch
	 * direction in a closed feedback loop.
	 */
	private final PIDController m_pitchPid;

	/**
	 * Creates a new subsystem using the motor id's from the turret constants, the
	 * corresponding pid constants from the turret constants, and a proper name.
	 */
	public Turret() {
		setName("Turret subsystem");
		{
			int id = Compliance.ensure(TurretConstants.Yaw.class, "kMotorPort");
			m_yawMotor = new SparkMax(id, MotorType.kBrushless);
			m_yawEncoder = m_yawMotor.getAbsoluteEncoder();
			int kP = Compliance.ensure(TurretConstants.Yaw.class, "kP");
			m_yawPid = new PIDController(kP, 0, 0);
		}
		{
			int id = Compliance.ensure(TurretConstants.Pitch.class, "kMotorPort");
			m_pitchMotor = new SparkMax(id, MotorType.kBrushless);
			m_pitchEncoder = m_pitchMotor.getAbsoluteEncoder();
			int kP = Compliance.ensure(TurretConstants.Pitch.class, "kP");
			m_pitchPid = new PIDController(kP, 0, 0);
		}
	}

	/**
	 * Wraps a {@link DoubleSupplier} in a {@link DoubleSupplier} in order to apply
	 * a deadzone on the signal.
	 * 
	 * @param supplier original signal
	 * @param deadzone size of the deadzone
	 * @return modified signal
	 */
	public DoubleSupplier addDeadzone(DoubleSupplier supplier, double deadzone) {
		return () -> MathUtil.applyDeadband(supplier.getAsDouble(), deadzone);
	}

	/**
	 * Creates a new command which aims the turret by moving in the direction of a
	 * joystick.
	 * 
	 * @param yaw yaw velocity signal from joystick
	 * @param pitch pitch velocity signal from joystick
	 * @return command
	 */
	public Command aimWithJoystick(DoubleSupplier yaw, DoubleSupplier pitch) {
		double deadzone = Compliance.ensure(TurretConstants.class, "kDeadzone");
		DoubleSupplier yawValue = addDeadzone(yaw, deadzone); // Add a deadzone to the yaw signal.
		DoubleSupplier pitchValue = addDeadzone(pitch, deadzone); // Add a deadzone to the pitch signal.
		return runEnd(
				() -> {
					m_yawMotor.set(yawValue.getAsDouble()); // Move the yaw motor in the yaw direction.
					m_pitchMotor.set(pitchValue.getAsDouble()); // Move the pitch motor in the pitch direction.
				},
				() -> {
					m_yawMotor.stopMotor(); // Stop both motors when the command ends.
					m_pitchMotor.stopMotor();
				})
						.withName("aimWithJoystick");
	}

	/**
	 * Creates a new command which aims the turret at a specific fixed angle.
	 * 
	 * @param yaw yaw angle in degrees
	 * @param pitch pitch angle in degrees
	 * @return command
	 */
	public Command aimAtAngle(double yaw, double pitch) {
		return aimAtAngle(
				() -> yaw, // Always aim in the fixed yaw direction.
				() -> pitch) // Always aim in the fixed pitch direction.
						.beforeStarting(
								() -> {
									m_yawPid.setSetpoint(yaw); // Set both the pitch and yaw setpoints when the command
																// is initialized.
									m_pitchPid.setSetpoint(pitch);
								})
						.until(
								() -> m_yawPid.atSetpoint() && m_pitchPid.atSetpoint()); // End the command when both
																							// the target pitch and
																							// target yaw angles have
																							// been reached.
	}

	/**
	 * Creates a new command which aims the turret at a constantly changing angle,
	 * likely derived from vision data.
	 * 
	 * @param yaw yaw angle signal in degrees
	 * @param pitch pitch angle signal in degrees
	 * @return command
	 */
	public Command aimAtAngle(DoubleSupplier yaw, DoubleSupplier pitch) {
		return runEnd(
				() -> {
					m_yawMotor.set(
							m_yawPid.calculate( // Use the yaw PID controller to calculate the yaw speed.
									m_yawEncoder.getPosition(), // Get the current yaw angle.
									yaw.getAsDouble())); // Specify the yaw target to be the value of the yaw angle
															// signal.
					m_pitchMotor.set(
							m_pitchPid.calculate( // Use the pitch PID controller to calculate the pitch speed.
									m_pitchEncoder.getPosition(), // Get the current pitch angle.
									pitch.getAsDouble())); // Specify the pitch target to be the value of the pitch
															// angle signal.
				},
				() -> {
					m_yawMotor.stopMotor(); // Stop both motors when the command ends.
					m_pitchMotor.stopMotor();
				})
						.withName("aimAtAngle");
	}
}
