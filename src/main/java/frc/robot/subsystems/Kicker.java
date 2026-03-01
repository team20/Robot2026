package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.TransportConstants;

/**
 * The Transport subsystem is responsible for moving game pieces from the intake
 * to the shooter. It uses a motor controller to achieve this functionality.
 */
public class Kicker extends SubsystemBase {
	private static Kicker s_theKicker;
	private final SparkMax m_kicker;

	/**
	 * Constructs a new Transport subsystem.
	 * This initializes the motor controller and ensures only one instance of the
	 * subsystem exists.
	 */
	public Kicker() {
		m_kicker = new SparkMax(TransportConstants.kKickerPort, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.smartCurrentLimit(TransportConstants.kCurrentLimit);
		m_kicker.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		if (s_theKicker == null) {
			s_theKicker = this;
		} else {
			throw new Error("The Kicker subsystem has already been created");
		}
	}

	/**
	 * Retrieves the singleton instance of the Transport subsystem.
	 *
	 * @return The singleton instance of the Transport subsystem.
	 */
	public static Kicker getKicker() {
		return s_theKicker;
	}

	/**
	 * Sets the motor speed for the transport subsystem.
	 *
	 * @param speed The speed to set, between -1.0 and 1.0.
	 */
	public static void setPower(double speed) {
		s_theKicker.m_kicker.set(speed);
	}

	/**
	 * Stops the transport motor by setting its speed to zero.
	 */
	public static void stop() {
		s_theKicker.m_kicker.set(0);
	}
}