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
public class Agitator extends SubsystemBase {
	private static Agitator s_theAgitator;
	private final SparkMax m_agitator;

	/**
	 * Constructs a new Transport subsystem.
	 * This initializes the motor controller and ensures only one instance of the
	 * subsystem exists.
	 */
	public Agitator() {
		m_agitator = new SparkMax(TransportConstants.kAgitatorMotorPort, MotorType.kBrushless);
		SparkMaxConfig config = new SparkMaxConfig();
		config.smartCurrentLimit(TransportConstants.kCurrentLimit);
		m_agitator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		if (s_theAgitator == null) {
			s_theAgitator = this;
		} else {
			throw new Error("The Agitator subsystem has already been created");
		}
	}

	/**
	 * Retrieves the singleton instance of the Transport subsystem.
	 *
	 * @return The singleton instance of the Transport subsystem.
	 */
	public static Agitator getAgitator() {
		return s_theAgitator;
	}

	/**
	 * Sets the motor speed for the transport subsystem.
	 *
	 * @param speed The speed to set, between -1.0 and 1.0.
	 */
	public static void setPower(double speed) {
		s_theAgitator.m_agitator.set(speed);
	}

	/**
	 * Stops the transport motor by setting its speed to zero.
	 */
	public static void stop() {
		s_theAgitator.m_agitator.set(0);
	}
}