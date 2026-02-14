package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.TransportConstants;

/**
 * The Transport subsystem is responsible for moving game pieces from the intake
 * to the shooter. It uses a motor controller to achieve this functionality.
 */
public class Transport extends SubsystemBase {
	private static Transport s_theTransport;
	private final SparkMax motor;

	/**
	 * Constructs a new Transport subsystem.
	 * This initializes the motor controller and ensures only one instance of the
	 * subsystem exists.
	 */
	public Transport() {
		motor = new SparkMax(TransportConstants.kMotorPort, MotorType.kBrushless);

		if (s_theTransport == null) {
			s_theTransport = this;
		} else {
			throw new Error("The Transport subsystem has already been created");
		}
	}

	/**
	 * Retrieves the singleton instance of the Transport subsystem.
	 *
	 * @return The singleton instance of the Transport subsystem.
	 */
	public static Transport getTransport() {
		return s_theTransport;
	}

	/**
	 * Sets the motor speed for the transport subsystem.
	 *
	 * @param speed The speed to set, between -1.0 and 1.0.
	 */
	public static void setPower(double speed) {
		s_theTransport.motor.set(speed);
	}

	/**
	 * Stops the transport motor by setting its speed to zero.
	 */
	public static void stop() {
		s_theTransport.motor.set(0);
	}
}
