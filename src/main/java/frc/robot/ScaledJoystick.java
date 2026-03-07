package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ScaledJoystick {
	private final DoubleSupplier m_xSupplier;
	private final DoubleSupplier m_ySupplier;
	private final double m_deadzone;
	private final double m_intensity;
	private double m_x;
	private double m_y;

	/**
	 * Applies a deadzone and scales joystick input to make it smoother.
	 * 
	 * @param x a supplier of the raw joystick x value
	 * @param y a supplier of the raw joystick y value
	 * @param deadzone the size of the deadzone
	 */
	public ScaledJoystick(DoubleSupplier x, DoubleSupplier y, double deadzone) {
		this(x, y, deadzone, 1);
	}

	/**
	 * Applies a deadzone and scales joystick input to make it smoother.
	 * 
	 * @param x a supplier of the raw joystick x value
	 * @param y a supplier of the raw joystick y value
	 * @param deadzone the size of the deadzone
	 * @param intensity a number from 0 to 1 of how sensitive the joystick is
	 */
	public ScaledJoystick(DoubleSupplier x, DoubleSupplier y, double deadzone, double intensity) {
		m_xSupplier = x;
		m_ySupplier = y;
		m_deadzone = deadzone;
		m_intensity = intensity;
	}

	/**
	 * Update the current x and y output
	 */
	public void update() {
		double x = m_xSupplier.getAsDouble();
		double y = m_ySupplier.getAsDouble();
		double magnitude = Math.min(Math.hypot(x, y), 1);
		double direction = Math.atan2(y, x);
		double deadzoned = magnitude <= m_deadzone ? 0 : magnitude;
		double intense = 2 * Math.asin(deadzoned) / Math.PI;
		double smooth = deadzoned * deadzoned * deadzoned;
		magnitude = m_intensity * intense + (1 - m_intensity) * smooth;
		m_x = magnitude * Math.cos(direction);
		m_y = magnitude * Math.sin(direction);
	}

	/**
	 * Gets the current x output
	 * 
	 * @return x value
	 */
	public double getX() {
		return m_x;
	}

	/**
	 * Gets the current y output
	 * 
	 * @return y value
	 */
	public double getY() {
		return m_y;
	}

	public static Command joystickTestCommand() {
		return Commands.runOnce(() -> {
			ScaledJoystick joystick = new ScaledJoystick(() -> .5, () -> .5, .05);
			joystick.update();
			System.out.printf("Joystick output: (%f, %f)\n", joystick.getX(), joystick.getY());
		});
	}
}