package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Subsystems.ShooterConstants;

public class Shooter extends SubsystemBase {
	private static Shooter s_theShooter;

	private final TalonFX m_motor = new TalonFX(ShooterConstants.kFlywheelPort);
	private final VelocityVoltage m_request = new VelocityVoltage(0);

	public Shooter() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kCurrentLimit;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.Slot0.kP = 0.4000;
		config.Slot0.kI = 0;
		config.Slot0.kD = 0;

		m_motor.getConfigurator().apply(config);
		if (s_theShooter == null) {
			s_theShooter = this;
		} else {
			throw new Error("Shooter already instantiated");
		}
	}

	public static Shooter getShooter() {
		return s_theShooter;
	}

	/**
	 * Sets setpoint to 0 and stops the motor.
	 */
	public static void stop() {
		s_theShooter.m_motor.stopMotor();
	}

	public static void setPower(double power) {
		s_theShooter.m_motor.set(power);
	}

	public static void setVoltage(double voltage) {
		s_theShooter.m_motor.setVoltage(voltage);
	}

	public static void setRPM(double rpm) {
		double voltage = rpm / getRPMperVolt();
		s_theShooter.m_motor.setControl(s_theShooter.m_request.withVelocity(RPM.of(rpm)).withFeedForward(voltage));
	}

	public static double getRPM() {
		return s_theShooter.m_motor.getVelocity().getValue().in(RPM);
	}

	public static double getRPMperVolt() {
		return ShooterConstants.kV;
	}

	@Override
	public void periodic() {
		if (Constants.kLogging) {
			SmartDashboard.putNumber("Shooter/RPM", getRPM());
		}
	}
}