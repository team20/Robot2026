package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.ShooterConstants;

public class Shooter extends SubsystemBase {

	private final TalonFX m_TalonFX = new TalonFX(ShooterConstants.kMotorPort);
	private double m_rpmSetpoint;

	public Shooter() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kCurrentLimit;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.Slot0.kP = 0.4000;
		config.Slot0.kI = 0;
		config.Slot0.kD = 0;

		m_TalonFX.getConfigurator().apply(config);

		m_rpmSetpoint = 0;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("RPM", m_rpmSetpoint);
	}

	public void velocityVoltage(VelocityVoltage request) {
		m_TalonFX.setControl(request);
	}

	/**
	 * Sets setpoint to 0 and stops the motor.
	 */
	public void stop() {
		m_rpmSetpoint = 0;
		m_TalonFX.set(0);
	}

	public void setPower(double power) {
		m_TalonFX.set(power);
	}

	public void setVoltage(double voltage) {
		m_TalonFX.setVoltage(voltage);
	}

	public void setRPM(double rpm) {
		VelocityVoltage request = new VelocityVoltage(RPM.of(rpm))
				.withFeedForward(rpm / this.getRPMperVolt());
		velocityVoltage(request);
	}

	/**
	 * Sets saved RPM setpoint and sends update to motor.
	 * 
	 * @param rpm
	 */
	public void setRPMSetpoint(double rpm) {
		m_rpmSetpoint = rpm;
		setRPM(m_rpmSetpoint);
	}

	public double getRPM() {
		return m_TalonFX.getVelocity().getValue().in(RPM);
	}

	public double getRPMSetpoint() {
		return m_rpmSetpoint;
	}

	public double getRPMperVolt() {
		return ShooterConstants.kV;
	}
}
