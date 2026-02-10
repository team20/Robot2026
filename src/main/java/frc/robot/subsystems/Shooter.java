package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.ShooterConstants;
import frc.robot.commands.ShooterCommands;

public class Shooter extends SubsystemBase {

	private final TalonFX m_motor = new TalonFX(ShooterConstants.kMotorPort);
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
	}

	/**
	 * Sets setpoint to 0 and stops the motor.
	 */
	public void stop() {
		m_motor.stopMotor();
	}

	public void setPower(double power) {
		m_motor.set(power);
	}

	public void setVoltage(double voltage) {
		m_motor.setVoltage(voltage);
	}

	public void setRPM(double rpm) {
		double voltage = rpm / this.getRPMperVolt();
		m_motor.setControl(m_request.withVelocity(RPM.of(rpm)).withFeedForward(voltage));
	}

	public double getRPM() {
		return m_motor.getVelocity().getValue().in(RPM);
	}

	public double getRPMperVolt() {
		return ShooterConstants.kV;
	}

	public ShooterCommands getCommands() {
		return new ShooterCommands(this);
	}
}
