// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunTurretAtSpeedForTime extends Command {
	private Turret m_turretSubsystem;
	private Timer m_timer;
	private double m_time;
	private double m_speed;

	/** Creates a new RunTurretAtSpeed. */
	public RunTurretAtSpeedForTime(Turret turretSubsystem, double speed, double time) {
		addRequirements(turretSubsystem);
		m_turretSubsystem = turretSubsystem;
		m_timer = new Timer();
		m_time = time;
		m_speed = speed;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_turretSubsystem.runMotorAtDutyCycle(m_speed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_timer.stop();
		m_turretSubsystem.runMotorAtDutyCycle(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.get() >= m_time;
	}
}