// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivePower extends Command {
	DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	double m_speed;

	/** Creates a new DriveDistance. */
	public DrivePower(double speed) {
		addRequirements(m_driveSubsystem);
		m_speed = m_driveSubsystem.getPose().getX() + speed;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_driveSubsystem.drive(m_speed, 0, 0, true);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopAllModules();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;// controller.atSetpoint();
	}
}
