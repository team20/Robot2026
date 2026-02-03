// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDistance extends Command {
	DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	PIDController controller = new PIDController(0.8, 0, 0);
	double m_setpoint;

	/** Creates a new DriveDistance. */
	public DriveDistance(double distance) {
		addRequirements(m_driveSubsystem);
		m_setpoint = m_driveSubsystem.getPose().getX() + distance;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		controller.setSetpoint(m_setpoint);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double speed = controller.calculate(m_driveSubsystem.getPose().getX());
		SmartDashboard.putNumber("Speed", speed);

		m_driveSubsystem.drive(speed, 0, 0, true);
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
