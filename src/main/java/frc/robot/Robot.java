// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);

	{
		new Drive();
		new Shooter();
		new Turret();
		new Hood();
		new Intake();
	}

	public Robot() {
		bindControls();
	}

	private void bindControls() {
		Drive.getDrive().setDefaultCommand(
				new DriveCommands.JoystickDrive(
						() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getCreateButton));
		Turret.getTurret().setDefaultCommand(
				new TurretCommands.RunToAngleHardwareSignal(m_operatorController::getLeftX,
						m_operatorController::getLeftY));
		m_operatorController.L1().whileTrue(new TurretCommands.RunAtPower(-.1, 0));
		m_operatorController.R1().whileTrue(new TurretCommands.RunAtPower(.1, 0));
		m_operatorController.triangle().toggleOnTrue(
				new ShooterCommands.RunAtDPadRPM(this, m_operatorController.povRight(),
						m_operatorController.povLeft()));
		m_operatorController.povDown().onTrue(new HoodCommands.RunAtPower(-.1, 0));
		m_operatorController.povUp().onTrue(new HoodCommands.RunAtPower(.1, 0));
		m_operatorController.L2().whileTrue(new IntakeCommands.SpinArmPower(0.1));
		m_operatorController.R2().whileTrue(new IntakeCommands.SpinArmPower(-0.1));
		m_operatorController.square().onTrue(new IntakeCommands.SpinIntake(.1));
		m_operatorController.circle().onTrue(Intake.getExtendCommand());
		m_operatorController.cross().onTrue(Intake.getRetractCommand());
	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();
		if (Constants.kLogging) {
			SmartDashboard.putData(m_scheduler);
		}
	}

	@Override
	public void autonomousInit() {
		m_scheduler.cancelAll();
		m_scheduler.schedule(Intake.getResetCommand());
	}

	@Override
	public void teleopInit() {
		m_scheduler.cancelAll();
	}

	@Override
	public void testInit() {
		m_scheduler.cancelAll();
		m_scheduler.schedule(Commands.sequence(ClampedP.testCommand(), ABBA.testBrownoutPreventionCommand()));
	}
}