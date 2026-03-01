// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.AimCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);
	private final Aim m_aim = new Aim.Linear();

	{
		new Drive();
		new Shooter();
		new Intake();
		new Kicker();
		new Agitator();
		Turret.create();
		Hood.create();
	}

	public Robot() {
		bindCompControls(); // Change to bindCompControls() for competition
	}

	// If code had comments then it is most likely
	// good for competition unless otherwise noted
	private void bindCompControls() {

		// *************** DRIVER BINDINGS ***************

		Drive.getDrive().setDefaultCommand(
				new DriveCommands.JoystickDrive(
						() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(), // L2 rotates left,
																								// R2 rotates right
						m_driverController.getHID()::getCreateButton));

		m_driverController.triangle().onTrue(new DriveCommands.SpinToAngle(0, 0.2)); // TODO: make sure commands work
		m_driverController.circle().onTrue(new DriveCommands.SpinToAngle(90, 0.2));
		m_driverController.cross().onTrue(new DriveCommands.SpinToAngle(180, 0.2));
		m_driverController.square().onTrue(new DriveCommands.SpinToAngle(270, 0.2));

		m_driverController.R1().onTrue(
				new SequentialCommandGroup(new IntakeCommands.MoveArmToPosition(1),
						new IntakeCommands.SpinIntake(1)));// Deploys arm TODO: tune position
		m_driverController.L1().onTrue(
				new SequentialCommandGroup(new IntakeCommands.MoveArmToPosition(0),
						new IntakeCommands.StopIntake()));// Retracts arm and stops power TODO: tune position
		m_driverController.povUp().whileTrue(new IntakeCommands.SpinArmPower(-.05)); // TODO: tune speed
		m_driverController.povDown().whileTrue(new IntakeCommands.SpinArmPower(.05));
		// TODO: Intake move up/down? Talk to drive team about bindings for these

		m_driverController.povUp().whileTrue(null); // TODO: add climber commands
		m_driverController.povDown().whileTrue(null);

		// *************** OPERATOR BINDINGS ***************

		m_operatorController.L2().whileTrue(new TurretCommands.RunAtPowerSignal(-.1));// Rotates left/counterclockwise
		m_operatorController.R2().whileTrue(new TurretCommands.RunAtPowerSignal(.1));// Rotates right/clockwise

		m_operatorController.square().toggleOnTrue(
				new AimCommands.RiyaAiming(m_operatorController.povUp(),
						m_operatorController.povDown(), Map.of(
								m_operatorController.cross(), 5.0,
								m_operatorController.circle(), 10.0,
								m_operatorController.triangle(), 15.0)));// TODO: Update command to be close preset

		// TODO: Change command to RunForPower (no time)
		m_operatorController.L1().toggleOnTrue(
				new ParallelCommandGroup(new TransportCommands.RunKickerAtPower(.2, 5),
						new ParallelCommandGroup(new TransportCommands.RunAgitatorAtPower(.2, 5))));
	}

	private void bindTestControls() {
		Drive.getDrive().setDefaultCommand(
				new DriveCommands.JoystickDrive(
						() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getCreateButton));

		m_driverController.cross().whileTrue(
				new TransportCommands.RunAgitatorAtPower(
						0.2, /* POWER */
						1)); /* TIME */
		m_driverController.square().whileTrue(
				new TransportCommands.RunKickerAtPower(
						0.2, /* POWER */
						1)); /* TIME */

		Turret.getTurret().setDefaultCommand(
				new TurretCommands.RunToAngleHardwareSignal(m_operatorController::getLeftX,
						m_operatorController::getLeftY));
		m_operatorController.L1().whileTrue(
				new TurretCommands.RunAtPower(
						-.1, /* POWER */
						0)); /* TIME */
		m_operatorController.R1().whileTrue(
				new TurretCommands.RunAtPower(
						.1, /* POWER */
						0)); /* TIME */

		m_operatorController.triangle().toggleOnTrue(
				new ShooterCommands.RunAtDPadRPM(this, m_operatorController.povRight(),
						m_operatorController.povLeft()));

		m_operatorController.povDown().whileTrue(
				new HoodCommands.RunAtPower(
						-.1, /* POWER */
						0)); /* TIME */
		m_operatorController.povUp().whileTrue(
				new HoodCommands.RunAtPower(
						.1, /* POWER */
						0)); /* TIME */

		m_operatorController.square().onTrue(
				new IntakeCommands.SpinIntake(
						.1)); /* POWER */
		m_operatorController.circle().onTrue(
				new IntakeCommands.MoveArmToPosition( // TODO: Turn this into an enum and tune position value
						1)); /* POSITION */
		m_operatorController.cross().onTrue(
				new IntakeCommands.MoveArmToPosition(
						0)); /* POSITION */

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
		m_scheduler.schedule(
				Commands.parallel(
						Commands.sequence(
								new TurretCommands.RunToAngleHardware(45), Commands.waitSeconds(1),
								new TurretCommands.RunToAngleHardware(225), Commands.waitSeconds(1),
								new TurretCommands.RunToAngleHardware(45), Commands.waitSeconds(1),
								new TurretCommands.RunToAngleHardware(225)),
						new ShooterCommands.RunAtDynamicRPM(2400).withTimeout(40)));
		// m_scheduler.schedule(m_aim.getAimCommand(13.5));
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