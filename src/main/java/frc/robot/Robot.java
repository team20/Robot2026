// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Subsystems.AgitatorConstants;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.autos.AutoComposer;
import frc.robot.commands.AimCommands;
import frc.robot.commands.AimCommands.AdjustAim;
import frc.robot.commands.AngularPositionCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	public static final boolean compControls = true;
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);
	private final Command m_auto;
	private final Command m_auto2;
	private final Command m_AUTO;
	private final AutoComposer m_autoComposer;
	private final SendableChooser<Command> m_autoChooser;
	private final SendableChooser<Boolean> m_isInPit = new SendableChooser<>();

	{ // Here are the individual subsystems
		new Drive();
		new Shooter();
		new IntakeWheels();
		new IntakeArm();
		new Climber();
		new Agitator();
		new Vision();
		Turret.create();
		Hood.create();
	}

	public void initSubsystems() {
		Hood.getHood().stop();
		Climber.getClimber().stopMotor();
		IntakeArm.getIntakeArm().stopMotor();
		IntakeWheels.stopWheel();
		Agitator.stop();
		Shooter.stop();
	}

	public Robot() {
		m_autoComposer = new AutoComposer(this);
		m_autoChooser = new SendableChooser<>();
		bindAutoOptions();
	}

	private void bindAutoOptions() {
		m_autoChooser.addOption("Left One Shoot Auto", m_autoComposer.getLeftOneShootAuto());
		m_autoChooser.addOption("Right One Shoot Auto", m_autoComposer.getRightOneShootAuto());
		m_autoChooser.addOption("Right Two Shoot Auto", m_autoComposer.getRightTwoShootAuto());
		m_autoChooser
				.addOption("Right Two Shoot Auto", m_autoComposer.getRightTwoShootAuto());
		m_autoChooser
				.addOption("Right Two Shoot Auto Practice", m_autoComposer.getRightTwoShootAutoPracticeModified());
		m_autoChooser
				.setDefaultOption(
						"Right Two Shoot Auto Practice", m_autoComposer.getRightTwoShootAutoPracticeModified());
		m_isInPit.addOption("Is in pit", true);
		m_isInPit.setDefaultOption("Not in pit", false);
	}

	{
		m_auto2 = new SequentialCommandGroup(
				new AngularPositionCommands.RunToAngleHardware(Turret.getTurret(), 36,
						Turret.getConstants()).withTimeout(.5),
				new AdjustAim(true, 11.6, this).withTimeout(.5),
				new WaitCommand(3),
				TransportCommands.getTimedShoot(6),
				new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 100,
						Hood.getConstants()));
		// new WaitCommand(10));

		m_auto = new SequentialCommandGroup(new DriveCommands.DrivePowerAndTime(.2, 0, 0, 3.8),
				new DriveCommands.DrivePowerAndTime(0, 0.2, 0, 1),
				// new DriveCommands.DrivePowerAndTime(-0.2, 0, 0, .5),
				new AngularPositionCommands.RunToAngleHardware(Turret.getTurret(), 65,
						Turret.getConstants()).withTimeout(.5),
				new AdjustAim(true, 18, this).withTimeout(.5),
				new WaitCommand(3),
				TransportCommands.getTimedShoot(20));

		m_AUTO = new SequentialCommandGroup(m_auto2, m_auto);
	}

	private void bindCompControls() {

		// *************** DRIVER BINDINGS ***************
		// Drive bindings

		Drive.getDrive().setDefaultCommand(
				new DriveCommands.JoystickDrive(
						() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(), // L2 rotates left,
						// R2 rotates right
						m_driverController.getHID()::getCreateButton));
		m_driverController.options().debounce(0.1).onTrue(new DriveCommands.ResetHeading());

		{ // Hood bindings

			// m_driverController.cross().whileTrue(
			// new AngularPositionCommands.RunAtPower(Hood.getHood(),
			// -.2, /* POWER */
			// 0).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)); /*
			// TIME */

			m_driverController.triangle().whileTrue(
					new AngularPositionCommands.RunAtPower(Hood.getHood(),
							.2, /* POWER */
							0)); /* TIME */

			m_driverController.cross().whileTrue( // Untested
					new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 100,
							Hood.getConstants()).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
		}

		{ // Intake bindings
			// TODO uncomment when robot fixed
			m_driverController.R1().debounce(0.1) // Untested
					.onTrue(Commands.sequence(IntakeCommands.getOutCommand(), new IntakeCommands.Spintake(1)));// Deploys
																												// arm
																												// TODO:
																												// tune
			// position
			m_driverController.L1().debounce(0.1).onTrue(
					Commands.sequence(
							new IntakeCommands.StopIntake(),
							new TransportCommands.StopAgitator(),
							IntakeCommands.getInCommand()));// Retracts arm and stops power TODO: tune position
			m_driverController.povRight().whileTrue(IntakeCommands.getRunArmAtPowerCommand(-0.2));
			m_driverController.povLeft().whileTrue(IntakeCommands.getRunArmAtPowerCommand(0.2));
		}

		{ // Climber bindings
			m_driverController.create().debounce(.1).onTrue(ClimberCommands.getResetCommand());
			m_driverController.povUp().debounce(.1).onTrue(ClimberCommands.getClimbCommand());
			m_driverController.povDown().debounce(.1).onTrue(ClimberCommands.getRetractCommand());
		}

		// *************** OPERATOR BINDINGS ***************

		{ // Turret bindings
			m_operatorController.L2().whileTrue(
					new AngularPositionCommands.RunAtPower(Turret.getTurret(), -.2, 0));// Rotates
			// left/counterclockwise
			m_operatorController.R2().whileTrue(new AngularPositionCommands.RunAtPower(Turret.getTurret(), .2, 0));// Rotates
			// right/clockwise
		}

		{ // Intake bindings
			m_operatorController.options().debounce(0.1).onTrue(IntakeCommands.getEncoderResetCommand());
			m_operatorController.L1().debounce(.05)
					.toggleOnTrue(new IntakeCommands.Spintake(IntakeConstants.kWheelPower));
			// m_operatorController.create().whileTrue(new
			// IntakeCommands.Spintake(-IntakeConstants.kWheelPower));
		}

		{ // Transport bindings
			m_operatorController.R1().whileTrue( // Runs agitator (for shooting) when pressed on R1
					new TransportCommands.RunAgitatorAtPower(AgitatorConstants.kTeleopPower));
			m_operatorController.povLeft()
					.whileTrue(new TransportCommands.RunAgitatorAtPower(-AgitatorConstants.kTeleopPower));
		}

		{ // Shooting bindings
			m_operatorController.square().onTrue(new ShooterCommands.Stop());
			boolean absolute = true;
			m_operatorController.triangle().debounce(.1).onTrue(new AimCommands.AdjustAim(absolute, 4.5, this));
			m_operatorController.circle().debounce(.1).onTrue(new AimCommands.AdjustAim(absolute, 11.7, this));
			m_operatorController.cross().debounce(.1).onTrue(new AimCommands.AdjustAim(absolute, 13, this));

			m_operatorController.create().whileTrue(
					new RepeatCommand(new AimCommands.AutoAim(Turret.getTurret(), Vision.getVision())));
			// m_operatorController.triangle().debounce(0.1).onTrue(
			// new ShooterCommands.RunAtDPadRPM(this, m_operatorController.povRight(),
			// m_operatorController.povLeft()));

			absolute = false;
			m_operatorController.povUp().whileTrue(new AimCommands.AdjustAim(absolute, 5, this)); // 5 ft/s increasing
			m_operatorController.povDown().whileTrue(new AimCommands.AdjustAim(absolute, -5, this));
			/*
			 * m_operatorController.square().toggleOnTrue(
			 * new AimCommands.RiyaAiming(m_operatorController.povUp(),
			 * m_operatorController.povDown(), Map.of(
			 * m_operatorController.cross(), 5.0,
			 * m_operatorController.circle(), 10.0,
			 * m_operatorController.triangle(), 15.0)));
			 */ // TODO: Update command to be close preset
		}
	}

	private void bindTestControls() {

		if (!m_isInPit.getSelected().booleanValue()) {
			Drive.getDrive().setDefaultCommand(
					new DriveCommands.JoystickDrive(
							() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
							() -> m_driverController.getR2Axis() - m_driverController.getL2Axis(),
							m_driverController.getHID()::getCreateButton));
		}

		m_driverController.cross().whileTrue(
				new TransportCommands.RunAgitatorAtPower(
						0.75 /* POWER */));
		m_operatorController.L2().whileTrue(
				new TransportCommands.RunAgitatorAtPower(
						-0.75)); // POWER

		/*
		 * Turret.getTurret().setDefaultCommand(
		 * new TurretCommands.RunToAngleHardwareSignal(m_operatorController::getLeftX,
		 * m_operatorController::getLeftY));
		 */
		m_operatorController.L1().whileTrue(
				new AngularPositionCommands.RunAtPower(Turret.getTurret(),
						.3, /* POWER */
						0)); /* TIME */
		m_operatorController.R1().whileTrue(
				new AngularPositionCommands.RunAtPower(Turret.getTurret(),
						-.3, /* POWER */
						0)); /* TIME */

		m_operatorController.triangle().debounce(0.1).onTrue(
				new ShooterCommands.RunAtDPadRPM(this, m_operatorController.povRight(),
						m_operatorController.povLeft()));

		/*
		 * m_operatorController.triangle().debounce(0.1).toggleOnTrue(
		 * new ShooterCommands.RunAtDynamicRPM(1000));
		 */

		m_operatorController.povDown().whileTrue(
				new AngularPositionCommands.RunAtPower(Hood.getHood(),
						-.2, /* POWER */
						0)); /* TIME */
		m_operatorController.povUp().whileTrue(
				new AngularPositionCommands.RunAtPower(Hood.getHood(),
						.2, /* POWER */
						0)); /* TIME */

		m_operatorController.touchpad()
				.onTrue(new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 0, Hood.getConstants()));

		{ // climber test bindings
			m_driverController.triangle().whileTrue(ClimberCommands.getRunAtPowerCommand(.6));
			m_driverController.circle().whileTrue(ClimberCommands.getRunAtPowerCommand(-0.6));
		}

		{ // intake test bindings
			m_operatorController.circle().whileTrue(IntakeCommands.getRunArmAtPowerCommand(-0.2));
			m_operatorController.cross().whileTrue(IntakeCommands.getRunArmAtPowerCommand(0.2));
			m_operatorController.square().debounce(0.1).toggleOnTrue(
					new IntakeCommands.Spintake(
							IntakeConstants.kWheelPower)); /* POWER */
		}

	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();
		SmartDashboard.putData("Real Auto Chooser", m_autoChooser);
		SmartDashboard.putData("Pit Selector 2000", m_isInPit);
		if (Constants.kLogging) {
			SmartDashboard.putData(m_scheduler);
		}
	}

	@Override
	public void autonomousInit() {
		initSubsystems();
		m_scheduler.cancelAll();
		Command kommander = m_autoChooser.getSelected();
		System.out.println(kommander.getName());
		m_scheduler.schedule(kommander);
	}

	@Override
	public void teleopInit() {
		initSubsystems();
		m_scheduler.cancelAll();
		if (compControls) {
			bindCompControls();
		} else {
			bindTestControls();
		}

	}

	@Override
	public void testInit() {
		initSubsystems();
		m_scheduler.cancelAll();
		m_scheduler.schedule(Commands.sequence(ClampedP.testCommand()));
	}
}
