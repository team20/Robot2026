// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.framework.TimedRobot;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.telemetry.Telemetry;
import frc.robot.commands.compat.Command;
import frc.robot.commands.compat.CommandScheduler;
import frc.robot.commands.compat.Commands;
import frc.robot.commands.compat.RepeatCommand;
import frc.robot.commands.compat.WaitCommand;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.Gamepad;
import frc.robot.Constants.Subsystems.AgitatorConstants;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.autos.AutoComposer;
import frc.robot.commands.AimCommands;
import frc.robot.commands.AngularPositionCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.Tolerances;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeExtraArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.USBSerialSubsystem;
import frc.robot.subsystems.USBSerialSubsystem.LightPattern;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	public static final boolean compControls = true;
	private final CommandGamepad m_driverController = new CommandGamepad(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandGamepad m_operatorController = new CommandGamepad(
			Constants.ControllerConstants.kOperatorControllerPort);
	private final AutoComposer m_autoComposer;
	private final Selectable<Command> m_autoChooser;
	private final Selectable<Boolean> m_isInPit = new Selectable<>();

	{ // Here are the individual subsystems
		new Drive();
		new Shooter();
		new IntakeWheels();
		new IntakeExtraArm();
		new IntakeArm();
		new Climber();
		new Agitator();
		new Vision();
		new USBSerialSubsystem();
		Turret.create();
		Hood.create();
	}

	public void initSubsystems() {
		Hood.getHood().stop();
		Climber.getClimber().stopMotor();
		IntakeArm.getIntakeArm().stopMotor();
		IntakeExtraArm.stopMotor();
		IntakeWheels.stopWheel();
		Agitator.stop();
		Shooter.stop();
		Drive.resetHeading();
	}

	public Robot() {
		m_autoComposer = new AutoComposer(this);
		m_autoChooser = new Selectable<>();
		bindAutoOptions();
		Tunables.publish("Real Auto Chooser", m_autoChooser);
		Tunables.publish("Pit Selector 2000", m_isInPit);
	}

	public static Command ledCommand(USBSerialSubsystem.LightPattern pattern, Command cmd) {
		return Commands.parallel(
				new USBSerialSubsystem.SetLightCommand(pattern),
				cmd);

	}

	private void bindAutoOptions() {
		m_autoChooser.add("No Auto", new WaitCommand(2));
		m_autoChooser.add("Left One Shoot Auto", m_autoComposer.getLeftOneShootAuto());
		m_autoChooser.add("Right One Shoot Auto", m_autoComposer.getRightOneShootAuto());
		m_autoChooser.add("Right Two Shoot Auto", m_autoComposer.getRightTwoShootAuto());
		m_autoChooser
				.add("Right Two Shoot Auto", m_autoComposer.getRightTwoShootAuto());
		m_autoChooser
				.add("Right Two Shoot Auto Front of Bump", m_autoComposer.getRightTwoShootAutoBump());
		m_autoChooser.add("MANUAL Right Shoot Auto", m_autoComposer.getManualRightShootAuto());
		m_autoChooser.add("Right Two Shoot Auto With Vision", m_autoComposer.getRightTwoShootAutoWithVision());
		m_autoChooser
				.addDefault(
						"Right Two Shoot Auto Front of Bump", m_autoComposer.getRightTwoShootAutoBump());
		m_autoChooser
				.addDefault(
						"Velocity 1 Test Command", m_autoComposer.getVelocity1Testcommand());
		m_isInPit.add("Is in pit", true);
		m_isInPit.addDefault("Not in pit", false);
	}

	private void bindCompControls() {

		// ***********************************************
		// *************** DRIVER BINDINGS ***************
		// ***********************************************

		m_driverController.touchpad().whileTrue(
				Commands.parallel(
						IntakeCommands.getArmOutCombinedCommand(),
						Commands.sequence(
								m_autoComposer.shoot(),
								HoodCommands.getHoodDownCommand(),
								m_autoComposer.getSingleTrajectory("sweep_outpost").withTimeout(14),
								m_autoComposer.shoot(),
								m_autoComposer.getSingleTrajectory("sweep_outpost2").withTimeout(3),
								m_autoComposer.shoot())));

		/*
		 * m_operatorController.touchpad().onTrue(
		 * Commands.sequence(
		 * new DriveCommands.NinjaStar(new Pose2d(12.15, 0.83, Rotation2d.kZero), .3,
		 * .3,
		 * Tolerances.FINE_TRANSLATION),
		 * new DriveCommands.NinjaStar(new Pose2d(10.1, 0.75, Rotation2d.kZero), .3, .3,
		 * Tolerances.FINE_TRANSLATION),
		 * Commands.print("Before arm out"),
		 * IntakeCommands.getArmOutCommand(),
		 * Commands.print("After arm out"),
		 * Commands.race(
		 * new DriveCommands.NinjaStar(new Pose2d(8.64, 1.54, Rotation2d.kCCW_90deg),
		 * 0.3, .3,
		 * Tolerances.COARSE),
		 * new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));
		 * m_driverController.touchpad().onTrue(
		 * Commands.sequence(
		 * new DriveCommands.NinjaStar(new Pose2d(12.15, 0.83, Rotation2d.kZero), .3,
		 * .3,
		 * Tolerances.FINE_TRANSLATION),
		 * new DriveCommands.NinjaStar(new Pose2d(10.1, 0.75, Rotation2d.kZero), .3, .3,
		 * Tolerances.FINE_TRANSLATION),
		 * Commands.print("Before arm out"),
		 * IntakeCommands.getArmOutCommand(),
		 * Commands.print("After arm out"),
		 * Commands.race(
		 * new DriveCommands.NinjaStar(new Pose2d(8.64, 1.54, Rotation2d.kCCW_90deg),
		 * 0.3, .3,
		 * Tolerances.COARSE),
		 * new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));
		 */

		{ // Drive bindings
			Drive.getDrive().setDefaultCommand(
					new DriveCommands.JoystickDrive(
							() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
							() -> m_driverController.getLeftTrigger() - m_driverController.getRightTrigger(),
							() -> false)); // TODO: fix robot/field-centric toggle
			m_driverController.start().debounce(Seconds.of(0.1)).onTrue(new DriveCommands.ResetHeading());
		}

		{ // Hood bindings
			m_driverController.faceUp().whileTrue(
					new AngularPositionCommands.RunAtPower(Hood.getHood(),
							.2, /* POWER */
							0)); /* TIME */
			m_driverController.faceDown().whileTrue(
					HoodCommands.getHoodDownCommand()
							// Overrides all conflicting commands
							.withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING));
		}

		{ // Intake bindings
			m_driverController.rightBumper().debounce(Seconds.of(0.1)).onTrue( // Deploys arm & spins wheels
					IntakeCommands.getArmOutCombinedCommand());
			m_driverController.leftBumper().debounce(Seconds.of(0.1)).onTrue( // Retracts arm & stops wheels
					IntakeCommands.getArmInCombinedCommand());

			// Manually drive intake up & down
			m_driverController.dpadRight().whileTrue(IntakeCommands.getRunArmAtPowerCommand(-0.2));
			m_driverController.dpadLeft().whileTrue(IntakeCommands.getRunArmAtPowerCommand(0.2));
		}

		{ // Climber bindings
			// m_driverController.back().debounce(Seconds.of(.1)).onTrue(ClimberCommands.getResetCommand());
			// m_driverController.dpadUp().debounce(Seconds.of(.1)).onTrue(ClimberCommands.getClimbCommand());
			// m_driverController.dpadDown().debounce(Seconds.of(.1)).onTrue(ClimberCommands.getRetractCommand());

			m_driverController.dpadUp().whileTrue(ClimberCommands.getRunAtPowerCommand(.75));
			m_driverController.dpadDown().whileTrue(ClimberCommands.getRunAtPowerCommand(-.75));
		}

		// *************************************************
		// *************** OPERATOR BINDINGS ***************
		// *************************************************

		{ // Turret bindings
			Turret.getTurret().setDefaultCommand(
					new TurretCommands.GradualAim(0.1, 0.45, 0.025,
							m_operatorController::getLeftTrigger,
							m_operatorController::getRightTrigger));
		}

		{ // Intake bindings
			m_operatorController.start().debounce(Seconds.of(0.1)).onTrue(IntakeCommands.getEncoderResetCommand());
			m_operatorController.leftBumper().debounce(Seconds.of(.1)).onTrue(new IntakeCommands.StopIntakeWheels());
			// IntakeWheels.getIntakeWheels().setDefaultCommand(
			// new IntakeCommands.Teletake(IntakeConstants.kWheelPower,
			// m_operatorController.leftBumper()));
		}

		{ // Agitator bindings
			m_operatorController.rightBumper().whileTrue(
					new TransportCommands.RunAgitatorAtPower(AgitatorConstants.kTeleopPower));

			/*
			 * m_operatorController.rightBumper().whileTrue(
			 * Commands.parallel(
			 * new IntakeCommands.Spintake(IntakeConstants.kWheelPower),
			 * Commands.sequence(
			 * Commands.parallel(
			 * Commands.repeatingSequence(
			 * new TransportCommands.RunAgitatorAtPower(
			 * AgitatorConstants.kTeleopPower)
			 * .withTimeout(1.5),
			 * new TransportCommands.RunAgitatorAtPower(
			 * -AgitatorConstants.kTeleopPower)
			 * .withTimeout(.25)),
			 * Commands.repeatingSequence(
			 * IntakeCommands.getRunArmAtPowerCommand(-1).withTimeout(1.75 / 2.0),
			 * IntakeCommands.getRunArmAtPowerCommand(1)
			 * .withTimeout(1.75 / 2.0))))))
			 * .onFalse(IntakeCommands.getArmOutCombinedCommand());
			 */

			m_operatorController.dpadLeft()
					.whileTrue(new TransportCommands.RunAgitatorAtPower(-AgitatorConstants.kTeleopPower));
		}

		{ // Shooting bindings
			// Stop flywheel
			m_operatorController.faceLeft().onTrue(new ShooterCommands.Stop());
			boolean absolute = true;

			// Presets
			m_operatorController.faceUp().debounce(Seconds.of(.1)).onTrue(new AimCommands.AdjustAim(absolute, 4.5, this));
			m_operatorController.faceRight().debounce(Seconds.of(.1)).onTrue(new AimCommands.AdjustAim(absolute, 11.7, this));
			m_operatorController.faceDown().debounce(Seconds.of(.1)).onTrue(new AimCommands.AdjustAim(absolute, 18, this));

			// Auto aim

			m_operatorController.getHID().axisLessThan(Gamepad.Axis.LEFT_TRIGGER.value, 0.025)
					.and(
							m_operatorController.getHID().axisLessThan(Gamepad.Axis.RIGHT_TRIGGER.value, 0.025)
									.and(m_operatorController.back()))
					.whileTrue(
							Commands.parallel(
									// Lights: one long-running command, sends only when the lock status changes
									USBSerialSubsystem.get().showAimStatus(
											Vision.getVision().getFieldPoseEstimator()::hasTargets),
									// Aiming: still a compat command that sets the target once, so repeat it every loop
									new RepeatCommand(new AimCommands.HubAimCommand(
											Vision.getVision().getFieldPoseEstimator()))));

			// Manual distance adjustment
			absolute = false;
			m_operatorController.dpadUp().whileTrue(new AimCommands.AdjustAim(absolute, 5, this)); // 5 ft/s increasing
			m_operatorController.dpadDown().whileTrue(new AimCommands.AdjustAim(absolute, -5, this));
		}
	}

	private void bindTestControls() {

		if (!m_isInPit.getSelected().booleanValue()) {
			Drive.getDrive().setDefaultCommand(
					new DriveCommands.JoystickDrive(
							() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
							() -> m_driverController.getRightTrigger() - m_driverController.getLeftTrigger(),
							m_driverController.getGamepad()::getBackButton));
		}

		m_driverController.back().onTrue(Commands.runOnce(() -> {
			Pose2d pose1 = Vision.getVision().getFieldPoseEstimator().getPose();
			Pose2d pose2 = new Pose2d(pose1.getTranslation(), Rotation2d.ZERO);
			m_scheduler.schedule(new DriveCommands.ResetOdometry(pose2));
		}));

		/*
		 * 12.15, 0.83, 0deg starting
		 * 10.1, 0.75, 0deg just out of trench
		 * -- open intake
		 * 8.64, 1.54, 90deg ready to intake
		 * -- spin wheels
		 */
		/*
		 * m_driverController.touchpad()
		 * .onTrue(new DriveCommands.NinjaStar(new Pose2d(14, 3.9, Rotation2d.kZero),
		 * .3, .3, Tolerances.FINEST));
		 */
		m_driverController.touchpad().onTrue(
				Commands.sequence(
						new DriveCommands.VisionDrivePose(new Pose2d(12.15, 0.83, Rotation2d.ZERO),
								Tolerances.FINE_TRANSLATION),
						new DriveCommands.VisionDrivePose(new Pose2d(10.1, 0.75, Rotation2d.ZERO),
								Tolerances.FINE_TRANSLATION),
						IntakeCommands.getArmOutCommand(),
						Commands.race(
								new DriveCommands.VisionDrivePose(new Pose2d(8.64, 1.54, Rotation2d.CCW_90DEG),
										Tolerances.COARSE),
								new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));
		m_operatorController.touchpad().onTrue(
				Commands.sequence(
						new DriveCommands.NinjaStar(new Pose2d(12.15, 0.83, Rotation2d.ZERO), .3, .3,
								Tolerances.FINE_TRANSLATION, true),
						new DriveCommands.NinjaStar(new Pose2d(10.1, 0.75, Rotation2d.ZERO), .3, .3,
								Tolerances.FINE_TRANSLATION, true),
						Commands.print("Before arm out"),
						IntakeCommands.getArmOutCommand(),
						Commands.print("After arm out"),
						Commands.race(
								new DriveCommands.NinjaStar(new Pose2d(8.64, 1.54, Rotation2d.CCW_90DEG), 0.3, .3,
										Tolerances.COARSE, true),
								new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));
		/*
		 * m_operatorController.touchpad()
		 * .onTrue(new DriveCommands.VisionDrivePose(new Pose2d(14, 3.9,
		 * Rotation2d.kZero)));
		 */
		// m_driverController.touchpad().onTrue(m_autoComposer.getVelocityTestCommand());
		// m_operatorController.touchpad().onTrue(m_autoComposer.getVelocityTestCommandReverse());

		m_driverController.faceDown().whileTrue(
				new TransportCommands.RunAgitatorAtPower(
						0.75 /* POWER */));
		m_operatorController.leftTrigger().whileTrue(
				new TransportCommands.RunAgitatorAtPower(
						-0.75)); // POWER

		/*
		 * Turret.getTurret().setDefaultCommand(
		 * new TurretCommands.RunToAngleHardwareSignal(m_operatorController::getLeftX,
		 * m_operatorController::getLeftY));
		 */
		m_operatorController.leftBumper().whileTrue(
				new AngularPositionCommands.RunAtPower(Turret.getTurret(),
						.3, /* POWER */
						0)); /* TIME */
		m_operatorController.rightBumper().whileTrue(
				new AngularPositionCommands.RunAtPower(Turret.getTurret(),
						-.3, /* POWER */
						0)); /* TIME */

		m_operatorController.faceUp().debounce(Seconds.of(0.1)).onTrue(
				new ShooterCommands.RunAtDPadRPM(this, m_operatorController.dpadRight(),
						m_operatorController.dpadLeft()));

		/*
		 * m_operatorController.faceUp().debounce(Seconds.of(0.1)).toggleOnTrue(
		 * new ShooterCommands.RunAtDynamicRPM(1000));
		 */

		m_operatorController.dpadDown().whileTrue(
				new AngularPositionCommands.RunAtPower(Hood.getHood(),
						-.2, /* POWER */
						0)); /* TIME */
		m_operatorController.dpadUp().whileTrue(
				new AngularPositionCommands.RunAtPower(Hood.getHood(),
						.2, /* POWER */
						0)); /* TIME */

		{ // climber test bindings
			m_driverController.faceUp().whileTrue(ClimberCommands.getRunAtPowerCommand(.6));
			m_driverController.faceRight().whileTrue(ClimberCommands.getRunAtPowerCommand(-0.6));
			m_driverController.start().debounce(Seconds.of(.1)).onTrue(ClimberCommands.getResetCommand());
		}

		{ // intake test bindings
			m_operatorController.faceRight().whileTrue(IntakeCommands.getRunArmAtPowerCommand(-0.2));
			m_operatorController.faceDown().whileTrue(IntakeCommands.getRunArmAtPowerCommand(0.2));
			m_operatorController.faceLeft().debounce(Seconds.of(0.1)).toggleOnTrue(
					new IntakeCommands.Spintake(
							IntakeConstants.kWheelPower)); /* POWER */
		}

	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();
		if (Constants.kLogging) {
			Telemetry.log("Scheduler", m_scheduler);
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

		m_scheduler.schedule(
				Commands.sequence(
						new WaitCommand(7),
						new USBSerialSubsystem.SetHubShiftCommand(),
						new USBSerialSubsystem.SetLightCommand(LightPattern.DEFAULT_GREEN),
						new WaitCommand(22),
						new USBSerialSubsystem.SetHubShiftCommand(),
						new USBSerialSubsystem.SetLightCommand(LightPattern.DEFAULT_GREEN),
						new WaitCommand(22),
						new USBSerialSubsystem.SetHubShiftCommand(),
						new USBSerialSubsystem.SetLightCommand(LightPattern.DEFAULT_GREEN),
						new WaitCommand(22),
						new USBSerialSubsystem.SetHubShiftCommand(),
						new USBSerialSubsystem.SetLightCommand(LightPattern.DEFAULT_GREEN),
						new WaitCommand(22),
						new USBSerialSubsystem.SetHubShiftCommand(),
						new USBSerialSubsystem.SetLightCommand(LightPattern.DEFAULT_GREEN)));

	}

	@Override
	public void disabledInit() {
		USBSerialSubsystem.get().setLightPattern(LightPattern.DEFAULT_GREEN);
	}

	@Override
	public void utilityInit() {
		initSubsystems();
		m_scheduler.cancelAll();
		m_scheduler.schedule(Commands.sequence(ClampedP.testCommand()));
	}
}