// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	public static final boolean compControls = true;
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);
	private final AutoComposer m_autoComposer;
	private final SendableChooser<Command> m_autoChooser;
	private final SendableChooser<Boolean> m_isInPit = new SendableChooser<>();

	{ // Here are the individual subsystems
		new Drive();
		new Shooter();
		new IntakeWheels();
		new IntakeExtraArm();
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
		IntakeExtraArm.stopMotor();
		IntakeWheels.stopWheel();
		Agitator.stop();
		Shooter.stop();
		Drive.resetHeading();
	}

	public Robot() {
		m_autoComposer = new AutoComposer(this);
		m_autoChooser = new SendableChooser<>();
		bindAutoOptions();
	}

	private void bindAutoOptions() {
		m_autoChooser.addOption("No Auto", new WaitCommand(2));
		m_autoChooser.addOption("Left One Shoot Auto", m_autoComposer.getLeftOneShootAuto());
		m_autoChooser.addOption("Right One Shoot Auto", m_autoComposer.getRightOneShootAuto());
		m_autoChooser.addOption("Right Two Shoot Auto", m_autoComposer.getRightTwoShootAuto());
		m_autoChooser
				.addOption("Right Two Shoot Auto", m_autoComposer.getRightTwoShootAuto());
		m_autoChooser
				.addOption("Right Two Shoot Auto Front of Bump", m_autoComposer.getRightTwoShootAutoBump());
		m_autoChooser.addOption("MANUAL Right Shoot Auto", m_autoComposer.getManualRightShootAuto());
		m_autoChooser.addOption("Right Two Shoot Auto With Vision", m_autoComposer.getRightTwoShootAutoWithVision());
		m_autoChooser
				.setDefaultOption(
						"Right Two Shoot Auto Front of Bump", m_autoComposer.getRightTwoShootAutoBump());
		m_autoChooser
				.setDefaultOption(
						"Velocity 1 Test Command", m_autoComposer.getVelocity1Testcommand());
		m_isInPit.addOption("Is in pit", true);
		m_isInPit.setDefaultOption("Not in pit", false);
	}

	private void bindCompControls() {

		// ***********************************************
		// *************** DRIVER BINDINGS ***************
		// ***********************************************

		m_operatorController.touchpad().onTrue(
				Commands.sequence(
						new DriveCommands.NinjaStar(new Pose2d(12.15, 0.83, Rotation2d.kZero), .3, .3,
								Tolerances.FINE_TRANSLATION),
						new DriveCommands.NinjaStar(new Pose2d(10.1, 0.75, Rotation2d.kZero), .3, .3,
								Tolerances.FINE_TRANSLATION),
						Commands.print("Before arm out"),
						IntakeCommands.getArmOutCommand(),
						Commands.print("After arm out"),
						Commands.race(
								new DriveCommands.NinjaStar(new Pose2d(8.64, 1.54, Rotation2d.kCCW_90deg), 0.3, .3,
										Tolerances.COARSE),
								new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));

		m_driverController.touchpad().onTrue(
				Commands.sequence(
						new DriveCommands.NinjaStar(new Pose2d(12.15, 0.83, Rotation2d.kZero), .3, .3,
								Tolerances.FINE_TRANSLATION),
						new DriveCommands.NinjaStar(new Pose2d(10.1, 0.75, Rotation2d.kZero), .3, .3,
								Tolerances.FINE_TRANSLATION),
						Commands.print("Before arm out"),
						IntakeCommands.getArmOutCommand(),
						Commands.print("After arm out"),
						Commands.race(
								new DriveCommands.NinjaStar(new Pose2d(8.64, 1.54, Rotation2d.kCCW_90deg), 0.3, .3,
										Tolerances.COARSE),
								new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));

		{ // Drive bindings
			Drive.getDrive().setDefaultCommand(
					new DriveCommands.JoystickDrive(
							() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
							() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
							() -> false)); // TODO: fix robot/field-centric toggle
			m_driverController.options().debounce(0.1).onTrue(new DriveCommands.ResetHeading());
		}

		{ // Hood bindings
			m_driverController.triangle().whileTrue(
					new AngularPositionCommands.RunAtPower(Hood.getHood(),
							.2, /* POWER */
							0)); /* TIME */
			m_driverController.cross().whileTrue(
					HoodCommands.getHoodDownCommand()
							// Overrides all conflicting commands
							.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
		}

		{ // Intake bindings
			m_driverController.R1().debounce(0.1).onTrue( // Deploys arm & spins wheels
					IntakeCommands.getArmOutCombinedCommand());
			m_driverController.L1().debounce(0.1).onTrue( // Retracts arm & stops wheels
					IntakeCommands.getArmInCombinedCommand());

			// Manually drive intake up & down
			m_driverController.povRight().whileTrue(IntakeCommands.getRunArmAtPowerCommand(-0.2));
			m_driverController.povLeft().whileTrue(IntakeCommands.getRunArmAtPowerCommand(0.2));
		}

		{ // Climber bindings
			// m_driverController.create().debounce(.1).onTrue(ClimberCommands.getResetCommand());
			m_driverController.povUp().debounce(.1).onTrue(ClimberCommands.getClimbCommand());
			m_driverController.povDown().debounce(.1).onTrue(ClimberCommands.getRetractCommand());
		}

		// *************************************************
		// *************** OPERATOR BINDINGS ***************
		// *************************************************

		{ // Turret bindings
			Turret.getTurret().setDefaultCommand(
					new TurretCommands.GradualAim(0.0, 0.2, 0.025,
							m_operatorController::getL2Axis,
							m_operatorController::getR2Axis));
		}

		{ // Intake bindings
			m_operatorController.options().debounce(0.1).onTrue(IntakeCommands.getEncoderResetCommand());
			m_operatorController.L1().debounce(.1).onTrue(new IntakeCommands.StopIntakeWheels());
			// IntakeWheels.getIntakeWheels().setDefaultCommand(
			// new IntakeCommands.Teletake(IntakeConstants.kWheelPower,
			// m_operatorController.L1()));
		}

		{ // Agitator bindings
			m_operatorController.R1().whileTrue(
					new TransportCommands.RunAgitatorAtPower(AgitatorConstants.kTeleopPower));

			/*
			 * m_operatorController.R1().whileTrue(
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

			m_operatorController.povLeft()
					.whileTrue(new TransportCommands.RunAgitatorAtPower(-AgitatorConstants.kTeleopPower));
		}

		{ // Shooting bindings
			// Stop flywheel
			m_operatorController.square().onTrue(new ShooterCommands.Stop());
			boolean absolute = true;

			// Presets
			m_operatorController.triangle().debounce(.1).onTrue(new AimCommands.AdjustAim(absolute, 4.5, this));
			m_operatorController.circle().debounce(.1).onTrue(new AimCommands.AdjustAim(absolute, 11.7, this));
			m_operatorController.cross().debounce(.1).onTrue(new AimCommands.AdjustAim(absolute, 18, this));

			// Auto aim
			m_operatorController.axisLessThan(3, 0.025)
					.and(
							m_operatorController.axisLessThan(4, 0.025)
									.and(m_operatorController.create()))
					.whileTrue(
							new RepeatCommand(
									new AimCommands.HubAimCommand(Vision.getVision().getFieldPoseEstimator())));

			// Manual distance adjustment
			absolute = false;
			m_operatorController.povUp().whileTrue(new AimCommands.AdjustAim(absolute, 5, this)); // 5 ft/s increasing
			m_operatorController.povDown().whileTrue(new AimCommands.AdjustAim(absolute, -5, this));
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

		m_driverController.create().onTrue(Commands.runOnce(() -> {
			Pose2d pose1 = Vision.getVision().getFieldPoseEstimator().getPose();
			Pose2d pose2 = new Pose2d(pose1.getTranslation(), Rotation2d.kZero);
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
						new DriveCommands.VisionDrivePose(new Pose2d(12.15, 0.83, Rotation2d.kZero),
								Tolerances.FINE_TRANSLATION),
						new DriveCommands.VisionDrivePose(new Pose2d(10.1, 0.75, Rotation2d.kZero),
								Tolerances.FINE_TRANSLATION),
						IntakeCommands.getArmOutCommand(),
						Commands.race(
								new DriveCommands.VisionDrivePose(new Pose2d(8.64, 1.54, Rotation2d.kCCW_90deg),
										Tolerances.COARSE),
								new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));
		m_operatorController.touchpad().onTrue(
				Commands.sequence(
						new DriveCommands.NinjaStar(new Pose2d(12.15, 0.83, Rotation2d.kZero), .3, .3,
								Tolerances.FINE_TRANSLATION),
						new DriveCommands.NinjaStar(new Pose2d(10.1, 0.75, Rotation2d.kZero), .3, .3,
								Tolerances.FINE_TRANSLATION),
						Commands.print("Before arm out"),
						IntakeCommands.getArmOutCommand(),
						Commands.print("After arm out"),
						Commands.race(
								new DriveCommands.NinjaStar(new Pose2d(8.64, 1.54, Rotation2d.kCCW_90deg), 0.3, .3,
										Tolerances.COARSE),
								new IntakeCommands.Spintake(IntakeConstants.kWheelPower))));
		/*
		 * m_operatorController.touchpad()
		 * .onTrue(new DriveCommands.VisionDrivePose(new Pose2d(14, 3.9,
		 * Rotation2d.kZero)));
		 */
		// m_driverController.touchpad().onTrue(m_autoComposer.getVelocityTestCommand());
		// m_operatorController.touchpad().onTrue(m_autoComposer.getVelocityTestCommandReverse());

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

		{ // climber test bindings
			m_driverController.triangle().whileTrue(ClimberCommands.getRunAtPowerCommand(.6));
			m_driverController.circle().whileTrue(ClimberCommands.getRunAtPowerCommand(-0.6));
			m_driverController.options().debounce(.1).onTrue(ClimberCommands.getResetCommand());
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