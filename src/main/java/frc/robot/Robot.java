// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	public static final boolean compControls = true;
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);
	private final MissileLauncher m_missileLauncher = new MissileLauncher(0);
	// private final CommandGenericHID m_hid = new CommandGenericHID(0);
	// private final CommandPS4Controller m_ps4 = new CommandPS4Controller(0);
	private final Aim m_aim = new Aim.Linear();
	private final Command m_auto = null;
	private final Command m_auto2 = null;

	{ // Here are the individual subsystems
		/*
		 * new Drive();
		 * new Shooter();
		 * new IntakeWheels();
		 * new IntakeArm();
		 * new Climber();
		 * new Kicker();
		 * new Agitator();
		 * new Vision();
		 * Turret.create();
		 * Hood.create();
		 */
	}

	public void initSubsystems() {
		/*
		 * Hood.getHood().stop();
		 * Climber.getClimber().stopMotor();
		 * IntakeArm.getIntakeArm().stopMotor();
		 * IntakeWheels.stopWheel();
		 * Agitator.stop();
		 * Kicker.stop();
		 * Shooter.stop();
		 */
	}

	{ // Here is the auto currently being run
		// m_auto = new SequentialCommandGroup(
		// new DriveCommands.DriveDistance(-1.5),
		// new DriveCommands.DriveDistance(1.5),
		// new DriveCommands.DriveDistance(-1.5),
		// new DriveCommands.DriveDistance(1.5),
		// new DriveCommands.DriveDistance(-1.5),
		// new DriveCommands.DriveDistance(1.5));

		/*
		 * m_auto = new SequentialCommandGroup(
		 * new AdjustAim(true, 12.2, this).withTimeout(2),
		 * new AngularPositionCommands.RunToAngleHardware(Turret.getTurret(), 36,
		 * Turret.getConstants()).withTimeout(1),
		 * new WaitCommand(3),
		 * TransportCommands.getTimedShoot(10));
		 * m_auto2 = new AngularPositionCommands.RunToAngleHardware(Turret.getTurret(),
		 * 36, Turret.getConstants());
		 */

		// Commands.sequence(
		// new AngularPositionCommands.RunToAngleSoftware(Turret.getTurret(), 0,
		// Turret.getConstants()),
		// Commands.waitSeconds(1),
		// new AngularPositionCommands.RunToAngleSoftware(Turret.getTurret(),
		// TurretConstants.kStraightAheadAngle,
		// Turret.getConstants()),
		// Commands.waitSeconds(1),
		// new AngularPositionCommands.RunToAngleSoftware(Turret.getTurret(),
		// TurretConstants.kMaxAngle,
		// Turret.getConstants()));
	}

	private void bindCompControls() {
		/*
		 * m_hid.button(11).whileTrue(Commands.run(() ->
		 * SmartDashboard.putBoolean("Pressed", true)));
		 * m_hid.button(11).whileFalse(Commands.run(() ->
		 * SmartDashboard.putBoolean("Pressed", false)));
		 * m_missileLauncher.bottomFarLeftButton()
		 * .whileTrue(Commands.run(() -> SmartDashboard.putBoolean("Pressed2", true)));
		 * m_missileLauncher.bottomFarLeftButton()
		 * .whileFalse(Commands.run(() -> SmartDashboard.putBoolean("Pressed2",
		 * false)));
		 */
		// m_ps4.options().whileTrue(Commands.run(() ->
		// SmartDashboard.putBoolean("Pressed", true)));
		// m_ps4.options().whileFalse(Commands.run(() ->
		// SmartDashboard.putBoolean("Pressed", false)));

		// *************** DRIVER BINDINGS ***************
		{ // Drive bindings
			/*
			 * Drive.getDrive().setDefaultCommand(
			 * new DriveCommands.JoystickDrive(
			 * () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
			 * () -> m_driverController.getL2Axis() - m_driverController.getR2Axis(), // L2
			 * rotates left,
			 * // R2 rotates right
			 * m_driverController.getHID()::getCreateButton));
			 * m_driverController.options().debounce(0.1).onTrue(new
			 * DriveCommands.ResetHeading());
			 */
		}

		{ // Hood bindings
			/*
			 * m_driverController.cross().whileTrue(
			 * new AngularPositionCommands.RunAtPower(Hood.getHood(),
			 * -.2,
			 * 0).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
			 * m_driverController.triangle().whileTrue(
			 * new AngularPositionCommands.RunAtPower(Hood.getHood(),
			 * .2,
			 * 0));
			 */
			// m_driverController.cross()
			// .whileTrue(new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 0,
			// Hood.getConstants()));
		}

		{ // Intake bindings
			// TODO uncomment when robot fixed
			// //m_operatorController.options().debounce(0.1).onTrue(IntakeCommands.getEncoderResetCommand());
			/*
			 * m_driverController.R1().debounce(0.1).onTrue(IntakeCommands.getOutCommand());
			 * // Deploys arm TODO: tune
			 * // position
			 * m_driverController.L1().debounce(0.1).onTrue(
			 * Commands.sequence(
			 * new IntakeCommands.StopIntake(),
			 * new TransportCommands.StopAgitator(),
			 * IntakeCommands.getInCommand()));// Retracts arm and stops power TODO: tune
			 * position
			 * m_driverController.povRight().whileTrue(IntakeCommands.
			 * getRunArmAtPowerCommand(-0.2));
			 * m_driverController.povLeft().whileTrue(IntakeCommands.getRunArmAtPowerCommand
			 * (0.2));
			 */
		}

		{ // Climber bindings
			/*
			 * m_driverController.povUp().whileTrue(ClimberCommands.getRunAtPowerCommand(.6)
			 * );
			 * m_driverController.povDown().whileTrue(ClimberCommands.getRunAtPowerCommand(-
			 * 0.6));
			 */
		}

		// *************** OPERATOR BINDINGS ***************

		{ // Turret bindings
			/*
			 * m_operatorController.L2().whileTrue(
			 * new AngularPositionCommands.RunAtPower(Turret.getTurret(), -.2, 0));//
			 * Rotates
			 * // left/counterclockwise
			 * m_operatorController.R2().whileTrue(new
			 * AngularPositionCommands.RunAtPower(Turret.getTurret(), .2, 0));// Rotates
			 */
			// right/clockwise
		}

		{ // Transport commands
			/*
			 * m_operatorController.R1().whileTrue( // Runs agitator and kicker (for
			 * shooting) when pressed on R1
			 * Commands.parallel(
			 * new TransportCommands.RunAgitatorAtPower(AgitatorConstants.kTeleopPower)));
			 * // new TransportCommands.RunKickerAtPower(KickerConstants.kTeleopPower)));
			 * m_operatorController.povLeft()
			 * .whileTrue(new
			 * TransportCommands.RunAgitatorAtPower(-AgitatorConstants.kTeleopPower));
			 * m_operatorController.L1().debounce(.05)
			 * .toggleOnTrue(new IntakeCommands.Spintake(IntakeConstants.kWheelPower));
			 */
			// m_operatorController.create().whileTrue(new
			// IntakeCommands.Spintake(-IntakeConstants.kWheelPower));
		}

		{ // Shooting bindings
			/*
			 * m_operatorController.square().onTrue(new ShooterCommands.Stop());
			 * boolean absolute = true;
			 * m_operatorController.triangle().debounce(.1).onTrue(new
			 * AimCommands.AdjustAim(absolute, 4.5, this));
			 * m_operatorController.circle().debounce(.1).onTrue(new
			 * AimCommands.AdjustAim(absolute, 9.1, this));
			 * m_operatorController.cross().debounce(.1).onTrue(new
			 * AimCommands.AdjustAim(absolute, 13, this));
			 * m_operatorController.create().whileTrue(
			 * new RepeatCommand(new AimCommands.AutoAim(Turret.getTurret(),
			 * Vision.getVision())));
			 */
			// m_operatorController.triangle().debounce(0.1).onTrue(
			// new ShooterCommands.RunAtDPadRPM(this, m_operatorController.povRight(),
			// m_operatorController.povLeft()));

			/*
			 * absolute = false;
			 * m_operatorController.povUp().whileTrue(new AimCommands.AdjustAim(absolute, 5,
			 * this)); // 5 ft/s increasing
			 * m_operatorController.povDown().whileTrue(new AimCommands.AdjustAim(absolute,
			 * -5, this));
			 */
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
		/*
		 * Drive.getDrive().setDefaultCommand(
		 * new DriveCommands.JoystickDrive(
		 * () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
		 * () -> m_driverController.getR2Axis() - m_driverController.getL2Axis(),
		 * m_driverController.getHID()::getCreateButton));
		 * m_driverController.cross().whileTrue(
		 * new TransportCommands.RunAgitatorAtPower(
		 * 0.75 ));
		 * m_driverController.square().whileTrue(
		 * new TransportCommands.RunKickerAtPower(
		 * 0.4 ));
		 * m_operatorController.L2().whileTrue(
		 * new TransportCommands.RunAgitatorAtPower(
		 * -0.75)); // POWER
		 */

		/*
		 * Turret.getTurret().setDefaultCommand(
		 * new TurretCommands.RunToAngleHardwareSignal(m_operatorController::getLeftX,
		 * m_operatorController::getLeftY));
		 */
		/*
		 * m_operatorController.L1().whileTrue(
		 * new AngularPositionCommands.RunAtPower(Turret.getTurret(),
		 * .3,
		 * 0));
		 * m_operatorController.R1().whileTrue(
		 * new AngularPositionCommands.RunAtPower(Turret.getTurret(),
		 * -.3,
		 * 0));
		 * m_operatorController.triangle().debounce(0.1).onTrue(
		 * new ShooterCommands.RunAtDPadRPM(this, m_operatorController.povRight(),
		 * m_operatorController.povLeft()));
		 */

		/*
		 * m_operatorController.triangle().debounce(0.1).toggleOnTrue(
		 * new ShooterCommands.RunAtDynamicRPM(1000));
		 */

		/*
		 * m_operatorController.povDown().whileTrue(
		 * new AngularPositionCommands.RunAtPower(Hood.getHood(),
		 * -.2,
		 * 0));
		 * m_operatorController.povUp().whileTrue(
		 * new AngularPositionCommands.RunAtPower(Hood.getHood(),
		 * .2,
		 * 0));
		 * m_operatorController.touchpad()
		 * .onTrue(new AngularPositionCommands.RunToAngleHardware(Hood.getHood(), 0,
		 * Hood.getConstants()));
		 */

		{ // climber test bindings
			/*
			 * m_driverController.triangle().whileTrue(ClimberCommands.getRunAtPowerCommand(
			 * .6));
			 * m_driverController.circle().whileTrue(ClimberCommands.getRunAtPowerCommand(-0
			 * .6));
			 */
		}

		{ // intake test bindings
			/*
			 * m_operatorController.circle().whileTrue(IntakeCommands.
			 * getRunArmAtPowerCommand(-0.2));
			 * m_operatorController.cross().whileTrue(IntakeCommands.getRunArmAtPowerCommand
			 * (0.2));
			 * m_operatorController.square().debounce(0.1).toggleOnTrue(
			 * new IntakeCommands.Spintake(
			 * IntakeConstants.kWheelPower));
			 */
		}

	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();
		if (Constants.kLogging) {
			SmartDashboard.putData(m_scheduler);
			SmartDashboard.putData("Missile Launcher", m_missileLauncher);
		}
	}

	@Override
	public void autonomousInit() {
		initSubsystems();
		m_scheduler.cancelAll();
		m_scheduler.schedule(m_auto);
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
