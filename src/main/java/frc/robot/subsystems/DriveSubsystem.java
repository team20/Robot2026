// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntFunction;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
	private final SwerveModule m_frontLeft = new SwerveModule(0, kFrontLeftCANCoderPort, kFrontLeftDrivePort,
			kFrontLeftSteerPort, true);
	private final SwerveModule m_frontRight = new SwerveModule(1, kFrontRightCANCoderPort, kFrontRightDrivePort,
			kFrontRightSteerPort, false);
	private final SwerveModule m_backLeft = new SwerveModule(2, kBackLeftCANCoderPort, kBackLeftDrivePort,
			kBackLeftSteerPort, true);
	private final SwerveModule m_backRight = new SwerveModule(3, kBackRightCANCoderPort, kBackRightDrivePort,
			kBackRightSteerPort, false);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private final SwerveDriveOdometry m_odometry;
	private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
	private final SimDouble m_gyroSim;
	private final SysIdRoutine m_sysidRoutine;

	private final StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("/SmartDashboard/Pose", Pose2d.struct).publish();
	private final StructPublisher<ChassisSpeeds> m_currentChassisSpeedsPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("/SmartDashboard/Chassis Speeds", ChassisSpeeds.struct).publish();
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher = NetworkTableInstance
			.getDefault().getStructArrayTopic("/SmartDashboard/Target Swerve Modules States", SwerveModuleState.struct)
			.publish();
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher = NetworkTableInstance
			.getDefault().getStructArrayTopic("/SmartDashboard/Current Swerve Modules States", SwerveModuleState.struct)
			.publish();
	private final StructPublisher<Rotation2d> m_targetHeadingPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("/SmartDashboard/Target Heading", Rotation2d.struct).publish();

	private final PIDController m_orientationController = new PIDController(kRotationP, kRotationI, kRotationD);

	private static class CoastState {
		private boolean coastState = false;

		public boolean shouldBeCoast() {
			boolean coast = coastState;
			coastState ^= true; // Toggle coast state each time
			return coast;
		}

		public void shouldBeCoast(boolean coast) {
			coastState = coast;
		}
	}

	private final CoastState coastState = new CoastState();

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		m_orientationController.enableContinuousInput(-Math.PI, Math.PI);
		// Adjust ramp rate, step voltage, and timeout to make sure robot doesn't
		// collide with anything
		m_sysidRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(Units.Volts.of(2.5).div(Units.Seconds.of(1)), null, Units.Seconds.of(3)),
				new SysIdRoutine.Mechanism(volt -> doModuleX(
						module -> module
								.setModuleState(new SwerveModuleState(volt.magnitude(), new Rotation2d(Math.PI / 2))),
						SwerveModuleState[]::new), null, this));
		m_gyro.zeroYaw();
		resetEncoders();
		// Wait 100 milliseconds to let all the encoders reset
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());
		if (RobotBase.isSimulation()) {
			m_gyroSim = new SimDeviceSim("navX-Sensor", m_gyro.getPort()).getDouble("Yaw");
		} else {
			m_gyroSim = null;
		}
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public Rotation2d getHeading() {
		return m_gyro.getRotation2d();
	}

	/**
	 * Resets drive encoders to zero.
	 */
	private void resetEncoders() {
		doModuleX(SwerveModule::resetDriveEncoder, Boolean[]::new);
	}

	/**
	 * Returns robot pose.
	 * 
	 * @return The pose of the robot.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Gets the module positions for each swerve module.
	 * 
	 * @return The module positions, in order of FL, FR, BL, BR
	 */
	public SwerveModulePosition[] getModulePositions() {
		return doModuleX(SwerveModule::getModulePosition, SwerveModulePosition[]::new);
	}

	private <T> T[] doModuleX(Function<SwerveModule, T> function, IntFunction<T[]> constructor) {
		return List.of(
				function.apply(m_frontLeft), function.apply(m_frontRight), function.apply(m_backLeft),
				function.apply(m_backRight)).toArray(constructor);
	}

	/**
	 * Calculates module states from a chassis speeds.
	 * 
	 * @param speeds The chassis speeds.
	 * @param isFieldRelative Whether or not the chassis speeds is field relative.
	 * @return The module states, in order of FL, FR, BL, BR
	 */
	private SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds, boolean isRobotRelative) {
		SmartDashboard.putNumber("Heading", getHeading().getDegrees());
		SmartDashboard.putBoolean("Robot Relative", isRobotRelative);
		if (!isRobotRelative)
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		speeds = ChassisSpeeds.discretize(speeds, 0.03);
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopDriveMaxSpeed);
		Double[] moduleAngles = doModuleX(SwerveModule::getModuleAngle, Double[]::new);
		for (int i = 0; i < states.length; i++) // Optimize target module states
			states[i].optimize(Rotation2d.fromDegrees(moduleAngles[i]));
		return states;
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speeds The chassis speeds.
	 */
	private void setModuleStates(SwerveModuleState[] states) {
		SmartDashboard.putNumber("Target number", states.length);
		m_targetModuleStatePublisher.set(states);
		doModuleX(module -> module.setModuleState(states[module.getIndex()]), SwerveModuleState[]::new);
	}

	public void stopAllModules() {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(), false));
	}

	public void setDriveMotorNeutralMode(NeutralModeValue mode) {
		// If we just set the motors to brake, when toggling, it should then switch to
		// coast
		coastState.shouldBeCoast(mode == NeutralModeValue.Brake);
		doModuleX(module -> module.setNeutralMode(mode), NeutralModeValue[]::new);
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}. Useful
	 * for updating subsystem-specific state.
	 */
	@Override
	public void periodic() {
		SwerveModuleState[] states = doModuleX(SwerveModule::getModuleState, SwerveModuleState[]::new);
		m_currentModuleStatePublisher.set(states);
		var speeds = m_kinematics.toChassisSpeeds(states);
		m_currentChassisSpeedsPublisher.set(speeds);
		if (RobotBase.isSimulation())// TODO: Use SysId to get feedforward model for rotation
			m_gyroSim.set(
					-Math.toDegrees(speeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod) + m_gyro.getYaw());
		m_posePublisher.set(m_odometry.update(getHeading(), getModulePositions()));
	}

	public Command toggleCoastMode() {
		return runOnce(
				() -> setDriveMotorNeutralMode(
						coastState.shouldBeCoast() ? NeutralModeValue.Coast : NeutralModeValue.Brake))
								.withName("Drive Toggle Coast Mode");
	}

	public Command setNeutralMode(NeutralModeValue mode) {
		return runOnce(() -> setDriveMotorNeutralMode(mode)).withName("Drive Enable Coast Mode");
	}

	/**
	 * Method for making the robot drive using speeds
	 * 
	 * @param forwardSpeed
	 * @param strafeSpeed
	 * @param rotation
	 * @param isRobotRelative
	 */
	public void drive(double forwardSpeed, double strafeSpeed, double rotation, boolean isRobotRelative) {
		setModuleStates(
				calculateModuleStates(
						chassisSpeeds(() -> forwardSpeed, () -> strafeSpeed, () -> rotation), !isRobotRelative));
	}

	/**
	 * Creates a {@code Command} to drive the robot with joystick input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation Rotation supplier. Positive values make
	 *        the robot rotate left (CCW direction).
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	public Command driveCommand(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation, BooleanSupplier isRobotRelative) {
		return runEnd(
				() -> drive(
						forwardSpeed.getAsDouble(), strafeSpeed.getAsDouble(), rotation.getAsDouble(),
						isRobotRelative.getAsBoolean()),
				this::stopAllModules)
						.withName("DefaultDriveCommand");
	}

	/**
	 * Creates a {@code ChassisSpeeds} instance to drive the robot with joystick
	 * input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation Rotation supplier. Positive values make
	 *        the robot rotate left (CCW direction).
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	public static ChassisSpeeds chassisSpeeds(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation) {
		double vxMetersPerSecond = MathUtil.applyDeadband(forwardSpeed.getAsDouble(), ControllerConstants.kDeadzone);
		vxMetersPerSecond = Math.signum(vxMetersPerSecond) * Math.pow(vxMetersPerSecond, 2) * kTeleopDriveMaxSpeed;
		double vyMetersPerSecond = MathUtil.applyDeadband(strafeSpeed.getAsDouble(), ControllerConstants.kDeadzone);
		vyMetersPerSecond = Math.signum(vyMetersPerSecond) * Math.pow(vyMetersPerSecond, 2) * kTeleopDriveMaxSpeed;
		vxMetersPerSecond = MathUtil.clamp(vxMetersPerSecond, -kTeleopDriveMaxSpeed, kTeleopDriveMaxSpeed);
		vyMetersPerSecond = MathUtil.clamp(vyMetersPerSecond, -kTeleopDriveMaxSpeed, kTeleopDriveMaxSpeed);
		double omegaRadiansPerSecond = MathUtil.applyDeadband(rotation.getAsDouble(), ControllerConstants.kDeadzone);
		omegaRadiansPerSecond = Math.signum(omegaRadiansPerSecond) * Math.pow(omegaRadiansPerSecond, 2)
				* kTeleopTurnMaxAngularSpeed;
		omegaRadiansPerSecond = MathUtil
				.clamp(omegaRadiansPerSecond, -kTeleopTurnMaxAngularSpeed, kTeleopTurnMaxAngularSpeed);
		return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public Command resetHeading() {
		return runOnce(m_gyro::zeroYaw).withName("ResetHeadingCommand");
	}

	public Command resetOdometry(Pose2d pose) {
		return runOnce(() -> m_odometry.resetPosition(getHeading(), getModulePositions(), pose))
				.withName("ResetOdometryCommand");
	}

	/**
	 * Creates a command to run a SysId quasistatic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.quasistatic(direction);
	}

	/**
	 * Creates a command to run a SysId dynamic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidDynamic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.dynamic(direction);
	}
}
