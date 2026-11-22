// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.stream.Stream;

import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.Units;
import org.wpilib.util.container.CircularBuffer;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.simulation.OnboardIMUSim;
import frc.robot.commands.compat.Command;
import frc.robot.commands.compat.CommandScheduler;
import frc.robot.commands.compat.SubsystemBase;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.util.Pair;
import frc.robot.Aim;
import frc.robot.Constants;
import frc.robot.Constants.Subsystems.VisionConstants;
import frc.robot.SwerveModule;

public class Drive extends SubsystemBase {
	private static Drive s_theDrive;

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
	private final SwerveDrivePoseEstimator m_odometry;
	// SystemCore onboard IMU. TODO: mounting is TBD; update MountOrientation to match the robot.
	private final OnboardIMU m_gyro = new OnboardIMU(MountOrientation.PORTRAIT);
	// Software heading offset (replaces the navX setAngleAdjustment)
	private Rotation2d m_headingOffset = Rotation2d.ZERO;
	// Simulated raw yaw in radians, integrated from chassis omega
	private double m_simYaw = 0;
	private final SysIdRoutine m_sysidRoutine;
	private final CircularBuffer<Pair<SwerveModulePosition[], Double>> m_modulePositions = new CircularBuffer<>(2);
	private final Timer periodicTimer = new Timer();


	private Translation2d m_velocity;
	// Average velocity over 6 values to reduce input noise
	private LinearFilter m_velocityXFilter = LinearFilter.movingAverage(6);
	private LinearFilter m_velocityYFilter = LinearFilter.movingAverage(6);
	private Translation2d m_filteredVelocity;

	private final PIDController m_orientationController = new PIDController(kP, kI, kD);

	private NeutralModeValue coastMode = NeutralModeValue.Coast;

	/** Creates a new DriveSubsystem. */
	public Drive() {
		if (s_theDrive == null) {
			s_theDrive = this;
		} else {
			throw new Error("Drive already instantiated");
		}
		m_orientationController.enableContinuousInput(-Math.PI, Math.PI);
		// Adjust ramp rate, step voltage, and timeout to make sure robot doesn't
		// collide with anything
		m_sysidRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(Units.Volts.of(2.5).div(Units.Seconds.of(1)), SysIdRoutine.Config.DEFAULT_STEP_VOLTAGE,
						Units.Seconds.of(3), null),
				new SysIdRoutine.SysIdMechanism(volt -> doModuleX(
						module -> module
								.setModuleState(new SwerveModuleVelocity(volt.magnitude(), new Rotation2d(Math.PI / 2))),
						SwerveModuleVelocity[]::new), null, this));
		m_gyro.resetYaw();
		resetEncoders();
		// Wait 100 milliseconds to let all the encoders reset
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getHeading(), getModulePositions(), Pose2d.ZERO);

		periodicTimer.start();
	}

	public static Drive getDrive() {
		return s_theDrive;
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public static Rotation2d getHeading() {
		return s_theDrive.m_gyro.getRotation2d().plus(s_theDrive.m_headingOffset);// .rotateBy(Rotation2d.k180deg); // Flipped due to auto start
		// .rotateBy((AutoConstants.isBackwardsAtStart) ? Rotation2d.k180deg :
		// Rotation2d.kZero);
	}

	/**
	 * Resets drive encoders to zero.
	 */
	private static void resetEncoders() {
		doModuleX(SwerveModule::resetDriveEncoder, Boolean[]::new);
	}

	/**
	 * Returns robot pose.
	 * 
	 * @return The pose of the robot.
	 */
	public static Pose2d getPose() {
		return s_theDrive.m_odometry.getEstimatedPosition();
	}

	/**
	 * Gets the module positions for each swerve module.
	 * 
	 * @return The module positions, in order of FL, FR, BL, BR
	 */
	public static SwerveModulePosition[] getModulePositions() {
		return doModuleX(SwerveModule::getModulePosition, SwerveModulePosition[]::new);
	}

	private static <T> T[] doModuleX(Function<SwerveModule, T> function, IntFunction<T[]> constructor) {
		return Stream.of(s_theDrive.m_frontLeft, s_theDrive.m_frontRight, s_theDrive.m_backLeft, s_theDrive.m_backRight)
				.map(function).toArray(constructor);
	}

	/**
	 * Calculates module states from a chassis speeds.
	 * 
	 * @param speeds The chassis speeds.
	 * @param isFieldRelative Whether or not the chassis speeds is field relative.
	 * @return The module states, in order of FL, FR, BL, BR
	 */
	private static SwerveModuleVelocity[] calculateModuleStates(ChassisVelocities speeds, boolean isRobotRelative) {
		if (Constants.kLogging) {
			Telemetry.log("Drive/Heading", getHeading().getDegrees());
			Telemetry.log("Drive/Robot Relative", isRobotRelative);
		}
		if (!isRobotRelative)
			speeds = speeds.toRobotRelative(getHeading());
		speeds = speeds.discretize(0.03);
		SwerveModuleVelocity[] states = SwerveDriveKinematics.desaturateWheelVelocities(
				s_theDrive.m_kinematics.toSwerveModuleVelocities(speeds), 1);
		Double[] moduleAngles = doModuleX(SwerveModule::getModuleAngle, Double[]::new);
		for (int i = 0; i < states.length; i++) // Optimize target module states
			states[i] = states[i].optimize(Rotation2d.fromDegrees(moduleAngles[i]));
		return states;
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speeds The chassis speeds.
	 */
	private static void setModuleStates(SwerveModuleVelocity[] states) {
		Telemetry.log("Target Swerve Modules States", states, SwerveModuleVelocity.struct);
		doModuleX(module -> module.setModuleState(states[module.getIndex()]), SwerveModuleVelocity[]::new);
	}

	public static void stop() {
		setModuleStates(calculateModuleStates(new ChassisVelocities(), false));
	}

	public static void setCoastMode(NeutralModeValue mode) {
		s_theDrive.coastMode = mode;
		doModuleX(module -> module.setNeutralMode(mode), NeutralModeValue[]::new);
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}. Useful
	 * for updating subsystem-specific state.
	 */
	@Override
	public void periodic() {
		double timestamp = periodicTimer.get();

		// Get and publish each swerve module's state
		SwerveModuleVelocity[] states = doModuleX(SwerveModule::getModuleState, SwerveModuleVelocity[]::new);
		Telemetry.log("Current Swerve Modules States", states, SwerveModuleVelocity.struct);

		// Find and publish chassis speeds (in terms of voltage, 0.0 to 12.0)
		var speeds = m_kinematics.toChassisVelocities(states);
		Telemetry.log("Chassis Speeds", speeds, ChassisVelocities.struct);

		// Simulate the gyro
		if (RobotBase.isSimulation())// TODO: Use SysId to get feedforward model for rotation
			OnboardIMUSim.setYaw(m_simYaw += speeds.omega * TimedRobot.DEFAULT_PERIOD);

		// Update current module positions & odometry pose
		// m_moduleStates stores last (current) and second to last module positions
		m_modulePositions.addFirst(new Pair<SwerveModulePosition[], Double>(getModulePositions(), timestamp));
		Telemetry.log("Pose", m_odometry.update(getHeading(), m_modulePositions.getFirst().getFirst()), Pose2d.struct);

		// Calculate difference between last and second to last module states
		// Find x and y velocity by taking the derivative / calculating slope
		// (change in module positions over 20 ms) = velocity in meters / sec
		Twist2d twist = m_kinematics
				.toTwist2d(m_modulePositions.getLast().getFirst(), m_modulePositions.getFirst().getFirst());
		double dt = m_modulePositions.getFirst().getSecond() - m_modulePositions.getLast().getSecond();
		m_velocity = new Translation2d(twist.dx / dt, twist.dy / dt);

		// Filter velocity (use every 6th value to avoid noise)
		m_filteredVelocity = new Translation2d(m_velocityXFilter.calculate(m_velocity.getX()),
				m_velocityYFilter.calculate(m_velocity.getY()));

		// Publish velocities and pose angle
		Telemetry.log("Velocity", m_velocity, Translation2d.struct);
		Telemetry.log("Filtered Velocity", m_filteredVelocity, Translation2d.struct);
		Telemetry.log("Odometry Pose Angle", getPose().getRotation().getDegrees());
	}

	/**
	 * Use this method to get the Pose2d of the hub for the alliance you are on.
	 * 
	 * @return the {@code Pose2d} of the current hub
	 */
	public static Pose2d getAimTarget() {
		// Get alliance and robot pose
		Alliance alliance = MatchState.getAlliance().orElse(Alliance.BLUE);
		Pose2d pose = getPose();

		// If robot is in neutral zone, just pass anywhere
		// within alliance zone colinear to hub
		if (pose.getX() > VisionConstants.kBlueNeutralZoneStart && pose.getX() < VisionConstants.kRedNeutralZoneStart) {
			Pose2d target = new Pose2d(switch (alliance) {
				case BLUE -> 1;
				case RED -> 13.5;
			}, pose.getY(), getHeading());
			Telemetry.log("Aim Pose", target, Pose2d.struct);
			return target;
		}

		// Define exact hub position
		Pose2d hub = switch (alliance) {
			case BLUE -> VisionConstants.kBlueHub;
			case RED -> VisionConstants.kRedHub;
		};

		// Get bot pose orientation and world-oriented velocity (not robot relative)
		Rotation2d robotAngle = pose.getRotation();
		Translation2d rotatedVelocity = s_theDrive.m_filteredVelocity.rotateBy(robotAngle);

		// Find vector between bot pose and hub, then calculate shot distance & airtime
		// Adjust aim target to compensate for changing robot position
		// (offset calculated using velocity & airtime)
		Translation2d difference = hub.minus(pose).getTranslation();
		double distance = difference.getNorm();
		// Find airtime using distance to target (in feet)
		double time = Aim.getShotAirtime(org.wpilib.math.util.Units.metersToFeet(distance));
		Translation2d offset = rotatedVelocity.times(-time);
		Pose2d aimTarget = new Pose2d(hub.getX() + offset.getX(),
				hub.getY() + offset.getY(), Rotation2d.ZERO);

		// Iterate multiple times (5) to account for changing airtime
		// (as aim target moves to compensate, airtime increases)
		for (int i = 0; i < 5; i++) {
			difference = aimTarget.minus(pose).getTranslation();
			distance = difference.getNorm();
			time = Aim.getShotAirtime(org.wpilib.math.util.Units.metersToFeet(distance));
			offset = rotatedVelocity.times(-time);
			// Slightly increment offset with each iteration
			aimTarget = new Pose2d(hub.getX() + offset.getX(), hub.getY() + offset.getY(), Rotation2d.ZERO);
		}

		// Publish raw & compensated targets and return compensated target
		Telemetry.log("Aim Pose", hub, Pose2d.struct);
		Telemetry.log("Aim Pose Compensated", aimTarget, Pose2d.struct);
		return aimTarget;
	}

	public static SwerveDrivePoseEstimator getEstimator() {
		return s_theDrive.m_odometry;
	}

	public static void toggleCoastMode() {
		setCoastMode(switch (s_theDrive.coastMode) {
			case Coast -> NeutralModeValue.Brake;
			case Brake -> NeutralModeValue.Coast;
		});
	}

	/**
	 * Method for making the robot drive using speeds
	 * 
	 * @param forwardSpeed
	 * @param strafeSpeed
	 * @param rotation
	 * @param isRobotRelative
	 */
	public static void drive(double forwardSpeed, double strafeSpeed, double rotation, boolean isRobotRelative) {
		setModuleStates(
				calculateModuleStates(new ChassisVelocities(forwardSpeed, strafeSpeed, rotation), isRobotRelative));
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public static void resetHeading() {
		s_theDrive.m_gyro.resetYaw();
		s_theDrive.m_headingOffset = Rotation2d.fromDegrees(180.0);
	}

	public static void resetOdometry(Pose2d pose) {
		s_theDrive.m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
	}

	/**
	 * Creates a command to run a SysId quasistatic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public static org.wpilib.command3.Command sysidQuasistatic(SysIdRoutine.Direction direction) {
		return s_theDrive.m_sysidRoutine.quasistatic(direction);
	}

	/**
	 * Creates a command to run a SysId dynamic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public static org.wpilib.command3.Command sysidDynamic(SysIdRoutine.Direction direction) {
		return s_theDrive.m_sysidRoutine.dynamic(direction);
	}
}