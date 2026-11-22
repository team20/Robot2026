package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import frc.robot.commands.compat.Command;
import frc.robot.AngleUtility;
import frc.robot.ClampedP;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.FieldPoseEstimator;

public class DriveCommands {

	public enum Tolerances {
		COARSE(0.15, 10),
		FINE_TRANSLATION(0.05, 10),
		FINE_ROTATION(0.15, 2.5),
		FINEST(0.05, 2.5);

		private final double m_translation;
		private final double m_rotation;

		private Tolerances(double translation, double rotation) {
			m_translation = translation;
			m_rotation = rotation;
		}

		public double getTranslation() {
			return m_translation;
		}

		public double getRotation() {
			return m_rotation;
		}
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
	 * @return a {@code ChassisVelocities} instance to drive the robot with joystick
	 *         input
	 */
	public static class JoystickDrive extends Command {
		private final DoubleSupplier m_forwardSpeed;
		private final DoubleSupplier m_strafeSpeed;
		private final DoubleSupplier m_rotation;
		private final BooleanSupplier m_isRobotRelative;

		public JoystickDrive(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
				DoubleSupplier rotation, BooleanSupplier isRobotRelative) {
			m_forwardSpeed = forwardSpeed;
			m_strafeSpeed = strafeSpeed;
			m_rotation = rotation;
			m_isRobotRelative = isRobotRelative;
			setName("Drive With Joysticks");
			addRequirements(Drive.getDrive());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			Translation2d translation = shapeTranslation(m_forwardSpeed.getAsDouble(), m_strafeSpeed.getAsDouble());
			double rotationStick = MathUtil.applyDeadband(m_rotation.getAsDouble(), kDeadzone);
			Telemetry.log("Drive/Translation Input", translation.getNorm());
			Telemetry.log("Drive/Rotation Stick", rotationStick);
			Drive.drive(translation.getX(), translation.getY(), rotationStick, m_isRobotRelative.getAsBoolean());
		}

		/**
		 * Turns the drive stick into drive power, treating the stick as one arrow (how far it's pushed, and
		 * which way) instead of two separate axes. The deadband and response curve change only how far it's
		 * pushed, so the robot always goes exactly the way the stick points, and a full push is full power in
		 * every direction, diagonals included.
		 *
		 * <p>Uses WPILib's vector versions of {@code applyDeadband} and {@code copyDirectionPow}.
		 *
		 * @param forward the forward stick value, -1 to 1 (positive = forward, +X)
		 * @param strafe the strafe stick value, -1 to 1 (positive = left, +Y)
		 * @return the forward (X) and strafe (Y) power, at most 1 long in total
		 */
		public static Translation2d shapeTranslation(double forward, double strafe) {
			Vector<N2> stick = VecBuilder.fill(forward, strafe);
			// Some gamepads report up to (1, 1) in the corners, which is 1.41 long. Limit the push to 1
			// (WPILib's deadband doesn't), so diagonals are never faster than straight lines.
			if (stick.norm() > 1) {
				stick = stick.unit();
			}
			stick = MathUtil.applyDeadband(stick, kDeadzone); // round deadband, no jump at its edge
			stick = MathUtil.copyDirectionPow(stick, kTranslationInputExponent); // response curve
			return new Translation2d(stick);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class DrivePowerAndTime extends Command {
		private final double m_fwdPower;
		private final double m_strafePower;
		private final double m_rotSpeed;
		private final double m_time;
		private Timer m_timer = new Timer();

		public DrivePowerAndTime(double fwdPower, double strafePower, double rot, double time) {
			m_fwdPower = fwdPower;
			m_strafePower = strafePower;
			m_rotSpeed = rot;
			m_time = time;
			setName("Drive For Power and Time");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			m_timer.reset();
			m_timer.start();
		}

		@Override
		public void execute() {
			Drive.drive(m_fwdPower, m_strafePower, m_rotSpeed, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_timer.stop();
			Drive.drive(0, 0, 0, true);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_time > 0 && m_timer.hasElapsed(m_time);
		}

	}

	public static class VisionDrivePose extends Command {
		private Pose2d m_targetPose;
		private final Tolerances m_tolerance;

		public VisionDrivePose(Pose2d pose, Tolerances tolerance) {
			m_targetPose = pose;
			m_tolerance = tolerance;
			setName("Drive For A Distance");
			addRequirements(Drive.getDrive());
		}

		public VisionDrivePose(Pose2d pose) {
			this(pose, Tolerances.FINE_TRANSLATION);
		}

		public double[] getError() {
			Pose2d currentPos = Drive.getPose();
			return new double[] {
					currentPos.getX() - m_targetPose.getX(), // X error
					currentPos.getY() - m_targetPose.getY(), // Y error
					currentPos.getRotation().getDegrees() - m_targetPose.getRotation().getDegrees() }; // Theta error
		}

		@Override
		public void execute() {
			double speedX, speedY, rotation;
			{
				double error = getError()[0];
				double minPower = .05;
				double maxPower = .3;
				double maxErr = .5; // When to start slowing down!!
				speedX = -ClampedP.clampedP(
						error, minPower, maxPower, maxErr,
						m_tolerance.getTranslation());
			}
			{
				double errorY = getError()[1];
				double minPower = .05;
				double maxPower = .3;
				double maxErr = .5; // When to start slowing down!!
				speedY = -ClampedP.clampedP(errorY, minPower, maxPower, maxErr, m_tolerance.getTranslation());
			}

			{
				double errorTheta = getError()[2];
				double minPower = .1;
				double maxPower = .3;
				double maxErr = 20; // When to start slowing down!!
				rotation = ClampedP.clampedP(errorTheta, minPower, maxPower, maxErr, m_tolerance.getRotation());
			}

			Telemetry.log("Drive Command/X Error", getError()[0]);
			Telemetry.log("Drive Command/Y Error", getError()[1]);
			Telemetry.log("Drive Command/Theta Error", getError()[2]);

			// rotation = .03;
			Drive.drive(speedX, speedY, rotation, false);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			// return (Math.abs(distanceY - Math.abs(m_distanceY)) < m_tolerance);
			return Math.abs(getError()[0]) < m_tolerance.getTranslation()
					&& Math.abs(getError()[1]) < m_tolerance.getTranslation() &&
					Math.abs(getError()[2]) < m_tolerance.getRotation();
		}

	}

	public static class VisionDriveDistance extends Command {
		private final double m_distanceX;
		private final double m_distanceY;
		private final double m_angle;
		private Pose2d m_targetPose;
		private static final double m_tolerance = .05; // When to stop trying!!
		private static final double m_thetaTolerance = 2;

		public VisionDriveDistance(double distanceX, double distanceY, double angle) {
			m_distanceX = distanceX;
			m_distanceY = distanceY;
			m_angle = angle;
			setName("Drive For A Distance");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Pose2d current = getCurrentPosition();

			m_targetPose = new Pose2d(current.getX() + m_distanceX, current.getY() + m_distanceY,
					Rotation2d.fromDegrees(current.getRotation().getDegrees() + m_angle));
		}

		public Pose2d getCurrentPosition() {
			Pose2d visionPose = Vision.getVision().getFieldPoseEstimator().getPose();
			Pose2d odometryPose = Drive.getPose();
			return new Pose2d(visionPose.getX(), visionPose.getY(), odometryPose.getRotation());
		}

		public double[] getError() {
			Pose2d currentPos = getCurrentPosition();
			return new double[] {
					currentPos.getX() - m_targetPose.getX(), // X error
					currentPos.getY() - m_targetPose.getY(), // Y error
					currentPos.getRotation().getDegrees() - m_targetPose.getRotation().getDegrees() }; // Theta error
		}

		@Override
		public void execute() {
			Pose2d pose = ((FieldPoseEstimator) Vision.getVision().getFieldPoseEstimator()).getPose(); // Drive.getPose().minus(m_initialPose);
			double speedX, speedY, rotation;
			{
				double error = getError()[0];
				double minPower = .05;
				double maxPower = .3;
				double maxErr = 1; // When to start slowing down!!
				speedX = -ClampedP.clampedP(
						error, minPower, maxPower, maxErr,
						m_tolerance);
			}
			{
				double errorY = getError()[1];
				double minPower = .05;
				double maxPower = .1;
				double maxErr = .5; // When to start slowing down!!
				speedY = -ClampedP.clampedP(errorY, minPower, maxPower, maxErr, m_tolerance);
			}

			{
				double errorTheta = getError()[2];
				double minPower = .1;
				double maxPower = .3;
				double maxErr = 20; // When to start slowing down!!
				rotation = ClampedP.clampedP(errorTheta, minPower, maxPower, maxErr, m_thetaTolerance);
			}

			Telemetry.log("Drive Command/X Error", getError()[0]);
			Telemetry.log("Drive Command/Y Error", getError()[1]);
			Telemetry.log("Drive Command/Theta Error", getError()[2]);

			// rotation = .03;
			Drive.drive(speedX, speedY, rotation, false);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			// return (Math.abs(distanceY - Math.abs(m_distanceY)) < m_tolerance);
			return (Math.abs(getError()[0]) < m_tolerance) && (Math.abs(getError()[1]) < m_tolerance) &&
					(Math.abs(getError()[2]) < m_thetaTolerance);
		}

	}

	public static class SpinToAngle extends Command {
		private final double m_angle;
		private final double m_speed;
		private Pose2d m_initialPose;

		public SpinToAngle(double angle, double speed) {
			m_angle = angle;
			m_speed = speed;
			setName("Spin To An Angle");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			m_initialPose = Drive.getPose();
		}

		@Override
		public void execute() {
			double angle = Drive.getPose().minus(m_initialPose).getRotation().getDegrees();
			double error = angle - m_angle;
			double speed = ClampedP.clampedP(error, 0.05, m_speed, 45, 5);
			Drive.drive(0, 0, speed, true);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			double angle = Drive.getPose().minus(m_initialPose).getRotation().getDegrees();
			return Math.abs(angle - m_angle) < 5;
		}
	}

	public static class NinjaStar extends Command {
		private final Pose2d m_pose;
		private final double m_translationSpeed;
		private final double m_rotationSpeed;
		private final Tolerances m_tolerance;
		private final boolean m_stop;

		public NinjaStar(Pose2d pose, double translationSpeed, double rotationSpeed, Tolerances tolerance,
				boolean stop) {
			m_pose = pose;
			m_translationSpeed = translationSpeed;
			m_rotationSpeed = rotationSpeed;
			m_tolerance = tolerance;
			m_stop = stop;
			setName("Drive To A Pose");
			addRequirements(Drive.getDrive());
		}

		public NinjaStar(Pose2d pose, double translationSpeed, double rotationSpeed, boolean stop) {
			this(pose, translationSpeed, rotationSpeed, Tolerances.FINE_TRANSLATION, stop);
		}

		public NinjaStar(Pose2d pose, boolean stop) {
			this(pose, 0.3, 0.3, stop);
		}

		@Override
		public void initialize() {
			Telemetry.log("Desired Pose", m_pose, Pose2d.struct);
		}

		@Override
		public void execute() {
			// Transform2d error = m_pose.minus(Drive.getPose());
			double errorX = m_pose.getX() - Drive.getPose().getX();
			double errorY = m_pose.getY() - Drive.getPose().getY();
			// double errorHypot = Math.hypot()
			// double errorTheta =
			double errorTheta = Drive.getPose().getRotation().getDegrees() - m_pose.getRotation().getDegrees();
			Telemetry.log("NinjaStar/X Error", errorX);
			Telemetry.log("NinjaStar/Y Error", errorY);
			Telemetry.log("NinjaStar/θ Error", errorTheta);
			double speedX = ClampedP.clampedP(errorX, 0.05, m_translationSpeed, 1, m_tolerance.getTranslation());
			double speedY = ClampedP.clampedP(errorY, 0.05, m_translationSpeed, 1, m_tolerance.getTranslation());
			double speedTheta = ClampedP.clampedP(errorTheta, 0.01, m_rotationSpeed, 45, m_tolerance.getRotation());
			Drive.drive(speedX, speedY, speedTheta, false);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			if (m_stop) {
				Drive.stop();
			}
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			double errorX = Math.abs(m_pose.getX() - Drive.getPose().getX());
			double errorY = Math.abs(m_pose.getY() - Drive.getPose().getY());
			double errorTheta = AngleUtility
					.minDifference(m_pose.getRotation().getDegrees(), Drive.getPose().getRotation().getDegrees());
			// System.out.printf("%f; %f%n", m_tolerance.getTranslation(),
			// m_tolerance.getRotation());
			return Math.max(errorX, errorY) < m_tolerance.getTranslation() && errorTheta < m_tolerance.getRotation();
			// Transform2d error = m_pose.minus(Drive.getPose());
			// return error.getTranslation().getNorm() < m_tolerance.getTranslation()
			// && Math.abs(error.getRotation().getDegrees()) < m_tolerance.getRotation();
		}
	}

	public static class YOLONinjaStar extends Command {
		private final Pose2d m_pose;
		private final double m_translationSpeed;
		private final double m_rotationSpeed;
		private final Tolerances m_tolerance;

		public YOLONinjaStar(Pose2d pose, double translationSpeed, double rotationSpeed, Tolerances tolerance) {
			m_pose = pose;
			m_translationSpeed = translationSpeed;
			m_rotationSpeed = rotationSpeed;
			m_tolerance = tolerance;
			setName("Drive To A Pose");
			addRequirements(Drive.getDrive());
		}

		public YOLONinjaStar(Pose2d pose, double translationSpeed, double rotationSpeed) {
			this(pose, translationSpeed, rotationSpeed, Tolerances.FINE_TRANSLATION);
		}

		public YOLONinjaStar(Pose2d pose) {
			this(pose, 0.3, 1);
		}

		@Override
		public void initialize() {
			Telemetry.log("Desired Pose YOLO", m_pose, Pose2d.struct);
		}

		@Override
		public void execute() {
			// Transform2d error = m_pose.minus(Drive.getPose());
			double errorX = Drive.getPose().getX() - m_pose.getX();
			double errorY = Drive.getPose().getY() - m_pose.getY();
			double errorHypot = Math.hypot(errorX, errorY);
			double speedX = m_translationSpeed * errorX / errorHypot;
			double speedY = m_translationSpeed * errorY / errorHypot;
			Telemetry.log("YOLONinjaStar/X Speed", speedX);
			Telemetry.log("YOLONinjaStar/Y Speed", speedY);
			double errorTheta = Drive.getPose().getRotation().getDegrees() - m_pose.getRotation().getDegrees();
			Telemetry.log("YOLONinjaStar/X Error", errorX);
			Telemetry.log("YOLONinjaStar/Y Error", errorY);
			Telemetry.log("YOLONinjaStar/θ Error", errorTheta);
			double speedTheta = ClampedP.clampedP(errorTheta, 0.01, m_rotationSpeed, 45, 0);
			Drive.drive(speedX, speedY, speedTheta, false);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			double errorX = Math.abs(m_pose.getX() - Drive.getPose().getX());
			double errorY = Math.abs(m_pose.getY() - Drive.getPose().getY());
			return Math.max(errorX, errorY) < m_tolerance.getTranslation();
		}
	}

	public static class ResetHeading extends Command {
		public ResetHeading() {
			setName("Reset Robot Heading");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.resetHeading();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class ResetOdometry extends Command {
		private final Pose2d m_pose;

		public ResetOdometry(Pose2d pose) {
			m_pose = pose;
			setName("Reset Drive Odometry");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.resetOdometry(m_pose);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class ToggleCoastMode extends Command {
		public ToggleCoastMode() {
			setName("Toggle Coast Mode");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.toggleCoastMode();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class SetCoastMode extends Command {
		private final NeutralModeValue m_mode;

		public SetCoastMode(NeutralModeValue mode) {
			m_mode = mode;
			setName("Set Coast Mode");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void execute() {
			Drive.setCoastMode(m_mode);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class Stop extends Command {
		public Stop() {
			setName("Stop Driving");
			addRequirements(Drive.getDrive());
		}

		@Override
		public void initialize() {
			Drive.stop();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}
}