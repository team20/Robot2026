package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.IntToDoubleFunction;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Subsystems.TurretConstants;
import frc.robot.Constants.Subsystems.VisionConstants;

public class PoseUtils {
	public record PoseResult(Pose2d pose, double xStdDev, double yStdDev, double thetaStdDev) {
	}

	public record AimResult(double setpoint, double feedforward) {
	}

	private static AprilTagFieldLayout s_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	/**
	 * Estimate the pose of the robot, including the standard deviations for each
	 * component of the pose. This hopefully may be more reliable than other methods
	 * if we want to implement a cutoff which ignores low quality data with high
	 * standard deviations.
	 * 
	 * @param result a {@code PhotonPiplineResult} which has the latest target data
	 * @return a {@code PoseResult} which contains the {@code Pose2d} of the robot
	 *         and the standard deviations
	 */
	public static Optional<PoseResult> estimateCamPoseStdDev(PhotonPipelineResult result) {
		int maxResults = result.getTargets().size();
		List<Double> data = new ArrayList<>(maxResults * 4);
		for (PhotonTrackedTarget target : result.getTargets()) {
			s_layout.getTagPose(target.getFiducialId()).ifPresent(pose -> {
				Pose2d camera = pose.transformBy(target.getBestCameraToTarget().inverse()).toPose2d();
				data.add(camera.getX());
				data.add(camera.getY());
				data.add(camera.getRotation().getSin());
				data.add(camera.getRotation().getCos());
			});
		}
		int targets = data.size() / 4;
		if (targets == 0) {
			return Optional.empty();
		}
		double x = findCenterOfData(index -> data.get(index * 4), targets);
		double y = findCenterOfData(index -> data.get(index * 4 + 1), targets);
		double sin = findCenterOfData(index -> data.get(index * 4 + 2), targets);
		double cos = findCenterOfData(index -> data.get(index * 4 + 3), targets);
		Pose2d pose = new Pose2d(x, y, Rotation2d.fromRadians(Math.atan2(sin, cos)));
		double xStdDev = findStdDevOfData(index -> data.get(index * 4), targets, x);
		double yStdDev = findStdDevOfData(index -> data.get(index * 4 + 1), targets, y);
		double sinStdDev = findStdDevOfData(index -> data.get(index * 4 + 2), targets, sin);
		double cosStdDev = findStdDevOfData(index -> data.get(index * 4 + 3), targets, cos);
		return Optional.of(new PoseResult(pose, xStdDev, yStdDev, Math.hypot(sinStdDev, cosStdDev)));
	}

	/**
	 * Estimate the pose of the robot, including the standard deviations for each
	 * component of the pose. This hopefully may be more reliable than other methods
	 * if we want to implement a cutoff which ignores low quality data with high
	 * standard deviations.
	 * 
	 * @param result a {@code PhotonPiplineResult} which has the latest target data
	 * @return a {@code PoseResult} which contains the {@code Pose2d} of the robot
	 *         and the standard deviations
	 */
	public static Optional<PoseResult> estimatePoseWithStdDev(PhotonPipelineResult result) {
		Transform2d cameraOffsetFromFrame = getCameraOffsetFromFrame();
		int maxResults = result.getTargets().size();
		List<Double> data = new ArrayList<>(maxResults * 4);
		for (PhotonTrackedTarget target : result.getTargets()) {
			s_layout.getTagPose(target.getFiducialId()).ifPresent(pose -> {
				Pose2d camera = pose.transformBy(target.getBestCameraToTarget().inverse()).toPose2d();
				Pose2d frame = camera.transformBy(cameraOffsetFromFrame.inverse());
				data.add(frame.getX());
				data.add(frame.getY());
				data.add(frame.getRotation().getSin());
				data.add(frame.getRotation().getCos());
			});
		}
		int targets = data.size() / 4;
		if (targets == 0) {
			return Optional.empty();
		}
		double x = findCenterOfData(index -> data.get(index * 4), targets);
		double y = findCenterOfData(index -> data.get(index * 4 + 1), targets);
		double sin = findCenterOfData(index -> data.get(index * 4 + 2), targets);
		double cos = findCenterOfData(index -> data.get(index * 4 + 3), targets);
		Pose2d pose = new Pose2d(x, y, Rotation2d.fromRadians(Math.atan2(sin, cos)));
		double xStdDev = findStdDevOfData(index -> data.get(index * 4), targets, x);
		double yStdDev = findStdDevOfData(index -> data.get(index * 4 + 1), targets, y);
		double sinStdDev = findStdDevOfData(index -> data.get(index * 4 + 2), targets, sin);
		double cosStdDev = findStdDevOfData(index -> data.get(index * 4 + 3), targets, cos);
		return Optional.of(new PoseResult(pose, xStdDev, yStdDev, Math.hypot(sinStdDev, cosStdDev)));
	}

	/**
	 * Finds the standard deviation of a set of data. This function actually
	 * computes the average absolute deviation, but they are essentially the same
	 * for our purposes and standard deviation doesn't work well when there are many
	 * outliers.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center the center of the data
	 * @return the standard deviation of the data
	 */
	public static double findStdDevOfData(IntToDoubleFunction data, int size, double center) {
		double deviation = 0;
		for (int i = 0; i < size; i++) {
			deviation += Math.abs(data.applyAsDouble(i) - center);
		}
		return deviation / size * Math.sqrt(Math.PI / 2);
	}

	/**
	 * Finds the center of a set of data. Internally this function uses a
	 * bisection algorithm to find the best center.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @return the best center of the data
	 */
	public static double findCenterOfData(IntToDoubleFunction data, int size) {
		return findCenterOfData(data, size, 5);
	}

	/**
	 * Finds the center of a set of data. It has a data-independent tunable
	 * parameter called {@code outlierRejectionAbility} which determines whether the
	 * center is closer to the mean or the median. Internally this function uses a
	 * bisection algorithm to find the best center.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param outlierRejectionAbility a parameter to tune what types of centers are
	 *        favored
	 * @return the best center of the data
	 */
	public static double findCenterOfData(IntToDoubleFunction data, int size, double outlierRejectionAbility) {
		double min = data.applyAsDouble(0), max = min;
		for (int i = 1; i < size; i++) {
			double point = data.applyAsDouble(i);
			max = Math.max(max, point);
			min = Math.min(min, point);
		}
		double k = Math.exp(-outlierRejectionAbility) * (max - min);
		double high = max, low = min, loss;
		do {
			double bisector = (low + high) / 2;
			loss = loss(data, size, bisector, k);
			if (loss < 0) {
				low = bisector;
			} else {
				high = bisector;
			}
		} while (Math.abs(loss) > 0.01);
		return (low + high) / 2;
	}

	/**
	 * Calculates how good of a center the chosen center is for a given set of data.
	 * When k is increased, the function favors the median. When k is decreased, the
	 * function favors the mean.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center a proposed center for the data
	 * @param k a parameter to tune what types of centers are favored
	 * @return how good the center is
	 */
	private static double loss(IntToDoubleFunction data, int size, double center, double k) {
		double total = 0;
		for (int i = 0; i < size; i++) {
			double difference = center - data.applyAsDouble(i);
			total += difference / Math.hypot(difference, k);
		}
		return total;
	}

	/**
	 * Use this method to determine the pose difference between the camera and the
	 * frame. This depends on the current angle of the turret and the overall
	 * geometry of the robot. Ensure that the constants are actually set in
	 * {@code TurretConstants.Geometry} before using this method.
	 * 
	 * @return the camera offset from the center of the frame
	 */
	public static Transform2d getCameraOffsetFromFrame() {
		// double rawTurretAngle = Turret.getTurret().getPosition() -
		// TurretConstants.Geometry.kStraightAheadAngle;
		double rawTurretAngle = 90 - TurretConstants.Geometry.kStraightAheadAngle;
		Rotation2d turretAngle = Rotation2d
				.fromDegrees(rawTurretAngle * TurretConstants.Geometry.kPositionConversionFactor);
		Translation2d cameraOffsetFromFrame = new Translation2d(
				TurretConstants.Geometry.kCameraOffsetFromTurret + TurretConstants.Geometry.kTurretOffsetFromFrame,
				0.0);
		Translation2d turretOffsetFromFrame = new Translation2d(TurretConstants.Geometry.kTurretOffsetFromFrame, 0.0);
		cameraOffsetFromFrame = cameraOffsetFromFrame.rotateAround(turretOffsetFromFrame, turretAngle);
		return new Transform2d(cameraOffsetFromFrame, turretAngle);
	}

	/**
	 * Use this method to determine the current pose of the turret on the field,
	 * given a specified robot pose. This value is useful for determining what
	 * direction the turret should point in.
	 * 
	 * @param robotPose the {@code Pose2d} of the robot
	 * @return the {@code Pose2d} of the turret
	 */
	public static Pose2d getTurretPose(Pose2d robotPose) {
		Transform2d turretOffsetFromFrame = new Transform2d(TurretConstants.Geometry.kTurretOffsetFromFrame, 0.0,
				Rotation2d.kZero);
		return robotPose.plus(turretOffsetFromFrame);
	}

	/**
	 * Use this method in order to figure out how to move the turret in order to aim
	 * at the hub. It needs to know the robot position on the field and the current
	 * speed the robot is moving at. The result is a setpoint for the turret and a
	 * minimum power level to move at. The minimum power level is assuming that the
	 * turret is already facing the hub. These will need a custom command to work
	 * together.
	 * 
	 * @param pose the {@code Pose2d} of the robot
	 * @param speeds the {@code ChassisSpeeds} of the robot
	 * @return the {@code AimResult} containing both the setpoint and power level
	 */
	public static AimResult aimToHub(Pose2d pose, ChassisSpeeds speeds) {
		Pose2d goalPose = getHub();
		AngularVelocity velocity = getTurretAngularVelocity(goalPose, pose, speeds);
		return new AimResult(getTurretAngle(goalPose, pose), getTurretFeedforward(velocity));
	}

	/**
	 * Use this method to the the Pose2d of the hub for the alliance you are on.
	 * 
	 * @return the {@code Pose2d} of the current hub
	 */
	public static Pose2d getHub() {
		return switch (DriverStation.getAlliance().orElse(Alliance.Red)) {
			case Blue -> VisionConstants.kBlueHub;
			case Red -> VisionConstants.kRedHub;
		};
	}

	/**
	 * Use this method to find out the power level need to apply to the turret in
	 * order to move it at a required angular velocity. This is used for converting
	 * the angular velocities given by {@code getTurretAngularVelocity} into power
	 * levels which you can actually use with the turret. Do not expect this to work
	 * properly without tuning.
	 * 
	 * @param velocity the angular velocity desired
	 * @return the power level to achieve the desired angular velocity
	 */
	public static double getTurretFeedforward(AngularVelocity velocity) {
		double rpm = velocity.in(RPM);
		double sign = Math.signum(rpm);
		rpm = Math.abs(rpm) * TurretConstants.Feedforward.kGearRatio;
		if (rpm < TurretConstants.Feedforward.kRPMDeadzone) {
			return 0;
		}
		double power = TurretConstants.Feedforward.kFrictionPower + rpm / TurretConstants.Feedforward.kRPMPerPower;
		return power * sign;
	}

	/**
	 * Use this method to determine how fast the turret should spin in order to
	 * continually face the hub during a "shooting-while-driving" maneuver. You will
	 * need to know where the robot is, where it is moving to, and what it is trying
	 * to aim at. A feedforward model of the turret (don't have this fully figured
	 * out yet) will be neccessary to do anything with this result. Be sure to note
	 * that the {@code ChassisSpeeds} have been modified to contain power instead of
	 * velocity.
	 * 
	 * @param goalPose the {@code Pose2d} of the target to aim at
	 * @param robotPose the {@code Pose2d} of the robot
	 * @param speeds the {@code ChassisSpeeds} of the robot
	 * @return angular velocity for the turret
	 */
	public static AngularVelocity getTurretAngularVelocity(Pose2d goalPose, Pose2d robotPose, ChassisSpeeds speeds) {
		Translation2d turretLocation = getTurretPose(robotPose).getTranslation();
		Translation2d turretTranslation = turretLocation.minus(robotPose.getTranslation());
		double rpmPerPower = RobotController.getBatteryVoltage() * 60 / DriveConstants.kV;
		double metersPerSecondPerPower = DriveConstants.kMetersPerMotorRotation * rpmPerPower / 60;
		double radiansPerSecondPerPower = metersPerSecondPerPower
				/ Math.hypot(DriveConstants.kModuleDistFromCenter, DriveConstants.kModuleDistFromCenter);
		double dTheta = speeds.omegaRadiansPerSecond * radiansPerSecondPerPower;
		double turretDX = speeds.vxMetersPerSecond * metersPerSecondPerPower - dTheta * turretTranslation.getY();
		double turretDY = speeds.vyMetersPerSecond * metersPerSecondPerPower + dTheta * turretTranslation.getX();
		Translation2d goalTranslation = goalPose.getTranslation().minus(turretLocation);
		dTheta += goalTranslation.cross(new Translation2d(turretDX, turretDY)) / goalTranslation.getSquaredNorm();
		return RadiansPerSecond.of(-dTheta);
	}

	/**
	 * Use this method when you have the position of the robot and a position (such
	 * as the hub) to point at. It will tell you the raw encoder value for the
	 * turret which aims at your target.
	 * 
	 * @param goalPose the {@code Pose2d} of the target to aim at
	 * @param robotPose the {@code Pose2d} of the robot
	 * @return encoder value for the turret
	 */
	public static double getTurretAngle(Pose2d goalPose, Pose2d robotPose) {
		Transform2d transform = goalPose.minus(getTurretPose(robotPose));
		double degrees = transform.getTranslation().getAngle().getDegrees();
		degrees /= TurretConstants.Geometry.kPositionConversionFactor;
		return degrees + TurretConstants.Geometry.kStraightAheadAngle;
	}
}