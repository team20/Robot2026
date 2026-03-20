package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Subsystems.TurretConstants;
import frc.robot.Constants.Subsystems.VisionConstants;
import frc.robot.subsystems.Turret;

public class PoseUtils {
	public record PoseResult(Pose2d pose, double xStdDev, double yStdDev, double thetaStdDev) {
	}

	public record AimResult(double setpoint, double feedforward) {
	}

	private static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	/**
	 * Method to get the estimated pose from a PhotonPipelineResult using
	 * PhotonPoseEstimator
	 * 
	 * @param result Input pipeline result to estimate pose from
	 * @param turretAngle Angle of the turret (for getting the actual camera to
	 *        robot translation)
	 * @return Estimated Pose3d
	 */
	public static Pose3d EstimatePoseFromPipelineResult(PhotonPipelineResult result) {
		Transform2d cameraOffsetFromFrame = getCameraOffsetFromFrame();
		PhotonPoseEstimator estimator = new PhotonPoseEstimator(
				layout,
				new Transform3d(cameraOffsetFromFrame.getX(), cameraOffsetFromFrame.getY(),
						TurretConstants.Geometry.kTurretHeightFromFloor,
						new Rotation3d(cameraOffsetFromFrame.getRotation())));

		Optional<EstimatedRobotPose> estimatedPose = estimator.estimateCoprocMultiTagPose(result);

		if (estimatedPose.isEmpty()) {
			estimatedPose = estimator.estimateLowestAmbiguityPose(result);
		}

		if (estimatedPose.isEmpty()) {
			return null;
		}

		return estimatedPose.get().estimatedPose;
	}

	public static PoseResult estimatePoseWithStdDev(PhotonPipelineResult result) {
		Transform2d cameraOffsetFromFrame = getCameraOffsetFromFrame();

		for (PhotonTrackedTarget target : result.targets) {

		}

		return new PoseResult(Pose2d.kZero, 0, 0, 0);
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
		double rawTurretAngle = Turret.getTurret().getPosition() - TurretConstants.Geometry.kStraightAheadAngle;
		Rotation2d turretAngle = Rotation2d
				.fromDegrees(rawTurretAngle * TurretConstants.Geometry.kPositionConversionFactor);
		Translation2d cameraOffsetFromFrame = new Translation2d(
				TurretConstants.Geometry.kCameraOffsetFromTurret + TurretConstants.Geometry.kTurretOffsetFromFrame,
				0.0);
		Translation2d turretOffsetFromFrame = new Translation2d(TurretConstants.Geometry.kTurretOffsetFromFrame, 0.0);
		cameraOffsetFromFrame = cameraOffsetFromFrame.rotateAround(turretOffsetFromFrame, turretAngle);
		return new Transform2d(turretOffsetFromFrame, turretAngle);
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
		Pose2d goalPose = switch (DriverStation.getAlliance().orElse(Alliance.Red)) {
			case Blue -> VisionConstants.kBlueHub;
			case Red -> VisionConstants.kRedHub;
		};
		AngularVelocity velocity = getTurretAngularVelocity(goalPose, pose, speeds);
		return new AimResult(getTurretAngle(goalPose, pose), getTurretFeedforward(velocity));
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
	 * out yet) will be
	 * neccessary to do anything with this result.
	 * 
	 * @param goalPose the {@code Pose2d} of the target to aim at
	 * @param robotPose the {@code Pose2d} of the robot
	 * @param speeds the {@code ChassisSpeeds} of the robot
	 * @return angular velocity for the turret
	 */
	public static AngularVelocity getTurretAngularVelocity(Pose2d goalPose, Pose2d robotPose, ChassisSpeeds speeds) {
		Pose2d turretPose = getTurretPose(robotPose);

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

	/**
	 * Method to average each component of a pose
	 * 
	 * @param poses Input poses to average
	 * @return Final average of pose
	 */
	public static Pose3d getAveragePose(List<Pose3d> poses) {
		double sums[] = new double[6];
		double averages[] = new double[6];
		for (Pose3d pose : poses) {
			if (pose == null) {
				continue;
			}

			sums[0] += pose.getX();
			sums[1] += pose.getY();
			sums[2] += pose.getZ();
			sums[3] += pose.getRotation().getX();
			sums[4] += pose.getRotation().getY();
			sums[5] += pose.getRotation().getZ();
		}

		for (int i = 0; i < 6; i++) {
			averages[i] = sums[i] / poses.size();
		}

		return new Pose3d(averages[0], averages[1], averages[2], new Rotation3d(averages[3], averages[4], averages[5]));
	}
}