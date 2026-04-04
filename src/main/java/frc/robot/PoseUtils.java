package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleUnaryOperator;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class PoseUtils {
	public record PoseResult(Pose2d pose, double xStdDev, double yStdDev, double thetaStdDev) {
	}

	private static final AprilTagFieldLayout s_layout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2026RebuiltWelded);
	private static final double TRANSLATION_MEAN_RADIUS = 0.1; // 0.1 meters (+/- 4 inch deviation expected)
	private static final double ROTATION_MEAN_RADIUS = 0.1; // 0.1 radians (+/- 6 degree deviation expected)

	public static List<Pose2d> extractAllPoses(List<PhotonTrackedTarget> targets, Transform2d transform) {
		int maxResults = targets.size();
		List<Pose2d> poses = new ArrayList<>(maxResults);
		for (PhotonTrackedTarget target : targets) {
			s_layout.getTagPose(target.getFiducialId()).ifPresent(pose -> {
				Pose2d camera = pose.transformBy(target.getBestCameraToTarget().inverse()).toPose2d();
				poses.add(camera.transformBy(transform));
			});
		}
		return poses;
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
	@SuppressWarnings("unchecked")
	public static Optional<PoseResult> estimateCamPoseStdDev(List<Pose2d> poses) {
		if (poses.isEmpty()) {
			return Optional.empty();
		}
		int count = poses.size();
		DoubleUnaryOperator wFunction = DataUtils.standardWFunction(TRANSLATION_MEAN_RADIUS);
		double[] position = DataUtils
				.findMultidimensionalCenter(count, wFunction, poses::get, Pose2d::getX, Pose2d::getY);
		wFunction = DataUtils.standardWFunction(ROTATION_MEAN_RADIUS);
		double[] heading = DataUtils.findMultidimensionalCenter(
				count, wFunction, index -> poses.get(index).getRotation(), Rotation2d::getCos, Rotation2d::getSin);
		Pose2d pose = new Pose2d(position[0], position[1], Rotation2d.fromRadians(Math.atan2(heading[1], heading[0])));
		double[] positionStdDev = DataUtils
				.findAverageAbsoluteDeviation(count, position, poses::get, Pose2d::getX, Pose2d::getY);
		double[] headingStdDev = DataUtils.findAverageAbsoluteDeviation(
				count, heading, index -> poses.get(index).getRotation(), Rotation2d::getCos, Rotation2d::getSin);
		return Optional.of(
				new PoseResult(pose, positionStdDev[0], positionStdDev[1],
						Math.hypot(headingStdDev[0], headingStdDev[1])));
	}
}
