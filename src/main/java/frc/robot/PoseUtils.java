package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.Subsystems.VisionConstants;

public class PoseUtils {
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
	public static Pose3d EstimatePoseFromPipelineResult(PhotonPipelineResult result, Angle turretAngle) {
		PhotonPoseEstimator estimator = new PhotonPoseEstimator(
				layout,
				new Transform3d(
						new Translation3d(
								-VisionConstants.cameraDist * Math.sin(turretAngle.in(Radians)),
								VisionConstants.cameraDist * Math.cos(turretAngle.in(Radians)),
								VisionConstants.cameraHeight),
						new Rotation3d(0, 0, 180 - turretAngle.in(Degrees))));

		Optional<EstimatedRobotPose> estimatedPose = estimator.estimateCoprocMultiTagPose(result);

		if (estimatedPose.isEmpty()) {
			estimatedPose = estimator.estimateLowestAmbiguityPose(result);
		}

		if (estimatedPose.isEmpty()) {
			return null;
		}

		return estimatedPose.get().estimatedPose;
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