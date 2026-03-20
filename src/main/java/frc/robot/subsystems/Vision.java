package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.VisionConstants;
import frc.robot.PoseUtils;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera;
	private double m_distanceToHub, m_angleToHubTag;
	private final StructPublisher<Pose3d> estimatedPoseTopic = NetworkTableInstance.getDefault()
			.getStructTopic("Estimated Vision Pose", Pose3d.struct)
			.publish(); // Initialize network table topic for estimated vision pose

	public Vision() {
		if (s_vision == null) {
			s_vision = this;
		} else {
			throw new Error("Vision already instantiated");
		}

		m_camera = new PhotonCamera("TurretCamera");
	}

	public static Vision getVision() {
		return s_vision;
	}

	public Pose3d getPose(List<PhotonPipelineResult> results) {
		List<Pose3d> poses = new ArrayList<Pose3d>();

		// Estimate pose for each pipeline result
		for (PhotonPipelineResult result : results) {
			poses.add(PoseUtils.EstimatePoseFromPipelineResult(result));
		}

		// Get initial average of each pose component (x, y, and z)
		Pose3d averagePose = PoseUtils.getAveragePose(poses);

		double components[] = {
				averagePose.getX(),
				averagePose.getY(),
				averagePose.getZ(),
				averagePose.getRotation().getX(),
				averagePose.getRotation().getY(),
				averagePose.getRotation().getZ() };

		// Cull any results that are not within 10 percent of the average
		for (int i = poses.size() - 1; i >= 0; i--) {
			boolean cull = false;

			if (Math.abs(poses.get(i).getX() - components[0]) > 0.1 * components[0]) {
				cull = true;
			}

			if (Math.abs(poses.get(i).getY() - components[1]) > 0.1 * components[1]) {
				cull = true;
			}

			if (Math.abs(poses.get(i).getZ() - components[2]) > 0.1 * components[2]) {
				cull = true;
			}

			if (Math.abs(poses.get(i).getRotation().getX() - components[3]) > 0.1 * components[3]) {
				cull = true;
			}

			if (Math.abs(poses.get(i).getRotation().getY() - components[4]) > 0.1 * components[4]) {
				cull = true;
			}

			if (Math.abs(poses.get(i).getRotation().getZ() - components[5]) > 0.1 * components[5]) {
				cull = true;
			}

			if (cull) {
				poses.remove(i);
			}
		}

		// Get more accurate average without outliers
		return PoseUtils.getAveragePose(poses);
	}

	public void periodic() {
		var results = m_camera.getAllUnreadResults();

		Pose3d estimatedPose = getPose(results);
		estimatedPoseTopic.set(estimatedPose);

		for (var result : results) {
			if (result.hasTargets()) {
				List<PhotonTrackedTarget> targets = result.getTargets();
				double distanceSum = 0;
				double angleSum = 0;
				double totalWeight = 0;
				double numTags = 0;
				for (PhotonTrackedTarget target : targets) {
					if (!VisionConstants.kTrackableTags.contains(target.fiducialId))
						continue;

					numTags++;
					double weight = Math.pow(target.getArea(), 2) * (1 -
							target.getPoseAmbiguity());
					totalWeight += weight;

					Transform3d botPose = target.getBestCameraToTarget();

					angleSum += target.yaw; // * weight;

					distanceSum += (botPose.getX() +
							Units.inchesToMeters(23.5));// * Math.sqrt(4 / Math.PI))) * weight;
				}

				m_distanceToHub = distanceSum / numTags;// totalWeight
				m_angleToHubTag = angleSum / numTags;// totalWeight

				SmartDashboard.putNumber("Vision/Angle to Tag", m_angleToHubTag);

				SmartDashboard.putNumber(
						"Vision/PoseX", Units.metersToFeet(m_distanceToHub));
			}

		}
	}

	public double getAngleToHubTag() {
		return m_angleToHubTag;
	}

	public double getDistanceToHub() {
		return Units.metersToFeet(m_distanceToHub);
	}
}
