package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Aim;
import frc.robot.Constants.Subsystems.VisionConstants;
import frc.robot.PoseUtils;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_turretCamera = new PhotonCamera("TurretCamera");
	private final PhotonCamera m_fixedCamera = new PhotonCamera("FixedCamera");
	private final PoseEstimator m_hubEstimator = new FieldPoseEstimator();
	private final AngleDistanceEstimator m_midpointEstimator = new FieldPoseEstimator();
	private final PowerDistribution m_pdh = new PowerDistribution(62, ModuleType.kRev);

	public interface AngleDistanceEstimator {
		public double getAngle();

		public Distance getDistance();

		public void update(List<PhotonTrackedTarget> turretTargets, List<PhotonTrackedTarget> fixedTargets);
	}

	public interface PoseEstimator extends AngleDistanceEstimator {
		public Pose2d getBotPose();

		public Pose2d getTurretPose();
	}

	public class FieldPoseEstimator implements PoseEstimator {
		private Pose2d m_turretPose;
		private Pose2d m_botPose;
		private double m_distance;
		private double m_angle;
		private final Aim.Linear m_aim = new Aim.Linear();
		private final StructPublisher<Pose2d> m_estimatedPoseTopic = NetworkTableInstance.getDefault()
				.getStructTopic("SmartDashboard/Vision/FieldPose/Estimated Camera Pose", Pose2d.struct)
				.publish();
		private final StructPublisher<Pose2d> m_estimatedBotPoseTopic = NetworkTableInstance.getDefault()
				.getStructTopic("SmartDashboard/Vision/FieldPose/Estimated Robot Pose", Pose2d.struct)
				.publish();

		public void update(List<PhotonTrackedTarget> turretTargets, List<PhotonTrackedTarget> fixedTargets) {
			Transform2d turretTransform = new Transform2d(0, 0,
					Rotation2d.fromDegrees(Turret.getTurret().getRobotRelativeAngle()));
			List<Pose2d> poses = new ArrayList<>();
			poses.addAll(PoseUtils.extractAllPoses(fixedTargets, Transform2d.kZero));
			poses.addAll(PoseUtils.extractAllPoses(turretTargets, turretTransform));
			PoseUtils.estimateCamPoseStdDev(poses).ifPresent(pose -> {
				m_botPose = pose.pose();
				m_turretPose = m_botPose.transformBy(turretTransform.inverse());
				Drive.getEstimator().addVisionMeasurement(
						m_botPose, RobotController.getFPGATime() * 1e-6,
						MatBuilder.fill(Nat.N3(), Nat.N1(), pose.xStdDev(), pose.yStdDev(), pose.thetaStdDev()));
				SmartDashboard
						.putNumber(
								"Robot Relative Angle from Turret",
								Turret.getTurret().getRobotRelativeAngle());
				m_estimatedPoseTopic.accept(pose.pose());
				m_estimatedBotPoseTopic.accept(m_botPose);

				SmartDashboard.putNumber(
						"Vision/FieldPose/Estimated Pose Angle",
						m_botPose.getRotation().getDegrees());

				SmartDashboard.putNumber("Vision/FieldPose/Angle To Hub", m_angle);
				SmartDashboard.putNumber("Vision/FieldPose/Distance To Hub", Units.metersToFeet(m_distance));
				SmartDashboard.putNumber("Vision/FieldPose/Estimate Pose Deviation/x", pose.xStdDev());
				SmartDashboard.putNumber("Vision/FieldPose/Estimate Pose Deviation/y", pose.yStdDev());
				SmartDashboard.putNumber(
						"Vision/FieldPose/Estimate Pose Deviation/theta",
						pose.thetaStdDev());
				SmartDashboard.putNumber(
						"Vision/FieldPose/Estimated Airtime", m_aim.getShotAirtime(Units.metersToFeet(m_distance)));
			});
		}

		public Pose2d getBotPose() {
			return m_botPose;
		}

		public Pose2d getTurretPose() {
			return m_turretPose;
		}

		public double getAngle() {
			return m_angle;
		}

		public Distance getDistance() {
			return Meters.of(m_distance);
		}
	}

	public class MidpointEstimator implements AngleDistanceEstimator {
		private double m_midpointAngle = 0;
		private double m_midpointDistance = 0;

		public void update(List<PhotonTrackedTarget> turretTargets, List<PhotonTrackedTarget> fixedTargets) {
			double minAngle = 0;
			double maxAngle = 0;
			double minDistance = 0;
			double maxDistance = 0;
			for (PhotonTrackedTarget target : turretTargets) {
				if (!VisionConstants.kTrackableTags.contains(target.fiducialId))
					continue;
				double distance = target.getBestCameraToTarget().getX();
				SmartDashboard.putNumber("Tag Dist/" + target.getFiducialId(), distance);
				if (minAngle == 0 && maxAngle == 0 && minDistance == 0 && maxDistance == 0) {
					minAngle = maxAngle = target.yaw;
					minDistance = maxDistance = distance;
				} else {
					minAngle = Math.min(target.yaw, minAngle);
					maxAngle = Math.max(target.yaw, maxAngle);
					minDistance = Math.min(distance, minDistance);
					maxDistance = Math.max(distance, maxDistance);
				}
			}
			// Get midpoint distance + offset to center of hub
			m_midpointDistance = (minDistance + maxDistance) / 2 + Units.inchesToMeters(23.5);
			m_midpointAngle = (minAngle + maxAngle) / 2;

			SmartDashboard.putNumber("Vision/Midpoint/Angle to Hub", m_midpointAngle);

			SmartDashboard.putNumber("Vision/Midpoint/Distance to Hub", Units.metersToFeet(m_midpointDistance));
		}

		public double getAngle() {
			return m_midpointAngle;
		}

		public Distance getDistance() {
			return Meters.of(m_midpointDistance);
		}
	}

	public Vision() {
		if (s_vision == null) {
			s_vision = this;
		} else {
			throw new Error("Vision already instantiated");
		}
	}

	public static Vision getVision() {
		return s_vision;
	}

	public void periodic() {
		List<PhotonPipelineResult> turretResults = m_turretCamera.getAllUnreadResults();
		List<PhotonPipelineResult> fixedResults = m_fixedCamera.getAllUnreadResults();
		List<PhotonTrackedTarget> turretTargets = turretResults.get(turretResults.size() - 1).getTargets();
		List<PhotonTrackedTarget> fixedTargets = fixedResults.get(fixedResults.size() - 1).getTargets();
		boolean empty = turretTargets.isEmpty() && fixedTargets.isEmpty();
		m_pdh.setSwitchableChannel(!empty);
		if (empty) {
			return;
		}
		m_hubEstimator.update(turretTargets, fixedTargets);
		m_midpointEstimator.update(turretTargets, fixedTargets);
	}

	public AngleDistanceEstimator getMidpointEstimator() {
		return m_midpointEstimator;
	}

	public PoseEstimator getFieldPoseEstimator() {
		return m_hubEstimator;
	}
}
