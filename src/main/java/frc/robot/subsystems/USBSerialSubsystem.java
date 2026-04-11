package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class USBSerialSubsystem extends SubsystemBase {
	private SerialPort usbSerial;
	private static USBSerialSubsystem s_SerialSubsystem;

	public static USBSerialSubsystem get() {
		return s_SerialSubsystem;
	}

	public USBSerialSubsystem() {
		if (s_SerialSubsystem == null) {
			s_SerialSubsystem = this;
		} else {
			throw new Error("USB Serial already instantiated");
		}

		try {
			// Initialize USB port (usually kUSB)
			usbSerial = new SerialPort(9600, SerialPort.Port.kUSB);
		} catch (Exception e) {
			System.out.println("USB Serial Port Init Failed: " + e.getMessage());
		}
	}

	public void sendData(byte[] data) {
		System.out.println("TRY SEND DATA: " + Arrays.toString(data));
		if (usbSerial != null) {
			usbSerial.write(data, data.length);
		}
	}

	public static enum LightPattern {
		DEFAULT_GREEN,
		VISION_AIM_LOCKED,
		VISION_AIM_NOT_LOCKED,
		TURRET_MANUAL,

		HUB_SHIFT,
		ENDGAME_SHIFT,

		STATE_COUNT
	}

	public static class SetLightAutoAimStatus extends Command {

		public SetLightAutoAimStatus() {
			addRequirements(USBSerialSubsystem.get(), Vision.getVision());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			USBSerialSubsystem.s_SerialSubsystem.sendData(
					new byte[] { (byte) (Vision.getVision().getFieldPoseEstimator().hasTargets()
							? LightPattern.VISION_AIM_LOCKED
							: LightPattern.VISION_AIM_NOT_LOCKED).ordinal() });
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class SetLightCommand extends Command {
		public final LightPattern m_pattern;

		public SetLightCommand(LightPattern pattern) {
			m_pattern = pattern;
			addRequirements(USBSerialSubsystem.s_SerialSubsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			USBSerialSubsystem.s_SerialSubsystem.sendData(new byte[] { (byte) m_pattern.ordinal() });
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

}
