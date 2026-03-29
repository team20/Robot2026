package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RainbowParty extends SubsystemBase {
/** The USB port that's used to connect to the Arduino. */
	private SerialPort m_usb;

		private SerialPort m_usb;

	/** The bytes that control the LED mode */
	public enum StatusCode {
		RESET((byte) 0),
		RAINBOW_PARTY_FUN_TIME((byte) 1),
		SOLID_ORANGE((byte) 2),
		SOLID_BLUE((byte) 3),
		BLINKING_YELLOW((byte) 4),
		BLINKING_PURPLE((byte) 5),
		BLINKING_RED((byte) 6),
		DEFAULT((byte) 20);

		public byte code;

		private StatusCode(byte c) {
			code = c;
		}
	}

		
	/** Creates a new ArduinoSubsystem. */
	public ArduinoSubsystem() {
		try {
			m_usb = new SerialPort(9600, Port.kUSB);
		} catch (Exception e) {
			DriverStation.reportError("Could not initialize Arduino over USB", false);
			m_usb = null;
		}
		setCode(StatusCode.DEFAULT);
	}



blic void setCode(StatusCode code) {
		if (m_usb != null) {

			m_usb.write(new byte[] { code.code }, 1);
		}
	}



lic Command writeStatus(StatusCode 
