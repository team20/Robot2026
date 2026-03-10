package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MissileLauncher extends CommandGenericHID implements Sendable {
	public enum Button {
		kTopFarLeftButton(18),
		kTopMiddleLeftButton(17),
		kTopMiddleRightButton(16),
		kTopFarRightButton(15),
		kBottomFarLeftButton(11),
		kBottomMiddleLeftButton(12),
		kBottomMiddleRightButton(13),
		kBottomFarRightButton(14),
		kLeftClicker(4),
		kRightClicker(3),
		kEngineStart(2),
		kLeftSwitch(8),
		kRightSwitch(7),
		kBottomRightDialButton(19),
		kBottomRightDialLeft(20),
		kBottomRightDialRight(21);

		public final int value;

		Button(int value) {
			this.value = value;
		}

		@Override
		public String toString() {
			return this.name().substring(1);
		}
	}

	private int m_bottomRightDial = 0;

	public MissileLauncher(int port) {
		super(port);
		bottomRightDialButton().onTrue(Commands.runOnce(() -> m_bottomRightDial = 0));
		bottomRightDialLeft().whileTrue(Commands.run(() -> m_bottomRightDial--));
		bottomRightDialRight().whileTrue(Commands.run(() -> m_bottomRightDial++));
	}

	public int bottomRightDial() {
		System.out.println(m_bottomRightDial);
		return m_bottomRightDial;
	}

	public Trigger bottomRightDialButton() {
		return button(Button.kBottomRightDialButton.value);
	}

	public Trigger bottomRightDialRight() {
		return button(Button.kBottomRightDialRight.value);
	}

	public Trigger bottomRightDialLeft() {
		return button(Button.kBottomRightDialLeft.value);
	}

	public Trigger topFarLeftButton() {
		return button(Button.kTopFarLeftButton.value);
	}

	public Trigger topMiddleLeftButton() {
		return button(Button.kTopMiddleLeftButton.value);
	}

	public Trigger topMiddleRightButton() {
		return button(Button.kTopMiddleRightButton.value);
	}

	public Trigger topFarRightButton() {
		return button(Button.kTopFarRightButton.value);
	}

	public Trigger bottomFarLeftButton() {
		return button(Button.kBottomFarLeftButton.value);
	}

	public Trigger bottomMiddleLeftButton() {
		return button(Button.kBottomMiddleLeftButton.value);
	}

	public Trigger bottomMiddleRightButton() {
		return button(Button.kBottomMiddleRightButton.value);
	}

	public Trigger bottomFarRightButton() {
		return button(Button.kBottomFarRightButton.value);
	}

	public Trigger leftClicker() {
		return button(Button.kLeftClicker.value);
	}

	public Trigger rightClicker() {
		return button(Button.kRightClicker.value);
	}

	public Trigger engineStart() {
		return button(Button.kEngineStart.value);
	}

	public Trigger leftSwitch() {
		return button(Button.kLeftSwitch.value);
	}

	public Trigger rightSwitch() {
		return button(Button.kRightSwitch.value);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("HID");
		builder.publishConstString("ControllerType", "Missile Launcher");
		GenericHID hid = getHID();
		builder.addBooleanProperty(
				"Top Far Left Button", () -> hid.getRawButton(Button.kTopFarLeftButton.value), null);
		builder.addBooleanProperty(
				"Top Middle Left Button", () -> hid.getRawButton(Button.kTopMiddleLeftButton.value), null);
		builder.addBooleanProperty(
				"Top Middle Right Button", () -> hid.getRawButton(Button.kTopMiddleRightButton.value), null);
		builder.addBooleanProperty(
				"Top Far Right Button", () -> hid.getRawButton(Button.kTopFarRightButton.value), null);
		builder.addBooleanProperty(
				"Bottom Far Left Button", () -> hid.getRawButton(Button.kBottomFarLeftButton.value), null);
		builder.addBooleanProperty(
				"Bottom Middle Left Button", () -> hid.getRawButton(Button.kBottomMiddleLeftButton.value), null);
		builder.addBooleanProperty(
				"Bottom Middle Right Button", () -> hid.getRawButton(Button.kBottomMiddleRightButton.value), null);
		builder.addBooleanProperty(
				"Bottom Far Right Button", () -> hid.getRawButton(Button.kBottomFarRightButton.value), null);
		builder.addBooleanProperty(
				"Bottom Right Dial Button", () -> hid.getRawButton(Button.kBottomRightDialButton.value), null);
		builder.addBooleanProperty(
				"Bottom Right Dial Left", () -> hid.getRawButton(Button.kBottomRightDialLeft.value), null);
		builder.addBooleanProperty(
				"Bottom Right Dial Right", () -> hid.getRawButton(Button.kBottomRightDialRight.value), null);
		builder.addBooleanProperty("Left Clicker", () -> hid.getRawButton(Button.kLeftClicker.value), null);
		builder.addBooleanProperty("Right Clicker", () -> hid.getRawButton(Button.kRightClicker.value), null);
		builder.addBooleanProperty("Engine Start", () -> hid.getRawButton(Button.kEngineStart.value), null);
		builder.addBooleanProperty("Left Switch", () -> hid.getRawButton(Button.kLeftSwitch.value), null);
		builder.addBooleanProperty("Right Switch", () -> hid.getRawButton(Button.kRightSwitch.value), null);
		builder.addIntegerProperty("Bottom Right Dial", this::bottomRightDial, null);
		System.out.println("Nice");
	}
}
