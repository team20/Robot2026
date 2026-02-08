package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArm;

public class ExtendArmCommand extends Command {
	/**
	 * Deploy the arm of the intake system.
	 */
	private IntakeArm m_subsystem;

	public ExtendArmCommand(IntakeArm subsystem) {
		addRequirements(m_subsystem);
		m_subsystem = subsystem;
	}

	public void initialize() {
		m_subsystem.setArmPower(0.5);
	}

	public void execute() {

	}

	// Update this to use limit switch instead of getArmAngle method.
	public boolean isFinished() {
		// Check if arm is at 77 degrees plus or minus 0.5
		if (m_subsystem.getArmAngle() >= 77 - 0.5) {
			if (m_subsystem.getArmAngle() <= 77 + 0.5) {
				return true;
			}
		}

		return false;
	}

	public void end() {
		m_subsystem.setArmPower(0);
	}
}
