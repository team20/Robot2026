package frc.robot.commands;

public class HoodCommands {
	/*
	 * public static class RunToAngleHardware extends Command {
	 * private final double m_angle;
	 * /** Creates a new RunHoodAtSpeed.
	 */
	/*
	 * public RunToAngleHardware(double angle) {
	 * setName("Run Hood To Angle Using Motor Controller");
	 * addRequirements(Hood.getHood());
	 * m_angle = angle;
	 * }
	 * // Called every time the scheduler runs while the command is scheduled.
	 * @Override
	 * public void initialize() {
	 * Hood.getHood().setAngle(m_angle);
	 * }
	 * // Called once the command ends or is interrupted.
	 * @Override
	 * public void end(boolean interrupted) {
	 * Hood.getHood().stop();
	 * }
	 * // Returns true when the command should end.
	 * @Override
	 * public boolean isFinished() {
	 * return Math.abs(Hood.getHood().getPosition() - m_angle) <
	 * HoodConstants.kTolerance;
	 * }
	 * }
	 * public static class RunToAngleSoftware extends Command {
	 * private final double m_angle;
	 * public RunToAngleSoftware(double angle) {
	 * setName("Run Hood To Angle Using PID Controller");
	 * addRequirements(Hood.getHood());
	 * m_angle = angle;
	 * }
	 * // Called every time the scheduler runs while the command is scheduled.
	 * @Override
	 * public void execute() {
	 * double error = Hood.getHood().getPosition() - m_angle;
	 * double power = ClampedP.clampedP(
	 * error, HoodConstants.kMinPower, HoodConstants.kMaxPower,
	 * HoodConstants.kMaxErr,
	 * HoodConstants.kTolerance);
	 * Hood.getHood().runAtDutyCycle(power);
	 * }
	 * // Called once the command ends or is interrupted.
	 * @Override
	 * public void end(boolean interrupted) {
	 * Hood.getHood().stop();
	 * }
	 * // Returns true when the command should end.
	 * @Override
	 * public boolean isFinished() {
	 * // return false;
	 * return Math.abs(Hood.getHood().getPosition() - m_angle) <
	 * HoodConstants.kTolerance;
	 * }
	 * }
	 * public static class RunAtPower extends Command {
	 * private Timer m_timer;
	 * private double m_time;
	 * private double m_speed;
	 * public RunAtPower(double speed, double time) {
	 * setName("Run Hood At Power");
	 * addRequirements(Hood.getHood());
	 * m_timer = new Timer();
	 * m_time = time;
	 * m_speed = speed;
	 * }
	 * // Called when the command is initially scheduled.
	 * @Override
	 * public void initialize() {
	 * m_timer.reset();
	 * m_timer.start();
	 * Hood.getHood().runAtDutyCycle(m_speed);
	 * }
	 * // Called once the command ends or is interrupted.
	 * @Override
	 * public void end(boolean interrupted) {
	 * m_timer.stop();
	 * Hood.getHood().stop();
	 * }
	 * // Returns true when the command should end.
	 * @Override
	 * public boolean isFinished() {
	 * return m_time > 0 && m_timer.hasElapsed(m_time);
	 * }
	 * }
	 * public static class RunAtPowerSignal extends Command {
	 * private DoubleSupplier m_speed;
	 * public RunAtPowerSignal(DoubleSupplier speed) {
	 * setName("Run Hood At Power From Trigger");
	 * addRequirements(Hood.getHood());
	 * m_speed = speed;
	 * }
	 * // Called every time the scheduler runs while the command is scheduled.
	 * @Override
	 * public void execute() {
	 * Hood.getHood().runAtDutyCycle(MathUtil.applyDeadband(m_speed.getAsDouble(),
	 * HoodConstants.kDeadzone));
	 * }
	 * // Called once the command ends or is interrupted.
	 * @Override
	 * public void end(boolean interrupted) {
	 * Hood.getHood().stop();
	 * }
	 * // Returns true when the command should end.
	 * @Override
	 * public boolean isFinished() {
	 * return false;
	 * }
	 * }
	 */
}