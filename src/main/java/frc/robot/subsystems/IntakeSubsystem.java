package frc.robot.subsystems;

// import motor stuff
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import more stuff
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem
{
  private SparkMax m_motor;
  // initialize stuff
  public IntakeSubsystem()
  {
	// can id is a placeholder; replace later.
    m_motor = new SparkMax(10, MotorType.kBrushless);
    // set current limits
    m_motor.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
	m_motor.setInverted(IntakeConstants.kInvert);

    }
  
  //set speed for motor
  public setSpeed(double speed)
  {
    m_motor.set(speed);
  }
}
