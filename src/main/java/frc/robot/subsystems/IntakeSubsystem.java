package frc.robot.subsystems;

// import motor stuff
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import more stuff
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;

public class IntakeSubsystem
{
  private SparkMax m_motor;
  // initialize stuff
  public IntakeSubsystem()
  {
    
  }
  //motro
  public motor(int id)
  {
    m_motor = new SparkMax(id, MotorType.kBrushless);
  }
  //set speed for motor
  public setSpeed(double speed)
  {
    m_motor.set(speed);
  }
}
