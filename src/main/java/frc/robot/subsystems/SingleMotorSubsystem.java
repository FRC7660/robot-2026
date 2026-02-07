package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Simple subsystem to control a single motor (PWM) for testing. */
public class SingleMotorSubsystem extends SubsystemBase {
  private final PWMSparkMax motor;

  /**
   * Create a new SingleMotorSubsystem.
   *
   * @param pwmPort PWM port the motor controller is connected to (e.g. 0)
   */
  public SingleMotorSubsystem(int pwmPort) {
    motor = new PWMSparkMax(pwmPort);
    // Ensure motor is stopped on construction to avoid unintended motion on startup
    motor.stopMotor();
  }

  /** Set motor speed (-1.0 to 1.0). */
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  /** Stop the motor. */
  public void stop() {
    motor.stopMotor();
  }
}
