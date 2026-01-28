package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/** Subsystem for testing motor outputs. */
public class Motor_Test extends SubsystemBase {
  private final Spark motor;
  private final DoubleSupplier speedSupplier;

  /**
   * Creates a new Motor_Test subsystem.
   *
   * @param speedSupplier a DoubleSupplier that provides the motor speed (-1 to 1)
   */
  public Motor_Test(DoubleSupplier speedSupplier) {
    this.motor = new Spark(0); // PWM port 0 (adjust as needed)
    this.speedSupplier = speedSupplier;
  }

  @Override
  public void periodic() {
    // Set motor speed from the supplier
    motor.set(speedSupplier.getAsDouble());
  }

  /**
   * Set the motor speed directly.
   *
   * @param speed motor speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  /** Stop the motor. */
  public void stop() {
    motor.set(0);
  }
}
