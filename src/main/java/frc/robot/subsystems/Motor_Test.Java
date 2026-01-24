package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Motor_Test extends SubsystemBase {
  private static final int TALON_ID = 1;
  private final WPI_TalonSRX motor = new WPI_TalonSRX(TALON_ID);
  // private final XboxController controller;
  private DoubleSupplier leftTrigger;

  // private static final double DEADBAND = 0.05;

  // Prefer injecting the controller from RobotContainer: new
  // Motor_Test(robotContainer.getController())
  public Motor_Test(DoubleSupplier leftTrigger) {
    this.leftTrigger = leftTrigger;

    // Reset Talon to known state and set neutral mode
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // if (controller == null) {
    //     System.out.println("Motor_Test: controller is null");
    //     motor.stopMotor();
    //     return;
    // }

    double raw = leftTrigger.getAsDouble(); // 0.0 - 1.0
    double speed = Math.abs(raw) < 0.2 ? 0.0 : raw;

    // WPI_TalonSRX.set accepts percent output (-1.0 to 1.0)
    motor.set(speed);

    // Diagnostics to help troubleshoot wiring/CAN/enable issues
    System.out.println(
        String.format(
            "Motor_Test: raw=%.3f speed=%.3f output=%.3f busV=%.2f curr=%.2f",
            raw,
            speed,
            motor.getMotorOutputPercent(),
            motor.getBusVoltage(),
            motor.getSupplyCurrent()));
  }

  public void stopMotor() {
    motor.stopMotor();
  }
}
