package frc.robot.subsystems;

import com.revrobotics.spark.*;
//import com.revrobotics.config.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// initializes index and funnel motors
public class Index extends SubsystemBase {
  private final SparkMax indexMotor =
      new SparkMax(Constants.IndexConstants.INDEX_MOTOR_ID, Constants.IndexConstants.INDEX_TYPE);

  private final SparkMax funnelMotor =
      new SparkMax(Constants.IndexConstants.FUNNEL_MOTOR_ID, Constants.IndexConstants.FUNNEL_TYPE);

  public void setIndexSpeed(double speed) {
    this.indexMotor.set(speed);
  }

  public void setFunnelSpeed(double speed) {
    this.funnelMotor.set(speed);
  }
}

//
