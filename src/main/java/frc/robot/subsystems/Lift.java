// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  private SparkFlex m_right;
  // private RelativeEncoder m_rightEncoder;
  private ProfiledPIDController m_PID;
  private double position;

  private SparkFlex m_left;
  // private RelativeEncoder m_leftEncoder;

  /** Creates a new Lift. */
  public Lift() {
    m_right = new SparkFlex(Constants.LiftConstants.kRightCANid, Constants.LiftConstants.kRightMotorType);
    m_right.configure(Configs.Lift.rightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_left = new SparkFlex(Constants.LiftConstants.kLeftCANid, Constants.LiftConstants.kLeftMotorType);
    m_left.configure(Configs.Lift.leftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_PID = new ProfiledPIDController(Constants.LiftConstants.kLiftLowP,
        Constants.LiftConstants.kLiftI,
        Constants.LiftConstants.kLiftD,
        new TrapezoidProfile.Constraints(Constants.LiftConstants.kLiftMaxSpeed, Constants.LiftConstants.kLiftMaxAcceleration));
  }

  public void setVoltage(double voltage) {
    m_left.setVoltage(voltage);
    m_right.setVoltage(voltage);
  }

  public boolean isInPostion() {
    return (m_right.getEncoder().getPosition() < 2);
  }

  public void setPostion(double position) {
    if (position < m_right.getEncoder().getPosition()) {
      m_PID.setP(Constants.LiftConstants.kLiftLowP);
      m_PID.setI(0);
    } else {
      m_PID.setP(Constants.LiftConstants.kLiftHighP);
    }
    this.position = position;
  }

  public void setZero() {
    // while(m_right.getBusVoltage() < Constants.LiftConstants.kZeroTolerance) {
    // position = m_rightEncoder.getPosition() -
    // Constants.WristConstants.kZeroSpeed;
    // }
    // m_rightEncoder.setPosition(0);
    // m_leftEncoder.setPosition(0);
  }

  public double getOutput() {
    // if ((position < m_right.getEncoder().getPosition() + 1) && (position > m_right.getEncoder().getPosition() - 1)) {
    //   return .1;
    // } else {
    //   return m_PID.calculate(m_right.getEncoder().getPosition(), position);
    // }
    return 0;
  }

  public void resetLift() {
    m_right.getEncoder().setPosition(0);
  }

  public double getPosition(){
    return (m_right.getEncoder().getPosition() + m_left.getEncoder().getPosition())/2;
  }

  @Override
  public void periodic() {
    setVoltage(MathUtil.clamp(m_PID.calculate(m_right.getEncoder().getPosition(), position), -12, 12));

    SmartDashboard.putNumber("lift PID", m_PID.calculate(getPosition(), position));
    SmartDashboard.putNumber("lift setpoint", position);
    SmartDashboard.putNumber("lift postion", getPosition());
    // SmartDashboard.putNumber("lift MP", m_right.GET());
    //SmartDashboard.putNumber("lift P", m_PID.getP());
    //SmartDashboard.putNumber("period", m_PID.getPeriod());
    //SmartDashboard.putNumber("lift error acc", m_PID.getAccumulatedError());
    // This method will be called once per scheduler run
  }
}
