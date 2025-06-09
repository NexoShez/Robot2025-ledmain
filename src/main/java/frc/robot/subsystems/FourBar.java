// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class FourBar extends SubsystemBase {
  private SparkMax m_motor;
  private AbsoluteEncoder m_motorEncoder;
  private SparkClosedLoopController m_motorPID;
  private double position = 0;

  /** Creates a new fourbar. */
  public FourBar() {
    m_motor = new SparkMax(Constants.FourBarConstants.kMotorCANid, Constants.FourBarConstants.kMotorType);
    m_motor.configure(Configs.FourBar.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_motorEncoder = m_motor.getAbsoluteEncoder();
    m_motorPID = m_motor.getClosedLoopController();
  }

  public void setPostion(double position) {
    this.position = position;
  }

  public double getPostion() {
    return m_motorEncoder.getPosition();
  }

  public boolean isResting() {
    return getPostion() < Constants.FourBarConstants.FourBarPostion.kPositionResting;
  }

  @Override
  public void periodic() {
    m_motorPID.setReference(position, ControlType.kPosition);
    // This method will be called once per scheduler run
  }
}
