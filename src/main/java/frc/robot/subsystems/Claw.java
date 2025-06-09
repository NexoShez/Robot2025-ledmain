// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private SparkMax m_right;
  private SparkMax m_left;
  private DigitalInput lms;

  /** Creates a new Outake. */
  public Claw() {
    m_right = new SparkMax(Constants.ClawConstants.kRightCANid, Constants.ClawConstants.kRightMotorType);
    m_right.configure(Configs.Claw.rightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_left = new SparkMax(Constants.ClawConstants.kLeftCANid, Constants.ClawConstants.kLeftMotorType);
    m_left.configure(Configs.Claw.leftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    lms = new DigitalInput(0);
  }

  public void setVoltage(double speed) {
    m_right.setVoltage(speed);
    m_left.setVoltage(speed);
  }
  
  public boolean hasAlgae() {
    return !lms.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("CLAW LMS", hasAlgae());
  }
}
