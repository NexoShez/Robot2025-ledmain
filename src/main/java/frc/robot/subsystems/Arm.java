// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private SparkMax m_motor;
  private SparkAbsoluteEncoder m_motorEncoder;
  private SparkClosedLoopController m_motorPID;
  // private PIDController m_PID;
  private double position = .6;
  // private Lift m_lift;

  /** Creates a new Wrist. */
  public Arm(Lift m_lift) {
    // this.m_lift = m_lift;
    m_motor = new SparkMax(Constants.ArmConstants.kMotorCANid, Constants.ArmConstants.kMotorType);
    m_motor.configure(Configs.Arm.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
        m_motorPID = m_motor.getClosedLoopController();

    m_motorEncoder = m_motor.getAbsoluteEncoder();
    // m_PID = new PIDController(Constants.ArmConstants.kArmP,
    //  Constants.ArmConstants.kArmI,
    //   Constants.ArmConstants.kArmD);
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void setPosition(double position) {
      this.position = position;

      // if (m_motorEncoder.getPosition() < .3){
      //   m_PID.setP(Constants.ArmConstants.kArmLP);
      // } else {
      //   m_PID.setP(Constants.ArmConstants.kArmP);
      // }
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("armABS", m_motorEncoder.getPosition());
    m_motorPID.setReference((position - .201) * 316.25, ControlType.kPosition);
    // if (m_motorEncoder.getPosition() < .3){
    //   m_PID.setP(Constants.ArmConstants.kArmLP);
    // }
    // setSpeed(MathUtil.clamp(m_PID.calculate(m_motorEncoder.getPosition(), position), -1, 1));
    // This method will be called once per scheduler run
  }
}
