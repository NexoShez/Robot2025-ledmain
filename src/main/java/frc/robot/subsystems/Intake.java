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

public class Intake extends SubsystemBase {
  private SparkMax kIntake;
  // private SparkFlex kDelivery;

  private DigitalInput proxSensor;

  /** Creates a new intake. */
  public Intake() {
    kIntake = new SparkMax(Constants.IntakeConstants.kIntakeCANid, Constants.IntakeConstants.kIntakeMotorType);
    kIntake.configure(Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    proxSensor = new DigitalInput(8);
  }

  public boolean hasCoral() {
    return !proxSensor.get();
  }

  public void setVoltage(double intakeSpeed) {
    kIntake.setVoltage(intakeSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake has Coral", hasCoral());
  }
}
