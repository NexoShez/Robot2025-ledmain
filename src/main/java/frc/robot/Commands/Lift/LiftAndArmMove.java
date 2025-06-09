// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.subsystems.FourBar;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
// import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LiftAndArmMove extends SequentialCommandGroup {
  /** Creates a new LiftAndWristMove. */
  public LiftAndArmMove(Lift m_lift, Arm m_arm, double liftHeight, double armSetpoint) {
addCommands(new RunCommand(
        () -> {m_arm.setPosition(armSetpoint);
          m_lift.setPostion(liftHeight);},
        m_arm, m_lift));
    //move the coral until it is in the outake
      addRequirements(m_lift, m_arm);
      //move the coral a little bit more
    addCommands();
  }
}
