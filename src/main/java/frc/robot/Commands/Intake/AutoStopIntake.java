// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.FourBar;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStopIntake extends SequentialCommandGroup {
  /** Creates a new AutoStop. */
  public AutoStopIntake(FourBar m_fourbar, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.runOnce(
        () -> m_fourbar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionCoral),
        m_fourbar),
        new RunCommand(
            () -> m_intake.setVoltage(Constants.IntakeConstants.kIntakeVolts),
            m_intake)
            .until(m_intake::hasCoral),
        new RunCommand(
            () -> m_intake.setVoltage(Constants.IntakeConstants.kIntakeVolts),
            m_intake)
            .withTimeout(.3),
        Commands.runOnce(
            () -> {
              m_fourbar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting);
              m_intake.setVoltage(2);
            },
            m_intake, m_fourbar));
    addRequirements(m_fourbar, m_intake);
  }
}
