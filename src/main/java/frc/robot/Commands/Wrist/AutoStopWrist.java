// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Wrist;

// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.FourBar;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
// import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStopWrist extends SequentialCommandGroup {
  /** Creates a new AutoStopWrist. */
  public AutoStopWrist(Lift m_lift, FourBar m_fourbar, Intake m_intake) {
    //get everything in the right position especly so the lift doesnt hit the outake
    addCommands(new RunCommand(
        () -> {m_fourbar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting);
          m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting);},
        m_fourbar, m_lift).until(m_lift::isInPostion),
    //set everything too stop
      new RunCommand(
        () -> m_intake.setVoltage(0),
          m_intake));
        addRequirements(m_fourbar, m_lift, m_intake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
