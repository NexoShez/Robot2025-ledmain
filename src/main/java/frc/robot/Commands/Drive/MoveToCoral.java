// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToCoral extends SequentialCommandGroup {
  /** Creates a new MoveToCoral. */
  public MoveToCoral(Vision m_vision) {
    // addCommands(new RunCommand(() -> m_vision.autoPickupCoral(), m_vision).until(Intake::has));
    // // Add your commands in the addCommands() call, e.g.
    // // addCommands(new FooCommand(), new BarCommand());
    // addCommands();
  }
}
