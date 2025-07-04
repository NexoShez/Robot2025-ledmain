// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LEDsC;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LEDXtras extends SequentialCommandGroup {
  private final LEDs leds;
  /** Creates a new LEDXtras. */
  public LEDXtras(LEDs x) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();

    leds=x;
  }

  public Command blinkCoral() {
    return Commands.runOnce(() -> leds.setPattern(LEDsC.kCoralFLASHPattern, LEDsC.kCoralV), leds).withTimeout(Seconds.of(50));
  }
}
