// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LEDsC;

public class LEDs extends SubsystemBase {
  

  private final AddressableLED led; // = new AddressableLED(0);
  private int length = 23;
  AddressableLEDBuffer buffer = LEDsC.kRobotLEDBuffer;

  /**
   * LED TeleOp Visuals
   * 
   * 
   * @apiNote 0 = Default
   * @apiNote 1 = Progress Mask
   */
  int set = 0;
  final int setMax = 1;
  final int setMin = 0;

  boolean rev = false;

  private LEDPattern changablePattern;

  private double confTime = .5;

  // private XboxController controller = new XboxController(0);

  /** Creates a new LEDs. */
  public LEDs(AddressableLED l) {
    led = l;
    led.setLength(length);
    // disabled.applyTo(buffer);
    led.start();

    for (Seconds.of(75);;) {
      if (!rev) {
        rev = true;
      } else {
      rev = false;
      }
    }
    
    // progress = LEDPattern.progressMaskLayer(() -> controller.getLeftY()/ -1);
    // progressMask = progress.mask(LEDPattern.solid(Color.kAntiqueWhite));
  }

  /**
   * Sets the LEDs color for the Autonomous Period
   */
  public void setAuto() {
    // led.close();
    LEDsC.kIdlePattern.applyTo(buffer);
    // led.start();

  }

  /**
   * Sets the LEDs color for when the robot is disabled. LEDS are still programmable when the robot is disabled c:
   */
  public void setDisabled() {
    // led.close();
    // LEDsC.kDisabledPattern.applyTo(buffer);

    if (rev == false) {
      LEDsC.kDisabledPattern.applyTo(buffer);
    } else {
      LEDsC.kDisabledPatternRev.applyTo(buffer);
    }
    // led.start();
  } 

  /**
   * Sets the LEDs color for the Tele-Operated period.
   */
  public void setTeleOp() {
    // led.close();
    LEDsC.kIdlePattern.applyTo(buffer);
    // led.start();
  }

  /**
   * Sets the LEDs color for the Progress Mask. Generally used for elevator/lift height visuals.
   */
  // public void setProgressMask() {
  //   // led.close();
  //   if (set==1) {      
  //   progressMask.applyTo(buffer);
  //   }
  //   // led.start();
  // }

//   public void changeSet(boolean backwards) {
//     if (backwards==true) {
//       set=0;
//       // if (set==setMin) {
//       //   set=setMax;
//       // } else {
//       //   set= set - 1;
//       // }
//     } else {
//       set=1;
//       // if (set==setMax) {
//       //   set=setMin;
//       // } else {
//       //   set= set + 1;
//       // }
//     }
//   }

//   public void resetSet() {
// set=0;
//   }

  /**
   * Sets a custom pattern.
   * 
   * @param pat Pattern to set the LEDS to
   */
  public void setPattern(LEDPattern pat, AddressableLEDBufferView buf) {
    changablePattern=pat;
    changablePattern.applyTo(buf);
  }

  public void setCoralConfirm() {
    setPattern(LEDsC.kCoralPattern, LEDsC.kCoralV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kLSHPattern, LEDsC.kCoralV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kCoralPattern, LEDsC.kCoralV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kLSHPattern, LEDsC.kCoralV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kCoralPattern, LEDsC.kCoralV);
  }

  public void setAlgaeConfirm() {
    setPattern(LEDsC.kAlgaePattern, LEDsC.kAlgaeV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kLSHPattern, LEDsC.kAlgaeV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kAlgaePattern, LEDsC.kAlgaeV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kLSHPattern, LEDsC.kAlgaeV);
    new WaitCommand(confTime);
    setPattern(LEDsC.kAlgaePattern, LEDsC.kAlgaeV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(buffer);

    SmartDashboard.putNumber("TELEOP SET", set);
  }
}
