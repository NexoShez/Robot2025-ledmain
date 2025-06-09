// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;
import frc.robot.subsystems.DriveSubsystem;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToAprilTag extends Command {
  private DriveSubsystem mDrive;
  private PIDController mTurn;
  private PIDController mfwd;
  private PIDController mside;

  private static final Map<Integer, Double> tagIDToAngle = Map.ofEntries(
    Map.entry(21, 180.0),
    Map.entry(7, 180.0),
    Map.entry(22, 120.0),
    Map.entry(6, 120.0),
    Map.entry(17, 60.0),
    Map.entry(11, 60.0),
    Map.entry(18, 0.0),
    Map.entry(10, 0.0),
    Map.entry(19, -60.0),
    Map.entry(9, -60.0),
    Map.entry(20, -120.0),
    Map.entry(8, -120.0)
);

  /** Creates a new AprilTag. */
  public MoveToAprilTag(DriveSubsystem drive) {
    this.mDrive = drive;
    PIDController fwd = new PIDController(.05, .0, .1);
    this.mfwd = fwd;

    PIDController side = new PIDController(.05, 0, .1);
    this.mside = side;

    PIDController turn = new PIDController(.013, 0.01, 0.01);
    this.mTurn = turn;

    addRequirements(mDrive);
  }

  public double correctedTX(){

    if (Math.abs(LimelightHelpers.getTX("")) < 2){
      return 0;
    } else {
      return LimelightHelpers.getTX("");
    }
  }

  public double correctedTY(){

    if (Math.abs(LimelightHelpers.getTY("")) < 2){
      return 0;
    } else {
      return LimelightHelpers.getTY("");
    }
  }

  public double parallelDiff(){
    return mDrive.getHeading();
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @SuppressWarnings("unlikely-arg-type")
  @Override
  public void execute() {
    mDrive.drive(
      -mfwd.calculate(correctedTY(), 0),
      mside.calculate(parallelDiff(),0),
      mTurn.calculate(mDrive.getHeading(),
       tagIDToAngle.get(LimelightHelpers.getFiducialID(""))),
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
