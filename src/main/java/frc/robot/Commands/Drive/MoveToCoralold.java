// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;
import frc.robot.subsystems.DriveSubsystem;

// import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToCoralold extends Command {
  private DriveSubsystem mDrive;
  private PIDController mTurn;
  @SuppressWarnings("unused")
  private PIDController mfwd;
  @SuppressWarnings("unused")
  private PIDController mside;
    private ProfiledPIDController m_PID;


  /** Creates a new AprilTag. */
  public MoveToCoralold(DriveSubsystem drive) {
    this.mDrive = drive;
    
    PIDController fwd = new PIDController(.05, .0, .1);
    this.mfwd = fwd;

    PIDController turn = new PIDController(.01, 0.0, 0.0);
    this.mTurn = turn;

    m_PID = new ProfiledPIDController(.025,0,0,
        new TrapezoidProfile.Constraints(.5, .1));

    addRequirements(mDrive);
  }

  public double correctedTX(){

    if (Math.abs(LimelightHelpers.getTX("limelight-coral")) < 2){
      return 0;
    } else {
      return LimelightHelpers.getTX("limelight-coral");
    }
  }

  public double correctedTY(){

    if (Math.abs(LimelightHelpers.getTY("limelight-coral")) < 2){
      return 0;
    } else {
      return LimelightHelpers.getTY("limelight-coral");
    }
  }

  public double parallelDiff(){
    return mDrive.getHeading();
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.drive(
      // mfwd.calculate(-correctedTY(), -4),
      -m_PID.calculate(correctedTY(),-5),
      0,
      mTurn.calculate(correctedTX(),0),
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
