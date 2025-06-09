// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private ProfiledPIDController m_PID = new ProfiledPIDController(.025,0,0,new TrapezoidProfile.Constraints(.5, .1));
  private PIDController turn = new PIDController(.01, 0.0, 0.0);
    private boolean hijack = false;
    /** Creates a new Vision. */
    public Vision(){}
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
  
      public void autoPickupCoral(){
        this.hijack = true;
    }

    public void stopAutoPickupCoral(){
      this.hijack = false;
    }

    public boolean getHijack(){
      return hijack;
    }

    public ChassisSpeeds getAutoPickupCoral(){
        return new ChassisSpeeds(
          -m_PID.calculate(correctedTY(),-5), 
          0,
          turn.calculate(correctedTX(),0));
    }
}
