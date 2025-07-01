// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class LiftConstants {
    public static final int kRightCANid = 12;
    public static final int kLeftCANid = 13;

    public static final MotorType kRightMotorType = MotorType.kBrushless;
    public static final MotorType kLeftMotorType = MotorType.kBrushless;

    public static final boolean kRightInverted = false;
    public static final boolean kLeftInverted = true;

    public static final int kRightCurrentLimit = 70;
    public static final int kLeftCurrentLimit = 70;

    public static final double kLiftHighP = 1;
    public static final double kLiftLowP = .5;
    public static final double kLiftI = .5;
    public static final double kLiftD = 0.0;
    public static final double kLiftMaxSpeed = 150;
    public static final double kLiftMaxAcceleration = 60;

    public final class LiftHeight {
      public static final double kPositionResting = -.1;
      public static final double kPositionL2 = 73;
      public static final double kPositionHolding = 50;
      public static final double kPositionL3 = 105;
      public static final double kPositionBarge = 110;
    }

    public static final double kZeroSpeed = 0.01;
    public static final double kZeroTolerance = 10;

  }

  public static final class ArmConstants {
    public static final int kMotorCANid = 14;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final boolean kInverted = true;

    public static final int kCurrentLimit = 50;

    public static final boolean kEncoderInverted = true;

    public static final double kArmP = .5;
    //public static final double kArmLP = 5;
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.0;

    public final class ArmPostion {
      public static final double kPositionResting = 0.225;
      public static final double kPositionGround = 0.445;
      public static final double kPositionL2 = 0.49;
      public static final double kPosistionL3 = 0.49;
      public static final double kPosistionProcesser = 0.38;
      public static final double kPosistionBarge = 0.3;
    }

  }

  public static final class FourBarConstants {
    public static final int kMotorCANid = 9;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final boolean kInverted = true;

    public static final int kCurrentLimit = 50;
  
    public static final double kFourBarP = 3;
    public static final double kFourBarI = 0.0;
    public static final double kFourBarD = 0.0;

    public final class FourBarPostion {
      public static final double kPositionResting = 0.31;
      public static final double kPositionAlgae = 0.5;
      public static final double kPositionCoral = 0.6;
      public static final double kPositionClimb = 0.45;
      public static final double kPositionL1 = .4;
    }

  }

    public static final class ClawConstants {
    public static final int kRightCANid = 15;
    public static final int kLeftCANid = 16;

    public static final MotorType kRightMotorType = MotorType.kBrushless;
    public static final MotorType kLeftMotorType = MotorType.kBrushless;

    public static final boolean kRightInverted = false;
    public static final boolean kLeftInverted = true;

    public static final int kRightCurrentLimit = 50;
    public static final int kLeftCurrentLimit = 50;

    public static final double kOutakeVolts = -12;
    public static final double kIntakeVolts = 7;
  }

  public static final class IntakeConstants {
    public static final int kIntakeCANid = 10;

    public static final MotorType kIntakeMotorType = MotorType.kBrushless;

    public static final boolean kIntakeInverted = true;

    public static final int kIntakeCurrentLimit = 50;

    public static final double kIntakeVolts = 10;
    public static final double kOutakeVolts = 5;
  }

  public static final class ClimberConstants {
    public static final int kMotorCANid = 17;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final boolean kMotorInverted = true;

    public static final int kMotorCurrentLimit = 70;

    public static final double kClimberP = 1.5;
    public static final double kClimberI = 0.0;
    public static final double kClimberD = 0.0;

    public final class ClimberPostion {
      public static final double kPositionResting = 0;
      public static final double kPositionClimbed = 0;
      public static final double kPositionAnchor = 0;
    }

  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  

  public static final class LEDsC {

    public static final int kRobotLEDLength = 23;
    // public static final int kLiftLEDLength = 120;

    public static double coralProg = 0;
    public static final double cPrMax = 1;
    public static final double cPrMin = 0;

    public static double algaeProg=0;
    public static final double aPrMax = 1;
    public static final double aPrMin = 0;

    public static final AddressableLEDBuffer kRobotLEDBuffer = new AddressableLEDBuffer(kRobotLEDLength);
    // public static final AddressableLEDBuffer kLiftLEDBuffer = new AddressableLEDBuffer(kLiftLEDLength);

    public static final Color kSOTAbotYellow = Color.fromHSV(23, 255, 255); 
    public static final Color kError = Color.kRed;
    public static final Color[] kDisabled = {
      Color.kWhite,
      Color.kBlack
    };
    public static final Color kCoral = Color.kPurple;
    public static final Color kAlgae = Color.kLightBlue;
    public static final Color kLimitSwitchHit = Color.kLightGreen;

    public static final Color kIdle = Color.kBlack;
    

    public static final Map<Number, Color> kMaskSteps = Map.of(0, Color.kWhite, 0.5, Color.kBlack);

    public static final LEDPattern kDisabledPattern = LEDPattern.steps(Map.of(0, kSOTAbotYellow, 0.5, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(100));
    public static final LEDPattern kDisabledPatternRev = LEDPattern.steps(Map.of(0, kSOTAbotYellow, 0.5, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(100)).reversed();
    public static final LEDPattern kIdlePattern = LEDPattern.solid(kIdle);
    public static final LEDPattern kAutoIdlePattern = LEDPattern.steps(kMaskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(1.25));
    
    public static final AddressableLEDBufferView kCoralV = new AddressableLEDBufferView(kRobotLEDBuffer, 0, 10);
    public static final AddressableLEDBufferView kAlgaeV = new AddressableLEDBufferView(kRobotLEDBuffer, 11, 22);

    public static final LEDPattern kCoralPattern = LEDPattern.solid(kCoral).breathe(Seconds.of(75));
    public static final LEDPattern kAlgaePattern = LEDPattern.solid(kAlgae).breathe(Seconds.of(75));
    public static final LEDPattern kCoralFLASHPattern = LEDPattern.gradient(GradientType.kContinuous ,kCoral, kLimitSwitchHit).blink(Seconds.of(75));
    public static final LEDPattern kAlgaeFLASHPattern = LEDPattern.gradient(GradientType.kContinuous ,kAlgae, kLimitSwitchHit).blink(Seconds.of(75)).reversed();

    public static final LEDPattern kLSHPattern = LEDPattern.steps(Map.of( //limit swsitch hit
      0, Color.kGreen,
      0.1, Color.kWhite, 
      0.2, Color.kGreen,
      0.3, Color.kWhite,
      0.4, Color.kGreen,
      0.5, Color.kWhite, 
      0.6, Color.kGreen,
      0.7, Color.kWhite,
      0.8, Color.kGreen,
      0.9, Color.kWhite
      )).blink(Seconds.of(.15));
  }
}
