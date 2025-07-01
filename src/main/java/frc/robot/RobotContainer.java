// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FourBar;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.Intake.AutoStopIntake;
import frc.robot.Commands.Lift.LiftAndArmMove;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final Intake m_intake = new Intake();
        private final FourBar m_fourBar = new FourBar();
        private final Lift m_lift = new Lift();
        private final Arm m_arm = new Arm(m_lift);
        private final Claw m_claw = new Claw();
        private final LEDs leds = new LEDs(new AddressableLED(0));
        // private final Outake m_outake = new Outake();
        private final Climber m_climber = new Climber();
        private final Vision m_vision = new Vision();
        private final SendableChooser<Command> autoChooser;
        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController m_manipulatorController = new CommandXboxController(
                        OIConstants.kManipulatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                registerNamedCommands();
                // Configure the button bindings
                configureButtonBindings();

                m_fourBar.setDefaultCommand(new RunCommand(
                                () -> m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting),
                                m_fourBar));

                m_lift.setDefaultCommand(new RunCommand(
                () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting),
                m_lift));

                m_arm.setDefaultCommand(new RunCommand(
                        () -> m_arm.setPosition(Constants.ArmConstants.ArmPostion.kPositionResting),
                        m_arm));

                // m_wrist.setDefaultCommand(new RunCommand(
                // () ->
                // m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting),
                // m_wrist));

                // m_climber.setDefaultCommand(new RunCommand(
                // () -> m_climber.setVoltage((-m_manipulatorController.getLeftY()) * 12),
                // m_climber));

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));

                // ...

                // Build an auto chooser. This will use Commands.none() as the default option.
                autoChooser = AutoBuilder.buildAutoChooser();

                // Another option that allows you to specify the default auto by its name
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void registerNamedCommands() {


                NamedCommands.registerCommand("ClawOutake", Commands.run(
                        () -> m_claw.setVoltage(
                        -12),
                        m_claw, m_claw));

                NamedCommands.registerCommand("Barge", new LiftAndArmMove(m_lift, m_arm,
                Constants.LiftConstants.LiftHeight.kPositionBarge,
                Constants.ArmConstants.ArmPostion.kPosistionBarge));

                NamedCommands.registerCommand("L3Pickup", new LiftAndArmMove(m_lift, m_arm,
                Constants.LiftConstants.LiftHeight.kPositionL3,
                Constants.ArmConstants.ArmPostion.kPositionResting).withTimeout(.5)
                .andThen(new LiftAndArmMove(m_lift, m_arm,
                Constants.LiftConstants.LiftHeight.kPositionL3,
                Constants.ArmConstants.ArmPostion.kPosistionL3)).alongWith(
                new RunCommand(() -> m_claw.setVoltage(5),m_claw)));

                NamedCommands.registerCommand("L2Pickup", new LiftAndArmMove(m_lift, m_arm,
                Constants.LiftConstants.LiftHeight.kPositionL2,
                Constants.ArmConstants.ArmPostion.kPositionResting).withTimeout(.25)
                .andThen(new LiftAndArmMove(m_lift, m_arm,
                Constants.LiftConstants.LiftHeight.kPositionL2,
                Constants.ArmConstants.ArmPostion.kPositionL2)).alongWith(
                new RunCommand(() -> m_claw.setVoltage(5),m_claw)));

                NamedCommands.registerCommand("LiftRest", new LiftAndArmMove(m_lift, m_arm,
                Constants.LiftConstants.LiftHeight.kPositionResting,
                Constants.ArmConstants.ArmPostion.kPositionResting).alongWith(Commands.run(
                        () -> m_claw.setVoltage(
                        0),
                        m_claw, m_claw)));

                NamedCommands.registerCommand("LiftProcesser", new LiftAndArmMove(m_lift, m_arm,
                Constants.LiftConstants.LiftHeight.kPositionResting,
                Constants.ArmConstants.ArmPostion.kPosistionProcesser));


                NamedCommands.registerCommand("Outake", Commands.run(
                                () -> m_intake.setVoltage(
                                                -4.5),
                                m_intake));

                NamedCommands.registerCommand("FourBarL1", Commands.run(
                                () -> m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionL1),
                                m_fourBar));

                NamedCommands.registerCommand("FourBarCoral", Commands.run(
                                () -> m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionCoral),
                                m_fourBar));

                NamedCommands.registerCommand("FourBarRest", Commands.run(
                                () -> m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting),
                                m_fourBar));

                NamedCommands.registerCommand("Intake", Commands.run(
                                () -> m_intake.setVoltage(
                                                Constants.IntakeConstants.kIntakeVolts),
                                m_intake));

                                NamedCommands.registerCommand("StopClaw", Commands.run(
                                        () -> m_claw.setVoltage(
                                                        0),
                                        m_claw));

                NamedCommands.registerCommand("StopIntake", Commands.run(
                                () -> m_intake.setVoltage(0),
                                m_intake));

                NamedCommands.registerCommand("AutoIntake", new AutoStopIntake(m_fourBar, m_intake));

                NamedCommands.registerCommand("MoveToCoral", Commands.run(
                                () -> m_vision.autoPickupCoral(), m_vision).until(m_intake::hasCoral)
                                .andThen(Commands.run(() -> m_vision.stopAutoPickupCoral(), m_vision)));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() { //hi jack ur kind a bum :thumbs_up:
                m_driverController.rightBumper()
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                m_driverController.start()
                                .onTrue(Commands.runOnce(
                                                () -> m_robotDrive.zeroHeading(),
                                                m_robotDrive));
                //climber down slow
                m_driverController.a().onTrue(new RunCommand(
                                () -> m_climber.setVoltage(3),
                                m_climber)).onFalse(new RunCommand(
                                                () -> m_climber.setVoltage(0),
                                                m_climber));
                //climber up fast
                m_driverController.y().onTrue(new RunCommand(
                                () -> m_climber.setVoltage(-12),
                                m_climber)).onFalse(new RunCommand(
                                                () -> m_climber.setVoltage(0),
                                                m_climber));
                //climber down fast
                m_driverController.b().onTrue(new RunCommand(
                                () -> m_climber.setVoltage(12),
                                m_climber)).onFalse(new RunCommand(
                                                () -> m_climber.setVoltage(0),
                                                m_climber));
                //climb 4bar out
                m_driverController.leftTrigger().onTrue(new RunCommand(
                                () -> m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionClimb),
                                m_fourBar)).onFalse(
                                                new RunCommand(
                                                                () -> m_fourBar.setPostion(
                                                                                Constants.FourBarConstants.FourBarPostion.kPositionResting),
                                                                m_fourBar));


                //arm score algea
                // m_manipulatorController.povUp().onTrue(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionBarge,
                // Constants.ArmConstants.ArmPostion.kPosistionBarge))
                // .onFalse(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionResting,
                // Constants.ArmConstants.ArmPostion.kPositionResting));

                // //arm score algea
                // m_manipulatorController.leftStick().onTrue(new LiftAndArmMove(m_lift, m_arm,
                //                 Constants.LiftConstants.LiftHeight.kPositionHolding,
                //                 Constants.ArmConstants.ArmPostion.kPosistionBarge))
                //                 .onFalse(new LiftAndArmMove(m_lift, m_arm,
                //                 Constants.LiftConstants.LiftHeight.kPositionHolding,
                //                 Constants.ArmConstants.ArmPostion.kPosistionBarge));

                // // arm pickup algea l3
                // m_manipulatorController.povRight().onTrue(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionL3,
                // Constants.ArmConstants.ArmPostion.kPositionResting).withTimeout(.5)
                // .andThen(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionL3,
                // Constants.ArmConstants.ArmPostion.kPosistionL3)).alongWith(
                // new RunCommand(() -> m_claw.setVoltage(5),m_claw)
                // ))
                // .onFalse(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionResting,
                // Constants.ArmConstants.ArmPostion.kPositionResting).alongWith(
                // new RunCommand(() -> m_claw.setVoltage(0),m_claw)
                // ));

                // //arm pickup algea l2
                // m_manipulatorController.povLeft().onTrue(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionL2,
                // Constants.ArmConstants.ArmPostion.kPositionResting).withTimeout(.25)
                // .andThen(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionL2,
                // Constants.ArmConstants.ArmPostion.kPositionL2)).alongWith(
                // new RunCommand(() -> m_claw.setVoltage(5),m_claw)
                // ))
                // .onFalse(new LiftAndArmMove(m_lift, m_arm,
                // Constants.LiftConstants.LiftHeight.kPositionResting,
                // Constants.ArmConstants.ArmPostion.kPositionResting).alongWith(
                // new RunCommand(() -> m_claw.setVoltage(0),m_claw)
                // ));


                //intake outake slow
                m_manipulatorController.leftTrigger().onTrue(new RunCommand(
                                () -> m_intake.setVoltage(
                                                -Constants.IntakeConstants.kOutakeVolts),
                                m_intake))
                                .onFalse(new RunCommand(
                                                () -> m_intake.setVoltage(0),
                                                m_intake));
                //intake outake fast
                m_manipulatorController.leftBumper().onTrue(new RunCommand(
                                () -> m_intake.setVoltage(
                                        -10),
                                m_intake))
                                .onFalse(new RunCommand(
                                        () -> m_intake.setVoltage(0),
                                m_intake));

                //intake intake speed
                m_manipulatorController.rightBumper().onTrue(new RunCommand(
                                () -> m_intake.setVoltage(
                                                Constants.IntakeConstants.kIntakeVolts),
                                m_intake))  
                                .onFalse(new RunCommand(
                                                () -> m_intake.setVoltage(0),
                                                m_intake));
                // claw outake
                m_manipulatorController.y().onTrue(new RunCommand(
                        () -> m_claw.setVoltage(
                        -12),
                        m_claw))  
                        .onFalse(new RunCommand(
                        () -> m_claw.setVoltage(0),
                        m_claw));

                // claw outake
                m_manipulatorController.x().onTrue(new RunCommand(
                                        () -> m_claw.setVoltage(
                                        5),
                                        m_claw))  
                                        .onFalse(new RunCommand(
                                        () -> m_claw.setVoltage(0),
                                        m_claw));

                        //arm go to processer
                        // m_manipulatorController.povDown().onTrue(
                        //         (new LiftAndArmMove(m_lift, m_arm,
                        //         Constants.LiftConstants.LiftHeight.kPositionResting,
                        //         Constants.ArmConstants.ArmPostion.kPosistionProcesser)))  
                        //         .onFalse(
                        //         (new LiftAndArmMove(m_lift, m_arm,
                        //         Constants.LiftConstants.LiftHeight.kPositionResting,
                        //         Constants.ArmConstants.ArmPostion.kPositionResting)));

                //claw intake from ground
                             m_manipulatorController.b().onTrue(new RunCommand(
                                                        () -> m_claw.setVoltage(
                                                                        5),
                                                        m_claw).alongWith(new LiftAndArmMove(m_lift, m_arm,
                                                        Constants.LiftConstants.LiftHeight.kPositionResting,
                                                        Constants.ArmConstants.ArmPostion.kPositionGround)))  
                                                        .onFalse(new RunCommand(
                                                                        () -> m_claw.setVoltage(0),
                                                                        m_claw).alongWith(new LiftAndArmMove(m_lift, m_arm,
                                                                        Constants.LiftConstants.LiftHeight.kPositionResting,
                                                                        Constants.ArmConstants.ArmPostion.kPositionResting)));

                // claw outtake from ground (USE FOR WHEN ELEVATOR BROKEN)
                m_manipulatorController.a().onTrue(new LiftAndArmMove(m_lift, m_arm, 
                LiftConstants.LiftHeight.kPositionResting, 
                Constants.ArmConstants.ArmPostion.kPosistionBarge))
                        .onFalse(new LiftAndArmMove(m_lift, m_arm,
                        Constants.LiftConstants.LiftHeight.kPositionResting,
                        Constants.ArmConstants.ArmPostion.kPositionResting));
                // fourbar intake then comeup and intake a little more
                // m_manipulatorController.a().onTrue(new RunCommand(
                //                 () -> {m_fourBar.setPostion(
                //                                 Constants.FourBarConstants.FourBarPostion.kPositionCoral);
                //                         m_intake.setVoltage(Constants.IntakeConstants.kIntakeVolts);},
                //                 m_fourBar, m_intake)).onFalse(new RunCommand(
                //                         () -> {m_fourBar.setPostion(
                //                                 Constants.FourBarConstants.FourBarPostion.kPositionResting);
                //                         m_intake.setVoltage(Constants.IntakeConstants.kIntakeVolts);},
                //                         m_fourBar, m_intake).withTimeout(.4).andThen(new RunCommand(
                //                                 () -> {m_fourBar.setPostion(
                //                                         Constants.FourBarConstants.FourBarPostion.kPositionResting);
                //                                 m_intake.setVoltage(0);},
                //                                 m_fourBar, m_intake)));
                // // fourbar go to l1
                // m_manipulatorController.rightTrigger().onTrue(new RunCommand(
                //         () -> m_fourBar.setPostion(
                //         Constants.FourBarConstants.FourBarPostion.kPositionL1),
                //         m_fourBar)).onFalse(new RunCommand(
                //         () -> m_fourBar.setPostion(
                //         Constants.FourBarConstants.FourBarPostion.kPositionResting),
                //         m_fourBar));

                // m_manipulatorController.rightStick().onTrue(new LiftAndWristMove(m_lift,
                // m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionCoralStation,
                // Constants.WristConstants.WristPostion.kPosistionCoralStation))
                // .onFalse(new LiftAndWristMove(m_lift, m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionResting,
                // Constants.WristConstants.WristPostion.kPositionResting));

                // m_manipulatorController.rightBumper().onTrue(new RunCommand(
                // () -> {
                // m_claw.setVoltage(
                // Constants.ClawConstants.kIntakeVolts);
                // },
                // m_claw))
                // .onFalse(new RunCommand(
                // () -> {
                // m_claw.setVoltage(0);
                // },
                // m_claw));

                // m_manipulatorController.leftBumper().onTrue(new RunCommand(
                // () -> {
                // m_claw.setVoltage(
                // Constants.ClawConstants.kOutakeVolts);
                // },
                // m_claw))
                // .onFalse(new RunCommand(
                // () -> {
                // m_claw.setVoltage(0);
                // },
                // m_claw));

                // move the delivery to the outake for emergencys
                // m_manipulatorController.rightBumper().onTrue(new RunCommand(
                // () -> {
                // m_intake.setVoltage(
                // Constants.IntakeConstants.kIntakeVolts,
                // Constants.IntakeConstants.kDeliveryVolts);
                // },
                // m_intake))
                // .onFalse(new RunCommand(
                // () -> {
                // m_intake.setVoltage(0, 0);
                // },
                // m_intake));





                // m_manipulatorController.y().onTrue(new RunCommand(
                //                 () -> m_fourBar.setPostion(
                //                                 Constants.FourBarConstants.FourBarPostion.kPositionAlgae),
                //                 m_fourBar)).onFalse(new RunCommand(
                //                                 () -> m_fourBar.setPostion(
                //                                                 Constants.FourBarConstants.FourBarPostion.kPositionResting),
                //                                 m_fourBar));

                // m_manipulatorController.a().onTrue(new AutoStopIntake(
                // m_fourBar, m_intake))
                // .onFalse(Commands.runOnce(
                // () -> m_intake.setVoltage(0),
                // m_intake));

                // m_manipulatorController.leftStick().onTrue(new MoveToCoral(m_robotDrive))
                // .onFalse(new MoveToCoral(m_robotDrive).withTimeout(.001));

                // m_manipulatorController.leftStick().onTrue(new RunCommand(
                // () ->
                // m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionL1),
                // m_fourBar)).onFalse(
                // new RunCommand(
                // () -> m_fourBar.setPostion(
                // Constants.FourBarConstants.FourBarPostion.kPositionResting),
                // m_fourBar));

                // m_manipulatorController.leftStick().onTrue(new RunCommand(
                // () -> m_climber.setVoltage(-6),
                // m_climber)).onFalse(
                // new RunCommand(
                // () -> m_climber.setVoltage(0),
                // m_climber));

                // m_manipulatorController.rightStick().onTrue(new RunCommand(
                // () -> m_climber.setVoltage(6),
                // m_climber)).onFalse(
                // new RunCommand(
                // () -> m_climber.setVoltage(0),
                // m_climber));

                // m_manipulatorController.back().onTrue(new RunCommand(
                // () ->
                // m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionClimb),
                // m_fourBar)).onFalse(
                // new RunCommand(
                // () ->
                // m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting),
                // m_fourBar));

                // m_manipulatorController.x().onTrue(
                // () -> m_climber.setVoltage(
                // Constants.ClimberConstants.kClimberVolts
                // ))
                // .onFalse(
                // () -> m_climber.setVoltage(
                // 0));

                // new JoystickButton(m_manipulatorController, Button.kStart.value)
                // .whileTrue(new RunCommand(
                // () -> {m_wrist.setZero();
                // m_lift.setZero();},
                // m_wrist, m_lift));

                // new JoystickButton(m_manipulatorController, Button.kA.value)
                // .onTrue(new AutoStopIntake(m_fourBar, m_intake)
                // .andThen(new AutoStopWrist(m_lift, m_wrist, m_outake, m_fourBar, m_intake)))
                // .onFalse(Commands.runOnce(
                // () -> m_intake.setVoltage(0, 0),
                // m_intake));

                // new JoystickButton(m_manipulatorController, Button.kA.value)
                // .onTrue(new AutoStopIntake(m_fourBar, m_intake))
                // .onFalse(Commands.runOnce(
                // () -> {m_intake.setVoltage(0, 0);
                // m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting);},
                // m_intake, m_fourBar));

                // new JoystickButton(m_manipulatorController, Button.kLeftStick.value)
                // .onTrue(new RunCommand(
                // () ->
                // {m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionAlgae);
                // m_intake.setVoltage(Constants.IntakeConstants.IntakeSpeeds.kSpeedAlgeaRight,
                // Constants.IntakeConstants.IntakeSpeeds.kSpeedAlgeaLeft);},
                // m_fourBar, m_intake))
                // .onFalse(Commands.runOnce(
                // () -> m_intake.setVoltage(0, 0),
                // m_intake));

                // new JoystickButton(m_manipulatorController, Button.kLeftBumper.value)
                // .onTrue(new RunCommand(
                // () ->
                // {m_intake.setVoltage(Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery,
                // Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery);
                // m_outake.setVoltage(Constants.OutakeConstants.OutakeSpeeds.kSpeedDelvery);}
                // ,m_intake, m_outake))
                // .onFalse(new RunCommand(
                // () -> {m_intake.setVoltage(0, 0);
                // m_outake.setVoltage(0);}
                // ,m_intake, m_outake));

                // new JoystickButton(m_manipulatorController, Button.kRightBumper.value)
                // .onTrue(new RunCommand(
                // () ->
                // {m_intake.setVoltage(-Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery,
                // -Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery);
                // m_outake.setVoltage(-Constants.OutakeConstants.OutakeSpeeds.kSpeedDelvery);}
                // ,m_intake, m_outake))
                // .onFalse(new RunCommand(
                // () -> {m_intake.setVoltage(0, 0);
                // m_outake.setVoltage(0);}
                // ,m_intake, m_outake));

                // new JoystickButton(m_manipulatorController, Button.kY.value)
                // .onTrue(new RunCommand(
                // () ->
                // {m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionL2);
                // m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL2);},
                // m_wrist,m_lift))
                // .onFalse( new RunCommand(
                // () ->
                // {m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting);
                // m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting);},
                // m_wrist,m_lift));

                // new JoystickButton(m_manipulatorController, Button.kB.value)
                // .onTrue(Commands.runOnce(
                // () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL4),
                // m_lift))
                // .onFalse( new RunCommand(
                // () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting),
                // m_lift));

                // new JoystickButton(m_manipulatorController, Button.kX.value)
                // .onTrue(new RunCommand(
                // () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL3),
                // m_lift))
                // .onFalse( new RunCommand(
                // () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting),
                // m_lift));

                // new JoystickButton(m_manipulatorController, Button.kB.value)
                // .onTrue(new LiftAndWristMove(m_lift, m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionCoralStation,
                // Constants.WristConstants.WristPostion.kPositionResting))
                // .onFalse(new LiftAndWristMove(m_lift, m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionResting,
                // Constants.WristConstants.WristPostion.kPositionResting));

                // new JoystickButton(m_manipulatorController, Button.kY.value)
                // .onTrue(new LiftAndWristMove(m_lift, m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionL3,
                // Constants.WristConstants.WristPostion.kPositionL23))
                // .onFalse(new LiftAndWristMove(m_lift, m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionResting,
                // Constants.WristConstants.WristPostion.kPositionResting));

                // new JoystickButton(m_manipulatorController, Button.kX.value)
                // .onTrue(new LiftAndWristMove(m_lift, m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionL2,
                // Constants.WristConstants.WristPostion.kPositionL23))
                // .onFalse(new LiftAndWristMove(m_lift, m_wrist,
                // Constants.LiftConstants.LiftHeight.kPositionResting,
                // Constants.WristConstants.WristPostion.kPositionResting));

                // new JoystickButton(m_manipulatorController, Button.kY.value)
                // .onTrue(new RunCommand(
                // () ->
                // m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionSafe),
                // m_wrist))
                // .onFalse( new RunCommand(
                // () ->
                // m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting),
                // m_wrist));

                // new JoystickButton(m_manipulatorController, Button.kRightStick.value)
                // .whileTrue(new RunCommand(
                // () ->
                // {m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionClimb);
                // m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionL23);},
                // m_fourBar,m_wrist))
                // .onFalse(new RunCommand(
                // () ->
                // {m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting);
                // m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting);},
                // m_fourBar,m_wrist));

                // new JoystickButton(m_manipulatorController, Button.kY.value)
                // .onTrue(new RunCommand(
                // () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL3),
                // m_lift));

                // new JoystickButton(m_manipulatorController, Button.kX.value)
                // .onTrue(new RunCommand(
                // () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL2),
                // m_lift));

                // new JoystickButton(m_manipulatorController, Button.kStart.value)
                // .onTrue(Commands.runOnce(
                // () -> m_lift.resetLift(),
                // m_lift));
        }
        
  public LEDs getLEDS() {
        return leds;
      }
    
      public Intake getInt() {
        return m_intake;
      }

      public Claw getClaw() {
        return m_claw;
      }

      public Lift getLift() {
        return m_lift;
      }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();

        }
}
