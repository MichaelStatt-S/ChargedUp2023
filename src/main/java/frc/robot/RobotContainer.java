// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Balance.ActiveBalance;
import frc.robot.commands.Balance.PassiveBalance;
import frc.robot.commands.Balance.SpeedyBalance;
import frc.robot.commands.Intake.intake;
import frc.robot.commands.Intake.retract;
import frc.robot.commands.Scoring.scoreCones;
import frc.robot.commands.Scoring.scoreHigh;
import frc.robot.commands.Scoring.scoreMid;
import frc.robot.commands.Scoring.scoreLow;
import frc.robot.commands.Scoring.scoreHighAuton;
import frc.robot.subsystems.*;

/* 
Summary:
This code is for the robot container and has a joy stick, joystick buttons, swerve subsystem, a sendable chooser for autonomous modes, autonomous modes, and methods for configuring button bindings and smart dashboard options. 
*/

public class RobotContainer {
        /* Controllers */
        public final static Joystick driver = new Joystick(0);
        private final Joystick codriver = new Joystick(1);

        /* Compressor */
        private Compressor compressor;

        // Gyro Sensor
        private static Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID);

        /* Drive Controls */
        private static final int translationAxis = XboxController.Axis.kLeftY.value;
        private static final int strafeAxis = XboxController.Axis.kLeftX.value;
        private static final int rotationAxis = XboxController.Axis.kRightX.value;
        private double SPEED_MULTIPLIER = 1.0;

        
        /* Driver Buttons */
        private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
        private final JoystickButton outtakeButton = new JoystickButton(codriver, XboxController.Button.kB.value);
        private final JoystickButton retractButton = new JoystickButton(codriver, XboxController.Button.kA.value);
        private final JoystickButton intakeButton = new JoystickButton(codriver, XboxController.Button.kX.value);
        private final JoystickButton lowButton = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
        private final JoystickButton coneButton = new JoystickButton(codriver,
                XboxController.Button.kRightBumper.value);
        public final static JoystickButton activeBalanceButton = new JoystickButton(driver,
                XboxController.Button.kRightBumper.value);
        private final JoystickButton passiveBalanceButton = new JoystickButton(driver,
                XboxController.Button.kLeftBumper.value);
        private final JoystickButton intakeFullButton = new JoystickButton(codriver, XboxController.Button.kY.value);
        private final JoystickButton robotCentric = new JoystickButton(driver,
                XboxController.Button.kRightBumper.value);

        /* Subsystems */
        // public final static VisionSubsystem s_visionSubsystem = new
        // VisionSubsystem();
        
        /* Commands */
        private final Swerve s_Swerve = new Swerve(gyro);
        private final Ground_Intake ground_intake = new Ground_Intake();
        private final LedSub ledsub = new LedSub();
        private final PassiveBalance passiveBalance = new PassiveBalance(s_Swerve);
        private final ActiveBalance activeBalance = new ActiveBalance(s_Swerve, gyro);
        private final Command speedyBalance = new SpeedyBalance(s_Swerve, gyro);
        public final Command outtakeFullAuto = new scoreHighAuton(ground_intake);
        public final scoreCones ScoreCones = new scoreCones(ground_intake);
        
        //public final static Led ledSubsystem = new Led(gyro, ground_intake);
        
        /* LED Commands */
        public final Command ledcommand = new ledCommand(gyro, ground_intake,ledsub);
        
        /* Pneumatics Commands */
        public final Command intake = new intake(ground_intake);
        public final Command outtake = new scoreMid(ground_intake);
        public final Command retract = new retract(ground_intake);
        public final Command outtakeFull = new scoreHigh(ground_intake);
        public final Command outtakeLow = new scoreLow(ground_intake);

        /* SendableChooser */
        public final SendableChooser<Command> autoChooser;

        //  private final VisionSubsytem visionSubsytem = new VisionSubsytem();
        

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                System.out.print("init");
                CameraServer.startAutomaticCapture();
                // CameraServer.startAutomaticCapture();
                compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
                compressor.enableDigital();

                s_Swerve.setDefaultCommand(
                        new TeleopSwerve(
                                s_Swerve,

                                () -> -driver.getRawAxis(translationAxis) * SPEED_MULTIPLIER,
                                () -> -driver.getRawAxis(strafeAxis) * SPEED_MULTIPLIER,
                                () -> -driver.getRawAxis(rotationAxis) * SPEED_MULTIPLIER,
                                () -> robotCentric.getAsBoolean()));
                SmartDashboard.putNumber("Speed Multipler", SPEED_MULTIPLIER);

                ledsub.setDefaultCommand(ledcommand);

                // visionSubsytem.setDefaultCommand(new AddVisionPose(visionSubsytem,s_Swerve));
                
                autoChooser = AutoBuilder.buildAutoChooser("New Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);

                // Configure the button bindings
                configureButtonBindings();

                // Configure Smart Dashboard options
                configureSmartDashboard();

                
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                /* Driver Buttons */
                zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

                /* Pneumatics Buttons */
                intakeButton.onTrue(intake);
                outtakeButton.onTrue(outtake);
                retractButton.onTrue(retract);
                activeBalanceButton.onTrue(activeBalance);
                passiveBalanceButton.onTrue(passiveBalance);
                intakeFullButton.onTrue(outtakeFull);
                lowButton.onTrue(outtakeLow);
                coneButton.onTrue(ScoreCones);

        }

        private void configureSmartDashboard() {
        
        }

        public void disabledInit() {
                s_Swerve.resetToAbsolute();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Executes the autonomous command chosen in smart dashboard
                return autoChooser.getSelected();
                        // return new PathPlannerAuto("New Auto");

        }
}