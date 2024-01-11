// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ground_Intake extends SubsystemBase {
	/* Solenoid */
	DoubleSolenoid clampSolenoid;
	DoubleSolenoid kickerSolenoid;
	DoubleSolenoid kickerSolenoid2;
	DoubleSolenoid tiltSolenoid;

	Mechanism2d intakeMechanism = new Mechanism2d(2, 2);
	MechanismRoot2d mechanismRoot = intakeMechanism.getRoot("ground intake", 1.5, 0);
	MechanismLigament2d mechanismTilt = mechanismRoot.append(new MechanismLigament2d("tilt", 1, 90));
	MechanismLigament2d mechanismKicker = mechanismTilt.append(new MechanismLigament2d("kicker", 1, -90));
	MechanismLigament2d mechanismClamp = mechanismKicker.append(new MechanismLigament2d("clamp", 0.05, 90));
	
	
	/** Creates a new Ground_Intake. */
	public Ground_Intake() {

		 clampSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
         kickerSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
         tiltSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
         kickerSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);


	}
	// open clamp
	public void openClamp() {
		clampSolenoid.set(Value.kForward);
	}

	// close clamp
	public void closeClamp() {
		clampSolenoid.set(Value.kReverse);
	}

	// push kicker out
	public void sendKicker() {
		kickerSolenoid.set(Value.kReverse);
		kickerSolenoid2.set(Value.kReverse);
		
	}

	// bring kicker back in
	public void returnKicker() {
		kickerSolenoid.set(Value.kForward);
		kickerSolenoid2.set(Value.kForward);
	}

	// Tilt robot forward
	public void tiltUpward() {
		tiltSolenoid.set(Value.kReverse);
	}

	// Tilt robot back upright
	public void tiltDownward() {
		tiltSolenoid.set(Value.kForward);
	}

	public Value getTilt(){
		return tiltSolenoid.get();
	}
	

	@Override
	public void periodic() {
		if (kickerSolenoid.get().equals(Value.kForward)) {
			mechanismKicker.setAngle(200);
		} else {
			mechanismKicker.setAngle(185);
		}
		if (tiltSolenoid.get().equals(Value.kForward)) {
			mechanismTilt.setAngle(90);
		} else {
			mechanismTilt.setAngle(100);
		}
		if (clampSolenoid.get().equals(Value.kForward)) {
			mechanismClamp.setLength(0.05);
		} else {
			mechanismClamp.setLength(0.25);
		}
	
		
		SmartDashboard.putData("Intake Mechanism", intakeMechanism);
		SmartDashboard.putData("Clamp Piston/s", clampSolenoid);
		SmartDashboard.putData("Kicker Piston", kickerSolenoid);
		SmartDashboard.putData("Tilt Piston", tiltSolenoid);

	}
	
}

