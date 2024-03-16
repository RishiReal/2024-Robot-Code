// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax climber1, climber2;
  private static RelativeEncoder climber1Encoder, climber2Encoder;  

  public final static double setpoint = 1000;

  /** Creates a new Climber. */
  public Climber() {
    climber1 = new CANSparkMax(ClimberConstants.CLIMBER_1, MotorType.kBrushless);
    climber2 = new CANSparkMax(ClimberConstants.CLIMBER_2, MotorType.kBrushless);

    climber1.restoreFactoryDefaults();
    climber2.restoreFactoryDefaults();

    climber1.setInverted(true);
    climber2.setInverted(false);

    climber1.setIdleMode(IdleMode.kBrake);
    climber1.setIdleMode(IdleMode.kBrake);   

    climber1.setSmartCurrentLimit(40);
    climber2.setSmartCurrentLimit(40);

    climber1.enableSoftLimit(SoftLimitDirection.kForward, true);
    climber1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    climber2.enableSoftLimit(SoftLimitDirection.kForward, true);
    climber2.enableSoftLimit(SoftLimitDirection.kReverse, true);

    climber1.setSoftLimit(SoftLimitDirection.kForward, 300);
    climber1.setSoftLimit(SoftLimitDirection.kReverse, 4);

    climber2.setSoftLimit(SoftLimitDirection.kForward, 291);
    climber2.setSoftLimit(SoftLimitDirection.kReverse, 5);

    climber1.burnFlash();
    climber2.burnFlash();

    climber1Encoder = climber1.getEncoder();
    climber2Encoder = climber2.getEncoder();

    climber1Encoder.setPositionConversionFactor(1 / 70);
    climber2Encoder.setPositionConversionFactor(1 / 70);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber1Encoder", climber1Encoder.getPosition());
    SmartDashboard.putNumber("Climber2Encoder", climber2Encoder.getPosition());
  }

  public void set1(double speed) { climber1.set(speed); }
  public void set2(double speed) { climber2.set(speed); }
  public void stop1() { climber1.set(0.0); }
  public void stop2() { climber2.set(0.0); }
  public void setPositions() {
    climber1Encoder.setPosition(0);
    climber2Encoder.setPosition(0);
  } 
}
