// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    
  private final WPI_TalonSRX shooter = new WPI_TalonSRX(ShooterConstants.kShooterMotorPort);

  private final double maxSpeed = 1;
  private double speed = 0.1;

  /** Creates a new Intake. */
  public Shooter() {
    addChild("Shooter", shooter);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", speed);
  }

  public void shoot() {
    shooter.set(ControlMode.PercentOutput, speed);
  }

  public void increaseSpeed() {
    if (speed < maxSpeed) {
    speed += 0.1;
    }
  }
}
