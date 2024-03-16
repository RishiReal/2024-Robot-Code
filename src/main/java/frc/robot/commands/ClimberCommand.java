// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {

  private final XboxController gamepad;
  private final Climber climber;

  public static final double UP = 0.8;
  public static final double DOWN = -0.8;
  public static final double STOP = 0.0;

  public double speed1 = 0;
  public double speed2 = 0;

  /** Creates a new ControlElevator. */
  public ClimberCommand(Climber climber, XboxController gamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.gamepad = gamepad;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    climber.setPositions();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    speed1 = 0;
    speed2 = 0;

    if (gamepad.getRawAxis(OIConstants.XBX_L_Y) < -0.2) {
      speed1 = UP;
    } else if (gamepad.getRawAxis(OIConstants.XBX_L_Y) > 0.2) {
      speed1 = DOWN;
    } else {
      speed1 = STOP;
    }

    if (gamepad.getRawAxis(OIConstants.XBX_R_Y) < -0.2) {
      speed2 = UP;
    } else if (gamepad.getRawAxis(OIConstants.XBX_R_Y) > 0.2) {
      speed2 = DOWN;
    } else {
      speed2 = STOP;
    }

    climber.set1(speed1);
    climber.set2(speed2);
    
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
