// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NoteHandlerConstants;
import frc.robot.subsystems.NoteHandler;

public class ArmNeutralizeAuto extends Command {

  private final NoteHandler noteHandler;
  private double encoderAngle;

  /** Creates a new IntakeNoteAuto. */
  public ArmNeutralizeAuto(NoteHandler noteHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.noteHandler = noteHandler;
    addRequirements(noteHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderAngle = Math.toRadians(10);
    noteHandler.setArmAngle(encoderAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(encoderAngle - noteHandler.getArmAngle()) <= NoteHandlerConstants.ANGLE_TOLERANCE);
  }
}
