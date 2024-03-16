// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHandler;

public class ARMDOWN extends Command {
  private final NoteHandler noteHandler;

  /** Creates a new IntakeNoteAuto. */
  public ARMDOWN(NoteHandler noteHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.noteHandler = noteHandler;
    addRequirements(noteHandler);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { noteHandler.setArmSpeed(.25); }

  @Override
  public void end(boolean interrupted) { noteHandler.setPositionEncoders(); }

  @Override
  public boolean isFinished() { return (noteHandler.getArmAngle() >= 1.75); }
}
