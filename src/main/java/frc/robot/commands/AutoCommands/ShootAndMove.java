// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.NoteHandler;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndMove extends SequentialCommandGroup {

  /** Creates a new ShootAndMove. */
  public ShootAndMove(NoteHandler noteHandler, SwerveSubsystem swerveSubsystem) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands(new BasicNoteShot(noteHandler).withTimeout(3.00));
    //addCommands(new SimpleAutoDrive(swerveSubsystem).withTimeout(1.75));
    addCommands(new SimpleAuto(swerveSubsystem, 0, 0, 2).withTimeout(0.75));
  }
}
