// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SimpleAutoDrive extends Command {

  private SwerveSubsystem swerveSubsystem;
  private ChassisSpeeds chassisSpeeds;
  private ChassisSpeeds discreteSpeeds;
  private SwerveModuleState[] moduleStates;
  

  /** Creates a new SimpleAutoDrive. */
  public SimpleAutoDrive(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSpeeds = new ChassisSpeeds(-2, 0, 0);
    discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);

        // 6. Output each module states to wheels
                swerveSubsystem.setModuleStates(moduleStates); 

       }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates); 


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
