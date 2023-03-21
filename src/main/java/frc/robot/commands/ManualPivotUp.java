// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class ManualPivotUp extends CommandBase {
  private final Pivot pivot;
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.27);
  private double power;
  
  /** Creates a new ManualPivotUp. */
  public ManualPivotUp(Pivot pivot, double power) {
    this.pivot = pivot;
    this.power = power;
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.testPivot(rotLimiter.calculate(power));
    SmartDashboard.putNumber("Slew Up Command", rotLimiter.calculate(power));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.UpperLimitExceeded() ;
    //return pivot.UpperLimitExceeded();
    //return false;
  }
}
