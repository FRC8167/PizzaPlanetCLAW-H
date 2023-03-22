// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase {
  
  private final TalonFX pivotMotor;
  private AnalogInput anglePotentiometer;
  private AnalogPotentiometer angleSensor;
  private Boolean upperRotationAllowed;
  private Boolean lowerRotationAllowed;
  private double currentAngle;
  private enum motionState {UP, DOWN, STATIONARY};
  private motionState motion;
  

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new TalonFX(Constants.PIVOT_MOTOR);
    // isManualPivotDone = false;
    configmotor();
    anglePotentiometer = new AnalogInput(Constants.POT_PORT);
    anglePotentiometer.setAverageBits(4);
    angleSensor = new AnalogPotentiometer(anglePotentiometer, 300, 20);
    currentAngle = angleSensor.get();
    upperRotationAllowed = UpperLimitExceeded();
    lowerRotationAllowed = LowerLimitExceeded();
    motion = motionState.STATIONARY;

  }
  
  private void configmotor() {
    pivotMotor.configFactoryDefault();
    pivotMotor.setNeutralMode(NeutralMode.Brake);
    pivotMotor.configNeutralDeadband(0.1, 30);
    
    pivotMotor.configClosedloopRamp(1.0);
    pivotMotor.configOpenloopRamp(0.5);

    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.PIVOT_pidLoopTimeout);
    pivotMotor.selectProfileSlot(0, 0);

    pivotMotor.config_kF(0, 0.04, Constants.PIVOT_pidLoopTimeout); 
    pivotMotor.config_kP(0, 0.058, Constants.PIVOT_pidLoopTimeout); //.049
    pivotMotor.config_kI(0, 0, Constants.PIVOT_pidLoopTimeout);
    pivotMotor.config_kD(0, 0, Constants.PIVOT_pidLoopTimeout);

    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.PIVOT_pidLoopTimeout);
    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.PIVOT_pidLoopTimeout);

    pivotMotor.setInverted(true);
    pivotMotor.setSensorPhase(false);

    pivotMotor.configNominalOutputForward(0, Constants.PIVOT_pidLoopTimeout);
    pivotMotor.configNominalOutputReverse(0, Constants.PIVOT_pidLoopTimeout);
    pivotMotor.configPeakOutputForward(1, Constants.PIVOT_pidLoopTimeout);
    pivotMotor.configPeakOutputReverse(-1, Constants.PIVOT_pidLoopTimeout);

    pivotMotor.configMotionCruiseVelocity(Constants.pivotCruiseVelocity, Constants.PIVOT_pidLoopTimeout);
    pivotMotor.configMotionAcceleration(Constants.pivotAcceleration, Constants.PIVOT_pidLoopTimeout);

    pivotMotor.configForwardSoftLimitThreshold(40900);
    pivotMotor.configForwardSoftLimitEnable(true);
    pivotMotor.configReverseSoftLimitThreshold(1000);
    pivotMotor.configReverseSoftLimitEnable(true);

    //Zero the encoder
    pivotMotor.setSelectedSensorPosition(0, 0, Constants.PIVOT_pidLoopTimeout);  //really 45 real pos
    }

    public void testPivot(double power) {
      if (power > 0 && upperRotationAllowed)  {
        pivotMotor.set(ControlMode.PercentOutput, power);  
        motion = motionState.UP;
      }  else if (power < 0 && lowerRotationAllowed)  {
        pivotMotor.set(ControlMode.PercentOutput, power);  
        motion = motionState.DOWN;
      } else  {
        stop();
      }
    }



    public void setPivotMotionMagic(double targetRotations) {
      double targetTicks = targetRotations * 80 * 2048;
      double horizontalHoldOutput = 0.15;
      double arbFeedFwdTerm = getFeedForward(horizontalHoldOutput);
      //pivotMotor.set(TalonFXControlMode.MotionMagic, targetTicks);
      pivotMotor.set(TalonFXControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, arbFeedFwdTerm);

      //Display PID Commanded Target and Resulting Error
      StringBuilder pivotmoreinfo = new StringBuilder();
      pivotmoreinfo.append("\tPIVOT Commanded Target:  ");
      pivotmoreinfo.append(targetTicks);
      pivotmoreinfo.append("\tPIVOT PID Error ");
      pivotmoreinfo.append(pivotMotor.getClosedLoopError());
      pivotmoreinfo.append("\tPIVOT SENSOR POSITION:  ");
      pivotmoreinfo.append(pivotMotor.getSelectedSensorPosition());

      System.out.println(pivotmoreinfo.toString());
    
    }

    public void stop(){
      pivotMotor.set(ControlMode.PercentOutput, 0);
      motion = motionState.STATIONARY;
    }

    public boolean UpperLimitExceeded() {
      if (angleSensor.get() >= Constants.PIVOT_MAX_ROTATION_LIMIT)  {
        upperRotationAllowed = false;
        return true;
      }  else  {
        upperRotationAllowed = true;
        return false;
      }
    }

    public boolean LowerLimitExceeded()  {
      if (angleSensor.get() <= Constants.PIVOT_MIN_ROTATION_LIMIT)  {
        lowerRotationAllowed = false;
        return true;
      }  else  {
        lowerRotationAllowed = true;
        return false;
      }
    }
    
    public double getPivotAngle()  {
        return angleSensor.get();
    }

    public double getPivotPosition() {
      return pivotMotor.getSelectedSensorPosition(0);
    }

    public void zeroPivotSensor() {
      pivotMotor.setSelectedSensorPosition(0);
     // _currentPosition = pivotMotor.getSelectedSensorPosition();
    }


    //**************the 45-degree offset is just a guess from sketches.  Change this angle per actual build.*****

    private double getFeedForward(double horizontalHoldOutput) {
      double pivotSensorPosition = pivotMotor.getSelectedSensorPosition(0);
      double pivotCurrentAngle = (pivotSensorPosition/80*360/2048);  //lowest position is 0 via sensor; needs 45 deg offset
      // double theta = Math.toRadians(90 - (pivotCurrentAngle + 45));  //angle of elevation for arm; 90 is max hold
      double theta = Math.toRadians(56 - pivotCurrentAngle);
      double gravityCompensation = Math.cos(theta);
      double arbFeedFwd = gravityCompensation * horizontalHoldOutput;
      return arbFeedFwd;
    }

    public void UpdateDashboard()  {
      SmartDashboard.putNumber("Pivot Angle [deg]", currentAngle);
      SmartDashboard.putBoolean("Upper Limit", upperRotationAllowed);
      SmartDashboard.putBoolean("Lower Limit", lowerRotationAllowed);
      SmartDashboard.putNumber("Pivot Encoder Position", getPivotPosition());

      //Angle Potentiometer raw values for debugging purposes
      SmartDashboard.putNumber("Pivot Pot Raw", anglePotentiometer.getValue());
      SmartDashboard.putNumber("Pivot Pot Voltage", anglePotentiometer.getVoltage());
      SmartDashboard.putNumber("Pivot Command", pivotMotor.getMotorOutputPercent());
      SmartDashboard.putString("Pivot Motor Mode", pivotMotor.getControlMode().name());


    }

  @Override
  public void periodic() {
    currentAngle = angleSensor.get();
    // switch (motion) {
    //   case UP:  if (UpperLimitExceeded()) stop();
    //   case DOWN: if (LowerLimitExceeded()) stop();
    //   case STATIONARY: stop();
    // }

    // UpperLimitExceeded();
    // LowerLimitExceeded();
    SmartDashboard.putNumber("Pivot Encoder Position", getPivotPosition());
    
    //SmartDashboard.putNumber("Pivot Motor Current", pdm.getCurrent();
    UpdateDashboard();
    // This method will be called once per scheduler run
  }
}
