// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final TalonFX armMotor;
  private DigitalInput flagSwitch;
  private boolean isExtensionAllowed;
  private boolean isRetractionAllowed;
  private double approxCurrentDistance;
  private enum armState {OUTGOING, INCOMING, STATIC};
  private armState motion;
  public String gamepiece = "cube";


  /** Creates a new Arm. */
  public Arm() {
    armMotor = new TalonFX(Constants.ARM_MOTOR);
    flagSwitch = new DigitalInput(Constants.FLAG_SWITCH);
    motion = armState.STATIC;

    configmotor();
    approxCurrentDistance = (20*2048*armMotor.getSelectedSensorPosition(0)) / (0.787 * Math.PI);
    isExtensionAllowed = MaxLimitExceeded();
    isRetractionAllowed = MinLimitExceeded();
  
    armMotor.configForwardSoftLimitThreshold(380000);
    armMotor.configForwardSoftLimitEnable(true);
    armMotor.configReverseSoftLimitThreshold(-512);
    armMotor.configReverseSoftLimitEnable(true);
  }

  private void configmotor(){
    armMotor.configFactoryDefault();
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configNeutralDeadband(0.1, 30);
    
    armMotor.configClosedloopRamp(1.0);
    armMotor.configOpenloopRamp(0.0);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.ARM_pidLoopTimeout);
    armMotor.selectProfileSlot(0, 0);

    armMotor.config_kF(0, 0.0, Constants.ARM_pidLoopTimeout);//.046W
    armMotor.config_kP(0, 0.1, Constants.ARM_pidLoopTimeout);//.049W
    armMotor.config_kI(0, 0.0, Constants.ARM_pidLoopTimeout);
    armMotor.config_kD(0, 0.05, Constants.ARM_pidLoopTimeout);//0.0W

    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.ARM_pidLoopTimeout);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.ARM_pidLoopTimeout);

    armMotor.setInverted(true);
    armMotor.setSensorPhase(false);

    armMotor.configNominalOutputForward(0, Constants.ARM_pidLoopTimeout);
    armMotor.configNominalOutputReverse(0, Constants.ARM_pidLoopTimeout);
    armMotor.configPeakOutputForward(1, Constants.ARM_pidLoopTimeout);
    armMotor.configPeakOutputReverse(-1, Constants.ARM_pidLoopTimeout);

    armMotor.configMotionCruiseVelocity(Constants.armCruiseVelocity, Constants.ARM_pidLoopTimeout);
    armMotor.configMotionAcceleration(Constants.armAcceleration, Constants.ARM_pidLoopTimeout);

    //Zero the encoder
    armMotor.setSelectedSensorPosition(0, 0, Constants.ARM_pidLoopTimeout);

    }


    public void testArm(double power) {  //MANUAL ARM CONTROL
      
        if (power < 0 && isRetractionAllowed ) {
          armMotor.set(ControlMode.PercentOutput, power);  
          motion = armState.INCOMING;
        }
        else if (power > 0 && isExtensionAllowed) 
        {
          armMotor.set(ControlMode.PercentOutput, power);  
          motion = armState.OUTGOING;
        }  else {
          stop();
        }
      }


        

    public void setArmMotionMagic(double targetRotations) {
      double targetTicks = targetRotations * 20 * 2048;
      armMotor.set(TalonFXControlMode.MotionMagic, targetTicks);
      
    
    }
    public boolean armIsFullyExtend() {
      return armMotor.getSelectedSensorPosition() < 350000;
    }

    public void stop(){
      armMotor.set(ControlMode.PercentOutput, 0);
      motion = armState.STATIC;
    }

    public double getArmPosition() {
      return armMotor.getSelectedSensorPosition(0);
    }

    public void zeroArmSensor() {
      armMotor.setSelectedSensorPosition(0);
    }

    public boolean MaxLimitExceeded()  {

      if (getArmPosition() >= Constants.ARM_MAX_EXTENSION_TICKS)  {
        isExtensionAllowed = false;
        return true;
      }  else  {
        isExtensionAllowed = true;
        return false;
      }
    }
    
    public boolean MinLimitExceeded()  {
      //return  !flagSwitch.get();
      if (!flagSwitch.get())  { //getArmPosition() <= Constants.ARM_MIN_RETRACTION_TICKS  ||
        isRetractionAllowed = false;
        return true;
      }  else  {
        isRetractionAllowed = true;
        return false;
      }
      }

      public String toggleGamepiece() {
        if (gamepiece == "cube")  {
          gamepiece = "cone";
        } else {
          gamepiece = "cube";
        }
        SmartDashboard.putString("Gamepiece", gamepiece);
        return gamepiece;
      }




    
      public void UpdateDashboard()  {
        SmartDashboard.putNumber("Approx Extension [in]", approxCurrentDistance);
        SmartDashboard.putBoolean("IsExtensionAllowed", isExtensionAllowed);
        SmartDashboard.putBoolean("IsRetractionAllowed", isRetractionAllowed);
        SmartDashboard.putNumber("Arm Encoder Position", getArmPosition());
  
        //Optical Sensor state for debugging purposes
        SmartDashboard.putBoolean("Flag Sensor", flagSwitch.get());  //low when fully retracted aka false
  
      }

  @Override
  public void periodic() {
    approxCurrentDistance = (20*2048*armMotor.getSelectedSensorPosition(0)) / (0.787 * Math.PI); 
    switch (motion) {
      case OUTGOING:  if (MaxLimitExceeded()) stop();
      case INCOMING: if (MinLimitExceeded()) stop();
      case STATIC: stop();
    }
    //flagSwitch.get();
    MinLimitExceeded();
    MaxLimitExceeded();

    UpdateDashboard();
    // This method will be called once per scheduler run
  }
}
