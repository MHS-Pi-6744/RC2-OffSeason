package frc.robot.majc4frc.motor_ctl.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.wpiutils.MotorSafetyImplem;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandTalonFXS extends SubsystemBase {
  private String s_motorName;
  private TalonFXS m_motor;

  private String m_description;

  public static final double kDefaultSafetyExpiration = 0.1;

  private MotorSafetyImplem m_motorSafety = null;
  private double m_motSafeExpiration = kDefaultSafetyExpiration;

//  private final StatusSignal<Double> m_dutyCycle = getDutyCycle(false).clone();

  public CommandTalonFXS(int deviceId, String name) {
    s_motorName = name + "/";
    m_motor = new TalonFXS(deviceId);
  }

  public CommandTalonFXS(int deviceId) {
    s_motorName = "TalonFXS #" + deviceId + "/";
    m_motor = new TalonFXS(deviceId);
  }

  private StatusCode _setControl(ControlRequest request) {
    var err = m_motor.setControl(request);
    if (!err.isOK()) {
      SmartDashboard.putString(s_motorName + request.getName() + "/Error/Name", err.getName());
      SmartDashboard.putString(s_motorName + request.getName() + "/Error/Description", err.getDescription());
    }
    return err;
  }

  public String getDescription() {
    return m_description;
  }

  private MotorSafetyImplem GetMotorSafety() {
    if (m_motorSafety == null) {
      m_motorSafety = new MotorSafetyImplem(this::stopMotor, getDescription());

      m_motorSafety.setExpiration(m_motSafeExpiration);
    }
    return m_motorSafety;
  }

  public final boolean isAlive() {
    return m_motor.isAlive();
  }

  public final boolean isSafetyEnabled() {
    return m_motor.isSafetyEnabled();
  }

  public final double getExpiration() {
    return m_motor.getExpiration();
  }

  public final void setExpiration(double expirationTime) {
  
  }

  public Command setControl(ControlRequest request) {
    return runOnce(
      () -> _setControl(request)
    );
  }

  public Command set(double speed) {
    return run(
      () -> m_motor.set(speed)
    );
  }

  public Command stopMotor() {
    return runOnce(
      () -> m_motor.stopMotor()
    );
  }

  public Command disable() {
    return runOnce(
      () -> m_motor.disable()
    );
  }
  
  @Override
  public void periodic() {
    // Put telemetry things here
    SmartDashboard.putNumber(s_motorName + "Output Status", m_motor.getMotorOutputStatus().getValueAsDouble());
  }
}
