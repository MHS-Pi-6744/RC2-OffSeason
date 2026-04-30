package frc.robot.majc4frc.motor_ctl.rev;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSparkFlex extends SubsystemBase {
  private SparkFlex m_otor;
  private RelativeEncoder e_ncoder;
  private SparkClosedLoopController p_controller;

  private String s_motorName;

  public CommandSparkFlex(int canID, SparkBaseConfig config, String name) {
    s_motorName = name + "/";
    m_otor = new SparkFlex(canID, MotorType.kBrushless);
    e_ncoder = m_otor.getEncoder();

    m_otor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    p_controller = m_otor.getClosedLoopController();
    e_ncoder.setPosition(0);
  }

  public CommandSparkFlex(int canID, SparkBaseConfig config) {
    s_motorName = "SparkFlex #" + canID + "/";
    m_otor = new SparkFlex(canID, MotorType.kBrushless);
    e_ncoder = m_otor.getEncoder();

    m_otor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    p_controller = m_otor.getClosedLoopController();
    e_ncoder.setPosition(0);
  }

  public CommandSparkFlex(int canID) {
    s_motorName = "SparkFlex #" + canID + "/";
    m_otor = new SparkFlex(canID, MotorType.kBrushless);
    e_ncoder = m_otor.getEncoder();

    m_otor.configure(
        new SparkFlexConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    p_controller = m_otor.getClosedLoopController();
    e_ncoder.setPosition(0);
  }

  public Command stopMotor() {
    return runOnce(() -> m_otor.stopMotor());
  }

  public Command set(double speed) {
    return runOnce(() -> m_otor.set(speed));
  }

  public Command clearFaults() {
    return runOnce(() -> m_otor.clearFaults());
  }

  public Command setVoltage(double volts) {
    return runOnce(() -> m_otor.setVoltage(volts));
  }

  public Command setVoltage(Voltage volts) {
    return runOnce(() -> m_otor.setVoltage(volts));
  }

  public Command setSetpoint(
      double setpoint,
      SparkBase.ControlType ctrl,
      ClosedLoopSlot slot,
      double arbFeedforward,
      ArbFFUnits arbFFUnits) {
    return runOnce(
        () -> p_controller.setSetpoint(setpoint, ctrl, slot, arbFeedforward, arbFFUnits));
  }

  public Command setSetpoint(
      double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot, double arbFeedforward) {
    return setSetpoint(setpoint, ctrl, slot, arbFeedforward, ArbFFUnits.kVoltage);
  }

  public Command setSetpoint(double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot) {
    return setSetpoint(setpoint, ctrl, slot, 0);
  }

  public Command setSetpoint(double setpoint, SparkBase.ControlType ctrl) {
    return setSetpoint(setpoint, ctrl, ClosedLoopSlot.kSlot0);
  }

  public Command setPosition(double pos) {
    return runOnce(() -> e_ncoder.setPosition(pos));
  }

  public double getVelocity() {
    return e_ncoder.getVelocity();
  }

  public Trigger isVelocityExactly(double vel) {
    return new Trigger(() -> e_ncoder.getVelocity() == vel);
  }

  public Trigger isVelocityCloseTo(double vel, double err) {
    return isVelocityGreaterThan(vel - (vel * err)).or(isVelocityLessThan(vel + (vel * err)));
  }

  public Trigger isVelocityCloseTo(double vel) {
    return isVelocityCloseTo(vel, 0.9);
  }

  public Trigger isVelocityGreaterThan(double vel) {
    return new Trigger(() -> e_ncoder.getVelocity() > vel);
  }

  public Trigger isVelocityLessThan(double vel) {
    return new Trigger(() -> e_ncoder.getVelocity() < vel);
  }

  public double getPosition() {
    return e_ncoder.getPosition();
  }

  public Trigger isPositionExactly(double pos) {
    return new Trigger(() -> e_ncoder.getPosition() == pos);
  }

  public Trigger isPositionCloseTo(double pos, double err) {
    return isPositionGreaterThan(pos - (pos * err)).or(isPositionLessThan(pos + (pos * err)));
  }

  public Trigger isPositionCloseTo(double pos) {
    return isPositionCloseTo(pos, 0.9);
  }

  public Trigger isPositionGreaterThan(double pos) {
    return new Trigger(() -> e_ncoder.getPosition() > pos);
  }

  public Trigger isPositionLessThan(double pos) {
    return new Trigger(() -> e_ncoder.getPosition() < pos);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(s_motorName + "Output", m_otor.getAppliedOutput());
    SmartDashboard.putNumber(s_motorName + "Current", m_otor.getOutputCurrent());

    SmartDashboard.putBoolean(s_motorName + "Faults/" + "Can", m_otor.getFaults().can);
    SmartDashboard.putBoolean(s_motorName + "Faults/" + "escEeprom", m_otor.getFaults().escEeprom);
    SmartDashboard.putBoolean(s_motorName + "Faults/" + "Firmware", m_otor.getFaults().firmware);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Gate Driver", m_otor.getFaults().gateDriver);
    SmartDashboard.putBoolean(s_motorName + "Faults/" + "Motor Type", m_otor.getFaults().motorType);
    SmartDashboard.putBoolean(s_motorName + "Faults/" + "Other", m_otor.getFaults().other);
    SmartDashboard.putBoolean(s_motorName + "Faults/" + "Sensor", m_otor.getFaults().sensor);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Temperature", m_otor.getFaults().temperature);

    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "Can", m_otor.getStickyFaults().can);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "escEeprom", m_otor.getStickyFaults().escEeprom);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "Firmware", m_otor.getStickyFaults().firmware);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "Gate Driver", m_otor.getStickyFaults().gateDriver);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "Motor Type", m_otor.getStickyFaults().motorType);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "Other", m_otor.getStickyFaults().other);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "Sensor", m_otor.getStickyFaults().sensor);
    SmartDashboard.putBoolean(
        s_motorName + "Faults/" + "Sticky/" + "Temperature", m_otor.getStickyFaults().temperature);

    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Brownout", m_otor.getWarnings().brownout);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "escEeprom", m_otor.getWarnings().escEeprom);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "extEeprom", m_otor.getWarnings().extEeprom);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Has Reset", m_otor.getWarnings().hasReset);
    SmartDashboard.putBoolean(s_motorName + "Warnings/" + "Other", m_otor.getWarnings().other);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Overcurrent", m_otor.getWarnings().overcurrent);
    SmartDashboard.putBoolean(s_motorName + "Warnings/" + "Sensor", m_otor.getWarnings().sensor);
    SmartDashboard.putBoolean(s_motorName + "Warnings/" + "Stall", m_otor.getWarnings().stall);

    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "Brownout", m_otor.getStickyWarnings().brownout);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "escEeprom", m_otor.getStickyWarnings().escEeprom);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "extEeprom", m_otor.getWarnings().extEeprom);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "Has Reset", m_otor.getWarnings().hasReset);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "Other", m_otor.getWarnings().other);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "Overcurrent", m_otor.getWarnings().overcurrent);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "Sensor", m_otor.getWarnings().sensor);
    SmartDashboard.putBoolean(
        s_motorName + "Warnings/" + "Sticky/" + "Stall", m_otor.getWarnings().stall);

    SmartDashboard.putNumber(s_motorName + "Position", e_ncoder.getPosition());
    SmartDashboard.putNumber(s_motorName + "Velocity", e_ncoder.getVelocity());

    SmartDashboard.putNumber(s_motorName + "Setpoint", p_controller.getSetpoint());
  }
}
