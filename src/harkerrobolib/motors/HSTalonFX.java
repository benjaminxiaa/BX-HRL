package harkerrobolib.motors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import harkerrobolib.util.Constants;
import harkerrobolib.util.PIDConfig;

public class HSTalonFX implements AutoCloseable {
    private static final double kCANTimeoutS = 0.1; // s
    private final String m_name;
    private final TalonFX m_controller;
    private final HSTalonFXConfiguration m_config;

    private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);
    private final VoltageOut m_voltageControl = new VoltageOut(0);
    private final TorqueCurrentFOC m_currentControl = new TorqueCurrentFOC(0);
    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0);
    private final PositionVoltage m_positionControl = new PositionVoltage(0);
    private final MotionMagicVoltage m_motionMagicControl = new MotionMagicVoltage(0);

    public static class HSTalonFXConfiguration {
        private NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        private boolean INVERTED = false;
        private double SENSOR_TO_MECH_RATIO = 1.0;
        private double SUPPLY_CURRENT_LIMIT = 80.0; // A
        private double STATOR_CURRENT_LIMIT = 80.0; // A
        private double FORWARD_SOFT_LIMIT = 0.0;
        private double REVERSE_SOFT_LIMIT = 0.0;
        private PIDConfig slot0Config = new PIDConfig();
        private PIDConfig slot1Config = new PIDConfig();
        private PIDConfig slot2Config = new PIDConfig();
        private double motionMagicCruiseVelocity = 0.0;
        private double motionMagicAcceleration = 0.0;
        private double motionMagicJerk = 0.0;

        public HSTalonFXConfiguration setBrakeMode() {
            NEUTRAL_MODE = NeutralModeValue.Brake;
            return this;
        }

        public HSTalonFXConfiguration setInverted(final boolean inverted) {
            INVERTED = inverted;
            return this;
        }

        public HSTalonFXConfiguration setStatorCurrentLimit(final double amps) {
            STATOR_CURRENT_LIMIT = amps;
            return this;
        }

        public HSTalonFXConfiguration setSupplyCurrentLimit(final double amps) {
            SUPPLY_CURRENT_LIMIT = amps;
            return this;
        }

        public HSTalonFXConfiguration setForwardSoftLimitThreshold(final double pos) {
            FORWARD_SOFT_LIMIT = pos;
            return this;
        }

        public HSTalonFXConfiguration setReverseSoftLimitThreshold(final double pos) {
            REVERSE_SOFT_LIMIT = pos;
            return this;
        }

        public HSTalonFXConfiguration setSensorToMechanismRatio(final double ratio) {
            SENSOR_TO_MECH_RATIO = ratio;
            return this;
        }

        public HSTalonFXConfiguration setPIDConfig(final int slot, final PIDConfig config) {
            switch (slot) {
                case 0:
                    slot0Config = config;
                    break;
                case 1:
                    slot1Config = config;
                    break;
                case 2:
                    slot2Config = config;
                    break;
                default:
                    throw new RuntimeException("Invalid PID slot " + slot);
            }
            return this;
        }

        public HSTalonFXConfiguration setMotionMagicConfig(
                final double cruiseVelocity, final double acceleration, final double jerk) {
            motionMagicCruiseVelocity = cruiseVelocity;
            motionMagicAcceleration = acceleration;
            motionMagicJerk = jerk;
            return this;
        }

        public TalonFXConfiguration toTalonFXConfiguration() {
            final TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NEUTRAL_MODE;
            config.MotorOutput.Inverted = INVERTED ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;

            config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECH_RATIO;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;

            config.Slot0 = slot0Config.fillCTRE(new Slot0Configs());
            config.Slot1 = slot1Config.fillCTRE(new Slot1Configs());
            config.Slot2 = slot2Config.fillCTRE(new Slot2Configs());

            config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
            config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT;

            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT;

            config.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
            config.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration;
            config.MotionMagic.MotionMagicJerk = motionMagicJerk;

            return config;
        }
    }

    public static HSTalonFXConfiguration makeDefaultConfig() {
        return new HSTalonFXConfiguration();
    }

    /** Follower constructor */
    public HSTalonFX(final int deviceID, final HSTalonFX leader, final boolean opposeLeader,
            final HSTalonFXConfiguration config) {
        this(deviceID, config);
        m_controller.setControl(new Follower(leader.getDeviceID(), opposeLeader));
    }

    /** Main constructor */
    public HSTalonFX(final int deviceID, final HSTalonFXConfiguration config) {
        m_name = "TalonFX " + deviceID;
        m_controller = new TalonFX(deviceID);
        m_config = config;

        // Clear sticky faults
        m_controller.clearStickyFaults();

        // Configure the motor
        setConfiguration();
    }

    public boolean setConfiguration() {
        // Set motor controller configuration
        final TalonFXConfiguration config = m_config.toTalonFXConfiguration();
        return m_controller.getConfigurator().apply(config, kCANTimeoutS).isOK();
    }

    public boolean checkFaultsAndReconfigureIfNecessary() {
        if (m_controller.hasResetOccurred()) {
            DriverStation.reportError(m_name + ": reset occurred", false);
            setConfiguration();
            return true;
        }
        return false;
    }

    @Override
    public void close() {
        m_controller.close();
    }

    public int getDeviceID() {
        return m_controller.getDeviceID();
    }

    public void setBrakeMode(final boolean on) {
        m_config.NEUTRAL_MODE = on ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_controller.getConfigurator().apply(m_config.toTalonFXConfiguration().MotorOutput);
    }

    public void setStatorCurrentLimit(final double amps) {
        m_config.STATOR_CURRENT_LIMIT = amps;
        m_controller.getConfigurator().apply(m_config.toTalonFXConfiguration().CurrentLimits, kCANTimeoutS);
    }

    public void setPercentOutput(final double percent) {
        m_dutyCycleControl.Output = percent;
        m_controller.setControl(m_dutyCycleControl);
    }

    public void setVoltageOutput(final double voltage) {
        m_voltageControl.Output = voltage;
        m_controller.setControl(m_voltageControl);
    }

    public void setCurrentOutput(final double current, final double maxAbsDutyCycle) {
        m_currentControl.Output = current;
        m_currentControl.MaxAbsDutyCycle = maxAbsDutyCycle;
        m_controller.setControl(m_currentControl);
    }

    public void setPositionSetpoint(final int slot, final double setpoint) {
        setPositionSetpoint(slot, setpoint, 0.0);
    }

    public void setPositionSetpoint(final int slot, final double setpoint, final double feedforwardVolts) {
        m_positionControl.Slot = slot;
        m_positionControl.Position = setpoint;
        m_positionControl.FeedForward = feedforwardVolts;
        m_controller.setControl(m_positionControl);
    }

    public void setMotionMagicPositionSetpoint(final int slot, final double setpoint) {
        setMotionMagicPositionSetpoint(slot, setpoint, 0.0);
    }

    public void setMotionMagicPositionSetpoint(final int slot, final double setpoint,
            final double feedforwardVolts) {
        m_motionMagicControl.Slot = slot;
        m_motionMagicControl.Position = setpoint;
        m_motionMagicControl.FeedForward = feedforwardVolts;
        m_controller.setControl(m_motionMagicControl);
    }

    public void setVelocitySetpoint(final int slot, final double setpoint) {
        setVelocitySetpoint(slot, setpoint, 0.0);
    }

    public void setVelocitySetpoint(final int slot, final double setpoint, final double feedforwardVolts) {
        m_velocityControl.Slot = slot;
        m_velocityControl.Velocity = setpoint;
        m_velocityControl.FeedForward = feedforwardVolts;
        m_controller.setControl(m_velocityControl);
    }

    public void setControl(ControlRequest request) {
        m_controller.setControl(request);
    }

    public double getStatorCurrent() {
        return m_controller.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_controller.getSupplyCurrent().getValueAsDouble();
    }

    public double getPercentOutput() {
        return m_controller.getDutyCycle().getValue();
    }

    public boolean getInverted() {
        return m_config.INVERTED;
    }

    public void zeroSensorPosition() {
        setSensorPosition(0.0);
    }

    public void setSensorPosition(final double pos) {
        m_controller.setPosition(pos);
    }

    public double getSensorPosition() {
        return m_controller.getPosition().getValueAsDouble();
    }

    public double getSensorVelocity() {
        return m_controller.getVelocity().getValueAsDouble();
    }
}