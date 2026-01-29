package frc.alotobots.rebuilt.subsystems.launcher.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.alotobots.rebuilt.subsystems.launcher.io.ShooterIOInputsAutoLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.launcher.constants.ShooterTalonFXConstants;
import frc.alotobots.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO { 
    private final CANBus canBus = new CANBus("rio");
    private final TalonFX motorLeft;
    private final TalonFX motorRight;
    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;
    private StatusSignal<AngularVelocity> leftVelocity;
    private StatusSignal<AngularVelocity> rightVelocity;
    private StatusSignal<AngularAcceleration> leftAcceleration;
    private StatusSignal<AngularAcceleration> rightAcceleration;
    private StatusSignal<Voltage> leftAppliedVoltage;
    private StatusSignal<Voltage> rightAppliedVoltage;
    private StatusSignal<Current> leftAppliedCurrent;
    private StatusSignal<Current> rightAppliedCurrent;
    private StatusSignal<Integer> currentPidSlot;
    private Debouncer leftConnectedDebounce = new Debouncer(0.1);
    private Debouncer rightConnectedDebounce = new Debouncer(0.1);
    

    public ShooterIOTalonFX() {
        motorLeft = new TalonFX(Constants.CanId.SHOOTER_LEFT_CAN_ID, canBus);
        motorRight = new TalonFX(Constants.CanId.SHOOTER_RIGHT_CAN_ID, canBus);

        var motorLeftConfig = new TalonFXConfiguration();
        motorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorLeftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ShooterTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
        motorLeftConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ShooterTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
        motorLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ShooterTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
        motorLeftConfig.Slot0.kP = ShooterTalonFXConstants.VELOCITY_P_GAIN;
        motorLeftConfig.Slot0.kI = ShooterTalonFXConstants.VELOCITY_I_GAIN;
        motorLeftConfig.Slot0.kD = ShooterTalonFXConstants.VELOCITY_D_GAIN;
        motorLeftConfig.Slot0.kV = ShooterTalonFXConstants.VELOCITY_V_GAIN;
        motorLeftConfig.Slot0.kS = ShooterTalonFXConstants.VELOCITY_S_GAIN;
        
        var motorRightConfig = new TalonFXConfiguration();
        motorRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorRightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorRightConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ShooterTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
        motorRightConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ShooterTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
        motorRightConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ShooterTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
        motorRightConfig.Slot0.kP = ShooterTalonFXConstants.VELOCITY_P_GAIN;
        motorRightConfig.Slot0.kI = ShooterTalonFXConstants.VELOCITY_I_GAIN;
        motorRightConfig.Slot0.kD = ShooterTalonFXConstants.VELOCITY_D_GAIN;
        motorRightConfig.Slot0.kV = ShooterTalonFXConstants.VELOCITY_V_GAIN;
        motorRightConfig.Slot0.kS = ShooterTalonFXConstants.VELOCITY_S_GAIN;

        PhoenixUtil.tryUntilOk(5, () -> motorLeft.getConfigurator().apply(motorLeftConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> motorRight.setControl(new Follower(motorRight.getDeviceID(), MotorAlignmentValue.Opposed)));

    leftPosition = motorLeft.getPosition();
    rightPosition = motorRight.getPosition();

    leftVelocity = motorLeft.getVelocity();
    rightVelocity = motorRight.getVelocity();

    leftAcceleration = motorLeft.getAcceleration();
    rightAcceleration = motorRight.getAcceleration();

    leftAppliedVoltage = motorLeft.getMotorVoltage();
    rightAppliedVoltage = motorRight.getMotorVoltage();

    leftAppliedCurrent = motorLeft.getStatorCurrent();
    rightAppliedCurrent = motorRight.getStatorCurrent();

    currentPidSlot = motorLeft.getClosedLoopSlot();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        rightPosition,
        leftVelocity,
        rightVelocity,
        leftAcceleration,
        rightAcceleration,
        leftAppliedVoltage,
        rightAppliedVoltage,
        leftAppliedCurrent,
        rightAppliedCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(motorLeft, motorRight);
    }

    @Override
    public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
        var leftSignals =
        BaseStatusSignal.refreshAll(
            leftPosition,
            leftVelocity,
            leftAcceleration,
            leftAppliedVoltage,
            leftAppliedCurrent);
    var rightSignals =
        BaseStatusSignal.refreshAll(
            rightPosition,
            rightVelocity,
            rightAcceleration,
            rightAppliedVoltage,
            rightAppliedCurrent);

    inputs.motorRightPIDSlot = switch (currentPidSlot.getValue()) {
        case 0 -> PIDSlots.DEFAULT_VELOCITY;
        default -> throw new IllegalStateException("Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT" + currentPidSlot.getValue());
    };

    inputs.motorLeftPIDSlot = switch (currentPidSlot.getValue()) {
        case 0 -> PIDSlots.DEFAULT_VELOCITY;
        default -> throw new IllegalStateException("Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT" + currentPidSlot.getValue());
    };

    inputs.motorLeftConnected = leftConnectedDebounce.calculate(leftSignals.isOK());
    inputs.motorRightConnected = rightConnectedDebounce.calculate(rightSignals.isOK());
    inputs.motorLeftVelocity = leftVelocity.getValue();
    inputs.motorRightVelocity = rightVelocity.getValue();
    inputs.motorLeftAcceleration = leftAcceleration.getValue();
    inputs.motorRightAcceleration = rightAcceleration.getValue();
    inputs.motorLeftAppliedVolts = leftAppliedVoltage.getValue();
    inputs.motorRightAppliedVolts = rightAppliedVoltage.getValue();
    inputs.motorLeftCurrent = leftAppliedCurrent.getValue();
    inputs.motorRightCurrent = rightAppliedCurrent.getValue();
    }

    @Override
    public void setShooterVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
        // Set shooter velocity with PID slot implementation
    }

    @Override
    public void setShooterVelocity(AngularVelocity velocity) {
        // Set shooter velocity implementation
        setShooterVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
    }

    @Override
    public void setShooterOpenLoop(double percentOutput) {
        // Set shooter open loop implementation
    }

    @Override
    public void stop() {
        // Stop shooter implementation
    }

}
