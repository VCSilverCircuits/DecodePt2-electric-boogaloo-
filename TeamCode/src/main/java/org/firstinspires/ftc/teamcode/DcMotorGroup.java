package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorGroup extends DcMotorImplEx {
    DcMotorEx[] motors;

    public DcMotorGroup(DcMotorEx... motors) {
        super(motors[0].getController(), motors[0].getPortNumber());
        this.motors = motors;
    }

    /**
     * Sets the logical direction in which this motor operates.
     *
     * @param direction the direction to set for this motor
     * @see #getDirection()
     */
    @Override
    public void setDirection(Direction direction) {
        for (DcMotorEx motor : motors) {
            motor.setDirection(direction);
        }
    }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum
     * possible power / speed supported according to the run mode in which the
     * motor is operating.
     *
     * <p>Setting a power level of zero will brake the motor</p>
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        for (DcMotorEx motor : motors) {
            motor.setMotorType(motorType);
        }
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     * @see #setPower(double)
     */
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * target position, {@link #isBusy()} will return true.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(RunMode)
     * @see RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    @Override
    public void setTargetPosition(int position) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    /**
     * Sets the current run mode for this motor
     *
     * @param mode the new current run mode for this motor
     * @see RunMode
     * @see #getMode()
     */
    @Override
    public void setMode(RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    /**
     * Individually energizes this particular motor
     *
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorEnable() {
        for (DcMotorEx motor : motors) {
            motor.setMotorEnable();
        }
    }

    /**
     * Individually de-energizes this particular motor
     *
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorDisable() {
        for (DcMotorEx motor : motors) {
            motor.setMotorDisable();
        }
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired ticks per second
     */
    @Override
    public void setVelocity(double angularRate) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(angularRate);
        }
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired angular rate, in units per second
     * @param unit        the units in which angularRate is expressed
     * @see #getVelocity(AngleUnit)
     */
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(angularRate, unit);
        }
    }

    /**
     * Sets the target positioning tolerance of this motor
     *
     * @param tolerance the desired tolerance, in encoder ticks
     */
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPositionTolerance(tolerance);
        }
    }

    /**
     * Sets the current alert for this motor
     *
     * @param current current alert
     * @param unit    current units
     */
    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        for (DcMotorEx motor : motors) {
            motor.setCurrentAlert(current, unit);
        }
    }
}
