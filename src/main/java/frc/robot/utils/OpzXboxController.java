package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** 經過讀數大小優化的XboxController Class */
public class OpzXboxController extends CommandXboxController {
    private double minValue;

    /**
     * Construct an OpzXboxController class
     * @param port 連接埠編號
     * @param minValue 優化基準最小值
     */
    public OpzXboxController(int port, double minValue) {
        super(port);
        this.minValue = minValue;
    }

    /**
     * Update minimum value of opz datum
     * @param minValue
     */
    public void setMinValue(double minValue) {
        this.minValue = minValue;
    }

    private double opzAxisValue(double value) {
        return Math.abs(value) >= minValue ? value : 0;
    }

    /**
     * 
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     * 
     */
    @Override
    public double getLeftX() {
        return opzAxisValue(super.getLeftX());
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        return opzAxisValue(super.getRightX());
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        return opzAxisValue(super.getLeftY());
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        return opzAxisValue(super.getRightY());
    }

    /**
     * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    @Override
    public double getLeftTriggerAxis() {
        return opzAxisValue(super.getLeftTriggerAxis());
    }

    /**
     * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    @Override
    public double getRightTriggerAxis() {
        return opzAxisValue(super.getRightTriggerAxis());
    }
}
