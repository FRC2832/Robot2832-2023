
package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class T16000M extends GenericHID {
  
  
  /** Represents a digital button on an Saitek. */
  public enum Button {
    trigger(1),
    middle (2),
    left(3),
    right(4),
    leftSideUpLeft(5),
    leftSideUpMiddle(6),
    leftSideUpRight(7),
    leftSideDownRight(8),
    leftSideDownMiddle(9),
    leftSideDownLeft(10),
    rightSideUpLeft(11),
    rightSideUpMiddle(12),
    rightSideUpRight(13),
    rightSideDownRight(14),
    rightSideDownMiddle(15),
    rightSideDownLeft(16);
  
    public final int value;
    Button(int value) {
      this.value = value;
    }


  }
  public enum Axis {
    xAxis(0),
    yAxis(1),
    rotate(2),
    slider(3);

    public final int value;

    Axis(int value) {
      this.value = value;
    }


  }

  public T16000M(final int port) {
    super(port);
  }



  public double getxAxis1() {
    return getRawAxis(Axis.xAxis.value);
  }

  public double getyAxis1() {
    return getRawAxis(Axis.yAxis.value);
  }
  
  public double rotate() {
    return getRawAxis(Axis.rotate.value);
  }
  public double slider() {
    return getRawAxis(Axis.slider.value);
  }



  //Trigger Detection
  public boolean getTrigger() {
    return getRawButton(Button.trigger.value);
  }


  public boolean getTriggerPressed() {
    return getRawButtonPressed(Button.trigger.value);
  }


  public boolean getTriggerReleased() {
    return getRawButtonReleased(Button.trigger.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent trigger(EventLoop loop) {
    return new BooleanEvent(loop, this::getTrigger);
  }
  
  public boolean getMiddle() {
    return getRawButton(Button.middle.value);
  }


  public boolean getMiddlePressed() {
    return getRawButtonPressed(Button.middle.value);
  }


  public boolean getMiddleReleased() {
    return getRawButtonReleased(Button.middle.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent middle(EventLoop loop) {
    return new BooleanEvent(loop, this::getMiddle);
  }
  
  public boolean getLeft() {
    return getRawButton(Button.left.value);
  }


  public boolean getLeftPressed() {
    return getRawButtonPressed(Button.left.value);
  }


  public boolean getLeftReleased() {
    return getRawButtonReleased(Button.left.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent left(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeft);
  }

  public boolean getRight() {
    return getRawButton(Button.middle.value);
  }


  public boolean getRightPressed() {
    return getRawButtonPressed(Button.right.value);
  }


  public boolean getRightReleased() {
    return getRawButtonReleased(Button.right.value);
  }


  @SuppressWarnings("MethodName")
  public BooleanEvent right(EventLoop loop) {
    return new BooleanEvent(loop, this::getRight);
  }


  public boolean getLeftSideUpLeft() {
    return getRawButton(Button.leftSideUpLeft.value);
  }


  public boolean getLeftSideUpLeftPressed() {
    return getRawButtonPressed(Button.leftSideUpLeft.value);
  }


  public boolean getLeftSideUpLeftReleased() {
    return getRawButtonReleased(Button.leftSideUpLeft.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent leftSideUpLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftSideUpLeft);
  }
  
  public boolean getLeftSideUpMiddle() {
    return getRawButton(Button.leftSideUpMiddle.value);
  }


  public boolean getLeftSideUpMiddlePressed() {
    return getRawButtonPressed(Button.leftSideUpMiddle.value);
  }


  public boolean getLeftSideUpMiddleReleased() {
    return getRawButtonReleased(Button.leftSideUpMiddle.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent leftSideUpMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftSideUpMiddle);
  }
  
  public boolean getLeftSideUpRight() {
    return getRawButton(Button.leftSideUpRight.value);
  }


  public boolean getLeftSideUpRightPressed() {
    return getRawButtonPressed(Button.leftSideUpRight.value);
  }


  public boolean getLeftSideUpRightReleased() {
    return getRawButtonReleased(Button.leftSideUpMiddle.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent leftSideUpRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftSideUpRight);
  }

  public boolean getLeftSideDownRight() {
    return getRawButton(Button.leftSideDownRight.value);
  }


  public boolean getLeftSideDownRightPressed() {
    return getRawButtonPressed(Button.leftSideDownRight.value);
  }


  public boolean getLeftSideDownRightReleased() {
    return getRawButtonReleased(Button.leftSideDownRight.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent leftSideDownRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftSideDownRight);
  }

  public boolean getLeftSideDownMiddle() {
    return getRawButton(Button.leftSideDownMiddle.value);
  }


  public boolean getLeftSideDownMiddlePressed() {
    return getRawButtonPressed(Button.leftSideDownMiddle.value);
  }


  public boolean getLeftSideDownMiddleReleased() {
    return getRawButtonReleased(Button.leftSideDownMiddle.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent leftSideDownMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftSideDownMiddle);
  }

  public boolean getLeftSideDownLeft() {
    return getRawButton(Button.leftSideDownLeft.value);
  }


  public boolean getLeftSideDownLeftPressed() {
    return getRawButtonPressed(Button.leftSideDownLeft.value);
  }


  public boolean getLeftSideDownLeftReleased() {
    return getRawButtonReleased(Button.leftSideDownLeft.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent leftSideDownLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftSideDownLeft);
  }
  public boolean getRightSideUpLeft() {
    return getRawButton(Button.rightSideUpLeft.value);
  }


  public boolean getRightSideUpLeftPressed() {
    return getRawButtonPressed(Button.rightSideUpLeft.value);
  }


  public boolean getRightSideUpLeftReleased() {
    return getRawButtonReleased(Button.leftSideUpLeft.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent rightSideUpLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightSideUpLeft);
  }
  
  public boolean getRightSideUpMiddle() {
    return getRawButton(Button.leftSideUpMiddle.value);
  }


  public boolean getRightSideUpMiddlePressed() {
    return getRawButtonPressed(Button.rightSideUpMiddle.value);
  }


  public boolean getRightSideUpMiddleReleased() {
    return getRawButtonReleased(Button.rightSideUpMiddle.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent rightSideUpMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightSideUpMiddle);
  }
  
  public boolean getRightSideUpRight() {
    return getRawButton(Button.rightSideUpRight.value);
  }


  public boolean getRightSideUpRightPressed() {
    return getRawButtonPressed(Button.rightSideUpRight.value);
  }


  public boolean getRightSideUpRightReleased() {
    return getRawButtonReleased(Button.rightSideUpMiddle.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent rightSideUpRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightSideUpRight);
  }

  public boolean getRightSideDownRight() {
    return getRawButton(Button.rightSideDownRight.value);
  }


  public boolean getRightSideDownRightPressed() {
    return getRawButtonPressed(Button.rightSideDownRight.value);
  }


  public boolean getRightSideDownRightReleased() {
    return getRawButtonReleased(Button.rightSideDownRight.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent rightSideDownRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightSideDownRight);
  }

  public boolean getRightSideDownMiddle() {
    return getRawButton(Button.rightSideDownMiddle.value);
  }


  public boolean getRightSideDownMiddlePressed() {
    return getRawButtonPressed(Button.rightSideDownMiddle.value);
  }


  public boolean getRightSideDownMiddleReleased() {
    return getRawButtonReleased(Button.rightSideDownMiddle.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent rightSideDownMiddle(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightSideDownMiddle);
  }

  public boolean getRightSideDownLeft() {
    return getRawButton(Button.rightSideDownLeft.value);
  }


  public boolean getRightSideDownLeftPressed() {
    return getRawButtonPressed(Button.rightSideDownLeft.value);
  }


  public boolean getRightSideDownLeftReleased() {
    return getRawButtonReleased(Button.rightSideDownLeft.value);
  }
  
  @SuppressWarnings("MethodName")
  public BooleanEvent rightSideDownLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightSideDownLeft);
  }
}