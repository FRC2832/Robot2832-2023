package frc.robot.faults;

public interface IFault {

    /** How many counts do we go up per loop failure */
    int Weight = 1;

    /** How many loops does it take to set/clear the fault */
    int Goal = 10;

    /** Did the fault fail this loop */
    protected boolean IsFailed;

    /** Did the fault pass this loop */
    boolean IsPassed;

    /** Run the failsafe logic */
    void CheckFault();
}
