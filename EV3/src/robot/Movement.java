package robot;

import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.subsumption.Behavior;
/**
 * This class implements the leJOS Behavior interface and controls the movement of the robot
 * @author Jeremias Korhonen
 *
 */
public class Movement implements Behavior{
	private Sensors sensor;
	private volatile boolean suppressed = false;
	
	public Movement(Sensors sensor) {
		this.sensor = sensor;
	}
	/**
	 * This is the lowest priority Behavior. Tells the robot to move to the assigned waypoint.
	 * @return Always returns true, unless a higher priority Behavior's takeControl() returns true
	 */
	@Override
    public boolean takeControl() { 
		return true; 
	}
	
	@Override
	public void suppress() { suppressed = true; }
	/**
	 * Commands the robot to move forward
	 * While this behavior is waiting for another to take over, Tutki() method is called for the camera to search the area.
	 */
	@Override
	public void action() {
		suppressed = false;
		try {
			sensor.RobotMene();
		} catch (DestinationUnreachableException e) {
			e.printStackTrace();
		}
		while(!suppressed) {
			if (!sensor.getKasky()) {
				sensor.setKasky(sensor.Tutki());
			}
			
			Thread.yield();
		}
	}

}
