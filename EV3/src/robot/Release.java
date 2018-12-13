package robot;

import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.subsumption.Behavior;
/**
 * This class implements the leJOS Behavior interface and releases the object
 * @author Jeremias Korhonen
 *
 */
public class Release implements Behavior{
	private Sensors sensor;
	private volatile boolean suppressed = false;
	/**
	 * A boolean variable, which is true after the action method is called
	 * This is used to make the action method run only once
	 */
	private volatile boolean palautettu = false;
	public Release(Sensors sensor) {
		this.sensor = sensor;
		
	}
	/**
	 * Checks if the robot is back home with the object.
	 * @return True, if the robot has an object within its grasps and at the drop off point.
	 */
	@Override
	public boolean takeControl() {
		if (sensor.getX()>=19 && sensor.getX()<=21 && sensor.getY()>=99 && sensor.getY()<=101 && !palautettu) {
			return true;
		}
		return false;

	}

	@Override
	public void action() {
		suppressed=false;
		if (!palautettu) {
			sensor.grab(1000);
			System.out.println("Oleppa hyva :)");
			Waypoint point = new Waypoint(500,150);
			try {
				sensor.asetaWP(point);
			} catch (DestinationUnreachableException e) {
				e.printStackTrace();
			}
			palautettu = true;
		}
		/**
		 * tekee jotain emaa tiia
		 */
		
	}

	@Override
	public void suppress() { suppressed=true;
		
	}
}