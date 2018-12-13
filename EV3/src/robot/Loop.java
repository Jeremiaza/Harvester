package robot;

import lejos.robotics.subsumption.Behavior;
/**
 * This class implements the leJOS Behavior interface and resets the robot for a new round
 * @author Jeremias Korhonen
 *
 */
public class Loop implements Behavior{
	private Sensors sensor;
	/**
	 * A boolean variable, which is true after the action method is called
	 * This is used to make the action method run only once
	 */
	private volatile boolean kierros=false;
	private volatile boolean suppressed = false;

	public Loop(Sensors sensor) {
		this.sensor = sensor;
		
	}
	/**
	 * Uses the Sensors class onkoPaassa() method to check, whether the robot is at the end of the map.
	 * @return True, if the robot is at the end of the map.
	 */
	@Override
	public boolean takeControl() {
		if (kierros) {
			return false;
		}
		return sensor.onkoPaassa();

	}
	/**
	 * Sets a waypoint back home and a boolean value, so we know to calculate the detected object's distance differently
	 */
	@Override
	public void action() {
		suppressed=false;
		if (!kierros) {
			try {
				sensor.asetaKohde(20, 200);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			sensor.Palaamassa();
			kierros=true;
		}
		
		
	}

	@Override
	public void suppress() { suppressed=true;
		
	}
}
