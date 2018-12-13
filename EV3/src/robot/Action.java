package robot;

import lejos.robotics.subsumption.Behavior;
/**
 * This class implements the leJOS Behavior interface and controls the grabber mechanism.
 * @author Jeremias Korhonen
 *
 */
public class Action implements Behavior{
	private Sensors sensor;
	private volatile boolean suppressed = false;
	/**
	 * A boolean variable, which is true after the action method is called
	 * This is used to make the action method run only once
	 */
	private volatile boolean kyydissa = false;
	
	public Action(Sensors sensor) {
		this.sensor = sensor;
	}
	/**
	 * This behavior takes control when an object is close enough to the IR sensor.
	 * @return True only once.
	 */
	@Override
	public boolean takeControl() {
		if (kyydissa) {
			return false;
		}
		return sensor.KohdeEdessa();
	}
	/**
	 * Grabs the object in front of it and sets a waypoint back.
	 */
	@Override
	public void action() {
		suppressed = false;
		try {
			sensor.asetaKohde(20, 100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		sensor.grab(-1000);
		kyydissa = true;
		
	}

	@Override
	public void suppress() {  
			suppressed = true; }

}
