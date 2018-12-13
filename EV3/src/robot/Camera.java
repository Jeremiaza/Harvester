package robot;

import lejos.robotics.subsumption.Behavior;
/**
 * This class implements the leJOS Behavior interface and controls the camera
 * @author Jeremias Korhonen
 *
 */
public class Camera implements Behavior{
	private Sensors sensor;
	/**
	 * A boolean variable, which is true after the action method is called
	 * This is used to make the action method run only once
	 */
	private volatile boolean kohde = false;
	private volatile boolean suppressed = false;

	public Camera(Sensors sensor) {
		this.sensor = sensor;
		
	}
	/**
	 * This behavior takes over when the camera detects an object within the given color parameters
	 * @return Returns true only once when an object is detected
	 */
	@Override
    public boolean takeControl() {

		if (kohde) {
			return false;
		}
    	return sensor.getKasky();
	}
	
	@Override
    public void suppress() {
        suppressed = true;
    }
	/**
	 * Calls the laskeX and Y methods of the Sensors class and sets the location using the asetaKohde method
	 */
    @Override
    public void action() {
    	suppressed = false;
        double x = sensor.laskeX();
        double y = sensor.laskeY();
        if (x < 600 && y < 300) {
        	try {
				sensor.asetaKohde(x, y);
				kohde=true;
				System.out.println("Kohde asetettu "+x + ", " +y);
				sensor.nollaaKamera();
				
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
        } else {
        	System.out.println("Valitettavasti kohde on alueen ulkopuolella");
        }
    }
	
}
