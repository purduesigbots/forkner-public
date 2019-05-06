/**
 * Vision Sensor Wrappers
 */
#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_


class Camera {
public:
	Camera(uint8_t port, bool ired);

	enum editModes {EXPOSURE = 0, WHITE_BALANCE = 1, RANGE = 2, EXIT = 3};

	void setColor(bool ired);

	bool capFlipped();

	/**
	 * Enables editing of various parameters while displaying logistics on the LCD
	 * Press Y and up on the left joystick to increase white balance and down to decrease white balance
	 * Press X and up on the left joystick to increase exposure and down to decrease exposure
	 * Press A and up on the left joystick to increase the sig range of sig_id and down to decrease the sig range
	 * Press B to set each value to their default values
	 *
	 * \param controller
	 * 		  The controller used to edit the params
	 * \param sig_id
	 * 		  The id of the signature whose range may be changed and whose information will be displayed on the LCD
	 */
	void editSensorParams(uint8_t sig_id);

	/**
	 * Checks buttons being pressed and edits the appropriate vision sensor parameter.
	 * Press the center button to change the edit mode
	 * In the EXPOSURE mode, press the right button to increase exposure, left to decrease
	 * In the WHITE_BALANCE mode, press the right button to increase white balance, left to decrease
	 * In the RANGE mode, press the right button to increase the sig range, left to decrease
	 * In the EXIT mode, press the right button to set each value to their default value, left to exit/return
	 * 
	 * \param sig_id
	 * 		  The id of the signature whose range may be changed and whose information will be displayed on the LCD
	 * 
	 */
	void editSensorParamsLCD(uint8_t sig_id);

	/**
	 * Shows vision data on the V5 LCD. Shows the # of objects visible with given sig,
	 * as well as the dimensions of the largest object and the current brightness
	 */
	void showSigLogistics(uint8_t sig_id);

private:
	bool red;
	pros::Vision vis;
	int32_t exposure;

	int8_t editMode = EXPOSURE;
};

extern std::shared_ptr<Camera> capCam;

#endif
