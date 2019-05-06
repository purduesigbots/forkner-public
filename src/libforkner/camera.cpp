#include <math.h>  //round
#include "main.h"
#define MIN_OBJ_SIZE 400
#define MIN_WIDTH 50
#define MIN_HEIGHT 30

std::shared_ptr<Camera> capCam;
pros::vision_signature_s_t RED_SIG;
pros::vision_signature_s_t BLUE_SIG;

#define RED_ID 1
#define BLUE_ID 2

Camera::Camera(uint8_t port, bool ired) : vis(port), red(ired) {
	exposure = 50;
	vis.set_exposure(exposure);

	RED_SIG = pros::Vision::signature_from_utility(RED_ID, 7367, 10037, 8702,
	                                               -2007, -689, -1348, 3.000, 0);
	BLUE_SIG = pros::Vision::signature_from_utility(BLUE_ID, -3537, -2185, -2861,
	                                                7561, 10845, 9203, 4.000, 0);

	vis.set_signature(RED_ID, &RED_SIG);
	vis.set_signature(BLUE_ID, &BLUE_SIG);
}

void Camera::setColor(bool ired) {
	red = ired;
}

bool Camera::capFlipped() {
	pros::vision_object_s_t obj;
	if (red) {
		obj = vis.get_by_sig(0, RED_ID);
	} else {
		obj = vis.get_by_sig(0, BLUE_ID);
	}
	printf(" %d %d %d\n", obj.signature, obj.width, obj.height);
	// check size, etc. ?
	if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > MIN_WIDTH &&
	    obj.height > MIN_HEIGHT) {
		printf("\n\nstopped\n");
		return true;
	}
	return false;
}

void Camera::editSensorParams(uint8_t sig_id) {
	okapi::Controller controller;
	float left_y_joystick = controller.getAnalog(okapi::ControllerAnalog::leftY);
	okapi::ControllerButton x_button = controller[okapi::ControllerDigital::X];
	okapi::ControllerButton y_button = controller[okapi::ControllerDigital::Y];
	okapi::ControllerButton a_button = controller[okapi::ControllerDigital::A];
	okapi::ControllerButton b_button = controller[okapi::ControllerDigital::B];
	okapi::ControllerButton down_button =
	    controller[okapi::ControllerDigital::down];

	while (!down_button.isPressed()) {
		pros::lcd::print(7, "Press Down to Exit.");
		if (b_button.changedToPressed()) {
			exposure = 50;
			vis.set_exposure(exposure);
			vis.set_white_balance(0xff);

			pros::vision_signature_s_t sig = vis.get_signature(sig_id);
			sig.range = 5.0;
			vis.set_signature(sig_id, &sig);
		} else if (y_button.isPressed()) {
			int32_t white_balance = vis.get_white_balance();

			if (left_y_joystick > 0.1 &&
			    white_balance <= 0xfe) {  // increase when pushed up
				if (white_balance <= 0xfe) white_balance += 0x01;
			} else if (left_y_joystick < -0.1 &&
			           white_balance >= 0x01) {  // decrease when pushed down
				white_balance -= 0x01;
			}

			vis.set_white_balance(white_balance);
		} else if (x_button.changedToPressed()) {
			if (left_y_joystick > 0.1 &&
			    exposure <=
			        149) {  // if exposure + 1 <= 150, to prevent out of range issues
				exposure += 1;
			} else if (left_y_joystick < -0.1 && exposure >= 1) {
				exposure -= 1;
			}
			/*
			Camera cam(11, true);

			while(true) {
			  cam.editSensorParams(master, 1); //RED_ID
			  pros::delay(100); //high delay for debugging purposes
			}
			*/
			vis.set_exposure(exposure);
		} else if (a_button.isPressed()) {
			pros::vision_signature_s_t sig = vis.get_signature(sig_id);

			if (left_y_joystick > 0.1 && sig.range <= 10.9) {
				sig.range += 0.1;
			} else if (left_y_joystick < -0.1 && sig.range >= 0.1) {
				sig.range -= 0.1;
			}

			sig.range = round(sig.range * 100) / 100;  // round to 2 decimal places
			vis.set_signature(sig_id, &sig);
		}

		showSigLogistics(sig_id);
		pros::delay(100);
	}
}

void Camera::showSigLogistics(uint8_t sig_id) {
	pros::vision_object_s_t object_arr[5];
	int32_t numObjects =
	    vis.read_by_sig(0, sig_id, 5, object_arr);  // read objects

	pros::lcd::print(0, "number of objects: %d", numObjects);

	if (numObjects > 0 && numObjects <= 5) {
		pros::vision_object_s_t obj = object_arr[0];
		pros::lcd::print(1, "Largest object: %dx%d", obj.width, obj.height);
	} else if (numObjects == 0) {
		pros::lcd::print(1, "no objects found");
	} else {
		pros::lcd::print(1, "error");
	}

	switch (editMode) {
		case EXPOSURE:
			pros::lcd::print(2, "Current Mode: Exposure");
			break;
		case WHITE_BALANCE:
			pros::lcd::print(2, "Current Mode: White Balance");
			break;
		case RANGE:
			pros::lcd::print(2, "Current Mode: Sig Range");
			break;
		case EXIT:
			pros::lcd::print(2, "Current Mode: Exit");
			break;
		default:
			pros::lcd::print(2, "Current Mode: Error");
	}

	pros::lcd::print(3, "brightness:%d/150", vis.get_exposure());
	pros::lcd::print(4, "white balance:0x%x", vis.get_white_balance());
	pros::vision_signature_s_t sig = vis.get_signature(sig_id);
	pros::lcd::print(5, "sig range:%.2f/11.0", sig.range);

	if (editMode == EXIT) {
		pros::lcd::print(7, " Reset              Select               Exit");
	} else {
		pros::lcd::print(7, "    -                 Select                 +");
	}
}

void Camera::editSensorParamsLCD(uint8_t sig_id) {
	while (true) {
		// update LCD
		showSigLogistics(sig_id);

		uint8_t button = pros::lcd::read_buttons();

		if (button == 2) {  // update settings
			editMode = (editMode + 1) % 4;
		} else if (editMode == EXPOSURE) {  // update params
			if (button == 1 && exposure <= 149) {
				exposure += 1;
			} else if (button == 4 && exposure >= 1) {
				exposure -= 1;
			}

			vis.set_exposure(exposure);
		} else if (editMode == WHITE_BALANCE) {
			int white = vis.get_white_balance();

			if (button == 1 && white <= 0xfe) {
				vis.set_white_balance(white + 0x01);
			} else if (button == 4 && white >= 0x01) {
				vis.set_white_balance(white - 0x01);
			}
		} else if (editMode == RANGE) {
			pros::vision_signature_s_t sig = vis.get_signature(sig_id);

			if (button == 1 && sig.range >= 0.1) {
				// increase range
				sig.range += 0.1;
				sig.range = round(sig.range * 100) / 100;  // round to 2 decimal places
				vis.set_signature(sig_id, &sig);
			} else if (button == 4 && sig.range <= 10.9) {
				// decrease range
				sig.range -= 0.1;
				sig.range = round(sig.range * 100) / 100;  // round to 2 decimal places
				vis.set_signature(sig_id, &sig);
			}
		} else if (editMode == EXIT) {
			if (button == 1) {  // exit
				break;
			} else if (button == 4) {  // reset
				exposure = 50;
				vis.set_exposure(exposure);
				vis.set_white_balance(0xff);

				pros::vision_signature_s_t sig = vis.get_signature(sig_id);
				sig.range = 5.0;
				vis.set_signature(sig_id, &sig);
			}
		}  // end if

		pros::delay(
		    175);  // high delay to prevent edit mode from updating too quickly
	}            // end while
}
