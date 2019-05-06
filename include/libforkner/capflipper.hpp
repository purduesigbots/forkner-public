/**
 * Functionality for interfacing with the cap flipping mechanism
 */
#ifndef _CAP_FLIPPER_HPP_
#define _CAP_FLIPPER_HPP_

namespace flipper {
enum class state { RETRACTED, DOWN, UP, TIP, INVALID };

/**
 * Initializes the cap flipper motor and tares its position.
 */
void init();

/**
 * Asynchronously returns the cap flipper to a position near its starting
 * position.
 */
void retract();

/**
 * Retracts the cap flipper and blocks until the mechanism is at its desired
 * position.
 */
void retractBlocking();

/**
 * Asynchronously moves the cap flipper to its lowest possible position.
 */
void down();

/**
 * Moves the cap flipper to its lowest possible position and blocks until the
 * mechanism is at its desired position.
 */
void downBlocking();

/**
 * Asynchronously actuates the cap flipper upwards to flip a cap.
 */
void up();

/**
 * Actuates the cap flipper upwards to flip a cap an blocks until the mechanism
 * is at its desired position.
 */
void upBlocking();

/**
 * Holds the cap flipper at a height that will tip caps over when hitting them.
 * Does not delay until the movement is finished.
 */
void tip();

/**
 * Holds the cap flipper at a height that will tip caps over when hitting them.
 * Delays until the movement is finished.
 */
void tipBlocking();

/**
 * Returns the position that the cap flipper is set to.
 */
state getState();
}  // namespace flipper

#endif
