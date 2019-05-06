/**
 * Scripts Header
 *
 * Declares the various autonomous scripts available for the bot.
 */
#ifndef _SCRIPTS_HPP_
#define _SCRIPTS_HPP_

#include "main.h"

#define MIN_FIRST_SHOT_DELAY 15
#define MIN_SECOND_SHOT_DELAY 38

namespace auton {
enum color { red = 0, blue = 1 };

void acConfig();
void caConfig();
void psConfig();
void defaultConfig();

/**
 * Goal: Shoot all alliance side flags, then all middle flags, then flip
 * ground cap and (maybe) fire at opposing alliance flag (just one of the high
 * flags).
 */
void allianceToCenter(color side, uint32_t allianceFireTime,
                      uint32_t centerFireTime);

/**
 * Generate the paths for the allianceToCenter script. Call this in initialize,
 * since this might take a little while.
 */
void allianceToCenterInit(color side);

/**
 * Goal: Center flags first, then alliance side flags
 */
void centerToAlliance(color side, uint32_t allianceFireTime,
                      uint32_t centerFireTime);

/**
 * Generate the paths for the centerToAlliance script. Call this in initialize,
 * since this might take a little while.
 */
void centerToAllianceInit(color side);

/**
 * Goal: Hit all of the flags, flip 5 caps, park. 26 points.
 */
void progSkills(color side, uint32_t ign1, uint32_t ign2);
void progSkillsBase(color side, uint32_t ign1, uint32_t ign2,
                    bool reckless = false);

void progSkillsReckless(color side, uint32_t allianceFireTime,
                        uint32_t centerFireTime);

/**
 * Generate the paths for the progSkill script. Call this in initialize, since
 * this might take a little while.
 */
void progSkillsInit(color side);

void defaultInit(color side);
/**
 * Shoots the preload and does nothing else. Used in case things start failing.
 * Line the bot up to the high alliance flag.
 */
void defaultScript(color side, uint32_t ign1, uint32_t ign2);
}  // namespace auton

#endif
