/*
 * mltotem_ioctl.h
 *
 * Magic Leap Totem Input Driver
 *
 * Copyright (c) 2017-2018, Magic Leap, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __ML_TOTEM_IOCTL_H__
#define __ML_TOTEM_IOCTL_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

#define TOTEM_ID_1              0x01
#define TOTEM_ID_2              0x02
#define TOTEM_ID_ALL            0xFF

/**
 * enum mlt_led_cmd - type of LED command.
 * @MLT_LED_PLAY_CMD:           Packet cmd for playing a pattern.
 * @MLT_LED_SIMPLE_DYNAMIC_CMD: Packet cmd for simple dynamic pattern.
 * @MLT_LED_ARC_DRAW_CMD:       Packet cmd for arc draw.
 * @MLT_LED_STOP_CMD:           Packet cmd for stopping the running pattern.
 * @MLT_LED_PAUSE_CMD:          Packet cmd for pausing the running pattern.
 * @MLT_LED_RESUME_CMD:         Packet cmd for resuming the paused pattern.
 * @MLT_LED_CARDINAL_MASK_CMD:  Packet cmd for sending cardinal specific data.
 * @MLT_LED_MASK_CMD:           Packet cmd for lighting up individual LEDs
 *                              based on mask.
 * @MLT_LED_CLEAR_CMD:          Packet cmd for clearing/destroying cached
 *                              pattern on totem.
 */
enum mlt_led_cmd {
	MLT_LED_PLAY_CMD = 0,
	MLT_LED_SIMPLE_DYNAMIC_CMD,
	MLT_LED_ARC_DRAW_CMD,
	MLT_LED_STOP_CMD,
	MLT_LED_PAUSE_CMD,
	MLT_LED_RESUME_CMD,
	MLT_LED_CARDINAL_MASK_CMD,
	MLT_LED_MASK_CMD,
	MLT_LED_CLEAR_CMD,
};

/**
 * struct mlt_led_mask - LED mask.
 * @val:       LED mask. 1 bit-state per 1 LED.
 * @r:         PWM of red diode.
 * @g:         PWM of green diode.
 * @b:         PWM of blue diode.
 */
struct mlt_led_mask {
	uint16_t val;
	uint8_t  r;
	uint8_t  g;
	uint8_t  b;
} __packed;

/**
 * struct mlt_led_set_type_t - Set LED pattern.
 * @color_id:        Color ID of the color specified.
 * @arc_type:        The type of arc animation to be rendered.
 * @start_position:  Position at which the pattern has to start.
 * @effect_id:       Effect ID of the effect.
 * @speed:           Speed at which the animation has to be rendered.
 * @pattern_index:   Index assigned to the pattern.
 * @duration:        Duration for which the pattern is supposed to run.
 * @repeat:          Number of times the pattern has to repeat.
 */
struct mlt_led_pattern {
	uint8_t  color_id       : 6;
	uint8_t  arc_type       : 2;
	uint8_t  start_position : 4;
	uint8_t  effect_id      : 4;
	uint8_t  speed          : 2;
	uint8_t  pattern_index  : 6;
	uint16_t duration       : 13;
	uint16_t repeat         : 3;
} __packed;

/**
 * struct mlt_led_rgba -rgba color.
 * @a:         Brightness at which the color is to be rendered.
 * @r:         Red color component of the color.
 * @g:         Green color component of the color.
 * @b:         Blue color component of the oclor.
 */
struct mlt_led_rgba {
	uint8_t  a;
	uint8_t  r;
	uint8_t  g;
	uint8_t  b;
} __packed;

/**
 * struct mlt_arc_draw - draw simple arc.
 * @num_leds:        Number of leds to be lit in the arc pattern.
 * @start_led:       The position of the first led from where arc
 *                   draw has to begin.
 * @start_color:     Color at the beginning of the pattern.
 * @end_color:       Color at the end of the pattern.
 * @pattern_index:   Index assigned to the pattern.
 * @duration:        Duration for which the pattern is supposed to run.
 * @repeat:          Number of times the pattern has to repeat.
 */
struct mlt_arc_draw {
	uint8_t             num_leds      : 4;
	uint8_t             start_led     : 4;
	struct mlt_led_rgba start_color;
	struct mlt_led_rgba end_color;
	uint8_t             pattern_index : 6;
	uint16_t            duration      : 13;
	uint16_t            repeat        : 3;
} __packed;

/**
 * struct mlt_play_patt - play pattern by index.
 * @duration:        Duration for which the pattern is supposed to run.
 * @repeat:          Number of times the pattern has to repeat.
 * @index:           Index assigned to the pattern.
 */
struct mlt_play_patt {
	uint16_t duration : 13;
	uint16_t repeat   : 3;
	uint8_t  index;
} __packed;

/**
 * struct mlt_led - LED Halo command structure.
 * @led_cmd:           One of mlt_led_cmd.
 * @over_ride:         Denotes if any playing pattern needs to be stopped
 *                     instead of being queued before playing this new pattern.
 * @finite:            Denotes if pattern is playing finitely or infinitely.
 * @cardinal_mask:     Cardinal mask.
 * @led_mask:          LED mask.
 * @patt_index:        Pattern index.
 * @pattern:           Simple dynamic pattern data.
 * @arc:               Arc draw specific data.
 */
struct mlt_led {
	uint8_t led_cmd   : 6;
	uint8_t over_ride : 1;
	uint8_t finite    : 1;
	union {
		uint8_t                 cardinal_mask;
		uint16_t                led_mask;
		struct mlt_led_pattern  pattern;
		struct mlt_arc_draw     arc;
		struct mlt_play_patt    patt_index;
	} u;
} __packed;

/**
 * struct mlt_led_req - this type associates LED Halo command with totem(s).
 * @totem_id:                  Totem id. TOTEM_ID_ALL for both totems.
 * @led:                       LED Halo command.
 */
struct mlt_led_req {
	uint8_t        totem_id;
	struct mlt_led led;
} __packed;

/**
 * enum mlt_haptics_cmd - type of haptics command.
 * @MLT_HAPTICS_SET_CMD:       Enable haptics feedback.
 * @MLT_HAPTICS_STOP_CMD:      Disable haptics feedback.
 */
enum mlt_haptics_cmd {
	MLT_HAPTICS_SET_CMD  = 0x00,
	MLT_HAPTICS_STOP_CMD = 0x01,
};

/**
 * enum mlt_haptics_motor_mask - haptics motor selection.
 * @MLT_HAPTICS_BODY:          Body haptics motor.
 * @MLT_HAPTICS_TOUCH:         Touchpad haptics motor.
 */
enum mlt_haptics_motor_mask {
	MLT_HAPTICS_BODY  = 1U,
	MLT_HAPTICS_TOUCH = 2U,
};

/**
 * enum mlt_haptics_lib - haptics libraries.
 */
enum mlt_haptics_lib {
	MLT_HAPTICS_LIB_A = 0x00,
	MLT_HAPTICS_LIB_B = 0x01,
	MLT_HAPTICS_LIB_C = 0x02,
	MLT_HAPTICS_LIB_D = 0x03,
	MLT_HAPTICS_LIB_E = 0x04,
	MLT_HAPTICS_LIB_F = 0x05,
};

/**
 * struct mlt_haptics_drv2605 - TI DRV2605 haptics command structure.
 * @haptics_cmd:               One of mlt_haptics_cmd.
 * @motor_mask:                One of mlt_haptics_motor_mask.
 * @library:                   One of mlt_haptics_lib.
 * @patt_index:                Haptics pattern index.
 * @patt_repeat:               Number of times to play selected feedback.
 */
struct mlt_haptics_drv2605 {
	uint8_t haptics_cmd;
	uint8_t motor_mask;
	uint8_t library;
	uint8_t patt_index;
	uint8_t patt_repeat;
};

/**
 * struct mlt_haptics_aft14a901 - AFT14A901 haptics command structure.
 * @haptics_cmd:               One of mlt_haptics_cmd.
 * @pulse_num:                 Number of pulses to play.
 * @pulse_time:                "On" time of each pulse in micro-sec.
 */
struct mlt_haptics_aft14a901 {
	uint8_t  haptics_cmd;
	uint8_t  pulse_num;
	uint16_t pulse_time;
};

/**
 * enum mlt_haptics_drv - haptics driver.
 */
enum mlt_haptics_drv {
	MLT_HAPTICS_DRV2605    = 0x00,
	MLT_HAPTICS_AFT14A901  = 0x01,
};

/**
 * struct mlt_haptics - Generic haptics command structure.
 * @drv2605:               TI DRV2605 haptics cmd.
 * @aft14a901:             ALPS AFT14A901 haptics cmd.
 */
struct mlt_haptics {
	union {
		struct mlt_haptics_drv2605 drv2605;
		struct mlt_haptics_aft14a901 aft14a901;
	} __packed u;
} __packed;

/**
 * struct mlt_haptics_req - this type associates haptics command with totem(s).
 * @totem_id:                  Totem id. TOTEM_ID_ALL for both totems.
 * @driver:                    One of mlt_haptics_drv.
 * @haptics:                   Haptics command.
 */
struct mlt_haptics_req {
	uint8_t                totem_id;
	uint8_t                driver;
	struct mlt_haptics     haptics;
} __packed;

#define MLT_MAX_RAW_PAYLOAD_SZ 28

/**
 * struct raw_pkt - this structure associates raw packet with a totem.
 * @totem_id:                  Totem id.
 * @len:                       Raw data length.
 * @raw:                       Raw data.
 */
struct mlt_raw_pkt {
	uint8_t        totem_id;
	uint8_t        len;
	uint8_t        raw[MLT_MAX_RAW_PAYLOAD_SZ];
} __packed;

#define TOTEM_INPUT_EM_AGC_MAX 7

/**
 * struct mlt_agc_pkt - this structure associates EM AGC packet with a totem.
 * @totem_id:                  Totem id.
 * @agc:                       Requested EM AGC value.
 */
struct mlt_agc_req {
	uint8_t        totem_id;
	uint8_t        agc;
} __packed;

#define MLT_IOCTL_MAGIC        'T'

#define MLT_IOCTL_LED_SET      \
	_IOW(MLT_IOCTL_MAGIC, 0x00, struct mlt_led_req *)
#define MLT_IOCTL_HAPTICS_SET  \
	_IOW(MLT_IOCTL_MAGIC, 0x01, struct mlt_haptics_req *)
#define MLT_IOCTL_RAW_GET      \
	_IOR(MLT_IOCTL_MAGIC, 0x02, struct mlt_raw_pkt *)
#define MLT_IOCTL_RAW_SET      \
	_IOW(MLT_IOCTL_MAGIC, 0x03, struct mlt_raw_pkt *)
#define MLT_IOCTL_AGC_SET      \
	_IOW(MLT_IOCTL_MAGIC, 0x04, struct mlt_agc_req *)
#define MLT_IOCTL_AGC_GET      \
	_IOWR(MLT_IOCTL_MAGIC, 0x05, struct mlt_agc_req *)

#endif /* __ML_TOTEM_IOCTL_H__ */

