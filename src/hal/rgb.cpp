#include <Adafruit_NeoPixel.h>
#include <stdint.h>
#include <math.h>
#include "hal.h"

/*
 * Hardware configuration
 */
#define LED_COUNT	20	/* Total number of LEDs in the strip */
#define BRIGHTNESS	50	/* Default brightness level (0-255) */

/*
 * LED grouping definitions
 */
#define FRONT_START	0	/* First front light LED index */
#define FRONT_END	3	/* Last front light LED index */
#define REAR_START	4	/* First rear light LED index */
#define REAR_END		7	/* Last rear light LED index */
#define AMBIENT_LEFT_START	8	/* First left ambient LED index */
#define AMBIENT_LEFT_END	13	/* Last left ambient LED index */
#define AMBIENT_RIGHT_START	14	/* First right ambient LED index */
#define AMBIENT_RIGHT_END	19	/* Last right ambient LED index */



static Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, CONFIG_RGB_PIN, 
						   NEO_GRB + NEO_KHZ800);
static enum light_mode current_mode = MODE_STANDBY;
static unsigned long previous_millis = 0;
static uint16_t rainbow_cycle_counter = 0;

/* Function prototypes */
static void set_front_lights(uint32_t color);
static void set_rear_lights(uint32_t color);
static void set_left_ambient(uint32_t color);
static void set_right_ambient(uint32_t color);
static void breathing_ambient(uint32_t color, unsigned long current_millis);
static void rainbow_cycle(void);
static void all_off(void);
static void breathing_ambient_color_cycle(unsigned long current_millis);
static uint32_t wheel(byte wheel_pos);

/*
 * setup() - Initialize the LED strip
 */
void HAL::rgb_init(void)
{
	strip.begin();
	strip.show(); /* Turn off all LEDs initially */
	strip.setBrightness(BRIGHTNESS);
}

/*
 * loop() - Main control loop
 */
void HAL::rgb_update(void)
{
	unsigned long current_millis = millis();

	switch (current_mode) {
	case MODE_OFF:
		all_off();
		break;
	case MODE_STANDBY:
		/* Front lights white, rear lights off, ambient rainbow cycle */
		set_front_lights(strip.Color(255, 255, 255));
		set_rear_lights(strip.Color(0, 0, 0));
		// breathing_ambient(strip.Color(0, 0, 255), current_millis);
        breathing_ambient_color_cycle(current_millis);
		break;
	case MODE_DRIVING:
		/* Front white, rear red, ambient blue breathing */
		set_front_lights(strip.Color(255, 255, 255));
		// set_rear_lights(strip.Color(255, 0, 0));
        set_rear_lights(strip.Color(0, 0, 0));
		set_right_ambient(strip.Color(0, 0, 0));
        set_left_ambient(strip.Color(0, 0, 0));
		break;
	case MODE_BRAKING:
		/* Front white, rear red blinking, ambient red breathing */
		set_front_lights(strip.Color(255, 255, 255));
		if ((current_millis / 200) % 2 == 0) {
			set_rear_lights(strip.Color(255, 0, 0));
		} else {
			set_rear_lights(strip.Color(100, 0, 0));
		}
		set_right_ambient(strip.Color(0, 0, 0));
        set_left_ambient(strip.Color(0, 0, 0));
		break;
	case MODE_TURNING_LEFT:
		/* Left turn signal (amber blinking), right ambient off */
		set_front_lights(strip.Color(255, 255, 255));
		if ((current_millis / 500) % 2 == 0) {
			set_left_ambient(strip.Color(255, 150, 0));
		} else {
			set_left_ambient(strip.Color(0, 0, 0));
		}
		set_right_ambient(strip.Color(0, 0, 0));
		break;
	case MODE_TURNING_RIGHT:
		/* Right turn signal (amber blinking), left ambient off */
		set_front_lights(strip.Color(255, 255, 255));
		if ((current_millis / 500) % 2 == 0) {
			set_right_ambient(strip.Color(255, 150, 0));
		} else {
			set_right_ambient(strip.Color(0, 0, 0));
		}
		set_left_ambient(strip.Color(0, 0, 0));
		break;
	default:
		/* Should never reach here */
		break;
	}

	strip.show(); /* Update LED strip */
}

/*
 * set_front_lights() - Set color for all front lights
 * @color: 32-bit color value
 */
static void set_front_lights(uint32_t color)
{
	int i;

	for (i = FRONT_START; i <= FRONT_END; i++)
		strip.setPixelColor(i, color);
}

/*
 * set_rear_lights() - Set color for all rear lights
 * @color: 32-bit color value
 */
static void set_rear_lights(uint32_t color)
{
	int i;

	for (i = REAR_START; i <= REAR_END; i++)
		strip.setPixelColor(i, color);
}

/*
 * set_left_ambient() - Set color for left ambient lights
 * @color: 32-bit color value
 */
static void set_left_ambient(uint32_t color)
{
	int i;

	for (i = AMBIENT_LEFT_START; i <= AMBIENT_LEFT_END; i++)
		strip.setPixelColor(i, color);
}

/*
 * set_right_ambient() - Set color for right ambient lights
 * @color: 32-bit color value
 */
static void set_right_ambient(uint32_t color)
{
	int i;

	for (i = AMBIENT_RIGHT_START; i <= AMBIENT_RIGHT_END; i++)
		strip.setPixelColor(i, color);
}

/*
 * breathing_ambient() - Create breathing effect for ambient lights
 * @color: Base color for the effect
 * @current_millis: Current time in milliseconds for animation timing
 */
static void breathing_ambient(uint32_t color, unsigned long current_millis)
{
	float breath;
	uint8_t r, g, b;

	/* Calculate breathing intensity (0-1) */
	breath = (exp(sin(current_millis / 2000.0 * M_PI)) - 0.36787944) * 0.4254590641;

	/* Extract RGB components */
	r = (uint8_t)((color >> 16) & 0xff);
	g = (uint8_t)((color >> 8) & 0xff);
	b = (uint8_t)(color & 0xff);

	/* Apply breathing effect */
	r = (uint8_t)(r * breath);
	g = (uint8_t)(g * breath);
	b = (uint8_t)(b * breath);

	/* Set both left and right ambient lights */
	set_left_ambient(strip.Color(r, g, b));
	set_right_ambient(strip.Color(r, g, b));
}

/*
 * rainbow_cycle() - Create rainbow cycle effect on ambient lights
 */
static void rainbow_cycle(void)
{
	int i;
	uint16_t led_count = AMBIENT_LEFT_END - AMBIENT_LEFT_START + 1;

	for (i = AMBIENT_LEFT_START; i <= AMBIENT_LEFT_END; i++) {
		strip.setPixelColor(i, wheel(((i * 256 / led_count) + 
					     rainbow_cycle_counter) & 255));
	}

	for (i = AMBIENT_RIGHT_START; i <= AMBIENT_RIGHT_END; i++) {
		strip.setPixelColor(i, wheel(((i * 256 / led_count) + 
					     rainbow_cycle_counter) & 255));
	}

	rainbow_cycle_counter++;
	if (rainbow_cycle_counter >= 256)
		rainbow_cycle_counter = 0;
}

/*
 * all_off() - Turn off all LEDs
 */
static void all_off(void)
{
	int i;

	for (i = 0; i < LED_COUNT; i++)
		strip.setPixelColor(i, strip.Color(0, 0, 0));
}

/*
 * wheel() - Input a value 0-255 to get a color value
 * @wheel_pos: Position on the color wheel (0-255)
 *
 * The colours are a transition r - g - b - back to r.
 *
 * Return: 32-bit color value
 */
static uint32_t wheel(byte wheel_pos)
{
	wheel_pos = 255 - wheel_pos;

	if (wheel_pos < 85) {
		return strip.Color(255 - wheel_pos * 3, 0, wheel_pos * 3);
	} else if (wheel_pos < 170) {
		wheel_pos -= 85;
		return strip.Color(0, wheel_pos * 3, 255 - wheel_pos * 3);
	} else {
		wheel_pos -= 170;
		return strip.Color(wheel_pos * 3, 255 - wheel_pos * 3, 0);
	}
}


/*
 * set_light_mode() - Set the current light mode
 * @new_mode: The new light mode to set
 *
 * This function provides a safe interface to change the lighting mode
 * from external code. It performs basic validation on the input.
 */
void HAL::rgb_set_mode(enum light_mode new_mode)
{
	/* Validate input mode */
	if (new_mode >= MODE_OFF && new_mode <= MODE_TURNING_RIGHT) {
		current_mode = new_mode;
		
		/* Immediately update lights when mode changes */
		// previous_millis = millis();
		// strip.show();
	}
	/* Silently ignore invalid mode values */
}

void HAL::rgb_set_mode_by_status(float speed, float steering)
{
    enum light_mode new_mode = MODE_STANDBY;
    if (speed < 0) {
        new_mode = MODE_DRIVING;
    } else if (speed > 0) {
        new_mode = MODE_BRAKING;
    }

    if (steering > 0) {
        new_mode = MODE_TURNING_RIGHT;
    } else if (steering < 0) {
        new_mode = MODE_TURNING_LEFT;
    }

    rgb_set_mode(new_mode);
}

/*
 * get_light_mode() - Get the current light mode
 *
 * Return: The current light mode
 */
enum light_mode get_light_mode(void)
{
	return current_mode;
}


/*
 * Breathing color sequence - add to defines section
 */
#define BREATHING_COLORS_COUNT 6
static const uint32_t breathing_colors[BREATHING_COLORS_COUNT] = {
    strip.Color(255, 0, 0),     /* Red */
    strip.Color(255, 150, 0),   /* Amber */
    strip.Color(0, 255, 0),     /* Green */
    strip.Color(0, 255, 255),   /* Cyan */
    strip.Color(0, 0, 255),     /* Blue */
    strip.Color(255, 0, 255)    /* Magenta */
};
static uint8_t current_breathing_color = 0;

/*
 * breathing_ambient_color_cycle() - Color cycling breathing effect
 * @current_millis: Current time in milliseconds for animation timing
 */
static void breathing_ambient_color_cycle(unsigned long current_millis)
{
    float breath;
    uint32_t current_color = breathing_colors[current_breathing_color];
    uint8_t r, g, b;

    /* Calculate breathing intensity (0-1) */
    breath = (exp(sin(current_millis / 2000.0 * M_PI)) - 0.36787944) * 0.4254590641;

    /* Extract RGB components */
    r = (uint8_t)((current_color >> 16) & 0xff);
    g = (uint8_t)((current_color >> 8) & 0xff);
    b = (uint8_t)(current_color & 0xff);

    /* Apply breathing effect */
    r = (uint8_t)(r * breath);
    g = (uint8_t)(g * breath);
    b = (uint8_t)(b * breath);

    /* Set ambient lights */
    set_left_ambient(strip.Color(r, g, b));
    set_right_ambient(strip.Color(r, g, b));

    /* Change color when breath completes (at minimum intensity) */
    if (breath <= 0.1) {
        current_breathing_color = (current_breathing_color + 1) % BREATHING_COLORS_COUNT;
    }
}
