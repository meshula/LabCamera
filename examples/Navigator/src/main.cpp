
/*-----------------------------------------------------------------------------
    Demonstrating the use of lab::Camera with ImGui and sokol.
 */

#include "lab_sokol_config.h"

#include "LabCamera.h"
#include "LabCameraImgui.h"

#define SOKOL_TRACE_HOOKS
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_time.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_imgui.h"
#include "sokol_gfx_imgui.h"
#include "sokol_time.h"
#include "sokol_glue.h"
#include "sokol_gl.h"
#include "gizmo.h"
#include <tiny-gizmo.hpp>
#include <mutex>
#include <vector>

#define TRACE_INTERACTION 0


/**
 * Gamepad Input Library
 * Sean Middleditch <sean@middleditch.us>
 * Copyright (C) 2010,2011  Sean Middleditch
 * LICENSE: MIT/X
 * 
 * This version is sourced from a fork: https://github.com/elanthis/gamepad
 * which adds support for Linux, under the same license.
 */

#if !defined(GAMEPAD_H)
#define GAMEPAD_H 1

#define GAMEPAD_STATIC_LIB

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(GAMEPAD_STATIC_LIB)
#	define GAMEPAD_API
#else
#	if defined(_WIN32)
#		if defined(GAMEPAD_EXPORT)
#			define GAMEPAD_API __declspec(dllexport)
#		else
#			define GAMEPAD_API __declspec(dllimport)
#		endif
#	elif defined(__GNUC__) && defined(GAMEPAD_EXPORT)
#		define GAMEPAD_API __attribute__((visibility("default")))
#	else
#		define GAMEPAD_API extern
#	endif
#endif

    /**
     * Enumeration of the possible devices.
     *
     * Only four devices are supported as this is the limit of Windows.
     */
    enum GAMEPAD_DEVICE {
        GAMEPAD_0 = 0,	/**< First gamepad */
        GAMEPAD_1 = 1,	/**< Second gamepad */
        GAMEPAD_2 = 2,	/**< Third gamepad */
        GAMEPAD_3 = 3,	/**< Fourth gamepad */

        GAMEPAD_COUNT	/**< Maximum number of supported gamepads */
    };

    /**
     * Enumeration of the possible buttons.
     */
    enum GAMEPAD_BUTTON {
        BUTTON_DPAD_UP = 0,	/**< UP on the direction pad */
        BUTTON_DPAD_DOWN = 1,	/**< DOWN on the direction pad */
        BUTTON_DPAD_LEFT = 2,	/**< LEFT on the direction pad */
        BUTTON_DPAD_RIGHT = 3,	/**< RIGHT on the direction pad */
        BUTTON_START = 4,	/**< START button */
        BUTTON_BACK = 5,	/**< BACK button */
        BUTTON_LEFT_THUMB = 6,	/**< Left analog stick button */
        BUTTON_RIGHT_THUMB = 7,	/**< Right analog stick button */
        BUTTON_LEFT_SHOULDER = 8,	/**< Left bumper button */
        BUTTON_RIGHT_SHOULDER = 9,	/**< Right bumper button */
        BUTTON_A = 12,	/**< A button */
        BUTTON_B = 13,	/**< B button */
        BUTTON_X = 14,	/**< X button */
        BUTTON_Y = 15,	/**< Y button */

        BUTTON_COUNT					/**< Maximum number of supported buttons */
    };

    /**
     * Enumeration of the possible pressure/trigger buttons.
     */
    enum GAMEPAD_TRIGGER {
        TRIGGER_LEFT = 0,	/**< Left trigger */
        TRIGGER_RIGHT = 1,	/**< Right trigger */

        TRIGGER_COUNT			/**< Number of triggers */
    };

    /**
     * Enumeration of the analog sticks.
     */
    enum GAMEPAD_STICK {
        STICK_LEFT = 0,	/**< Left stick */
        STICK_RIGHT = 1,	/**< Right stick */

        STICK_COUNT		/**< Number of analog sticks */
    };

    /**
     * Enumeration of main stick directions.
     *
     * This is used for some of the convenience routines in the library.
     */
    enum GAMEPAD_STICKDIR {
        STICKDIR_CENTER = 0,	/**< CENTER, no direction */
        STICKDIR_UP = 1,	/**< UP direction */
        STICKDIR_DOWN = 2,	/**< DOWN direction */
        STICKDIR_LEFT = 3,	/**< LEFT direction */
        STICKDIR_RIGHT = 4,	/**< RIGHT direction */

        STICKDIR_COUNT
    };

    /**
     * Enumeration for true/false values
     */
    enum GAMEPAD_BOOL {
        GAMEPAD_FALSE = 0,	/**< FALSE value for boolean parameters */
        GAMEPAD_TRUE = 1		/**< TRUE value for boolean parameters */
    };

    typedef enum GAMEPAD_DEVICE GAMEPAD_DEVICE;
    typedef enum GAMEPAD_BUTTON GAMEPAD_BUTTON;
    typedef enum GAMEPAD_TRIGGER GAMEPAD_TRIGGER;
    typedef enum GAMEPAD_STICK GAMEPAD_STICK;
    typedef enum GAMEPAD_STICKDIR GAMEPAD_STICKDIR;
    typedef enum GAMEPAD_BOOL GAMEPAD_BOOL;

#define GAMEPAD_DEADZONE_LEFT_STICK		7849	/**< Suggested deadzone magnitude for left analog stick */
#define	GAMEPAD_DEADZONE_RIGHT_STICK	8689	/**< Suggested deadzone magnitude for right analog stick */
#define GAMEPAD_DEADZONE_TRIGGER		30		/**< Suggested deadzone for triggers */

    /**
     * Initialize the library.
     *
     * This is critical on non-Windows platforms.
     */
    GAMEPAD_API void GamepadInit(void);

    /**
     * Shutdown the library.
     *
     * This will release resources allocated by the library internally.
     *
     * This should be called after forking as well.
     */
    GAMEPAD_API void GamepadShutdown(void);

    /**
     * Updates the state of the gamepads.
     *
     * This must be called (at least) once per game loop.
     */
    GAMEPAD_API void GamepadUpdate(void);

    /**
     * Test if a particular gamepad is connected.
     *
     * \param device The device to check.
     * \returns GAMEPAD_TRUE if the device is connected, GAMEPAD_FALSE if it is not.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadIsConnected(GAMEPAD_DEVICE device);

    /**
     * Test if a particular button is being pressed.
     *
     * \param device The device to check.
     * \param button The button to check.
     * \returns GAMEPAD_TRUE if the button is down, GAMEPAD_FALSE if it is not.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadButtonDown(GAMEPAD_DEVICE device, GAMEPAD_BUTTON button);

    /**
     * Test if a particular button has been depressed since the previous call to GamepadUpdate.
     *
     * \param device The device to check.
     * \param button The button to check.
     * \returns GAMEPAD_TRUE if the button has been pressed, GAMEPAD_FALSE if it is not or if it was depressed the previous frame.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadButtonTriggered(GAMEPAD_DEVICE device, GAMEPAD_BUTTON button);

    /**
     * Test if a particular button has been released since the previous call to GamepadUpdate.
     *
     * \param device The device to check.
     * \param button The button to check.
     * \returns GAMEPAD_TRUE if the button has been released, GAMEPAD_FALSE if it is down or if it was not down the previous frame.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadButtonReleased(GAMEPAD_DEVICE device, GAMEPAD_BUTTON button);

    /**
     * Get the trigger value (depression magnitude) in its raw form.
     *
     * \param device The device to check.
     * \param trigger The trigger to check.
     * \returns Trigger depression magnitude (0 to 32767).
     */
    GAMEPAD_API int GamepadTriggerValue(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger);

    /**
     * Get the trigger value (depression magnitude) in normalized form.
     *
     * \param device The device to check.
     * \param trigger The trigger to check.
     * \returns Trigger depression magnitude (0 to 1).
     */
    GAMEPAD_API float GamepadTriggerLength(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger);

    /**
     * Test if a trigger is depressed
     *
     * \param device The device to check.
     * \param trigger The trigger to check.
     * \returns GAMEPAD_TRUE if down, GAMEPAD_FALSE otherwise.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadTriggerDown(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger);

    /**
     * Test if a trigger is depressed
     *
     * \param device The device to check.
     * \param trigger The trigger to check.
     * \returns GAMEPAD_TRUE if triggered, GAMEPAD_FALSE otherwise.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadTriggerTriggered(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger);

    /**
     * Test if a trigger is depressed
     *
     * \param device The device to check.
     * \param trigger The trigger to check.
     * \returns GAMEPAD_TRUE if released, GAMEPAD_FALSE otherwise.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadTriggerReleased(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger);

    /**
     * Set the rumble motors on/off.
     *
     * To turn off the rumble effect, set values to 0 for both motors.
     *
     * The left motor is the low-frequency/strong motor, and the right motor is the high-frequency/weak motor.
     *
     * \param device The device to update.
     * \param left Left motor strengh (0 to 1).
     * \param right Right motor strengh (0 to 1).
     */
    GAMEPAD_API void GamepadSetRumble(GAMEPAD_DEVICE device, float left, float right);

    /**
     * Query the position of an analog stick as raw values.
     *
     * The values retrieved by this function represent the magnitude of the analog
     * stick in each direction.  Note that it shouldn't be possible to get full
     * magnitude in one direction unless the other direction has a magnitude of
     * zero, as the stick has a circular movement range.
     *
     * \param device The device to check.
     * \param stick The stick to check.
     * \param outX Pointer to integer to store the X magnitude in (-32767 to 32767).
     * \param outX Pointer to integer to store the Y magnitude in (-32767 to 32767).
     */
    GAMEPAD_API void GamepadStickXY(GAMEPAD_DEVICE device, GAMEPAD_STICK stick, int* outX, int* outY);

    /**
     * Query the position of an analog stick as normalized values.
     *
     * The values retrieved by this function represent the magnitude of the analog
     * stick in each direction.  Note that it shouldn't be possible to get full
     * magnitude in one direction unless the other direction has a magnitude of
     * zero, as the stick has a circular movement range.
     *
     * \param device The device to check.
     * \param stick The stick to check.
     * \param outX Pointer to float to store the X magnitude in (-1 to 1).
     * \param outX Pointer to float to store the Y magnitude in (-1 to 1).
     */
    GAMEPAD_API void GamepadStickNormXY(GAMEPAD_DEVICE device, GAMEPAD_STICK stick, float* outX, float* outY);

    /**
     * Query the magnitude of an analog stick.
     *
     * This returns the normalized value of the magnitude of the stick.  That is,
     * if the stick is pushed all the way in any direction, it returns 1.0.
     *
     * \param device The device to check.
     * \param stick The stick to check.
     * \returns The magnitude of the stick (0 to 1).
     */
    GAMEPAD_API float GamepadStickLength(GAMEPAD_DEVICE device, GAMEPAD_STICK stick);

    /**
     * Query the direction of a stick (in radians).
     *
     * This returns the direction of the stick.  This value is in radians, not
     * degrees.  Zero is to the right, and the angle increases in a
     * counter-clockwise direction.
     *
     * \param device The device to check.
     * \param stick The stick to check.
     * \returns The angle of the stick (0 to 2*PI).
     */
    GAMEPAD_API float GamepadStickAngle(GAMEPAD_DEVICE device, GAMEPAD_STICK stick);

    /**
     * Get the direction the stick is pushed in (if any).
     *
     * This is a useful utility function for when the stick should be treated as a simple
     * directional pad, such as for menu UIs.
     *
     * \param device The device to check.
     * \param stick The trigger to check.
     * \returns The stick's current direction.
     */
    GAMEPAD_API GAMEPAD_STICKDIR GamepadStickDir(GAMEPAD_DEVICE device, GAMEPAD_STICK stick);

    /**
     * Test whether a stick has been pressed in a particular direction since the last update.
     *
     * This only returns true if the stick was centered last frame.
     *
     * This is a useful utility function for when the stick should be treated as a simple
     * directional pad, such as for menu UIs.
     *
     * \param device The device to check.
     * \param stick The trigger to check.
     * \param stickdir The direction to check for.
     * \returns GAMEPAD_TRUE if the stick is pressed in the specified direction, GAMEPAD_FALSE otherwise.
     */
    GAMEPAD_API GAMEPAD_BOOL GamepadStickDirTriggered(GAMEPAD_DEVICE device, GAMEPAD_STICK stick, GAMEPAD_STICKDIR dir);

#if defined(__cplusplus)
} /* extern "C" */
#endif





/**
 * Gamepad Input Library
 * Sean Middleditch
 * Copyright (C) 2010  Sean Middleditch
 * LICENSE: MIT/X
 */

#include <math.h>
#include <string.h>
#include <errno.h>
#include <malloc.h>

#define GAMEPAD_EXPORT 1
//#include "gamepad.h"

 /* Platform-specific includes */
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#	undef UNICODE
#	include "windows.h"
#	include "xinput.h"
#	pragma comment(lib, "xinput.lib")
#elif defined(__linux__)
#	include <linux/joystick.h>
#	include <stdio.h>
#	include <fcntl.h>
#	include <unistd.h>
#	include <libudev.h>
#else
#	error "Unknown platform in gamepad.c"
#endif

#define BUTTON_TO_FLAG(b) (1 << (b))

/* Axis information */
typedef struct GAMEPAD_AXIS GAMEPAD_AXIS;
struct GAMEPAD_AXIS {
    int x, y;
    float nx, ny;
    float length;
    float angle;
    GAMEPAD_STICKDIR dirLast, dirCurrent;
};

/* Trigger value information */
typedef struct GAMEPAD_TRIGINFO GAMEPAD_TRIGINFO;
struct GAMEPAD_TRIGINFO {
    int value;
    float length;
    GAMEPAD_BOOL pressedLast, pressedCurrent;
};

/* Structure for state of a particular gamepad */
typedef struct GAMEPAD_STATE GAMEPAD_STATE;
struct GAMEPAD_STATE {
    GAMEPAD_AXIS stick[STICK_COUNT];
    GAMEPAD_TRIGINFO trigger[TRIGGER_COUNT];
    int bLast, bCurrent, flags;
#if defined(__linux__)
    char* device;
    int fd;
    int effect;
#endif
};

/* State of the four gamepads */
static GAMEPAD_STATE STATE[4];

/* Note whether a gamepad is currently connected */
#define FLAG_CONNECTED	(1<<0)
#define FLAG_RUMBLE		(1<<1)

/* Prototypes for utility functions */
static void GamepadResetState(GAMEPAD_DEVICE gamepad);
static void GamepadUpdateCommon(void);
static void GamepadUpdateDevice(GAMEPAD_DEVICE gamepad);
static void GamepadUpdateStick(GAMEPAD_AXIS* axis, float deadzone);
static void GamepadUpdateTrigger(GAMEPAD_TRIGINFO* trig);

/* Various values of PI */
#define PI_1_4	0.78539816339744f
#define PI_1_2	1.57079632679489f
#define PI_3_4	2.35619449019234f
#define PI		3.14159265358979f

/* Platform-specific implementation code */
#if defined(_WIN32)

void GamepadInit(void) {
    int i;
    for (i = 0; i != GAMEPAD_COUNT; ++i) {
        STATE[i].flags = 0;
    }
}

void GamepadUpdate(void) {
    GamepadUpdateCommon();
}

static void GamepadUpdateDevice(GAMEPAD_DEVICE gamepad) {
    XINPUT_STATE xs;
    if (XInputGetState(gamepad, &xs) == 0) {
        /* reset if the device was not already connected */
        if ((STATE[gamepad].flags & FLAG_CONNECTED) == 0) {
            GamepadResetState(gamepad);
        }

        /* mark that we are connected w/ rumble support */
        STATE[gamepad].flags |= FLAG_CONNECTED | FLAG_RUMBLE;

        /* update state */
        STATE[gamepad].bCurrent = xs.Gamepad.wButtons;
        STATE[gamepad].trigger[TRIGGER_LEFT].value = xs.Gamepad.bLeftTrigger;
        STATE[gamepad].trigger[TRIGGER_RIGHT].value = xs.Gamepad.bRightTrigger;
        STATE[gamepad].stick[STICK_LEFT].x = xs.Gamepad.sThumbLX;
        STATE[gamepad].stick[STICK_LEFT].y = xs.Gamepad.sThumbLY;
        STATE[gamepad].stick[STICK_RIGHT].x = xs.Gamepad.sThumbRX;
        STATE[gamepad].stick[STICK_RIGHT].y = xs.Gamepad.sThumbRY;
    }
    else {
        /* disconnected */
        STATE[gamepad].flags &= ~FLAG_CONNECTED;
    }
}

void GamepadShutdown(void) {
    /* no Win32 shutdown required */
}

void GamepadSetRumble(GAMEPAD_DEVICE gamepad, float left, float right) {
    if ((STATE[gamepad].flags & FLAG_RUMBLE) != 0) {
        XINPUT_VIBRATION vib;
        memset(&vib, 0, sizeof(vib));
        vib.wLeftMotorSpeed = (WORD)(left * 65535);
        vib.wRightMotorSpeed = (WORD)(right * 65535);
        XInputSetState(gamepad, &vib);
    }
}

#elif defined(__linux__)

/* UDev handles */
static struct udev* UDEV = NULL;
static struct udev_monitor* MON = NULL;

static void GamepadAddDevice(const char* devPath);
static void GamepadRemoveDevice(const char* devPath);

/* Helper to add a new device */
static void GamepadAddDevice(const char* devPath) {
    int i;

    /* try to find a free controller */
    for (i = 0; i != GAMEPAD_COUNT; ++i) {
        if ((STATE[i].flags & FLAG_CONNECTED) == 0) {
            break;
        }
    }
    if (i == GAMEPAD_COUNT) {
        return;
    }

    /* copy the device path */
    STATE[i].device = strdup(devPath);
    if (STATE[i].device == NULL) {
        return;
    }

    /* reset device state */
    GamepadResetState(i);

    /* attempt to open the device in read-write mode, which we need fo rumble */
    STATE[i].fd = open(STATE[i].device, O_RDWR | O_NONBLOCK);
    if (STATE[i].fd != -1) {
        STATE[i].flags = FLAG_CONNECTED | FLAG_RUMBLE;
        return;
    }

    /* attempt to open in read-only mode if access was denied */
    if (errno == EACCES) {
        STATE[i].fd = open(STATE[i].device, O_RDONLY | O_NONBLOCK);
        if (STATE[i].fd != -1) {
            STATE[i].flags = FLAG_CONNECTED;
            return;
        }
    }

    /* could not open the device at all */
    free(STATE[i].device);
    STATE[i].device = NULL;
}

/* Helper to remove a device */
static void GamepadRemoveDevice(const char* devPath) {
    int i;
    for (i = 0; i != GAMEPAD_COUNT; ++i) {
        if (STATE[i].device != NULL && strcmp(STATE[i].device, devPath) == 0) {
            if (STATE[i].fd != -1) {
                close(STATE[i].fd);
                STATE[i].fd = -1;
            }
            free(STATE[i].device);
            STATE[i].device = 0;
            STATE[i].flags = 0;
            break;
        }
    }
}

void GamepadInit(void) {
    struct udev_list_entry* devices;
    struct udev_list_entry* item;
    struct udev_enumerate* enu;
    int i;

    /* initialize connection state */
    for (i = 0; i != GAMEPAD_COUNT; ++i) {
        STATE[i].flags = 0;
        STATE[i].fd = STATE[i].effect = -1;
    }

    /* open the udev handle */
    UDEV = udev_new();
    if (UDEV == NULL) {
        /* FIXME: flag error? */
        return;
    }

    /* open monitoring device (safe to fail) */
    MON = udev_monitor_new_from_netlink(UDEV, "udev");
    /* FIXME: flag error if hot-plugging can't be supported? */
    if (MON != NULL) {
        udev_monitor_enable_receiving(MON);
        udev_monitor_filter_add_match_subsystem_devtype(MON, "input", NULL);
    }

    /* enumerate joypad devices */
    enu = udev_enumerate_new(UDEV);
    udev_enumerate_add_match_subsystem(enu, "input");
    udev_enumerate_scan_devices(enu);
    devices = udev_enumerate_get_list_entry(enu);

    udev_list_entry_foreach(item, devices) {
        const char* name;
        const char* sysPath;
        const char* devPath;
        struct udev_device* dev;

        name = udev_list_entry_get_name(item);
        dev = udev_device_new_from_syspath(UDEV, name);
        sysPath = udev_device_get_syspath(dev);
        devPath = udev_device_get_devnode(dev);

        if (sysPath != NULL && devPath != NULL && strstr(sysPath, "/js") != 0) {
            GamepadAddDevice(devPath);
        }

        udev_device_unref(dev);
    }

    /* cleanup */
    udev_enumerate_unref(enu);
}

void GamepadUpdate(void) {
    if (MON != NULL) {
        fd_set r;
        struct timeval tv;
        int fd = udev_monitor_get_fd(MON);

        /* set up a poll on the udev device */
        FD_ZERO(&r);
        FD_SET(fd, &r);

        tv.tv_sec = 0;
        tv.tv_usec = 0;

        select(fd + 1, &r, 0, 0, &tv);

        /* test if we have a device change */
        if (FD_ISSET(fd, &r)) {
            struct udev_device* dev = udev_monitor_receive_device(MON);
            if (dev) {
                const char* devNode = udev_device_get_devnode(dev);
                const char* sysPath = udev_device_get_syspath(dev);
                const char* action = udev_device_get_action(dev);
                sysPath = udev_device_get_syspath(dev);
                action = udev_device_get_action(dev);

                if (strstr(sysPath, "/js") != 0) {
                    if (strcmp(action, "remove") == 0) {
                        GamepadRemoveDevice(devNode);
                    }
                    else if (strcmp(action, "add") == 0) {
                        GamepadAddDevice(devNode);
                    }
                }

                udev_device_unref(dev);
            }
        }
    }

    GamepadUpdateCommon();
}

static void GamepadUpdateDevice(GAMEPAD_DEVICE gamepad) {
    if (STATE[gamepad].flags & FLAG_CONNECTED) {
        struct js_event je;
        while (read(STATE[gamepad].fd, &je, sizeof(je)) > 0) {
            int button;
            switch (je.type) {
            case JS_EVENT_BUTTON:
                /* determine which button the event is for */
                switch (je.number) {
                case 0: button = BUTTON_A; break;
                case 1: button = BUTTON_B; break;
                case 2: button = BUTTON_X; break;
                case 3: button = BUTTON_Y; break;
                case 4: button = BUTTON_LEFT_SHOULDER; break;
                case 5: button = BUTTON_RIGHT_SHOULDER; break;
                case 6: button = BUTTON_BACK; break;
                case 7: button = BUTTON_START; break;
                case 8: button = 0; break; /* XBOX button  */
                case 9: button = BUTTON_LEFT_THUMB; break;
                case 10: button = BUTTON_RIGHT_THUMB; break;
                default: button = 0; break;
                }

                /* set or unset the button */
                if (je.value) {
                    STATE[gamepad].bCurrent |= BUTTON_TO_FLAG(button);
                }
                else {
                    STATE[gamepad].bCurrent ^= BUTTON_TO_FLAG(button);
                }

                break;
            case JS_EVENT_AXIS:
                /* normalize and store the axis */
                switch (je.number) {
                case 0:	STATE[gamepad].stick[STICK_LEFT].x = je.value; break;
                case 1:	STATE[gamepad].stick[STICK_LEFT].y = -je.value; break;
                case 2:	STATE[gamepad].trigger[TRIGGER_LEFT].value = (je.value + 32768) >> 8; break;
                case 3:	STATE[gamepad].stick[STICK_RIGHT].x = je.value; break;
                case 4:	STATE[gamepad].stick[STICK_RIGHT].y = -je.value; break;
                case 5:	STATE[gamepad].trigger[TRIGGER_RIGHT].value = (je.value + 32768) >> 8; break;
                case 6:
                    if (je.value == -32767) {
                        STATE[gamepad].bCurrent |= BUTTON_TO_FLAG(BUTTON_DPAD_LEFT);
                        STATE[gamepad].bCurrent &= ~BUTTON_TO_FLAG(BUTTON_DPAD_RIGHT);
                    }
                    else if (je.value == 32767) {
                        STATE[gamepad].bCurrent |= BUTTON_TO_FLAG(BUTTON_DPAD_RIGHT);
                        STATE[gamepad].bCurrent &= ~BUTTON_TO_FLAG(BUTTON_DPAD_LEFT);
                    }
                    else {
                        STATE[gamepad].bCurrent &= ~BUTTON_TO_FLAG(BUTTON_DPAD_LEFT) & ~BUTTON_TO_FLAG(BUTTON_DPAD_RIGHT);
                    }
                    break;
                case 7:
                    if (je.value == -32767) {
                        STATE[gamepad].bCurrent |= BUTTON_TO_FLAG(BUTTON_DPAD_UP);
                        STATE[gamepad].bCurrent &= ~BUTTON_TO_FLAG(BUTTON_DPAD_DOWN);
                    }
                    else if (je.value == 32767) {
                        STATE[gamepad].bCurrent |= BUTTON_TO_FLAG(BUTTON_DPAD_DOWN);
                        STATE[gamepad].bCurrent &= ~BUTTON_TO_FLAG(BUTTON_DPAD_UP);
                    }
                    else {
                        STATE[gamepad].bCurrent &= ~BUTTON_TO_FLAG(BUTTON_DPAD_UP) & ~BUTTON_TO_FLAG(BUTTON_DPAD_DOWN);
                    }
                    break;
                default: break;
                }

                break;
            default:
                break;
            }
        }
    }
}

void GamepadShutdown(void) {
    int i;

    /* cleanup udev */
    udev_monitor_unref(MON);
    udev_unref(UDEV);

    /* cleanup devices */
    for (i = 0; i != GAMEPAD_COUNT; ++i) {
        if (STATE[i].device != NULL) {
            free(STATE[i].device);
        }

        if (STATE[i].fd != -1) {
            close(STATE[i].fd);
        }
    }
}

void GamepadSetRumble(GAMEPAD_DEVICE gamepad, float left, float right) {
    if (STATE[gamepad].fd != -1) {
        struct input_event play;

        /* delete any existing effect */
        if (STATE[gamepad].effect != -1) {
            /* stop the effect */
            play.type = EV_FF;
            play.code = STATE[gamepad].effect;
            play.value = 0;

            write(STATE[gamepad].fd, (const void*)&play, sizeof(play));

            /* delete the effect */
            ioctl(STATE[gamepad].fd, EVIOCRMFF, STATE[gamepad].effect);
        }

        /* if rumble parameters are non-zero, start the new effect */
        if (left != 0.f || right != 0.f) {
            struct ff_effect ff;

            /* define an effect for this rumble setting */
            ff.type = FF_RUMBLE;
            ff.id = -1;
            ff.u.rumble.strong_magnitude = (unsigned short)(left * 65535);
            ff.u.rumble.weak_magnitude = (unsigned short)(right * 65535);
            ff.replay.length = 5;
            ff.replay.delay = 0;

            /* upload the effect */
            if (ioctl(STATE[gamepad].fd, EVIOCSFF, &ff) != -1) {
                STATE[gamepad].effect = ff.id;
            }

            /* play the effect */
            play.type = EV_FF;
            play.code = STATE[gamepad].effect;
            play.value = 1;

            write(STATE[gamepad].fd, (const void*)&play, sizeof(play));
        }
    }
}

#else /* !defined(_WIN32) && !defined(__linux__) */

#	error "Unknown platform in gamepad.c"

#endif /* end of platform implementations */

GAMEPAD_BOOL GamepadIsConnected(GAMEPAD_DEVICE device) {
    return (STATE[device].flags & FLAG_CONNECTED) != 0 ? GAMEPAD_TRUE : GAMEPAD_FALSE;
}

GAMEPAD_BOOL GamepadButtonDown(GAMEPAD_DEVICE device, GAMEPAD_BUTTON button) {
    return (STATE[device].bCurrent & BUTTON_TO_FLAG(button)) != 0 ? GAMEPAD_TRUE : GAMEPAD_FALSE;
}

GAMEPAD_BOOL GamepadButtonTriggered(GAMEPAD_DEVICE device, GAMEPAD_BUTTON button) {
    return ((STATE[device].bLast & BUTTON_TO_FLAG(button)) == 0 &&
        (STATE[device].bCurrent & BUTTON_TO_FLAG(button)) != 0) ? GAMEPAD_TRUE : GAMEPAD_FALSE;
}

GAMEPAD_BOOL GamepadButtonReleased(GAMEPAD_DEVICE device, GAMEPAD_BUTTON button) {
    return ((STATE[device].bCurrent & BUTTON_TO_FLAG(button)) == 0 &&
        (STATE[device].bLast & BUTTON_TO_FLAG(button)) != 0) ? GAMEPAD_TRUE : GAMEPAD_FALSE;
}

int GamepadTriggerValue(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger) {
    return STATE[device].trigger[trigger].value;
}

float GamepadTriggerLength(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger) {
    return STATE[device].trigger[trigger].length;
}

GAMEPAD_BOOL GamepadTriggerDown(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger) {
    return STATE[device].trigger[trigger].pressedCurrent;
}

GAMEPAD_BOOL GamepadTriggerTriggered(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger) {
    return (STATE[device].trigger[trigger].pressedCurrent &&
        !STATE[device].trigger[trigger].pressedLast) ? GAMEPAD_TRUE : GAMEPAD_FALSE;
}

GAMEPAD_BOOL GamepadTriggerReleased(GAMEPAD_DEVICE device, GAMEPAD_TRIGGER trigger) {
    return (!STATE[device].trigger[trigger].pressedCurrent &&
        STATE[device].trigger[trigger].pressedLast) ? GAMEPAD_TRUE : GAMEPAD_FALSE;
}

void GamepadStickXY(GAMEPAD_DEVICE device, GAMEPAD_STICK stick, int* outX, int* outY) {
    *outX = STATE[device].stick[stick].x;
    *outY = STATE[device].stick[stick].y;
}

float GamepadStickLength(GAMEPAD_DEVICE device, GAMEPAD_STICK stick) {
    return STATE[device].stick[stick].length;
}

void GamepadStickNormXY(GAMEPAD_DEVICE device, GAMEPAD_STICK stick, float* outX, float* outY) {
    *outX = STATE[device].stick[stick].nx;
    *outY = STATE[device].stick[stick].ny;
}

float GamepadStickAngle(GAMEPAD_DEVICE device, GAMEPAD_STICK stick) {
    return STATE[device].stick[stick].angle;
}

GAMEPAD_STICKDIR GamepadStickDir(GAMEPAD_DEVICE device, GAMEPAD_STICK stick) {
    return STATE[device].stick[stick].dirCurrent;
}

GAMEPAD_BOOL GamepadStickDirTriggered(GAMEPAD_DEVICE device, GAMEPAD_STICK stick, GAMEPAD_STICKDIR dir) {
    return (STATE[device].stick[stick].dirCurrent == dir &&
        STATE[device].stick[stick].dirCurrent != STATE[device].stick[stick].dirLast) ? GAMEPAD_TRUE : GAMEPAD_FALSE;
}

/* initialize common gamepad state */
static void GamepadResetState(GAMEPAD_DEVICE gamepad) {
    memset(STATE[gamepad].stick, 0, sizeof(STATE[gamepad].stick));
    memset(STATE[gamepad].trigger, 0, sizeof(STATE[gamepad].trigger));
    STATE[gamepad].bLast = STATE[gamepad].bCurrent = 0;
}

/* Update individual sticks */
static void GamepadUpdateCommon(void) {
    int i;
    for (i = 0; i != GAMEPAD_COUNT; ++i) {
        /* store previous button state */
        STATE[i].bLast = STATE[i].bCurrent;

        /* per-platform update routines */
        GamepadUpdateDevice((GAMEPAD_DEVICE)i);

        /* calculate refined stick and trigger values */
        if ((STATE[i].flags & FLAG_CONNECTED) != 0) {
            GamepadUpdateStick(&STATE[i].stick[STICK_LEFT], GAMEPAD_DEADZONE_LEFT_STICK);
            GamepadUpdateStick(&STATE[i].stick[STICK_RIGHT], GAMEPAD_DEADZONE_RIGHT_STICK);

            GamepadUpdateTrigger(&STATE[i].trigger[TRIGGER_LEFT]);
            GamepadUpdateTrigger(&STATE[i].trigger[TRIGGER_RIGHT]);
        }
    }
}

/* Update stick info */
static void GamepadUpdateStick(GAMEPAD_AXIS* axis, float deadzone) {
    // determine magnitude of stick
    axis->length = sqrtf((float)(axis->x * axis->x) + (float)(axis->y * axis->y));

    if (axis->length > deadzone) {
        // clamp length to maximum value
        if (axis->length > 32767.0f) {
            axis->length = 32767.0f;
        }

        // normalized X and Y values
        axis->nx = axis->x / axis->length;
        axis->ny = axis->y / axis->length;

        // adjust length for deadzone and find normalized length
        axis->length -= deadzone;
        axis->length /= (32767.0f - deadzone);

        // find angle of stick in radians
        axis->angle = atan2f((float)axis->y, (float)axis->x);
    }
    else {
        axis->x = axis->y = 0;
        axis->nx = axis->ny = 0.0f;
        axis->length = axis->angle = 0.0f;
    }

    /* update the stick direction */
    axis->dirLast = axis->dirCurrent;
    axis->dirCurrent = STICKDIR_CENTER;

    /* check direction to see if it's non-centered */
    if (axis->length != 0.f) {
        if (axis->angle >= PI_1_4 && axis->angle < PI_3_4) {
            axis->dirCurrent = STICKDIR_UP;
        }
        else if (axis->angle >= -PI_3_4 && axis->angle < -PI_1_4) {
            axis->dirCurrent = STICKDIR_DOWN;
        }
        else if (axis->angle >= PI_3_4 || axis->angle < -PI_3_4) {
            axis->dirCurrent = STICKDIR_LEFT;
        }
        else /* if (axis->angle < PI_1_4 && axis->angle >= -PI_1_4) */ {
            axis->dirCurrent = STICKDIR_RIGHT;
        }
    }
}

/* Update trigger info */
static void GamepadUpdateTrigger(GAMEPAD_TRIGINFO* trig) {
    trig->pressedLast = trig->pressedCurrent;

    if (trig->value > GAMEPAD_DEADZONE_TRIGGER) {
        trig->length = ((trig->value - GAMEPAD_DEADZONE_TRIGGER) / (255.0f - GAMEPAD_DEADZONE_TRIGGER));
        trig->pressedCurrent = GAMEPAD_TRUE;
    }
    else {
        trig->value = 0;
        trig->length = 0.0f;
        trig->pressedCurrent = GAMEPAD_FALSE;
    }
}


















#endif








































/*-----------------------------------------------------------------------------
    Utility
 */

static bool intersect_ray_plane(const lab::camera::Ray& ray, const lab::camera::v3f& point, const lab::camera::v3f& normal,
    lab::camera::v3f* intersection = nullptr, float* outT = nullptr)
{
    const float PLANE_EPSILON = 0.001f;
    const float d = ray.dir.x * normal.x + ray.dir.y * normal.y + ray.dir.z * normal.z;

    // Make sure we're not parallel to the plane
    if (std::abs(d) > PLANE_EPSILON)
    {
        float w = normal.x * point.x + normal.y * point.y + normal.z * point.z;
        w = -w;

        float distance = ray.pos.x * normal.x + ray.pos.y * normal.y + ray.pos.z * normal.z + w;
        float t = -distance / d;

        if (t >= PLANE_EPSILON)
        {
            if (outT) *outT = t;
            if (intersection)
            {
                lab::camera::v3f result = ray.pos;
                result.x += t * ray.dir.x;
                result.y += t * ray.dir.y;
                result.z += t * ray.dir.z;
                *intersection = result;
            }
            return true;
        }
    }
    if (outT) *outT = std::numeric_limits<float>::max();
    return false;
}

/*-----------------------------------------------------------------------------
    Application State
 */

enum class UIStateMachine
{
    None = 0, UI, Gizmo, DeltaCamera, TTLCamera
};

const char* name_state(UIStateMachine s)
{
    switch (s)
    {
    case UIStateMachine::None: return "None";
    case UIStateMachine::UI: return "UI";
    case UIStateMachine::Gizmo: return "Gizmo";
    case UIStateMachine::DeltaCamera: return "DeltaCamera";
    case UIStateMachine::TTLCamera: return "TTLCamera";
    }
    return "";
}

struct MouseState
{
    float initial_mousex{ 0 };          // set when mouse transitions from not dragging to dragging
    float initial_mousey{ 0 };
    float mousex{ 0 };                  // current mouse position in window space
    float mousey{ 0 };
    bool  click_initiated{ false };     // true only on the frame when the mouse transitioned from not dragging to dragging
    bool  dragging{ false };            // true as long as the button is held
    bool  click_ended{ false };         // true only on the frame when the mouse transitioned from dragging to not dragging
};

void mouse_state_update(MouseState* ms,
                        float mousex, float mousey, bool left_button_down)
{
    ms->click_ended = ms->dragging && !left_button_down;
    ms->click_initiated = !ms->dragging && left_button_down;
    ms->dragging = left_button_down;
    ms->mousex = mousex;
    ms->mousey = mousey;

    if (ms->click_initiated)
    {
        ms->initial_mousex = mousex;
        ms->initial_mousey = mousey;
    };

    if (TRACE_INTERACTION && ms->click_ended)
        printf("button released\n");
    if (TRACE_INTERACTION && ms->click_initiated)
        printf("button clicked\n");
}

struct AppState
{
    // camera and application state
    lab::camera::Camera camera;
    lab::camera::v3f initial_hit_point;
    lab::camera::PanTiltController main_pan_tilt;
    lab::camera::PanTiltController joystick_pan_tilt;
    UIStateMachine ui_state = UIStateMachine::UI;
    uint64_t last_time = 0;
    MouseState mouse;
    LCNav_PanelState* navigator_panel = nullptr;

    // gizmo state
    tinygizmo::m44f gizmo_transform;
    tinygizmo::gizmo_application_state gizmo_state;
    tinygizmo::gizmo_context gizmo_ctx;

    // UI elements and flags
    bool show_navigator = true;
    bool show_look_at = true;
    bool show_state = true;
    bool show_view_plane_intersect = false;
    bool show_manip_plane_intersect = false;
    bool quit = false;

    // sokol
    sgl_pipeline gl_pipelne;
    sg_imgui_t sg_imgui;
    sg_pass_action pass_action;
} gApp;


bool update_mouseStatus_in_current_imgui_window(MouseState* mouse)
{
    // assuming the 3d viewport is the current window, fetch the content region
    ImGuiWindow* win = ImGui::GetCurrentWindow();
    ImRect edit_rect = win->ContentRegionRect;
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 imgui_cursor_pos = ImGui::GetCursorPos();
    ImGui::SetCursorScreenPos(edit_rect.Min);

    // detect that the mouse is in the content region
    bool click_finished = ImGui::InvisibleButton("###GIZMOREGION", edit_rect.GetSize());
    bool in_canvas = click_finished || ImGui::IsItemHovered();

    ImVec2 mouse_pos = io.MousePos - ImGui::GetCurrentWindow()->Pos;
    mouse_state_update(mouse, mouse_pos.x, mouse_pos.y, io.MouseDown[0] && io.MouseDownOwned[0]);

    // restore the ImGui state
    ImGui::SetCursorPos(imgui_cursor_pos);
    return in_canvas;
}

/*-----------------------------------------------------------------------------
    Miscellaneous graphics helpers
 */

static void start_gl_rendering()
{
    static std::once_flag once;
    std::call_once(once, []()
    {
        /* setup sokol-gl */
        sgl_desc_t sgl_desc;
        memset(&sgl_desc, 0, sizeof(sgl_desc_t));
        sgl_desc.sample_count = sapp_sample_count();
        sgl_setup(&sgl_desc);

        /* a pipeline object with less-equal depth-testing */
        sg_pipeline_desc sg_p;
        memset(&sg_p, 0, sizeof(sg_pipeline_desc));
        sg_p.depth_stencil.depth_write_enabled = true;
        sg_p.depth_stencil.depth_compare_func = SG_COMPAREFUNC_LESS_EQUAL;
        gApp.gl_pipelne = sgl_make_pipeline(&sg_p);
    });

    sgl_defaults();
    sgl_push_pipeline();
    sgl_load_pipeline(gApp.gl_pipelne);
}

static void end_gl_rendering()
{
    sgl_pop_pipeline();
    sgl_draw();
}

static void draw_grid(float y, const lab::camera::m44f& m)
{
    sgl_matrix_mode_projection();

    lab::camera::m44f proj = gApp.camera.perspective();
    sgl_load_matrix(&proj.x.x);

    lab::camera::m44f view = gApp.camera.mount.gl_view_transform();
    lab::camera::m44f mv = gApp.camera.mount.model_view_transform(m);

    sgl_matrix_mode_modelview();
    sgl_load_matrix(&mv.x.x);

    sgl_c3f(1.0f, 0.0f, 1.0f);

    const float num = 32;
    const float step = 1.0f;
    sgl_begin_lines();
    for (float x = -num; x < num; x += step) {
        sgl_v3f(x, y, -num * step);
        sgl_v3f(x, y,  num * step);
    }
    for (float z = -num; z < num; z += step) {
        sgl_v3f(-num * step, y, z);
        sgl_v3f( num * step, y, z);
    }
    sgl_end();
}

static void draw_jack(float s, const lab::camera::m44f& m)
{
    lab::camera::m44f proj = gApp.camera.perspective();
    sgl_matrix_mode_projection();
    sgl_load_matrix(&proj.x.x);

    lab::camera::m44f mv = gApp.camera.mount.model_view_transform(&m.x.x);
    sgl_matrix_mode_modelview();
    sgl_load_matrix(&mv.x.x);

    sgl_begin_lines();
    sgl_c3f( 1,  0,  0);
    sgl_v3f(-s,  0,  0);
    sgl_v3f( s,  0,  0);
    sgl_c3f( 0,  1,  0);
    sgl_v3f( 0, -s,  0);
    sgl_v3f( 0,  s,  0);
    sgl_c3f( 0,  0,  1);
    sgl_v3f( 0,  0, -s);
    sgl_v3f( 0,  0,  s);
    sgl_end();
}



#if 1



static const char* button_names[] = {
    "d-pad up",
    "d-pad down",
    "d-pad left",
    "d-pad right",
    "start",
    "back",
    "left thumb",
    "right thumb",
    "left shoulder",
    "right shoulder",
    "???",
    "???",
    "A",
    "B",
    "X",
    "Y"
};

static int line = 0;

static void logevent(const char* format, ...) {
    va_list va;

    va_start(va, format);
    vprintf(format, va);
    va_end(va);

    if (++line == 14) {
        line = 0;
    }
}

static void update(GAMEPAD_DEVICE dev) {
    float lx, ly, rx, ry;

    if (!GamepadIsConnected(dev)) {
        return;
    }

    GamepadStickNormXY(dev, STICK_LEFT, &lx, &ly);
    GamepadStickNormXY(dev, STICK_RIGHT, &rx, &ry);

    printf("%d) L:(%+.3f,%+.3f :: %+.3f,%+.3f) R:(%+.3f, %+.3f :: %+.3f,%+.3f) LT:%+.3f RT:%+.3f ",
        dev,
        lx, ly,
        GamepadStickAngle(dev, STICK_LEFT),
        GamepadStickLength(dev, STICK_LEFT),
        rx, ry,
        GamepadStickAngle(dev, STICK_RIGHT),
        GamepadStickLength(dev, STICK_RIGHT),
        GamepadTriggerLength(dev, TRIGGER_LEFT),
        GamepadTriggerLength(dev, TRIGGER_RIGHT));
    printf("U:%d D:%d L:%d R:%d ",
        GamepadButtonDown(dev, BUTTON_DPAD_UP),
        GamepadButtonDown(dev, BUTTON_DPAD_DOWN),
        GamepadButtonDown(dev, BUTTON_DPAD_LEFT),
        GamepadButtonDown(dev, BUTTON_DPAD_RIGHT));
    printf("A:%d B:%d X:%d Y:%d Bk:%d St:%d ",
        GamepadButtonDown(dev, BUTTON_A),
        GamepadButtonDown(dev, BUTTON_B),
        GamepadButtonDown(dev, BUTTON_X),
        GamepadButtonDown(dev, BUTTON_Y),
        GamepadButtonDown(dev, BUTTON_BACK),
        GamepadButtonDown(dev, BUTTON_START));
    printf("LB:%d RB:%d LS:%d RS:%d\n",
        GamepadButtonDown(dev, BUTTON_LEFT_SHOULDER),
        GamepadButtonDown(dev, BUTTON_RIGHT_SHOULDER),
        GamepadButtonDown(dev, BUTTON_LEFT_THUMB),
        GamepadButtonDown(dev, BUTTON_RIGHT_THUMB));
}

int joy_main() {
    int i, j, k;
    bool rumble = false;

    GamepadInit();

    while (true) {
        GamepadUpdate();

        if (rumble) {
            for (i = 0; i != GAMEPAD_COUNT; ++i) {
                GamepadSetRumble((GAMEPAD_DEVICE)i, 0.25f, 0.25f);
            }
        }

        update(GAMEPAD_0);
        update(GAMEPAD_1);
        update(GAMEPAD_2);
        update(GAMEPAD_3);

        for (i = 0; i != GAMEPAD_COUNT; ++i) {
            GAMEPAD_DEVICE dev_i = static_cast<GAMEPAD_DEVICE>(i);
            if (GamepadIsConnected(dev_i)) {
                for (j = 0; j != BUTTON_COUNT; ++j) {
                    GAMEPAD_BUTTON dev_j = static_cast<GAMEPAD_BUTTON>(j);
                    if (GamepadButtonTriggered(dev_i, dev_j)) {
                        logevent("[%d] button triggered: %s", i, button_names[j]);
                    }
                    else if (GamepadButtonReleased(dev_i, dev_j)) {
                        logevent("[%d] button released:  %s", i, button_names[j]);
                    }
                }
                for (j = 0; j != TRIGGER_COUNT; ++j) {
                    GAMEPAD_TRIGGER dev_j = static_cast<GAMEPAD_TRIGGER>(j);
                    if (GamepadTriggerTriggered(dev_i, dev_j)) {
                        logevent("[%d] trigger pressed:  %d", i, j);
                    }
                    else if (GamepadTriggerReleased(dev_i, dev_j)) {
                        logevent("[%d] trigger released: %d", i, j);
                    }
                }
                for (j = 0; j != STICK_COUNT; ++j) {
                    GAMEPAD_STICK dev_j = static_cast<GAMEPAD_STICK>(j);
                    for (k = 0; k != STICKDIR_COUNT; ++k) {
                        GAMEPAD_STICKDIR dev_k = static_cast<GAMEPAD_STICKDIR>(j);
                        if (GamepadStickDirTriggered(dev_i, dev_j, dev_k)) {
                            logevent("[%d] stick direction:  %d -> %d", i, j, k);
                        }
                    }
                }
            }
        }

    }

    return 0;
}










#endif




void initialize_graphics()
{
    // &&&

#if 0
    glfwInit(); // initialize joysticks
    printf("Number of joysticks: %d\n", _win32.winmm.joyGetNumDevs());

    for (int i = 0; i < GLFW_JOYSTICK_LAST; ++i)
    {
        printf("Joy %d %s present: %s\n", i, glfwGetJoystickName(i), glfwJoystickPresent(i)? "Y" : "N");
    }

    int button_count = -1;
    const unsigned char* results = glfwGetJoystickButtons(0, &button_count);
    printf("button %d\n", button_count);
    for (int i = 0; i < 16; ++i)
    {
        printf("%d %d\n", i, (int)results[i]);
    }
#endif

#if 1

    GamepadInit();
    gApp.joystick_pan_tilt.set_speed(0.1f, 0.05f);

#endif


    gApp.navigator_panel = create_navigator_panel();

    // setup sokol-gfx, sokol-time and sokol-imgui
    sg_desc desc = { };
    desc.context = sapp_sgcontext();
    sg_setup(&desc);
    stm_setup();

    // setup debug inspection header(s)
    sg_imgui_init(&gApp.sg_imgui);

    // setup sokol-imgui, but provide our own font
    simgui_desc_t simgui_desc = { };
    simgui_desc.no_default_font = true;
    simgui_desc.sample_count = sapp_sample_count();
    simgui_desc.dpi_scale = sapp_dpi_scale();
    simgui_setup(&simgui_desc);

    // initial clear color
    gApp.pass_action.colors[0].action = SG_ACTION_CLEAR;
    gApp.pass_action.colors[0].val[0] = 0.0f;
    gApp.pass_action.colors[0].val[1] = 0.5f;
    gApp.pass_action.colors[0].val[2] = 0.7f;
    gApp.pass_action.colors[0].val[3] = 1.0f;

    uint8_t* data = nullptr;
    int32_t width = 0;
    int32_t height = 0;

    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->GetTexDataAsRGBA32(&data, &width, &height);

    // Upload new font texture atlas
    unsigned char* font_pixels;
    int font_width, font_height;
    io.Fonts->GetTexDataAsRGBA32(&font_pixels, &font_width, &font_height);
    sg_image_desc img_desc = { };
    img_desc.width = font_width;
    img_desc.height = font_height;
    img_desc.pixel_format = SG_PIXELFORMAT_RGBA8;
    img_desc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
    img_desc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
    img_desc.min_filter = SG_FILTER_LINEAR;
    img_desc.mag_filter = SG_FILTER_LINEAR;
    img_desc.content.subimage[0][0].ptr = font_pixels;
    img_desc.content.subimage[0][0].size = font_width * font_height * 4;
    io.Fonts->TexID = (ImTextureID)(uintptr_t)sg_make_image(&img_desc).id;
}

void shutdown_graphics() 
{
    simgui_shutdown();
    sg_imgui_discard(&gApp.sg_imgui);
    sgl_shutdown();
    sg_shutdown();

    release_navigator_panel(gApp.navigator_panel);
    gApp.navigator_panel = nullptr;
}

/*-----------------------------------------------------------------------------
    Run the gizmo which is tied to a virtual plane to help exercise various
    ray-intersection methods in lab::Camera
 */

struct GizmoTriangles
{
    GizmoTriangles()
    {
        indices.push_back(0);
        vertices.push_back(0);
    }

    int triangle_count = 0;
    std::vector<uint32_t> indices;
    std::vector<float>    vertices;
};
static GizmoTriangles gizmo_triangles;

// return true if the gizmo was interacted
bool run_gizmo(MouseState* ms, float width, float height)
{
    auto& cmt = gApp.camera.mount.transform();
    lab::camera::v3f camera_pos = cmt.position;
    lab::camera::Ray ray = gApp.camera.get_ray_from_pixel({ ms->mousex, ms->mousey }, { 0, 0 }, { width, height });
    lab::camera::quatf camera_orientation = cmt.orientation;

    gApp.gizmo_state.mouse_left = ms->dragging;
    gApp.gizmo_state.viewport_size = tinygizmo::v2f{ width, height };
    gApp.gizmo_state.cam.near_clip = gApp.camera.optics.znear;
    gApp.gizmo_state.cam.far_clip = gApp.camera.optics.zfar;
    gApp.gizmo_state.cam.yfov = gApp.camera.vertical_FOV().value;
    gApp.gizmo_state.cam.position = tinygizmo::v3f{ camera_pos.x, camera_pos.y, camera_pos.z };
    gApp.gizmo_state.cam.orientation = tinygizmo::v4f{ camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w };
    gApp.gizmo_state.ray_origin = tinygizmo::v3f{ ray.pos.x, ray.pos.y, ray.pos.z };
    gApp.gizmo_state.ray_direction = tinygizmo::v3f{ ray.dir.x, ray.dir.y, ray.dir.z };
    //gApp.gizmo_state.screenspace_scale = 80.f; // optional flag to draw the gizmos at a constant screen-space scale

    gApp.gizmo_ctx.begin(gApp.gizmo_state);

    static tinygizmo::rigid_transform xform_a;
    static tinygizmo::rigid_transform xform_a_last;
    static std::once_flag once;
    std::call_once(once, []() 
    {
        tinygizmo::m44f tx = xform_a.matrix();
        memcpy(&gApp.gizmo_transform, &tx, sizeof(float) * 16);
        xform_a_last = xform_a;
    });

    bool result = gApp.gizmo_ctx.transform_gizmo("first-example-gizmo", xform_a);
    if (result)
    {
        //std::cout << get_local_time_ns() << " - " << "First Gizmo Hovered..." << std::endl;
        //if (xform_a != xform_a_last) std::cout << get_local_time_ns() << " - " << "First Gizmo Changed..." << std::endl;
        xform_a_last = xform_a;
        tinygizmo::m44f tx = xform_a_last.matrix();
        memcpy(&gApp.gizmo_transform, &tx, sizeof(float) * 16);
    }

    // update index buffer
    gizmo_triangles.triangle_count = gApp.gizmo_ctx.triangles(nullptr, 0);
    if (gizmo_triangles.triangle_count > gizmo_triangles.indices.size())
        gizmo_triangles.indices.resize((size_t)gizmo_triangles.triangle_count * 3);

    gApp.gizmo_ctx.triangles(gizmo_triangles.indices.data(), gizmo_triangles.triangle_count);

    constexpr int vertex_float_count = 10;
    constexpr int vertex_byte_stride = sizeof(float) * vertex_float_count;

    // update vertex buffer
    int vertex_count = gApp.gizmo_ctx.vertices(nullptr, vertex_byte_stride, 0, 0, 0);
    int required_floats = vertex_float_count * vertex_count;
    if (required_floats > (int) gizmo_triangles.vertices.size())
        gizmo_triangles.vertices.resize((size_t)required_floats);

    gApp.gizmo_ctx.vertices(gizmo_triangles.vertices.data(),
        vertex_byte_stride,
        //0                 // position offset @TODO
        sizeof(float) * 3,  // normal offset
        sizeof(float) * 6,  // color offset
        vertex_count);

    gApp.gizmo_ctx.end(gApp.gizmo_state);
    return result;
}


/*-----------------------------------------------------------------------------
    Application logic
 */

void run_application_logic()
{
    using lab::camera::m44f;
    using lab::camera::v2f;
    using lab::camera::v3f;

    //--- Joystick
    //
    GamepadUpdate();
    v2f left_stick = { 0,0 };
    v2f right_stick = { 0,0 };

    int pad_index = GAMEPAD_COUNT;
    for (pad_index = 0; pad_index < GAMEPAD_COUNT; ++pad_index) 
    {
        if (GamepadIsConnected(static_cast<GAMEPAD_DEVICE>(pad_index))) {
            break;
        }
    }

    if (pad_index < GAMEPAD_COUNT)
    {
        GAMEPAD_DEVICE dev = static_cast<GAMEPAD_DEVICE>(pad_index);
        GamepadStickNormXY(dev, STICK_LEFT, &left_stick.x, &left_stick.y);
        GamepadStickNormXY(dev, STICK_RIGHT, &right_stick.x, &right_stick.y);
    }

    //--- set up projection and camera
    //
    const int window_width = sapp_width();
    const int window_height = sapp_height();
    const float w = (float)sapp_width();
    const float h = (float)sapp_height();
    const double delta_time = std::max(stm_sec(stm_laptime(&gApp.last_time)), 1. / 60.);

    lab::camera::PanTiltController& ptc = gApp.main_pan_tilt;

    float fovy = lab::camera::degrees_from_radians(gApp.camera.vertical_FOV());
    gApp.camera.optics.focal_length = gApp.camera.sensor.focal_length_from_vertical_FOV(lab::camera::radians_from_degrees(60));
    gApp.camera.optics.squeeze = w / h;
    m44f proj = gApp.camera.perspective();
    m44f view = gApp.camera.mount.gl_view_transform();
    m44f view_t = gApp.camera.mount.gl_view_transform_inv();
    m44f view_proj = gApp.camera.view_projection(1.f);

    auto& cmt = gApp.camera.mount.transform();
    v3f pos = cmt.position;

    //--- draw things in the 3d view
    //
    sg_begin_default_pass(&gApp.pass_action, window_width, window_height);

    draw_gizmo(&view_t.x.x, &view_proj.x.x, gizmo_triangles.triangle_count, gizmo_triangles.indices.data(), gizmo_triangles.vertices.data());

    start_gl_rendering();

    m44f identity = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    draw_grid(0, identity);
    draw_grid(0, *(const m44f*)&gApp.gizmo_transform);

    {
        m44f m = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

        // display look at
        if (gApp.show_look_at)
        {
            v3f lookat = gApp.navigator_panel->pan_tilt.orbit_center_constraint();
            m.w = { lookat.x, lookat.y, lookat.z, 1.f };
            draw_jack(1, m);
        }

        m.w = { gApp.initial_hit_point.x, gApp.initial_hit_point.y, gApp.initial_hit_point.z, 1.f };
        draw_jack(0.25, m);

        // hit point on manipulator plane
        if (gApp.show_manip_plane_intersect)
        {
            lab::camera::HitResult hit = gApp.camera.hit_test(
                { gApp.mouse.mousex, gApp.mouse.mousey },
                { (float)window_width, (float)window_height },
                *(v3f*)(&gApp.gizmo_transform.w),
                *(v3f*)(&gApp.gizmo_transform.y));

            if (hit.hit)
            {
                m.w = { hit.point.x, hit.point.y, hit.point.z, 1.f };
                draw_jack(0.5f, m);
            }
        }

        // intersection of mouse ray with image plane at 1 unit distance
        if (gApp.show_view_plane_intersect)
        {
            v3f cam_pos = cmt.position;
            v3f cam_nrm = cmt.forward();
            cam_nrm.x *= -1.f;
            cam_nrm.y *= -1.f;
            cam_nrm.z *= -1.f;
            cam_pos.x += cam_nrm.x;
            cam_pos.y += cam_nrm.y;
            cam_pos.z += cam_nrm.z;

            lab::camera::HitResult hit = gApp.camera.hit_test(
                { gApp.mouse.mousex, gApp.mouse.mousey },
                { (float)window_width, (float)window_height },
                cam_pos, cam_nrm);

            if (hit.hit)
            {
                m.w = { hit.point.x, hit.point.y, hit.point.z, 1.f };
                draw_jack(0.1f, m);
            }
        }
    }

    end_gl_rendering();

    //--- run the Dear Imgui portion
    //
    simgui_new_frame(window_width, window_height, delta_time);

    ImVec2 canvas_offset = ImGui::GetCursorPos();

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Playground")) {
            ImGui::MenuItem("Quit", 0, &gApp.quit);
            if (gApp.quit)
                sapp_request_quit();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Target")) {
            static bool local = true;
            static bool world = false;
            static bool translate = true;
            static bool rotate = false;
            static bool scale = false;
            if (ImGui::MenuItem("Local", 0, &local))
            {
                gApp.gizmo_ctx.set_frame(tinygizmo::reference_frame::local);
                world = false;
            }
            if (ImGui::MenuItem("World", 0, &world))
            {
                gApp.gizmo_ctx.set_frame(tinygizmo::reference_frame::global);
                local = false;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Translate", 0, &translate))
            {
                gApp.gizmo_ctx.set_mode(tinygizmo::transform_mode::translate);
                rotate = false;
                scale = false;
            }
            if (ImGui::MenuItem("Rotate", 0, &rotate))
            {
                gApp.gizmo_ctx.set_mode(tinygizmo::transform_mode::rotate);
                translate = false;
                scale = false;
            }
            if (ImGui::MenuItem("Scale", 0, &scale))
            {
                gApp.gizmo_ctx.set_mode(tinygizmo::transform_mode::scale);
                translate = false;
                rotate = false;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Show Look at point", 0, &gApp.show_look_at))
            {
            }
            if (ImGui::MenuItem("Show manip plane intersect", 0, &gApp.show_manip_plane_intersect))
            {
            }
            if (ImGui::MenuItem("Show view plane intersect", 0, &gApp.show_view_plane_intersect))
            {
            }

            //if (key == GLFW_KEY_LEFT_CONTROL) gizmo_state.hotkey_ctrl = (action != GLFW_RELEASE);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Windows")) {
            if (ImGui::MenuItem("Show Navigator", 0, &gApp.show_navigator))
            {
            }
            if (ImGui::MenuItem("Show Application State", 0, &gApp.show_state))
            {
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Sokol")) {
            ImGui::MenuItem("Buffers", 0, &gApp.sg_imgui.buffers.open);
            ImGui::MenuItem("Images", 0, &gApp.sg_imgui.images.open);
            ImGui::MenuItem("Shaders", 0, &gApp.sg_imgui.shaders.open);
            ImGui::MenuItem("Pipelines", 0, &gApp.sg_imgui.pipelines.open);
            ImGui::MenuItem("Calls", 0, &gApp.sg_imgui.capture.open);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    ImGui::SetNextWindowPos({ 0, 0 });
    ImGui::SetNextWindowSize({ (float)window_width, (float)window_height });
    static bool begin_flag = false;
    ImGui::Begin("###FULLSCREEN", &begin_flag,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImVec2 canvas_size = ImGui::GetContentRegionAvail();

    ImVec2 cursor_screen_pos = ImGui::GetCursorScreenPos();

    // draw a pixel ruler on the left side of the window
    for (float y = 0; y < window_height; y += 100)
    {
        char buff[32];
        sprintf(buff, "%d", (int)y);
        ImGui::SetCursorScreenPos(ImVec2{ 0, y } + canvas_offset);
        ImGui::TextUnformatted(buff);
    }

    // show where the 2d projected 3d projected 2d hit point is
    {
        v3f cam_pos = cmt.position;
        v3f cam_nrm = cmt.forward();
        cam_nrm.x *= -1.f;
        cam_nrm.y *= -1.f;
        cam_nrm.z *= -1.f;
        cam_pos.x += cam_nrm.x;
        cam_pos.y += cam_nrm.y;
        cam_pos.z += cam_nrm.z;

        // project onto a plane one unit in front of the camera
        lab::camera::HitResult hit = gApp.camera.hit_test(
            { gApp.mouse.mousex, gApp.mouse.mousey },
            { (float)window_width, (float)window_height },
            cam_pos, cam_nrm);

        if (hit.hit)
        {
            v2f vp_sz{ (float)window_width, (float)window_height };
            v2f vp_or = { 0, 0 };
            v2f xy = gApp.camera.project_to_viewport(vp_or, vp_sz, hit.point);
            ImGui::SetCursorScreenPos(ImVec2{ xy.x, xy.y } + canvas_offset - ImVec2{ 4,4 });
            ImGui::TextUnformatted("O");
        }
    }

    ImGui::SetCursorScreenPos(cursor_screen_pos);
    ImVec2 mouse_pos = ImGui::GetMousePos();
    v2f viewport{ (float)canvas_size.x, (float)canvas_size.y };
    lab::camera::InteractionPhase phase = lab::camera::InteractionPhase::Continue;
    bool mouse_in_viewport = update_mouseStatus_in_current_imgui_window(&gApp.mouse);

    if (gApp.show_state)
    {
        ImGui::Begin("App State");
        ImGui::Text("state: %s", name_state(gApp.ui_state));
        v3f ypr = gApp.camera.mount.ypr();
        ImGui::Text("ypr: (%f, %f, %f)", ypr.x, ypr.y, ypr.z);
        v3f pos = cmt.position;
        ImGui::Text("pos: (%f, %f, %f)", pos.x, pos.y, pos.z);
        ImGui::Separator();
        ImGui::Text("imouse: %f, %f", gApp.mouse.initial_mousex, gApp.mouse.initial_mousey);
        ImGui::Text(" mouse: %f, %f", gApp.mouse.mousex, gApp.mouse.mousey);
        ImGui::Text(" click: %s", gApp.mouse.dragging ? "X" : "-");
        ImGui::End();
    }

    LabCameraNavigatorPanelInteraction in = LCNav_None;
    if (gApp.show_navigator)
    {
        ptc.sync_constraints(gApp.navigator_panel->pan_tilt);
        in = run_navigator_panel(gApp.navigator_panel, gApp.camera, static_cast<float>(delta_time));
    }

    const lab::camera::rigid_transform rt = gApp.camera.mount.transform();
    camera_minimap(320, 240, &rt, gApp.main_pan_tilt.orbit_center_constraint());

    if (in > LCNav_None)
    {
        gApp.ui_state = UIStateMachine::None;
        run_gizmo(&gApp.mouse, (float)window_width, (float)window_height);
    }
    else if (mouse_in_viewport)
    {
        if (gApp.ui_state == UIStateMachine::Gizmo)
        {
            if (!run_gizmo(&gApp.mouse, (float)window_width, (float)window_height))
            {
                gApp.ui_state = UIStateMachine::None;
                sapp_lock_mouse(false);
            }
        }
        else if (gApp.ui_state <= UIStateMachine::UI)
        {
            if (run_gizmo(&gApp.mouse, (float)window_width, (float)window_height))
            {
                gApp.ui_state = UIStateMachine::Gizmo;
            }
            else if (gApp.mouse.click_initiated)
            {
                // hit test versus the gizmo's plane
                lab::camera::HitResult hit = gApp.camera.hit_test(
                    { gApp.mouse.mousex, gApp.mouse.mousey },
                    { (float)window_width, (float)window_height },
                    *(v3f*)(&gApp.gizmo_transform.w),
                    *(v3f*)(&gApp.gizmo_transform.y));

                if (gApp.mouse.click_initiated && gApp.ui_state <= UIStateMachine::UI)
                {
                    if (hit.hit)
                    {
                        gApp.initial_hit_point = hit.point;
                        gApp.ui_state = UIStateMachine::TTLCamera;
                    }
                    else
                    {
                        v3f cam_pos = cmt.position;
                        v3f cam_nrm = cmt.forward();
                        cam_nrm.x *= -1.f;
                        cam_nrm.y *= -1.f;
                        cam_nrm.z *= -1.f;
                        cam_pos.x += cam_nrm.x;
                        cam_pos.y += cam_nrm.y;
                        cam_pos.z += cam_nrm.z;

                        hit = gApp.camera.hit_test(
                            { gApp.mouse.mousex, gApp.mouse.mousey },
                            { (float)window_width, (float)window_height },
                            cam_pos, cam_nrm);

                        if (hit.hit)
                        {
                            gApp.initial_hit_point = hit.point;
                            gApp.ui_state = UIStateMachine::DeltaCamera;
                        }
                    }
                }
            }
        }

        if (gApp.ui_state == UIStateMachine::DeltaCamera || gApp.ui_state == UIStateMachine::TTLCamera)
        {
            if (gApp.mouse.click_initiated)
                phase = lab::camera::InteractionPhase::Start;
            else if (gApp.mouse.click_ended)
                phase = lab::camera::InteractionPhase::Finish;

            ImGui::CaptureMouseFromApp(true);

            if (gApp.ui_state == UIStateMachine::TTLCamera)
            {
                ImGui::SetCursorScreenPos(ImVec2{ mouse_pos.x, mouse_pos.y });
                ImGui::TextUnformatted("TTL+");

                // through the lens mode
                lab::camera::InteractionToken tok = ptc.begin_interaction(viewport);
                ptc.constrained_ttl_interaction(
                    gApp.camera,
                    tok, phase, gApp.navigator_panel->camera_interaction_mode,
                    { mouse_pos.x, mouse_pos.y },
                    gApp.initial_hit_point, static_cast<float>(delta_time));
                ptc.end_interaction(tok);
            }
            else
            {
                ImGui::SetCursorScreenPos(ImVec2{ mouse_pos.x, mouse_pos.y });
                ImGui::TextUnformatted("TTL");

                // virtual joystick mode
                lab::camera::InteractionToken tok = ptc.begin_interaction(viewport);
                ptc.ttl_interaction(
                    gApp.camera,
                    tok, phase, gApp.navigator_panel->camera_interaction_mode,
                    { mouse_pos.x, mouse_pos.y }, static_cast<float>(delta_time));
                ptc.end_interaction(tok);
            }

            if (gApp.mouse.click_ended)
            {
                gApp.ui_state = UIStateMachine::None;
                sapp_lock_mouse(false);
            }
        }
    }

    ImGui::End(); // full screen

    sg_imgui_draw(&gApp.sg_imgui);
    simgui_render();

    // the sokol_gfx draw pass
    sg_end_pass();
    sg_commit();

    //--- run the joystick controller
    //
    float len_l = left_stick.x * left_stick.x + left_stick.y * left_stick.y;
    float len_r = right_stick.x * right_stick.x + right_stick.y * right_stick.y;
    if (len_l > 0.05f || len_r > 0.05f)
    {
        ptc.sync_constraints(gApp.joystick_pan_tilt);
        lab::camera::InteractionToken tok = gApp.joystick_pan_tilt.begin_interaction(viewport);
        gApp.joystick_pan_tilt.dual_stick_interaction(
            gApp.camera, 
            tok, gApp.navigator_panel->camera_interaction_mode, 
            { left_stick.x, 0, left_stick.y }, { right_stick.x, 0, right_stick.y },
            static_cast<float>(delta_time));
        gApp.joystick_pan_tilt.end_interaction(tok);
    }
}

void imgui_callback_handler(const sapp_event* event)
{
    simgui_handle_event(event);
}

sapp_desc sokol_main(int argc, char* argv[]) 
{
    sapp_desc desc = { };
    desc.init_cb = initialize_graphics;
    desc.frame_cb = run_application_logic;
    desc.cleanup_cb = shutdown_graphics;
    desc.event_cb = imgui_callback_handler;
    desc.width = 1200;
    desc.height = 1000;
    desc.gl_force_gles2 = true;
    desc.window_title = "LabSound Playground";
    desc.ios_keyboard_resizes_canvas = false;
    return desc;
}
