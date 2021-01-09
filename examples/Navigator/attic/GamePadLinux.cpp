
#if 0

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
#   define GAMEPAD_API
#else
#   if defined(_WIN32)
#       if defined(GAMEPAD_EXPORT)
#           define GAMEPAD_API __declspec(dllexport)
#       else
#           define GAMEPAD_API __declspec(dllimport)
#       endif
#   elif defined(__GNUC__) && defined(GAMEPAD_EXPORT)
#       define GAMEPAD_API __attribute__((visibility("default")))
#   else
#       define GAMEPAD_API extern
#   endif
#endif

    /**
     * Enumeration of the possible devices.
     *
     * Only four devices are supported as this is the limit of Windows.
     */
    enum GAMEPAD_DEVICE {
        GAMEPAD_0 = 0,  /**< First gamepad */
        GAMEPAD_1 = 1,  /**< Second gamepad */
        GAMEPAD_2 = 2,  /**< Third gamepad */
        GAMEPAD_3 = 3,  /**< Fourth gamepad */

        GAMEPAD_COUNT   /**< Maximum number of supported gamepads */
    };

    /**
     * Enumeration of the possible buttons.
     */
    enum GAMEPAD_BUTTON {
        BUTTON_DPAD_UP = 0, /**< UP on the direction pad */
        BUTTON_DPAD_DOWN = 1,   /**< DOWN on the direction pad */
        BUTTON_DPAD_LEFT = 2,   /**< LEFT on the direction pad */
        BUTTON_DPAD_RIGHT = 3,  /**< RIGHT on the direction pad */
        BUTTON_START = 4,   /**< START button */
        BUTTON_BACK = 5,    /**< BACK button */
        BUTTON_LEFT_THUMB = 6,  /**< Left analog stick button */
        BUTTON_RIGHT_THUMB = 7, /**< Right analog stick button */
        BUTTON_LEFT_SHOULDER = 8,   /**< Left bumper button */
        BUTTON_RIGHT_SHOULDER = 9,  /**< Right bumper button */
        BUTTON_A = 12,  /**< A button */
        BUTTON_B = 13,  /**< B button */
        BUTTON_X = 14,  /**< X button */
        BUTTON_Y = 15,  /**< Y button */

        BUTTON_COUNT                    /**< Maximum number of supported buttons */
    };

    /**
     * Enumeration of the possible pressure/trigger buttons.
     */
    enum GAMEPAD_TRIGGER {
        TRIGGER_LEFT = 0,   /**< Left trigger */
        TRIGGER_RIGHT = 1,  /**< Right trigger */

        TRIGGER_COUNT           /**< Number of triggers */
    };

    /**
     * Enumeration of the analog sticks.
     */
    enum GAMEPAD_STICK {
        STICK_LEFT = 0, /**< Left stick */
        STICK_RIGHT = 1,    /**< Right stick */

        STICK_COUNT     /**< Number of analog sticks */
    };

    /**
     * Enumeration of main stick directions.
     *
     * This is used for some of the convenience routines in the library.
     */
    enum GAMEPAD_STICKDIR {
        STICKDIR_CENTER = 0,    /**< CENTER, no direction */
        STICKDIR_UP = 1,    /**< UP direction */
        STICKDIR_DOWN = 2,  /**< DOWN direction */
        STICKDIR_LEFT = 3,  /**< LEFT direction */
        STICKDIR_RIGHT = 4, /**< RIGHT direction */

        STICKDIR_COUNT
    };

    /**
     * Enumeration for true/false values
     */
    enum GAMEPAD_BOOL {
        GAMEPAD_FALSE = 0,  /**< FALSE value for boolean parameters */
        GAMEPAD_TRUE = 1        /**< TRUE value for boolean parameters */
    };

    typedef enum GAMEPAD_DEVICE GAMEPAD_DEVICE;
    typedef enum GAMEPAD_BUTTON GAMEPAD_BUTTON;
    typedef enum GAMEPAD_TRIGGER GAMEPAD_TRIGGER;
    typedef enum GAMEPAD_STICK GAMEPAD_STICK;
    typedef enum GAMEPAD_STICKDIR GAMEPAD_STICKDIR;
    typedef enum GAMEPAD_BOOL GAMEPAD_BOOL;

#define GAMEPAD_DEADZONE_LEFT_STICK     7849    /**< Suggested deadzone magnitude for left analog stick */
#define GAMEPAD_DEADZONE_RIGHT_STICK    8689    /**< Suggested deadzone magnitude for right analog stick */
#define GAMEPAD_DEADZONE_TRIGGER        30      /**< Suggested deadzone for triggers */

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

#   undef UNICODE
#   include "windows.h"
#   include "xinput.h"
#   pragma comment(lib, "xinput.lib")
#elif defined(__linux__)
#   include <linux/joystick.h>
#   include <stdio.h>
#   include <fcntl.h>
#   include <unistd.h>
#   include <libudev.h>
#else
#   error "Unknown platform in gamepad.c"
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
#define FLAG_CONNECTED  (1<<0)
#define FLAG_RUMBLE     (1<<1)

/* Prototypes for utility functions */
static void GamepadResetState(GAMEPAD_DEVICE gamepad);
static void GamepadUpdateCommon(void);
static void GamepadUpdateDevice(GAMEPAD_DEVICE gamepad);
static void GamepadUpdateStick(GAMEPAD_AXIS* axis, float deadzone);
static void GamepadUpdateTrigger(GAMEPAD_TRIGINFO* trig);

/* Various values of PI */
#define PI_1_4  0.78539816339744f
#define PI_1_2  1.57079632679489f
#define PI_3_4  2.35619449019234f
#define PI      3.14159265358979f

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
                case 0: STATE[gamepad].stick[STICK_LEFT].x = je.value; break;
                case 1: STATE[gamepad].stick[STICK_LEFT].y = -je.value; break;
                case 2: STATE[gamepad].trigger[TRIGGER_LEFT].value = (je.value + 32768) >> 8; break;
                case 3: STATE[gamepad].stick[STICK_RIGHT].x = je.value; break;
                case 4: STATE[gamepad].stick[STICK_RIGHT].y = -je.value; break;
                case 5: STATE[gamepad].trigger[TRIGGER_RIGHT].value = (je.value + 32768) >> 8; break;
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

#   error "Unknown platform in gamepad.c"

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


#endif




#if 0



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







