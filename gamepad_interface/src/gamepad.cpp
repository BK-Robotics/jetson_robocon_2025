#include "gamepad_interface/gamepad.hpp"
#include <linux/input.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <stdexcept>
#include <cmath>

using namespace std;

DualSenseDriver::DualSenseDriver(const string &dev) : fd_(-1)
{
    fd_ = open(dev.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0)
        throw runtime_error("Cannot open " + dev + ": " + strerror(errno));
}

/*------------------ auto_detect -----------------------------------------*/
string DualSenseDriver::auto_detect()
{
    DIR *dir = opendir("/dev/input");
    if (!dir)
        throw runtime_error("Cannot open /dev/input: " + string(strerror(errno)));

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL)
    {
        if (strncmp(ent->d_name, "event", 5) != 0)
            continue;

        string path = string("/dev/input/") + ent->d_name;
        int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            continue;

        struct input_id id;
        if (ioctl(fd, EVIOCGID, &id) == 0)
        {
            if (id.vendor == 0x054C && id.product == 0x0CE6) // Sony DualSense
            {
                ::close(fd);
                closedir(dir);
                return path;
            }
        }
        ::close(fd);
    }
    closedir(dir);
    throw runtime_error("DualSense event device not found");
}

/*------------------ read() ----------------------------------------------*/
bool DualSenseDriver::read(GamepadState &out)
{
    if (fd_ < 0)
        return false;

    bool updated = false;
    struct input_event ev;
    while (::read(fd_, &ev, sizeof(ev)) == sizeof(ev))
    {
        updated = true;

        if (ev.type == EV_ABS)
        {
            switch (ev.code)
            {
            case ABS_X:
                state_.axes[0] = (ev.value - 128) / 128.f;
                break; // L-Joy X
            case ABS_Y:
                state_.axes[1] = (ev.value - 128) / 128.f;
                break; // L-Joy Y
            case ABS_RX:
                state_.axes[2] = (ev.value - 128) / 128.f;
                break; // R-Joy X
            case ABS_RY:
                state_.axes[3] = (ev.value - 128) / 128.f;
                break; // R-Jot Y
            case ABS_Z:
                state_.axes[4] = ev.value / 255.f;
                break; // L2
            case ABS_RZ:
                state_.axes[5] = ev.value / 255.f;
                break; // R2
            case ABS_HAT0X:
                state_.axes[6] = static_cast<float>(ev.value);
                break; // D-Pad X
            case ABS_HAT0Y:
                state_.axes[7] = static_cast<float>(ev.value);
                break; // D-Pad Y
            default:
                break;
            }
        }
        else if (ev.type == EV_KEY)
        {
            switch (ev.code)
            {
            case BTN_SOUTH:
                state_.buttons[0] = ev.value;
                break; // CROSS
            case BTN_EAST:
                state_.buttons[1] = ev.value;
                break; // CIRCLE
            case BTN_NORTH:
                state_.buttons[2] = ev.value;
                break; // TRIANGLE
            case BTN_WEST:
                state_.buttons[3] = ev.value;
                break; // SQUARE
            case BTN_TL:
                state_.buttons[4] = ev.value;
                break; // L1
            case BTN_TR:
                state_.buttons[5] = ev.value;
                break; // R1
            case BTN_TL2:
                state_.buttons[6] = ev.value;
                break; // L2 (digital)
            case BTN_TR2:
                state_.buttons[7] = ev.value;
                break; // R2 (digital)
            case BTN_SELECT:
                state_.buttons[8] = ev.value;
                break; // Create
            case BTN_START:
                state_.buttons[9] = ev.value;
                break; // Options
            case BTN_MODE:
                state_.buttons[10] = ev.value;
                break; // PS
            case BTN_THUMBL:
                state_.buttons[11] = ev.value;
                break; // L3
            case BTN_THUMBR:
                state_.buttons[12] = ev.value;
                break; // R3
            default:
                break;
            }
        }
    }

    if (updated)
        out = state_;
    return updated;
}