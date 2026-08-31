### About
This C application connects directly to a DJI/Ryze Tello over Wi-Fi. It decodes
the drone's H.264 camera stream in an SDL window and sends continuous flight
controls from the keyboard or mouse. It can also detect an ArUco marker in the
video and optionally auto-steer the drone's yaw toward it.

### Controls

| Input | Action |
| --- | --- |
| `Ctrl+T` | Take off |
| `Space` or `L` | Land |
| `W` / `S` | Fly forward / backward |
| `A` / `D` | Fly left / right |
| Left / Right arrow | Yaw left / right |
| Up / Down arrow | Rise / descend |
| Hold `Shift` | High-speed mode |
| `Esc` or close window | Stop controls, land if airborne, and quit |
| Left-drag in left half | Mouse yaw / altitude stick |
| Left-drag in right half | Mouse strafe / forward stick |
| Right click | Land |
| `K` | Toggle **keyframe-only** mode (NDI-style: only I-frames are shown, so packet loss can never cause blockiness; trades frame rate for a clean image) |
| `,` | Shorten the keyframe request interval by 100 ms (more frequent keyframes; in keyframe-only mode this raises the frame rate) |
| `.` | Lengthen the keyframe request interval by 100 ms (less frequent keyframes) |
| `0` | Set the keyframe interval to 33 ms (30 fps) |
| `5` | Toggle **auto-steer**: the drone's yaw is driven toward the detected ArUco marker |

Releasing the keys or mouse immediately centers the controls. Losing window
focus also centers all controls.

The app starts in keyframe-only mode at 30 fps.

### ArUco

The camera feed is scanned for a 5x5 ArUco marker (dictionary `DICT_5X5_50`,
80 mm) on every new frame. When one is found, the overlay draws the marker
corners, its ID, the estimated distance, and a yaw arrow showing the intended
correction. With auto-steer enabled (`5`), the yaw stick is driven
proportionally to the marker's horizontal offset, with a small deadband.

The marker size and dictionary are defined at the top of `aruco.cpp`
(`MARKER_SIZE_M` and `DICT_5X5_50`) if you use a different marker.
***
### Installation

On Debian, install the build dependencies while connected to the internet:

```
sudo apt install build-essential pkg-config libsdl2-dev libavcodec-dev libavutil-dev libswscale-dev libopencv-dev
```

Connect the computer to the Tello Wi-Fi network, power on the drone, then build
and run without root privileges:

```
make
./tello
```

Use the application in a clear indoor area for the first test. Confirm that the
battery and Wi-Fi values appear in the window title before taking off. Keep a
finger on `L` (or use right-click) while testing controls.

The title also reports low/critical battery flags received from the drone. A
Tello battery provides at most about 13 minutes of flight, and
running the camera while grounded still consumes the battery and generates
heat. Close the controller and power off the aircraft between extended bench
tests. If it shuts down with substantial battery remaining and feels unusually
hot, let it cool to room temperature before using or charging it.

To install the executable system-wide:

```
sudo make install
```
tello is installed into /usr/local by default this can be changed by setting DESTDIR.
```
sudo make DESTDIR=/usr install
```
***
### Removal
```
make uninstall
```
***
### Custom Project Example
`tello.h` / `tello.c` are a small standalone library for the drone's protocol,
so you can build your own front end on top of them:

```c
#include "tello.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// create a tello object this will hold all the information of the tello
struct tello tello;

// when the tello sends us data this function will be called
void tello_data_callback(int id)
{
	// print drones forward vector
	float q[] = {tello.rotation_x, tello.rotation_y, tello.rotation_z, tello.rotation_w};
	float v[] = {2 * (q[0]*q[2] + q[3]*q[1]), 2 * (q[1]*q[2] - q[3]*q[0]), 1 - 2 * (q[0]*q[0] + q[1]*q[1])};
	printf("%f, %f, %f\n", v[0], v[1], v[2])
}

// when the tello sends camera data this function will be called
void tello_camera_callback(uint8_t *data, int size)
{
	printf("Camera data size: %d\n", size);
}

int main(int argc, char *argv[])
{
	// connect to tello with the camera port 6038 and a timeout of 2 seconds
	while (tello_connect(&tello, 6038, 2) < 0) printf("Connection Failed\n");

	// register are tello data callback
	tello.data_callback = &tello_data_callback;

	// register are tello camera data callback
	//tello.camera_callback = &tello_camera_callback;

	// start loop
	while (1) {
		// wait for command (enter key)
		char input[512]; fgets(input, 512, stdin); input[strcspn(input, "\n")] = '\0';
		if (strcmp(input, "takeoff") == 0) tello_takeoff(&tello);
		if (strcmp(input, "land") == 0) tello_land(&tello);
		if (strcmp(input, "exit") == 0) break;
	}

	// disconnect the tello
	tello_disconnect(&tello);
}
```
