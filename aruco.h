#ifndef _ARUCO_H_
#define _ARUCO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Result of a single ArUco detection pass. */
struct aruco_result {
	int found;
	int id;
	float distance_m;      /* rough distance to the marker center, meters */
	float yaw_error_deg;   /* + = marker right of image center */
	float pitch_error_deg; /* + = marker below image center */
};

/* Detect a 5x5 ArUco marker (80 mm) in an RGB24 image and draw the overlay
 * (marker corners, ID, distance, and the intended yaw motion arrow) directly
 * onto the image buffer. Returns the detection result.
 *
 * The buffer must be tightly packed (stride == width * 3). */
struct aruco_result aruco_detect_and_draw(
	uint8_t *rgb, int width, int height, int stride);

#ifdef __cplusplus
}
#endif

#endif
