#include "aruco.h"

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d.hpp>

#include <stdio.h>
#include <math.h>

#define MARKER_SIZE_M 0.08f
#define RAD2DEG 57.29577951308232f

/* Approximate Tello camera intrinsics (960x720 native), scaled to the
 * display size. Good enough for a rough distance / orientation estimate. */
static void camera_intrinsics(int width, int height, cv::Mat &camera, cv::Mat &dist)
{
	double scale = width / 960.0;
	camera = (cv::Mat_<double>(3, 3) <<
		480.0 * scale, 0.0, width / 2.0,
		0.0, 480.0 * scale, height / 2.0,
		0.0, 0.0, 1.0);
	dist = cv::Mat::zeros(4, 1, CV_64F);
}

struct aruco_result aruco_detect_and_draw(
	uint8_t *rgb, int width, int height, int stride)
{
	struct aruco_result r = {};
	if (rgb == NULL || width <= 0 || height <= 0 || stride != width * 3)
		return r;

	/* The shared buffer is RGB24; OpenCV works in BGR. Convert in, detect and
	 * draw on the BGR copy, then convert back so the buffer stays RGB24 for SDL. */
	cv::Mat rgb_mat(height, width, CV_8UC3, rgb);
	cv::Mat img;
	cv::cvtColor(rgb_mat, img, cv::COLOR_RGB2BGR);

	cv::aruco::ArucoDetector detector(
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50));
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;
	detector.detectMarkers(img, corners, ids);
	if (ids.empty())
		return r;

	r.found = 1;
	r.id = ids[0];
	const std::vector<cv::Point2f> &c = corners[0];

	/* Pose via solvePnP: the marker is a flat square of side MARKER_SIZE_M
	 * centred at the origin, with object points in the same clockwise order
	 * that detectMarkers returns. */
	cv::Mat camera, dist, rvec, tvec;
	camera_intrinsics(width, height, camera, dist);
	std::vector<cv::Point3f> obj;
	float s = MARKER_SIZE_M;
	obj.push_back(cv::Point3f(-s / 2, -s / 2, 0));
	obj.push_back(cv::Point3f( s / 2, -s / 2, 0));
	obj.push_back(cv::Point3f( s / 2,  s / 2, 0));
	obj.push_back(cv::Point3f(-s / 2,  s / 2, 0));
	if (cv::solvePnP(obj, c, camera, dist, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
		r.distance_m = (float)cv::norm(tvec);
		r.yaw_error_deg = (float)(atan2(tvec.at<double>(0), tvec.at<double>(2)) * RAD2DEG);
		r.pitch_error_deg = (float)(atan2(tvec.at<double>(1), tvec.at<double>(2)) * RAD2DEG);
	}

	/* Draw the marker corners and connecting edges. */
	cv::Scalar green(0, 255, 0);
	for (int i = 0; i < 4; i++) {
		cv::Point2f a = c[i], b = c[(i + 1) % 4];
		cv::line(img, a, b, green, 2, cv::LINE_AA);
		cv::circle(img, a, 4, green, -1, cv::LINE_AA);
	}

	/* Pose text near the marker center. */
	cv::Point2f center = (c[0] + c[2]) * 0.5f;
	int tx = (int)center.x, ty = (int)center.y - 14;
	char text[64];
	snprintf(text, sizeof(text), "ID %d", r.id);
	cv::putText(img, text, cv::Point(tx, ty),
		cv::FONT_HERSHEY_SIMPLEX, 0.6, green, 2, cv::LINE_AA);
	if (r.distance_m > 0.0f) {
		snprintf(text, sizeof(text), "%.2f m", r.distance_m);
		cv::putText(img, text, cv::Point(tx, ty + 22),
			cv::FONT_HERSHEY_SIMPLEX, 0.6, green, 2, cv::LINE_AA);
	}

	/* Intended motion: a yaw arrow at the image center. A marker to the right
	 * (yaw_error > 0) means we should yaw right, so the arrow points right. */
	cv::Scalar yellow(0, 255, 255);
	cv::Point2f mid(width / 2.0f, height / 2.0f);
	float magnitude = r.yaw_error_deg < 0 ? -r.yaw_error_deg : r.yaw_error_deg;
	float len = 40.0f + 2.0f * magnitude;
	if (len > 120.0f) len = 120.0f;
	if (r.yaw_error_deg > 2.0f)
		cv::arrowedLine(img, mid, cv::Point2f(mid.x + len, mid.y), yellow, 3, cv::LINE_AA, 0.03);
	else if (r.yaw_error_deg < -2.0f)
		cv::arrowedLine(img, mid, cv::Point2f(mid.x - len, mid.y), yellow, 3, cv::LINE_AA, 0.03);

	/* Write the annotated frame back into the shared RGB24 buffer. */
	cv::cvtColor(img, rgb_mat, cv::COLOR_BGR2RGB);
	return r;
}
