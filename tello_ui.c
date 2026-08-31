#include "tello.h"
#include "aruco.h"

#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <SDL2/SDL.h>

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAMERA_PORT 6038
#define MAX_VIDEO_PACKET 1460
#define VIDEO_OUTPUT_WIDTH 640
#define VIDEO_OUTPUT_HEIGHT 480
#define STICK_RADIUS 110.0f

struct video_decoder {
	AVCodecContext *codec;
	AVCodecParserContext *parser;
	AVFrame *frame;
	struct SwsContext *scaler;
	uint8_t *pixels;
	int pixels_size;
	int width;
	int height;
	unsigned long serial;
	int wait_for_keyframe;
	/* Set from the camera thread (corruption detected) and consumed by the
	 * main thread. Atomic so the two threads never race on it. */
	atomic_int needs_keyframe;
	pthread_mutex_t mutex;
};

struct mouse_stick {
	int active;
	int left_side;
	float origin_x;
	float origin_y;
	float x;
	float y;
};

static struct tello drone;
static struct video_decoder video;
static struct mouse_stick mouse_stick;
static atomic_int running = 1;
static atomic_int connected = 0;
/* Timestamp (SDL_GetTicks) of the last successfully decoded frame. Used to
 * detect decoder stalls so we only request keyframes when actually needed. */
static atomic_uint last_frame_ms = 0;
/* NDI-style "keyframe-only" mode: the decoder discards every non-key frame,
 * so a corrupted P-frame can never propagate into the displayed image. This
 * trades frame rate for zero blockiness. Toggle with K. */
static atomic_int keyframe_only = 1;
/* How often (ms) to request a keyframe from the drone. In keyframe-only mode
 * this is the effective frame rate (1000 ms = 1 fps). Adjust with , (shorter,
 * more frequent) and . (longer, less frequent). */
static atomic_uint keyframe_interval_ms = 33;
/* ArUco auto-steer: when enabled, the drone's yaw is driven toward the
 * detected marker. Toggle with 5. */
static atomic_int auto_steer = 0;
/* Most recent ArUco detection (written and read on the main thread only). */
static struct aruco_result last_aruco;

static float clamp_axis(float value)
{
	if (value < -1.0f) return -1.0f;
	if (value > 1.0f) return 1.0f;
	return value;
}

static int video_init(void)
{
	const AVCodec *decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
	if (decoder == NULL) return -1;

	video.parser = av_parser_init(AV_CODEC_ID_H264);
	video.codec = avcodec_alloc_context3(decoder);
	video.frame = av_frame_alloc();
	if (video.parser == NULL || video.codec == NULL || video.frame == NULL) return -1;
	video.codec->flags |= AV_CODEC_FLAG_LOW_DELAY;
	video.codec->thread_count = 1;
	video.codec->skip_frame = AVDISCARD_NONKEY;
	if (avcodec_open2(video.codec, decoder, NULL) < 0) return -1;
	video.wait_for_keyframe = 1;
	pthread_mutex_init(&video.mutex, NULL);
	return 0;
}

static void video_destroy(void)
{
	pthread_mutex_destroy(&video.mutex);
	sws_freeContext(video.scaler);
	av_free(video.pixels);
	av_frame_free(&video.frame);
	av_parser_close(video.parser);
	avcodec_free_context(&video.codec);
}

static void store_decoded_frames(void)
{
	int ret;
	while ((ret = avcodec_receive_frame(video.codec, video.frame)) == 0) {
		if (video.wait_for_keyframe) {
			if (!(video.frame->flags & AV_FRAME_FLAG_KEY)) continue;
			video.wait_for_keyframe = 0;
			video.codec->skip_frame = atomic_load(&keyframe_only)
				? AVDISCARD_NONKEY : AVDISCARD_DEFAULT;
		}
		/* Keyframe-only mode: skip every non-key frame so a corrupted P-frame
		 * can never reach the display. The decoder still decodes them (they are
		 * reference frames) but we simply never show them. */
		if (atomic_load(&keyframe_only) && !(video.frame->flags & AV_FRAME_FLAG_KEY))
			continue;
		int required = av_image_get_buffer_size(AV_PIX_FMT_RGB24,
			VIDEO_OUTPUT_WIDTH, VIDEO_OUTPUT_HEIGHT, 1);
		if (required <= 0) continue;

		pthread_mutex_lock(&video.mutex);
		if (required > video.pixels_size) {
			uint8_t *resized = av_realloc(video.pixels, required);
			if (resized == NULL) {
				pthread_mutex_unlock(&video.mutex);
				continue;
			}
			video.pixels = resized;
			video.pixels_size = required;
		}
		video.scaler = sws_getCachedContext(video.scaler,
			video.frame->width, video.frame->height, video.frame->format,
			VIDEO_OUTPUT_WIDTH, VIDEO_OUTPUT_HEIGHT, AV_PIX_FMT_RGB24,
			SWS_FAST_BILINEAR, NULL, NULL, NULL);
		if (video.scaler != NULL) {
			uint8_t *destination[] = {video.pixels};
			int destination_stride[] = {VIDEO_OUTPUT_WIDTH * 3};
			sws_scale(video.scaler, (const uint8_t *const *)video.frame->data,
				video.frame->linesize, 0, video.frame->height,
				destination, destination_stride);
			video.width = VIDEO_OUTPUT_WIDTH;
			video.height = VIDEO_OUTPUT_HEIGHT;
			video.serial++;
			atomic_store(&last_frame_ms, SDL_GetTicks());
		}
		pthread_mutex_unlock(&video.mutex);
	}
	/* A broken reference chain (UDP packet loss corrupting a P-frame) makes the
	 * decoder report invalid data. This is the strongest corruption signal, so
	 * request a keyframe and flush the decoder so the next keyframe starts from
	 * a clean state instead of inheriting the corrupted reference frames. */
	if (ret == AVERROR_INVALIDDATA) {
		atomic_store(&video.needs_keyframe, 1);
		avcodec_flush_buffers(video.codec);
	}
}

static void decode_video(const uint8_t *data, int size)
{
	while (size > 0) {
		uint8_t *packet_data = NULL;
		int packet_size = 0;
		int consumed = av_parser_parse2(video.parser, video.codec,
			&packet_data, &packet_size, data, size,
			AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
		if (consumed < 0) return;
		data += consumed;
		size -= consumed;
		if (packet_size > 0) {
			AVPacket packet = {0};
			packet.data = packet_data;
			packet.size = packet_size;
			/* The parser flags a packet CORRUPT when it cannot fully parse it
			 * (e.g. a truncated NAL from UDP packet loss). Once a P-frame is
			 * corrupted, every frame that references it stays blocky until the
			 * next keyframe, so flag a resync as soon as we see one. */
			if (packet.flags & AV_PKT_FLAG_CORRUPT)
				atomic_store(&video.needs_keyframe, 1);
			if (avcodec_send_packet(video.codec, &packet) == AVERROR(EAGAIN)) {
				store_decoded_frames();
				avcodec_send_packet(video.codec, &packet);
			}
			store_decoded_frames();
		}
		if (consumed == 0) break;
	}
}

static void camera_callback(uint8_t *data, int size)
{
	if (size <= 2) return;
	int payload_size = size - 2;
	if (payload_size > MAX_VIDEO_PACKET) return;
	uint8_t padded[MAX_VIDEO_PACKET + AV_INPUT_BUFFER_PADDING_SIZE];
	memcpy(padded, data + 2, payload_size);
	memset(padded + payload_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
	decode_video(padded, payload_size);
}

static void data_callback(int id)
{
	(void)id;
}

static void *connection_thread(void *unused)
{
	(void)unused;
	while (atomic_load(&running)) {
		if (tello_connect(&drone, CAMERA_PORT, 2) == 0) {
			if (!atomic_load(&running)) {
				tello_disconnect(&drone);
				return NULL;
			}
			drone.data_callback = data_callback;
			drone.camera_callback = camera_callback;
			tello_start_video(&drone);
			atomic_store(&connected, 1);
			fprintf(stderr, "Connected to Tello.\n");
			return NULL;
		}
		fprintf(stderr, "Tello not found at 192.168.10.1; retrying...\n");
	}
	return NULL;
}

static void set_controls(const uint8_t *keys)
{
	float yaw = (keys[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f)
		- (keys[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f);
	float throttle = (keys[SDL_SCANCODE_UP] ? 1.0f : 0.0f)
		- (keys[SDL_SCANCODE_DOWN] ? 1.0f : 0.0f);
	float strafe = (keys[SDL_SCANCODE_D] ? 1.0f : 0.0f)
		- (keys[SDL_SCANCODE_A] ? 1.0f : 0.0f);
	float forward = (keys[SDL_SCANCODE_W] ? 1.0f : 0.0f)
		- (keys[SDL_SCANCODE_S] ? 1.0f : 0.0f);

	if (mouse_stick.active) {
		float horizontal = clamp_axis((mouse_stick.x - mouse_stick.origin_x) / STICK_RADIUS);
		float vertical = clamp_axis((mouse_stick.origin_y - mouse_stick.y) / STICK_RADIUS);
		if (mouse_stick.left_side) {
			yaw += horizontal;
			throttle += vertical;
		} else {
			strafe += horizontal;
			forward += vertical;
		}
	}

	drone.left_x = clamp_axis(yaw);
	drone.left_y = clamp_axis(throttle);
	drone.right_x = clamp_axis(strafe);
	drone.right_y = clamp_axis(forward);
	drone.speed_mode = keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT];

	/* ArUco auto-steer: override yaw to face the detected marker. Positive
	 * yaw_error means the marker is to the right, so yaw right (positive). */
	if (atomic_load(&auto_steer) && last_aruco.found) {
		float cmd = last_aruco.yaw_error_deg / 25.0f;
		if (cmd > 1.0f) cmd = 1.0f;
		if (cmd < -1.0f) cmd = -1.0f;
		if (cmd > -0.15f && cmd < 0.15f) cmd = 0.0f;   /* deadband */
		drone.left_x = cmd;
	}
}

static void stop_controls(void)
{
	memset(&mouse_stick, 0, sizeof(mouse_stick));
	drone.left_x = 0.0f;
	drone.left_y = 0.0f;
	drone.right_x = 0.0f;
	drone.right_y = 0.0f;
	drone.speed_mode = 0;
}

/* Switch the decoder between normal and keyframe-only mode. When enabling,
 * force a resync so the next displayed frame is a clean keyframe. */
static void set_keyframe_only(int enable)
{
	atomic_store(&keyframe_only, enable);
	pthread_mutex_lock(&video.mutex);
	video.codec->skip_frame = enable ? AVDISCARD_NONKEY : AVDISCARD_DEFAULT;
	if (enable) video.wait_for_keyframe = 1;
	pthread_mutex_unlock(&video.mutex);
	atomic_store(&video.needs_keyframe, 1);
	fprintf(stderr, "Keyframe-only mode: %s\n", enable ? "ON" : "OFF");
}

static void set_keyframe_interval(uint32_t interval)
{
	if (interval < 33) interval = 33;   /* 33 ms = 30 fps, the practical floor */
	if (interval > 5000) interval = 5000;
	atomic_store(&keyframe_interval_ms, interval);
	fprintf(stderr, "Keyframe interval: %u ms\n", interval);
}

static void adjust_keyframe_interval(int delta_ms)
{
	set_keyframe_interval(atomic_load(&keyframe_interval_ms) + delta_ms);
}

static void set_auto_steer(int enable)
{
	atomic_store(&auto_steer, enable);
	fprintf(stderr, "Auto-steer: %s\n", enable ? "ON" : "OFF");
}

static void handle_key(const SDL_KeyboardEvent *key)
{
	if (key->repeat || !atomic_load(&connected)) return;
	if (key->keysym.scancode == SDL_SCANCODE_T && (key->keysym.mod & KMOD_CTRL))
		tello_takeoff(&drone);
	else if (key->keysym.scancode == SDL_SCANCODE_L || key->keysym.scancode == SDL_SCANCODE_SPACE)
		tello_land(&drone);
	else if (key->keysym.scancode == SDL_SCANCODE_K)
		set_keyframe_only(!atomic_load(&keyframe_only));
	else if (key->keysym.scancode == SDL_SCANCODE_COMMA)
		adjust_keyframe_interval(-100);
	else if (key->keysym.scancode == SDL_SCANCODE_PERIOD)
		adjust_keyframe_interval(100);
	else if (key->keysym.scancode == SDL_SCANCODE_0)
		set_keyframe_interval(33);   /* 30 fps */
	else if (key->keysym.scancode == SDL_SCANCODE_5)
		set_auto_steer(!atomic_load(&auto_steer));
}

static void handle_event(const SDL_Event *event, int window_width)
{
	switch (event->type) {
	case SDL_QUIT:
		atomic_store(&running, 0);
		break;
	case SDL_KEYDOWN:
		if (event->key.keysym.scancode == SDL_SCANCODE_ESCAPE)
			atomic_store(&running, 0);
		else
			handle_key(&event->key);
		break;
	case SDL_WINDOWEVENT:
		if (event->window.event == SDL_WINDOWEVENT_FOCUS_LOST) stop_controls();
		break;
	case SDL_MOUSEBUTTONDOWN:
		if (event->button.button == SDL_BUTTON_LEFT) {
			mouse_stick.active = 1;
			mouse_stick.left_side = event->button.x < window_width / 2;
			mouse_stick.origin_x = mouse_stick.x = event->button.x;
			mouse_stick.origin_y = mouse_stick.y = event->button.y;
		} else if (atomic_load(&connected) && event->button.button == SDL_BUTTON_RIGHT) {
			tello_land(&drone);
		}
		break;
	case SDL_MOUSEBUTTONUP:
		if (event->button.button == SDL_BUTTON_LEFT) memset(&mouse_stick, 0, sizeof(mouse_stick));
		break;
	case SDL_MOUSEMOTION:
		if (mouse_stick.active) {
			mouse_stick.x = event->motion.x;
			mouse_stick.y = event->motion.y;
		}
		break;
	}
}

static void draw_stick(SDL_Renderer *renderer, int center_x, int center_y,
	float horizontal, float vertical, SDL_Color color)
{
	SDL_SetRenderDrawColor(renderer, 220, 225, 229, 100);
	SDL_Rect horizontal_line = {center_x - 70, center_y - 1, 140, 2};
	SDL_Rect vertical_line = {center_x - 1, center_y - 70, 2, 140};
	SDL_RenderFillRect(renderer, &horizontal_line);
	SDL_RenderFillRect(renderer, &vertical_line);

	SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
	SDL_Rect marker = {
		center_x + (int)(horizontal * 55.0f) - 8,
		center_y - (int)(vertical * 55.0f) - 8,
		16, 16
	};
	SDL_RenderFillRect(renderer, &marker);
}

static void draw_dashboard(SDL_Renderer *renderer, int width, int height)
{
	int panel_height = 170;
	SDL_Rect panel = {0, height - panel_height, width, panel_height};
	SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
	SDL_SetRenderDrawColor(renderer, 14, 20, 24, 235);
	SDL_RenderFillRect(renderer, &panel);

	SDL_Color left_color = {235, 181, 63, 255};
	SDL_Color right_color = {49, 184, 164, 255};
	draw_stick(renderer, width / 4, height - panel_height / 2,
		drone.left_x, drone.left_y, left_color);
	draw_stick(renderer, width * 3 / 4, height - panel_height / 2,
		drone.right_x, drone.right_y, right_color);

	int battery_width = width > 320 ? width / 5 : 60;
	SDL_Rect battery_back = {12, height - panel_height + 12, battery_width, 8};
	SDL_Rect battery_fill = battery_back;
	battery_fill.w = battery_width * drone.battery_percentage / 100;
	SDL_SetRenderDrawColor(renderer, 72, 78, 82, 255);
	SDL_RenderFillRect(renderer, &battery_back);
	SDL_SetRenderDrawColor(renderer,
		drone.battery_percentage < 20 ? 218 : 67,
		drone.battery_percentage < 20 ? 73 : 189, 86, 255);
	SDL_RenderFillRect(renderer, &battery_fill);

	/* Temperature gauge: 0C maps to empty, 70C maps to full. */
	int temp_width = battery_width;
	float temp_frac = drone.imu_temperature / 70.0f;
	if (temp_frac < 0.0f) temp_frac = 0.0f;
	if (temp_frac > 1.0f) temp_frac = 1.0f;
	SDL_Rect temp_back = {12, height - panel_height + 28, temp_width, 8};
	SDL_Rect temp_fill = temp_back;
	temp_fill.w = (int)(temp_width * temp_frac);
	SDL_SetRenderDrawColor(renderer, 72, 78, 82, 255);
	SDL_RenderFillRect(renderer, &temp_back);
	if (drone.imu_temperature >= 55.0f)
		SDL_SetRenderDrawColor(renderer, 218, 73, 73, 255);
	else if (drone.imu_temperature >= 45.0f)
		SDL_SetRenderDrawColor(renderer, 235, 181, 63, 255);
	else
		SDL_SetRenderDrawColor(renderer, 67, 189, 86, 255);
	SDL_RenderFillRect(renderer, &temp_fill);
}

static void update_title(SDL_Window *window)
{
	char title[256];
	if (atomic_load(&connected)) {
		const char *warning = "";
		if (drone.battery_lower)
			warning = " | CRITICAL BATTERY";
		else if (drone.battery_low)
			warning = " | LOW BATTERY";
		snprintf(title, sizeof(title),
			"Tello | Battery %d%% | Wi-Fi %d%% | Height %.1fm | IMU %.1fC | Keepalive %s%s | %s | K keyframe-only, , . interval %u ms, 0 = 30fps, 5 auto-steer %s | WASD move, Ctrl+T take off, Space/L land",
			drone.battery_percentage, drone.wifi_strength, drone.height / 10.0,
			drone.imu_temperature, drone.keepalive_acknowledged ? "OK" : "WAIT", warning,
			atomic_load(&keyframe_only) ? "KEYFRAMES ONLY" : "ALL FRAMES",
			atomic_load(&keyframe_interval_ms),
			atomic_load(&auto_steer) ? "ON" : "OFF");
	} else {
		snprintf(title, sizeof(title), "Tello | Connecting to 192.168.10.1...");
	}
	SDL_SetWindowTitle(window, title);
}

int main(void)
{
	av_log_set_level(AV_LOG_ERROR);
	if (video_init() < 0) {
		fprintf(stderr, "Could not initialize the H.264 decoder.\n");
		return EXIT_FAILURE;
	}
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_TIMER) < 0) {
		fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
		video_destroy();
		return EXIT_FAILURE;
	}

	SDL_Window *window = SDL_CreateWindow("Tello | Connecting...",
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 960, 720,
		SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Renderer *renderer = window == NULL ? NULL : SDL_CreateRenderer(window, -1,
		SDL_RENDERER_ACCELERATED);
	if (window == NULL || renderer == NULL) {
		fprintf(stderr, "Could not create the SDL window: %s\n", SDL_GetError());
		SDL_DestroyWindow(window);
		SDL_Quit();
		video_destroy();
		return EXIT_FAILURE;
	}

	pthread_t connector;
	pthread_create(&connector, NULL, connection_thread, NULL);
	SDL_Texture *texture = NULL;
	int texture_width = 0;
	int texture_height = 0;
	unsigned long displayed_serial = 0;
	uint32_t last_title_update = 0;
	uint32_t last_video_headers = 0;

	while (atomic_load(&running)) {
		int window_width;
		int window_height;
		SDL_GetWindowSize(window, &window_width, &window_height);
		SDL_Event event;
		while (SDL_PollEvent(&event)) handle_event(&event, window_width);
		if (atomic_load(&connected)) set_controls(SDL_GetKeyboardState(NULL));

		int need_keyframe = atomic_exchange(&video.needs_keyframe, 0);
		pthread_mutex_lock(&video.mutex);
		if (video.serial != displayed_serial && video.width > 0 && video.height > 0) {
			if (texture_width != video.width || texture_height != video.height) {
				SDL_DestroyTexture(texture);
				texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24,
					SDL_TEXTUREACCESS_STREAMING, video.width, video.height);
				texture_width = video.width;
				texture_height = video.height;
			}
			last_aruco = aruco_detect_and_draw(video.pixels, video.width, video.height, video.width * 3);
			if (texture != NULL) SDL_UpdateTexture(texture, NULL, video.pixels, video.width * 3);
			displayed_serial = video.serial;
		}
		pthread_mutex_unlock(&video.mutex);

		SDL_SetRenderDrawColor(renderer, 8, 12, 15, 255);
		SDL_RenderClear(renderer);
		if (texture != NULL) {
			int available_height = window_height - 170;
			float scale = fminf((float)window_width / texture_width,
				(float)available_height / texture_height);
			SDL_Rect destination = {
				(window_width - (int)(texture_width * scale)) / 2,
				(available_height - (int)(texture_height * scale)) / 2,
				(int)(texture_width * scale), (int)(texture_height * scale)
			};
			SDL_RenderCopy(renderer, texture, NULL, &destination);
		}
		draw_dashboard(renderer, window_width, window_height);
		SDL_RenderPresent(renderer);

		uint32_t now = SDL_GetTicks();
		/* Request a keyframe when the decoder has stalled, when it reports a
		 * corrupted frame (packet loss), or on the user-tunable interval (the
		 * safety net, and the effective frame rate in keyframe-only mode). A
		 * short cooldown prevents a burst of requests. Forcing keyframes
		 * constantly starves the fixed-bitrate encoder and makes the whole
		 * stream blocky, so we only ask when a clean resync is actually needed. */
		if (atomic_load(&connected)) {
			uint32_t last_frame = atomic_load(&last_frame_ms);
			uint32_t interval = atomic_load(&keyframe_interval_ms);
			/* Allow a little slack beyond the interval so a slow keyframe does
			 * not count as a stall (matters in keyframe-only mode). */
			uint32_t stall_threshold = interval + 250;
			if (stall_threshold < 500) stall_threshold = 500;
			int stalled = last_frame != 0 && now - last_frame > stall_threshold;
			/* In keyframe-only mode the interval is the frame rate, so the
			 * cooldown must be shorter than the interval to allow 30 fps. */
			uint32_t cooldown = atomic_load(&keyframe_only) ? 30 : 150;
			int cooldown_ok = now - last_video_headers >= cooldown;
			if ((stalled || need_keyframe || now - last_video_headers >= interval)
				&& cooldown_ok) {
				tello_request_iframe(&drone);
				last_video_headers = now;
			}
		}
		if (now - last_title_update >= 500) {
			update_title(window);
			last_title_update = now;
		}
	}

	stop_controls();
	if (atomic_load(&connected) && drone.sky) tello_land(&drone);
	atomic_store(&running, 0);
	pthread_join(connector, NULL);
	tello_disconnect(&drone);
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	video_destroy();
	return EXIT_SUCCESS;
}