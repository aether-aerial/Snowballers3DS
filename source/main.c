// SnowballersAR - 3DS homebrew prototype
// Camera feed + gyroscope aiming + stereoscopic 3D penguins to hit with snowballs.
// Build with the devkitPro 3DS application template (libctru only, no extra libs).
// Needs penguin.h in the same source/ folder.
//
// Controls: move/turn the 3DS to aim, A = throw snowball, SELECT = recenter, START = quit
// Tuning:   D-pad UP/DOWN = flip up/down aim, D-pad LEFT/RIGHT = flip left/right aim,
//           L / R = gyro sensitivity down / up

#include <3ds.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "penguin.h"

#define CAM_W 400
#define CAM_H 240
#define CAM_BYTES (CAM_W * CAM_H * 2)
#define CAM_WAIT_NS 200000000LL     // restart the camera if no frame for 200 ms

#define NUM_TARGETS 5
#define PX_PER_RAD 400.0f           // how far targets slide on screen per radian you turn
#define MAX_DISPARITY 10            // stereo pop-out strength at full slider

// Raw gyro units -> radians/sec. Starting guess; TUNE THIS on hardware.
// If penguins move too slowly when you turn, raise it. Too fast, lower it.
#define GYRO_SCALE 0.0012f

// Starting aim direction (can also be flipped live with the D-pad).
#define AIM_X_SIGN -1.0f    // left/right
#define AIM_Y_SIGN 1.0f     // up/down

#define COL565(r, g, b) (u16)((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))

typedef struct {
    float yaw, pitch;   // direction in the world (radians)
    float depth;        // 0 = at screen, 1 = popped out toward the player
    int h;              // drawn height in pixels
} Target;

// Top screen framebuffer is rotated: 240 tall columns, 400 of them.
static inline void putpx(u16 *fb, int x, int y, u16 c)
{
    if (x < 0 || x >= 400 || y < 0 || y >= 240) return;
    fb[x * 240 + (239 - y)] = c;
}

static void blit_camera(u16 *fb, const u16 *cam)
{
    for (int y = 0; y < CAM_H; y++)
        for (int x = 0; x < CAM_W; x++)
            fb[x * 240 + (239 - y)] = cam[y * CAM_W + x];
}

static inline int target_w(int h)
{
    return h * PENGUIN_W / PENGUIN_H;
}

// Draw the penguin centered at (cx, cy), scaled to height h (nearest neighbor).
static void draw_penguin(u16 *fb, int cx, int cy, int h)
{
    int w = target_w(h);
    int x0 = cx - w / 2, y0 = cy - h / 2;
    for (int y = 0; y < h; y++) {
        int sy = y * PENGUIN_H / h;
        for (int x = 0; x < w; x++) {
            u16 c = penguin_px[sy * PENGUIN_W + x * PENGUIN_W / w];
            if (c != PENGUIN_KEY)
                putpx(fb, x0 + x, y0 + y, c);
        }
    }
}

static void draw_crosshair(u16 *fb)
{
    u16 c = COL565(255, 255, 255);
    for (int i = -10; i <= 10; i++) {
        if (i > -3 && i < 3) continue;
        putpx(fb, 200 + i, 120, c);
        putpx(fb, 200, 120 + i, c);
    }
}

static float wrap_angle(float a)
{
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void spawn(Target *t)
{
    t->yaw   = ((rand() % 628) - 314) / 100.0f;   // anywhere around you
    t->pitch = ((rand() % 50) - 25) / 100.0f;     // a bit above/below eye level
    t->depth = (rand() % 100) / 100.0f;
    t->h     = 60 + rand() % 40;
}

// ---------------------------------------------------------------------------
// Camera runs on its own thread so it never waits on the game loop.
// Three buffers: one the camera is filling, one holding the newest finished
// frame, and one the game may be drawing from. They never overlap, so no tearing.
// ---------------------------------------------------------------------------
static u16 *camBuf[3];
static volatile int camLatest = -1;   // newest finished frame (-1 = none yet)
static volatile int camInUse = -1;    // buffer the game is drawing from
static volatile bool camRunning = true;
static u32 camUnit = 0;
static Handle camErrEvent = 0;

static void cam_thread(void *arg)
{
    (void)arg;
    Handle ev[2] = {camErrEvent, 0};    // [0] = capture broke, [1] = frame finished
    int recv = 0;

    CAMU_ClearBuffer(PORT_CAM1);
    CAMU_StartCapture(PORT_CAM1);

    while (camRunning) {
        if (!ev[1])
            CAMU_SetReceiving(&ev[1], camBuf[recv], PORT_CAM1, CAM_BYTES, (s16)camUnit);

        s32 idx = -1;
        Result rc = svcWaitSynchronizationN(&idx, ev, 2, false, CAM_WAIT_NS);

        if (rc == 0 && idx == 1) {
            // Frame done: publish it, then pick a buffer that's neither the
            // newest frame nor the one the game is drawing.
            svcCloseHandle(ev[1]);
            ev[1] = 0;
            camLatest = recv;
            for (int i = 0; i < 3; i++)
                if (i != camLatest && i != camInUse) { recv = i; break; }
        } else {
            // Buffer error or no frame in time: restart capture.
            if (ev[1]) { svcCloseHandle(ev[1]); ev[1] = 0; }
            CAMU_StopCapture(PORT_CAM1);
            CAMU_ClearBuffer(PORT_CAM1);
            CAMU_StartCapture(PORT_CAM1);
        }
    }

    if (ev[1]) svcCloseHandle(ev[1]);
    CAMU_StopCapture(PORT_CAM1);
}

int main(void)
{
    gfxInitDefault();
    gfxSetScreenFormat(GFX_TOP, GSP_RGB565_OES);  // match camera output, no conversion
    gfxSet3D(true);
    consoleInit(GFX_BOTTOM, NULL);

    srand((unsigned)osGetTime());

    // --- Camera setup (outer right camera) ---
    camInit();
    CAMU_SetSize(SELECT_OUT1, SIZE_CTR_TOP_LCD, CONTEXT_A);
    CAMU_SetOutputFormat(SELECT_OUT1, OUTPUT_RGB_565, CONTEXT_A);
    CAMU_SetFrameRate(SELECT_OUT1, FRAME_RATE_30);
    CAMU_SetNoiseFilter(SELECT_OUT1, true);
    CAMU_SetAutoExposure(SELECT_OUT1, true);
    CAMU_SetAutoWhiteBalance(SELECT_OUT1, true);
    CAMU_SetTrimming(PORT_CAM1, false);

    CAMU_GetMaxBytes(&camUnit, CAM_W, CAM_H);
    CAMU_SetTransferBytes(PORT_CAM1, camUnit, CAM_W, CAM_H);
    CAMU_Activate(SELECT_OUT1);

    CAMU_GetBufferErrorInterruptEvent(&camErrEvent, PORT_CAM1);
    for (int i = 0; i < 3; i++) {
        camBuf[i] = (u16 *)linearAlloc(CAM_BYTES);
        memset(camBuf[i], 0, CAM_BYTES);
    }

    // Start the camera thread one priority step above the game so it always
    // gets to re-arm the camera the moment a frame finishes.
    s32 prio = 0x30;
    svcGetThreadPriority(&prio, CUR_THREAD_HANDLE);
    Thread camThread = threadCreate(cam_thread, NULL, 0x4000, prio - 1, -2, false);

    // --- Gyro ---
    HIDUSER_EnableGyroscope();

    Target targets[NUM_TARGETS];
    for (int i = 0; i < NUM_TARGETS; i++) spawn(&targets[i]);

    float yaw = 0.0f, pitch = 0.0f;
    float aimX = AIM_X_SIGN, aimY = AIM_Y_SIGN, gyroScale = GYRO_SCALE;
    int score = 0, throws = 0;
    u64 last = osGetTime();

    while (aptMainLoop()) {
        hidScanInput();
        u32 kDown = hidKeysDown();
        if (kDown & KEY_START) break;
        if (kDown & KEY_SELECT) { yaw = 0.0f; pitch = 0.0f; }
        if (kDown & (KEY_DUP | KEY_DDOWN))    { aimY = -aimY; yaw = 0.0f; pitch = 0.0f; }
        if (kDown & (KEY_DLEFT | KEY_DRIGHT)) { aimX = -aimX; yaw = 0.0f; pitch = 0.0f; }
        if (kDown & KEY_L) gyroScale *= 0.8f;
        if (kDown & KEY_R) gyroScale *= 1.25f;

        // Integrate gyro into a view direction.
        u64 now = osGetTime();
        float dt = (now - last) / 1000.0f;
        last = now;

        angularRate rate;
        hidGyroRead(&rate);
        yaw   = wrap_angle(yaw + aimX * rate.y * gyroScale * dt);
        pitch = pitch + aimY * rate.x * gyroScale * dt;
        if (pitch > 1.2f) pitch = 1.2f;
        if (pitch < -1.2f) pitch = -1.2f;

        // --- Camera: grab the newest finished frame ---
        int show = camLatest;
        camInUse = show;
        const u16 *cam = show >= 0 ? camBuf[show] : NULL;
        if (cam) GSPGPU_InvalidateDataCache(cam, CAM_BYTES);   // make sure we read fresh pixels

        float slider = osGet3DSliderState();
        u16 *fbL = (u16 *)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
        u16 *fbR = (u16 *)gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL);

        if (cam) {
            blit_camera(fbL, cam);
            blit_camera(fbR, cam);
        } else {
            memset(fbL, 0, 400 * 240 * 2);
            memset(fbR, 0, 400 * 240 * 2);
        }

        int thrown = (kDown & KEY_A) != 0;
        if (thrown) throws++;
        int onScreen = 0;

        for (int i = 0; i < NUM_TARGETS; i++) {
            Target *t = &targets[i];
            float dyaw = wrap_angle(t->yaw - yaw);
            if (fabsf(dyaw) > 0.8f) continue;   // behind you / off screen

            int sx = 200 + (int)(dyaw * PX_PER_RAD);
            int sy = 120 + (int)((pitch - t->pitch) * PX_PER_RAD);
            int w = target_w(t->h);
            if (sx < -w || sx > 400 + w || sy < -t->h || sy > 240 + t->h) continue;
            onScreen++;

            // Crossed disparity: left eye shifted right, right eye shifted left = pops out.
            int d = (int)(t->depth * slider * MAX_DISPARITY);
            draw_penguin(fbL, sx + d, sy, t->h);
            draw_penguin(fbR, sx - d, sy, t->h);

            if (thrown) {
                // Hit if the crosshair is inside the penguin's box (slightly shrunk).
                if (abs(sx - 200) <= w * 2 / 5 && abs(sy - 120) <= t->h * 2 / 5) {
                    score++;
                    thrown = 0;         // one hit per snowball
                    spawn(t);
                }
            }
        }

        draw_crosshair(fbL);
        draw_crosshair(fbR);

        // Bottom screen HUD
        printf("\x1b[1;1H  SNOWBALLERS AR");
        printf("\x1b[3;1H  Score: %d      ", score);
        printf("\x1b[4;1H  Snowballs: %d      ", throws);
        printf("\x1b[6;1H  Penguins in view: %d   ", onScreen);
        printf("\x1b[8;1H  Turn around to find penguins!");
        printf("\x1b[10;1H  A: throw  SELECT: recenter");
        printf("\x1b[11;1H  START: quit");
        printf("\x1b[13;1H  yaw %+.2f  pitch %+.2f   ", yaw, pitch);
        printf("\x1b[14;1H  gyro x%+6d y%+6d z%+6d ", rate.x, rate.y, rate.z);
        printf("\x1b[16;1H  Aim X %+.0f  Aim Y %+.0f  Sens %.5f   ", aimX, aimY, gyroScale);
        printf("\x1b[17;1H  DPAD: flip aim   L/R: sensitivity");

        gfxFlushBuffers();
        gfxSwapBuffers();
        gspWaitForVBlank();
    }

    // --- Cleanup ---
    HIDUSER_DisableGyroscope();
    camRunning = false;
    threadJoin(camThread, U64_MAX);
    threadFree(camThread);
    if (camErrEvent) svcCloseHandle(camErrEvent);
    CAMU_Activate(SELECT_NONE);
    camExit();
    for (int i = 0; i < 3; i++) linearFree(camBuf[i]);
    gfxExit();
    return 0;
}
