/* SPDX-License-Identifier: MIT */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "sandbox.h"
#include <stdio.h>
#include <stdlib.h>
static bool running = true, jump_pressed = false, capturiptide_pressed = false;
static LRESULT CALLBACK window_proc(HWND window, UINT msg, WPARAM w, LPARAM l) {
    if (msg == WM_DESTROY) {
        running = false;
        PostQuitMessage(0);
        return 0;
    }
    if (msg == WM_KEYDOWN && !(l & (1L << 30))) {
        if (w == VK_ESCAPE)
            DestroyWindow(window);
        if (w == 'J')
            jump_pressed = true;
        if (w == VK_F12)
            capturiptide_pressed = true;
    }
    return DefWindowProcW(window, msg, w, l);
}
static bool held(int key) {
    return (GetAsyncKeyState(key) & 0x8000) != 0;
}
int WINAPI WinMain(HINSTANCE instance, HINSTANCE previous, LPSTR command, int show) {
    (void)previous;
    (void)command;
    WNDCLASSW wc = {0};
    wc.lpfnWndProc = window_proc;
    wc.hInstance = instance;
    wc.lpszClassName = L"RiptideSandbox";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    if (!RegisterClassW(&wc))
        return 1;
    HWND window = CreateWindowW(wc.lpszClassName,
                                L"Riptide Engine | W/S drive  A/D steer  Space brake  Shift drift  "
                                L"Ctrl boost  J jump  F12 capture",
                                WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 1024, 680, NULL,
                                NULL, instance, NULL);
    if (!window)
        return 1;
    Sandbox s;
    if (!sandbox_init(&s, 640, 400)) {
        MessageBoxW(window, L"Could not allocate sandbox buffers.", L"Riptide Engine", MB_ICONERROR);
        DestroyWindow(window);
        return 1;
    }
    RiptideClock clock;
    riptide_clock_init(&clock, 120, 12);
    LARGE_INTEGER frequency, last;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&last);
    unsigned char *bgra = malloc((size_t)s.width * s.height * 4);
    if (!bgra) {
        sandbox_free(&s);
        DestroyWindow(window);
        return 1;
    }
    BITMAPINFO bitmap = {0};
    bitmap.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bitmap.bmiHeader.biWidth = (LONG)s.width;
    bitmap.bmiHeader.biHeight = -(LONG)s.height;
    bitmap.bmiHeader.biPlanes = 1;
    bitmap.bmiHeader.biBitCount = 32;
    bitmap.bmiHeader.biCompression = BI_RGB;
    ShowWindow(window, show);
    unsigned serial = 0;
    while (running) {
        MSG msg;
        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (!running)
            break;
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        double elapsed = (double)(now.QuadPart - last.QuadPart) / frequency.QuadPart;
        last = now;
        if (IsIconic(window) || GetForegroundWindow() != window) {
            clock.accumulator = 0;
            Sleep(20);
            continue;
        }
        RiptideControls input = {.throttle = (held('W') || held(VK_UP) ? 1.f : 0) -
                                        (held('S') || held(VK_DOWN) ? 1.f : 0),
                            .steering = (held('D') || held(VK_RIGHT) ? 1.f : 0) -
                                        (held('A') || held(VK_LEFT) ? 1.f : 0),
                            .brake = held(VK_SPACE) ? 1.f : 0,
                            .drift = held(VK_SHIFT),
                            .boost = held(VK_CONTROL)};
        if (jump_pressed) {
            if (s.player.grounded)
                riptide_vehicle_impulse(&s.player, &s.handling, (RiptideVec3){0, s.handling.mass * 7, 0});
            jump_pressed = false;
        }
        unsigned steps = riptide_clock_advance(&clock, elapsed);
        for (unsigned i = 0; i < steps; i++)
            sandbox_step(&s, input, (float)clock.step);
        if (!sandbox_draw(&s, riptide_clock_alpha(&clock))) {
            running = false;
            break;
        }
        const unsigned char *pixels = riptide_canvas_pixels(s.canvas);
        if (capturiptide_pressed) {
            char path[64];
            snprintf(path, sizeof path, "riptide-sandbox-%04u.png", serial++);
            if (!riptide_image_save(path, pixels, s.width, s.height, s.width * 4, false))
                MessageBoxW(window, L"Could not save the screenshot in the working directory.",
                            L"Riptide Engine", MB_ICONERROR);
            capturiptide_pressed = false;
        }
        for (size_t i = 0; i < (size_t)s.width * s.height; i++) {
            bgra[i * 4] = pixels[i * 4 + 2];
            bgra[i * 4 + 1] = pixels[i * 4 + 1];
            bgra[i * 4 + 2] = pixels[i * 4];
            bgra[i * 4 + 3] = 255;
        }
        RECT area;
        GetClientRect(window, &area);
        HDC dc = GetDC(window);
        StretchDIBits(dc, 0, 0, area.right, area.bottom, 0, 0, (int)s.width, (int)s.height, bgra,
                      &bitmap, DIB_RGB_COLORS, SRCCOPY);
        ReleaseDC(window, dc);
        Sleep(1);
    }
    free(bgra);
    sandbox_free(&s);
    if (IsWindow(window))
        DestroyWindow(window);
    return 0;
}
