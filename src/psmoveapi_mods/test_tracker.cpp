
 /**
  * call scripts/visualc/build_msvc.bat 2022 x64
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012, 2022 Thomas Perl <m@thp.io>
 * Copyright (c) 2012 Benjamin Venditt <benjamin.venditti@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

#include <stdio.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "psmove.h"
#include "psmove_tracker.h"
#include "psmove_tracker_opencv.h"
#include "psmove_format.h"

#include <memory>
#include <thread>


static const char *
TEST_TRACKER_WINDOW_NAME = "PS Move API: Tracker Test";


struct ControllerState {
    ControllerState(PSMove *move) : move(move) {}

    ~ControllerState()
    {
        psmove_set_leds(move, 0, 0, 0);
        psmove_update_leds(move);
        psmove_disconnect(move);
    }

    ControllerState(const ControllerState &) = delete;
    ControllerState(ControllerState &&) = delete;
    ControllerState &operator=(const ControllerState &) = delete;
    ControllerState &operator=(ControllerState &&) = delete;

    PSMove *move { nullptr };
    cv::Rect ui_rect;
    bool is_tracking { false };
    bool is_hovering { false };
};

bool terminate_thread = false;

void print_status(PSMove *move, PSMove *move2) {
    char *serial = psmove_get_serial(move);
    while (true) {
        if (terminate_thread) {
            return;
        }
        int res = psmove_poll(move);
        int res2 = psmove_poll(move2);
        if (res) {
            int x1, y1, z1, x2, y2, z2, x3, y3, z3;
            psmove_get_accelerometer(move, &x1, &y1, &z1);
            psmove_get_gyroscope(move, &x2, &y2, &z2);
            psmove_get_magnetometer(move, &x3, &y3, &z3);
            printf("state %s %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %x %x %5d\n", serial, psmove_get_trigger(move), x1, y1, z1, x2, y2, z2, x3, y3, z3, psmove_get_buttons(move), psmove_get_buttons(move2), psmove_get_trigger(move2));
            fflush(stdout);
        }
    }
}

struct TrackerTestApp {
    ~TrackerTestApp()
    {
        psmove_tracker_free(tracker);

        controllers.clear();
    }

    void set_exposure(float exposure)
    {
        psmove_tracker_set_exposure(tracker, exposure);
        this->exposure = psmove_tracker_get_exposure(tracker);
    }

    void init(int count)
    {
        PSMoveTrackerSettings settings;
        psmove_tracker_settings_set_default(&settings);

        settings.camera_exposure = exposure;
        settings.camera_mirror = true;

        PSMOVE_INFO("Trying to init PSMoveTracker...");
        tracker = psmove_tracker_new_with_settings(&settings);
        PSMOVE_VERIFY(tracker != nullptr, "Could not init PSMoveTracker");

        for (int i=0; i<count; i++) {
            PSMOVE_INFO("Opening controller %d", i);
            controllers.emplace_back(std::make_unique<ControllerState>(psmove_connect_by_id(i)));
            PSMOVE_VERIFY(controllers.back(), "Controller %d not found", i);
        }

    }


    void update()
    {
        psmove_tracker_update_image(tracker);
        psmove_tracker_update(tracker, NULL);
        psmove_tracker_annotate(tracker, draw_statusbar, draw_rois);

        IplImage *frame = psmove_tracker_opencv_get_frame(tracker);
        if (frame) {
            CvFont fontSmall = cvFont(0.8, 1);

            int x = 30;
            int y = 30;

            for (auto &controller: controllers) {
                int w = 130;
                int h = 20;

                controller->ui_rect = cv::Rect(x, y, w, h);

                CvPoint tl { controller->ui_rect.tl().x, controller->ui_rect.tl().y };
                CvPoint br { controller->ui_rect.br().x, controller->ui_rect.br().y };

                CvScalar color { 255, 255, 255, 255 };

                uint8_t r, g, b;
                controller->is_tracking = (psmove_tracker_get_color(tracker, controller->move, &r, &g, &b) > 0);
                if (controller->is_tracking) {
                    uint8_t m = std::max(r, std::max(g, b));
                    if (m != 0) {
                        color.val[2] = r * 255 / m;
                        color.val[1] = g * 255 / m;
                        color.val[0] = b * 255 / m;
                    }
                } else {
                    if (controller->is_hovering) {
                        uint8_t r = 255, g = 255, b = 255;
                        psmove_tracker_get_next_unused_color(tracker, &r, &g, &b);
                        color.val[2] = r;
                        color.val[1] = g;
                        color.val[0] = b;

                        if (!controller->is_tracking) {
                            psmove_set_leds(controller->move, r, g, b);
                            psmove_update_leds(controller->move);
                        }
                    } else {
                        color.val[0] *= 0.8;
                        color.val[1] *= 0.8;
                        color.val[2] *= 0.8;
                    }
                }

                cvRectangle(frame, tl, br, cvScalar(0, 0, 0, 128), CV_FILLED);
                cvRectangle(frame, tl, br, color, 1);

                CvPoint txt = tl;
                txt.x += 5;
                txt.y += h / 2 + 5;

                char *serial = psmove_get_serial(controller->move);
                
                if (psmove_connection_type(controller->move) != Conn_USB) {
                    float x4, y4, radi;
                    psmove_tracker_get_position(tracker, controller->move, &x4, &y4, &radi);
                    printf("state2 %s %i %f %f %f\n", serial, controller->is_tracking, x4, y4, radi);
                    fflush(stdout);
                }
                cvPutText(frame, serial, txt, &fontSmall, color);

                psmove_free_mem(serial);

                y += h + 5;
            }

            CvPoint txt { x, y };

            auto println = [&] (const std::string &text) {
                txt.y += 10;
                txt.x -= 1; txt.y -= 1;
                cvPutText(frame, text.c_str(), txt, &fontSmall, CvScalar{0.0, 0.0, 0.0, 0.0});
                txt.x += 1; txt.y += 1;
                cvPutText(frame, text.c_str(), txt, &fontSmall, CvScalar{255.0, 255.0, 255.0, 255.0});
            };

            println(format("Hover to highlight, click to toggle tracking"));
            println(format("Press 'R' to reset tracking for all"));
            println(format("Press 'D' to toggle ROI, 'S' to toggle statusbar"));
            println(format("Press 'H' to hue-calibrate, 'B' to blink-calibrate"));
            println(format("Exposure: %.2f (press 'E' to cycle)", exposure));

            const auto camera_info = psmove_tracker_get_camera_info(tracker);

            println(format("%s %dx%d (%s)", camera_info->camera_name, camera_info->width, camera_info->height, camera_info->camera_api));

            cvShowImage(TEST_TRACKER_WINDOW_NAME, frame);
        }
    }

    void onmouse(int event, int x, int y, int flags)
    {
        for (auto &controller: controllers) {
            uint8_t r, g, b;
            controller->is_tracking = (psmove_tracker_get_color(tracker, controller->move, &r, &g, &b) > 0);

            if (controller->ui_rect.contains(cv::Point(x, y))) {
                if (event == CV_EVENT_LBUTTONDOWN) {
                    if (!controller->is_tracking) {
                        enum PSMoveTracker_Status result = psmove_tracker_enable(tracker, controller->move);

                        if (result == Tracker_CALIBRATED) {
                            PSMOVE_INFO("Tracking started");
                        } else {
                            PSMOVE_WARNING("Tracker enable failed");
                        }
                    } else {
                        psmove_tracker_disable(tracker, controller->move);
                        psmove_set_leds(controller->move, 0, 0, 0);
                        psmove_update_leds(controller->move);
                    }
                }
                controller->is_hovering = true;
            } else {
                if (!controller->is_tracking) {
                    psmove_set_leds(controller->move, 0, 0, 0);
                    psmove_update_leds(controller->move);
                }
                controller->is_hovering = false;
            }
        }
    }

    void reset()
    {
        for (auto &controller: controllers) {
            psmove_tracker_disable(tracker, controller->move);
            psmove_set_leds(controller->move, 0, 0, 0);
            psmove_update_leds(controller->move);
        }

        for (auto &controller: controllers) {
            psmove_tracker_enable(tracker, controller->move);
        }
    }

    void hue_calibration()
    {
        int res = psmove_tracker_hue_calibration(tracker, controllers[0]->move);
        PSMOVE_INFO("Hue calibration found %d colors", res);
    }

    void reset_color_calibration()
    {
        psmove_tracker_reset_color_calibration(tracker);
    }

    PSMoveTracker *tracker { nullptr };
    std::vector<std::unique_ptr<ControllerState>> controllers;
    bool draw_statusbar { false };
    bool draw_rois { false };
    float exposure { 0.1f };
};

static void
test_tracker_on_mouse(int event, int x, int y, int flags, void *userdata)
{
    auto app = static_cast<TrackerTestApp *>(userdata);

    app->onmouse(event, x, y, flags);
}

int
main(int arg, char *args[])
{
    int count = psmove_count_connected();

    PSMOVE_INFO("%d controllers connected", count);

    if (count == 0) {
        return 1;
    }

    TrackerTestApp app;

    cvNamedWindow(TEST_TRACKER_WINDOW_NAME);
    cvSetMouseCallback(TEST_TRACKER_WINDOW_NAME, test_tracker_on_mouse, &app);

    app.init(count);

    PSMove *move;
    move = psmove_connect_by_id(1);
    PSMove *move2;
    move2 = psmove_connect_by_id(0);
    int res = 0;

    // psmove_set_rate_limiting(move, 1);
    std::thread thread_obj(print_status, move, move2);
    
    while (true) {
        int key = cvWaitKey(1) & 0xFF;
        if (key == 27) {
            terminate_thread = true;
            break;
        }
        if (cvGetWindowProperty(TEST_TRACKER_WINDOW_NAME, CV_WND_PROP_VISIBLE) < 1) {
            terminate_thread = true;
            break;
        }
        if (key == 'r') {
            app.reset();
        } else if (key == 's') {
            app.draw_statusbar = !app.draw_statusbar;
        } else if (key == 'd') {
            app.draw_rois = !app.draw_rois;
        } else if (key == 'e') {
            if (app.exposure == 0.1f) {
                app.set_exposure(0.5f);
            } else if (app.exposure == 0.5f) {
                app.set_exposure(1.f);
            } else {
                app.set_exposure(0.1f);
            }
        } else if (key == 'h') {
            app.reset_color_calibration();
            app.hue_calibration();
            app.reset();
        } else if (key == 'b') {
            app.reset_color_calibration();
            app.reset();
        }

        
        app.update();
    }
    thread_obj.join();
    printf("estado_final");
    fflush(stdout);
    return 0;
}
