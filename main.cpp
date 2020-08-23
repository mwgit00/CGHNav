// MIT License
//
// Copyright(c) 2020 Mark Whitney
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#include "FakeLidar.h"
#include "FakeRobot.h"
#include "GHNav.h"


using namespace cv;


#define MOVIE_PATH  ".\\movie\\"    // user may need to create or change this
#define DATA_PATH   ".\\data\\"     // user may need to change this

#define SCA_BLACK   (cv::Scalar(0,0,0))
#define SCA_RED     (cv::Scalar(0,0,255))
#define SCA_GREEN   (cv::Scalar(0,255,0))
#define SCA_YELLOW  (cv::Scalar(0,255,255))
#define SCA_BLUE    (cv::Scalar(255,0,0))
#define SCA_WHITE   (cv::Scalar(255,255,255))
#define SCA_DKGRAY  (cv::Scalar(64,64,64))



const double vspinfac = 0.08;
const double avel1 = 1 * vspinfac;
const double avel2 = 2 * vspinfac;
const double avel3 = 4 * vspinfac;
const double avel4 = 7 * vspinfac;
const double avelf = 1.6;


static bool is_rec_enabled = false;
static bool is_loc_enabled = false;
static bool is_resync = true;   // resync map (do on first iteration)
static bool is_rehome = true;   // new position (do on first iteration)
const char* stitle = "CGHNav Test Application";
int n_record_ctr = 0;

cpoz::FakeLidar theLidar;
cpoz::FakeRobot theRobot;



bool wait_and_check_keys(const int delay_ms = 1)
{
    bool result = true;

    int nkey = waitKey(delay_ms);
    char ckey = static_cast<char>(nkey);

    // check that a keypress has been returned
    if (nkey >= 0)
    {
        if (ckey == 27)
        {
            // done if ESC has been pressed
            result = false;
        }
        else
        {
            switch (ckey)
            {
            case 'r': is_rec_enabled = !is_rec_enabled; break;
            case '1':
            {
                // noiseless LIDAR
                theLidar.get_params().range_dec_pt_adjust = 1000;
                theLidar.get_params().jitter_angle_deg_u = 0.0;
                theLidar.get_params().jitter_range_u = 0.0;
                theLidar.get_params().jitter_sync_deg_u = 0.0;
                break;
            }
            case '2':
            {
                // slightly noisy LIDAR
                theLidar.get_params().range_dec_pt_adjust = 1;
                theLidar.get_params().jitter_angle_deg_u = 0.2;
                theLidar.get_params().jitter_range_u = 0.5;
                theLidar.get_params().jitter_sync_deg_u = 0.25;
                break;
            }
            case '3':
            {
                // noisy LIDAR
                theLidar.get_params().range_dec_pt_adjust = 1;
                theLidar.get_params().jitter_angle_deg_u = 0.3;
                theLidar.get_params().jitter_range_u = 2.0;
                theLidar.get_params().jitter_sync_deg_u = 0.35;
                break;
            }
            case '4':
            {
                // really noisy LIDAR
                theLidar.get_params().range_dec_pt_adjust = 1;
                theLidar.get_params().jitter_angle_deg_u = 0.5;
                theLidar.get_params().jitter_range_u = 4.0;
                theLidar.get_params().jitter_sync_deg_u = 0.5;
                break;
            }
            case 'a': theRobot.set_vLR({ -avel4, avel4 }); break;
            case 's': theRobot.set_vLR({ -avel3, avel3 }); break;
            case 'd': theRobot.set_vLR({ -avel2, avel2 }); break;
            case 'f': theRobot.set_vLR({ -avel1, avel1 }); break;
            case 'g':
            case '\'':
            {
                theRobot.set_vLR({ avelf, avelf });
                break;
            }
            case 'h': theRobot.set_vLR({ avel1, -avel1 }); break;
            case 'j': theRobot.set_vLR({ avel2, -avel2 }); break;
            case 'k': theRobot.set_vLR({ avel3, -avel3 }); break;
            case 'l': theRobot.set_vLR({ avel4, -avel4 }); break;
            case 'b':
            case ';':
            {
                theRobot.set_vLR({ 0.0, 0.0 });
                break;
            }
            case '[': theRobot.accel({ -0.2, 0.0 }, avelf); break;
            case ']': theRobot.accel({ 0.0, -0.2 }, avelf); break;
            case '=': is_resync = true; break;
            case '0': is_rehome = true; break;
            default: break;
            }
        }
    }

    return result;
}


void image_output(cv::Mat& rimg)
{
    static int ctr = 0;
    
    // save each frame to a file if recording
    if (is_rec_enabled)
    {
        ctr++;
        Scalar sca = ((ctr >> 3) & 1) ? SCA_RED : SCA_DKGRAY;
        
        // blinking red box in upper right corner if recording
        int x = rimg.size().width - 20;
        rectangle(rimg, { x, 0, x + 20, 20 }, sca, -1);

        std::ostringstream osx;
        osx << MOVIE_PATH << "img_" << std::setfill('0') << std::setw(5) << n_record_ctr << ".png";
        imwrite(osx.str(), rimg);
        n_record_ctr++;
    }

    imshow(stitle, rimg);
}


void loop(void)
{
    Mat img_orig;
    Mat img_viewer;
    Mat img_viewer_bgr;
    Point pt_drawing_offset;
    int ticker = 0;

    cpoz::GHNav ghnav;
    Point match_offset = { 0, 0 };
    double match_angle = 0.0;

    // various starting positions in default floorplan
    std::vector<Point2d> vhomepos;
    vhomepos.push_back({ 560.0, 360.0 });
    vhomepos.push_back({ 560.0, 540.0 });
    vhomepos.push_back({ 650.0, 140.0 });
    vhomepos.push_back({ 870.0, 360.0 });
    vhomepos.push_back({ 930.0, 630.0 });
    vhomepos.push_back({ 1040.0, 110.0 });
    vhomepos.push_back({ 1180.0, 440.0 });
    size_t iivhomepos = 0;

    ghnav.init_scan_angs();
    theLidar.set_scan_angs(ghnav.get_scan_angs());
    theLidar.load_floorplan(".\\docs\\apt_1cmpp_720p.png");

    img_orig = imread(".\\docs\\apt_1cmpp_720p.png", IMREAD_GRAYSCALE);

    // and the processing loop is running...
    bool is_running = true;

    while (is_running)
    {
        // handle keyboard events and end when ESC is pressed
        is_running = wait_and_check_keys(25);

        if (is_rehome)
        {
            // one-shot keypress
            is_rehome = false;

            // reset offset for drawing map info
            // stop robot and put it at new position and orientation
            // will need to resync to new position
            pt_drawing_offset = vhomepos[iivhomepos];
            theRobot.set_vLR({ 0.0, 0.0 });
            theRobot.set_xypos_ang(vhomepos[iivhomepos], 0.0);
            is_resync = true;

            iivhomepos++;
            if (iivhomepos == vhomepos.size()) iivhomepos = 0;
        }

        // determine robot's new position and heading
        theRobot.update();

        // robot has a new position so take a new LIDAR scan...
        theLidar.set_world_pos(theRobot.get_xypos());
        theLidar.set_world_ang(theRobot.get_ang());
        theLidar.run_scan();

        if (is_resync)
        {
            // apply latest scan as new waypoint
            ghnav.update_match_templates(theLidar.get_last_scan());
            is_resync = false;
        }

        // check match against the current waypoint
        ghnav.perform_match(theLidar.get_last_scan(), match_offset, match_angle);

        // now update the screen view...
        
        // init image output with source floorplan (gray)
        img_orig.copyTo(img_viewer);

        // preprocess the scan for drawing
        cpoz::GHNav::T_PREPROC preproc;
        ghnav.preprocess_scan(preproc, theLidar.get_last_scan(), 0, ghnav.get_search_resize());

        // draw "snapshot" image of current LIDAR scan (gray)
        Mat img_current_scan;
        Point img_current_scan_pt0;
        ghnav.draw_preprocessed_scan(img_current_scan, img_current_scan_pt0, preproc);
        Rect mroi = { {0,0}, img_current_scan.size() };
        img_current_scan.copyTo(img_viewer(mroi));

        // switch to BGR...
        cvtColor(img_viewer, img_viewer_bgr, COLOR_GRAY2BGR);

        // draw LIDAR scan lines over floorplan
        theLidar.draw_last_scan(img_viewer_bgr, SCA_DKGRAY);

        // draw robot position and direction in LIDAR scan in upper left
        circle(img_viewer_bgr, img_current_scan_pt0, 3, SCA_GREEN, -1);
        line(img_viewer_bgr, img_current_scan_pt0, img_current_scan_pt0 + Point{ 10, 0 }, SCA_GREEN, 1);

        // draw robot in floorplan
        const int r = theRobot.get_radius();
        Point ibotpos = theRobot.get_xypos();
        circle(img_viewer_bgr, ibotpos, r, SCA_WHITE, -1);
        circle(img_viewer_bgr, ibotpos, r, SCA_BLACK, 1);
        circle(img_viewer_bgr, ibotpos, 7, SCA_GREEN, -1);
        Point2d angvec = theRobot.get_ang_vec();
        int angx = static_cast<int>(angvec.x * r * 0.8);
        int angy = static_cast<int>(angvec.y * r * 0.8);
        line(img_viewer_bgr, ibotpos, ibotpos + Point{ angx, angy }, SCA_GREEN, 3);

        {
            // print robot position in image
            std::ostringstream oss;
            oss << " IMG:XY@ = " << std::setw(4) << ibotpos.x << ", " << ibotpos.y;
            oss << "  " << std::fixed << std::setprecision(1) << theRobot.get_ang();
            putText(img_viewer_bgr, oss.str(), { 0, 400 }, FONT_HERSHEY_PLAIN, 2.0, SCA_BLACK, 2);
        }

        {
            std::ostringstream oss;
            oss << " MATCH = " << std::setw(4) << match_offset.x << ", " << match_offset.y;
            oss << "  " << std::fixed << std::setprecision(1) << match_angle;
            putText(img_viewer_bgr, oss.str(), { 0, 430 }, FONT_HERSHEY_PLAIN, 2.0, SCA_BLUE, 2);
        }

        // show image of the Hough bins but scaled and equalized to make it pretty
        Mat img_acc;
        Mat img_acc8;
        Mat img_acc8_bgr;
        normalize(ghnav.m_img_acc, img_acc, 0, 255, cv::NORM_MINMAX);
        img_acc.convertTo(img_acc8, CV_8UC1);
        equalizeHist(img_acc8, img_acc8);
        cvtColor(img_acc8, img_acc8_bgr, COLOR_GRAY2BGR);
        circle(img_acc8_bgr, ghnav.m_img_acc_pt, 2, { 0,0,255 }, -1);
        Rect mroix = { { 25, 460 }, Size(ghnav.m_acc_fulldim, ghnav.m_acc_fulldim) };
        img_acc8_bgr.copyTo(img_viewer_bgr(mroix));

        // show the BGR image
        image_output(img_viewer_bgr);
    }

    // when everything is done, release the capture device and windows
    cv::destroyAllWindows();
}


int main()
{
    std::cout << stitle << std::endl;
    loop();
    return 0;
}
