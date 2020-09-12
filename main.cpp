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
#include <chrono>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#include "FakeLidar.h"
#include "FakeRobot.h"
#include "GHNav.h"
#include "util.h"


using namespace cv;


#define MOVIE_PATH_PREFIX   ".\\movie_"

#define SCA_BLACK   (cv::Scalar(0,0,0))
#define SCA_RED     (cv::Scalar(0,0,255))
#define SCA_GREEN   (cv::Scalar(0,255,0))
#define SCA_YELLOW  (cv::Scalar(0,255,255))
#define SCA_BLUE    (cv::Scalar(255,0,0))
#define SCA_CYAN    (cv::Scalar(255,255,0))
#define SCA_WHITE   (cv::Scalar(255,255,255))
#define SCA_DKGRAY  (cv::Scalar(64,64,64))



const double vspinfac = 0.08;
const double avel1 = 1 * vspinfac;
const double avel2 = 2 * vspinfac;
const double avel3 = 4 * vspinfac;
const double avel4 = 7 * vspinfac;
const double avelf = 1.6;

static int g_fps_ct = 0;
static int g_fps = 0;

static bool is_rec_enabled = false;
static bool is_loc_enabled = false;
static bool is_resync = true;   // resync map (do on first iteration)
static bool is_rehome = true;   // new position (do on first iteration)
const char* stitle = "CGHNav Test Application";
static int g_movie_ctr = 0;
static int g_record_ctr = 0;
static std::string g_sdir;

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
            case 'r':
            {
                is_rec_enabled = !is_rec_enabled;
                if (is_rec_enabled)
                {
                    // recording has been turned on
                    g_movie_ctr++;
                    g_record_ctr = 0;
                    
                    std::ostringstream osx;
                    osx << MOVIE_PATH_PREFIX;
                    osx << std::setfill('0') << std::setw(2) << g_movie_ctr;
                    osx << "\\";
                    g_sdir = osx.str();
                    if (!(dir_exists(g_sdir)))
                    {
                        // create it
                        (void)create_dir(g_sdir);
                    }
                }
                break;
            }
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

    // display frames per second at top of screen
    std::ostringstream oss;
    oss << g_fps << "fps";
    putText(rimg, oss.str(), { 1000, 32 }, FONT_HERSHEY_PLAIN, 2.0, SCA_BLACK, 2);

    // save each frame to a file if recording
    if (is_rec_enabled)
    {
        ctr++;
        Scalar sca = ((ctr >> 3) & 1) ? SCA_RED : SCA_DKGRAY;
        
        // blinking red box in upper right corner if recording
        int x = rimg.size().width - 20;
        rectangle(rimg, { x, 0, x + 20, 20 }, sca, -1);

        std::ostringstream osx;
        osx << "img_" << std::setfill('0') << std::setw(5) << g_record_ctr << ".png";

        std::string sframe = g_sdir + osx.str();
        imwrite(sframe, rimg);
        g_record_ctr++;
    }

    imshow(stitle, rimg);
}


void loop(void)
{
    Mat img_orig;
    Mat img_viewer;
    Mat img_viewer_bgr;

    cpoz::GHNav ghnav;
    Point match_offset = { 0, 0 };
    double match_angle = 0.0;
    
    Point home_pos;
    Point2d home_ang_vec;
    double home_ang;

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

    theLidar.set_scan_angs(ghnav.get_scan_angs());
    theLidar.load_floorplan(".\\docs\\apt_1cmpp_720p.png");

    img_orig = imread(".\\docs\\apt_1cmpp_720p.png", IMREAD_GRAYSCALE);

    // and the processing loop is running...
    auto t_start = std::chrono::steady_clock::now();
    bool is_running = true;
    size_t ts_prev = 0;

    while (is_running)
    {
        // keep track of frames per second
        auto t_now = std::chrono::steady_clock::now();
        size_t ts = std::chrono::duration_cast<std::chrono::seconds>(t_now - t_start).count();
        if (ts != ts_prev)
        {
            g_fps = g_fps_ct;
            g_fps_ct = 0;
        }
        g_fps_ct++;
        ts_prev = ts;

        // handle keyboard events and end when ESC is pressed
        is_running = wait_and_check_keys(25);

        if (is_rehome)
        {
            // one-shot keypress
            is_rehome = false;

            // stop robot and put it at new position and orientation
            // will need to resync to new position
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
            home_pos = theRobot.get_xypos();
            home_ang = theRobot.get_ang();
            home_ang_vec = theRobot.get_ang_vec();
            match_angle = 0.0;
            match_offset = { 0,0 };
            ghnav.update_match_templates(theLidar.get_last_scan());
            is_resync = false;
        }

        // check match against the current waypoint
        // but limit orientation search to a small range around last match angle
        int search_deg = 10;
        int imatchang = static_cast<int>(match_angle);
        ghnav.perform_match(
            theLidar.get_last_scan(),
            imatchang - search_deg,
            imatchang + search_deg,
            match_offset, match_angle);

        // now update the screen view...
        
        // init image output with source floorplan (gray)
        img_orig.copyTo(img_viewer);

        // preprocess the scan for drawing
        cpoz::GHNav::T_PREPROC preproc;
        ghnav.preprocess_scan(theLidar.get_last_scan(), preproc);

        // draw shrunken "snapshot" image of current LIDAR scan (gray)
        Mat img_current_scan;
        Point img_current_scan_pt0;
        ghnav.draw_preprocessed_scan(img_current_scan, img_current_scan_pt0, preproc, 2);
        Rect mroi = { {0,0}, img_current_scan.size() };
        img_current_scan.copyTo(img_viewer(mroi));

        // switch to BGR...
        cvtColor(img_viewer, img_viewer_bgr, COLOR_GRAY2BGR);

        // draw LIDAR scan lines over floorplan
        theLidar.draw_last_scan(img_viewer_bgr, SCA_DKGRAY);

        // draw current home position with orientation line
        {
            circle(img_viewer_bgr, home_pos, 4, SCA_CYAN, -1);
            int angx = static_cast<int>(home_ang_vec.x * 12.0);
            int angy = static_cast<int>(home_ang_vec.y * 12.0);
            line(img_viewer_bgr, home_pos, home_pos + Point{ angx, angy }, SCA_CYAN, 2);
        }

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

        // show where the robot thinks it is relative to current home point
        // also draw a line showing what robot thinks its orientation is
        Point p0 = (-match_offset) / ghnav.get_match_params().prescale;
        Point prot;
        double rang_rad = (home_ang + match_angle) * CV_PI / 180.0;
        double cos0 = cos(rang_rad);
        double sin0 = sin(rang_rad);
        prot.x = static_cast<int>(p0.x * cos0 + p0.y * -sin0);
        prot.y = static_cast<int>(p0.x * sin0 + p0.y * cos0);
        Point guesspt = home_pos + prot;
        circle(img_viewer_bgr, guesspt, 4, SCA_RED, -1);
        rang_rad = (home_ang + match_angle) * CV_PI / 180.0;
        int dx = static_cast<int>(cos(rang_rad) * 12);
        int dy = static_cast<int>(sin(rang_rad) * 12);
        line(img_viewer_bgr, guesspt, guesspt + Point({ dx, dy }), SCA_RED, 2);

        {
            // print robot position in image
            std::ostringstream oss;
            oss << " BotXY. = ";
            oss << std::setfill('0') << std::setw(4) << ibotpos.x << ", ";
            oss << std::setfill('0') << std::setw(4) << ibotpos.y << ", ";
            oss << std::fixed << std::setprecision(1) << theRobot.get_ang();
            putText(img_viewer_bgr, oss.str(), { 0, 400 }, FONT_HERSHEY_PLAIN, 2.0, SCA_BLACK, 2);
        }

        {
            // print latest position and orientation match
            std::ostringstream oss;
            oss << " MATCH = ";
            oss << std::setfill('0') << std::setw(4) << guesspt.x << ", ";
            oss << std::setfill('0') << std::setw(4) << guesspt.y << ", ";
            double xang = match_angle + home_ang;
            xang = (xang > 360.0) ? xang - 360.0 : xang;
            oss << std::fixed << std::setprecision(1) << xang;
            putText(img_viewer_bgr, oss.str(), { 0, 430 }, FONT_HERSHEY_PLAIN, 2.0, SCA_BLUE, 2);
        }


#ifdef DEMO_IMG_ACC
        // show image of the Hough bins for best orientation match
        // normalize values and enlarge for better visualization
        Mat img_acc;
        Mat img_acc8;
        Mat img_acc8_bgr;
        Mat img_acc8_bgr_big;
        normalize(ghnav.m_img_acc, img_acc, 0, 255, cv::NORM_MINMAX);
        img_acc.convertTo(img_acc8, CV_8UC1);
#if 0
        // enable this to see more structure in Hough bins
        equalizeHist(img_acc8, img_acc8);
#endif
        cvtColor(img_acc8, img_acc8_bgr, COLOR_GRAY2BGR);
        int cx = img_acc8_bgr.size().width / 2;
        int cy = img_acc8_bgr.size().height / 2;
#if 1
        // draw circle at max
        circle(img_acc8_bgr, ghnav.m_img_acc_pt + Point(cx - 1, cy - 1), 2, SCA_RED, -1);
#else
        // draw multiple max regions
        std::vector<std::vector<cv::Point>> contours;
        Mat match_mask = (img_acc > 240);
        findContours(match_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        drawContours(img_acc8_bgr, contours, -1, SCA_RED, -1, LINE_8, noArray(), INT_MAX);
#endif
        resize(img_acc8_bgr, img_acc8_bgr_big, {}, 3.0, 3.0);
        Rect mroix = { { 25, 460 }, img_acc8_bgr_big.size() };
        img_acc8_bgr_big.copyTo(img_viewer_bgr(mroix));
#endif

        // show the BGR image
        image_output(img_viewer_bgr);
    }

    // when everything is done, release the capture device and windows
    cv::destroyAllWindows();
}


int main()
{
    std::cout << stitle << std::endl;
#if 0
    std::list<std::string> lfiles;
    std::string spath = ".\\movie_01\\";
    get_dir_list(spath, "*.png", lfiles);
    make_video(20, spath, "demo_01.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), lfiles);
#else
    loop();
#endif
    return 0;
}
