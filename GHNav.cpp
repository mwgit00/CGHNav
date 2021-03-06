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

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <vector>
#include <map>

#include "GHNav.h"



namespace cpoz
{
    using namespace cv;

    const int PAD_BORDER = 25;      // add some space around border of scan images when drawing

    constexpr double CONV_RAD2DEG = 180.0 / CV_PI;
    constexpr double CONV_DEG2RAD = CV_PI / 180.0;


    static double xy_to_360deg(int y, int x)
    {
        // convert X,Y to angle in degrees [0,360)
        double angdeg = atan2(y, x) * CONV_RAD2DEG;
        return (angdeg < 0.0) ? angdeg + 360.0 : angdeg;
    }


    static uint8_t encode_ang(double angdeg, uint8_t ct)
    {
        // convert angle to a byte code
        // at 360 the angle code is equal to ct so wrap it back
        // to angle code 0 to keep angle code in range 0 to (ct - 1)
        double dct = static_cast<double>(ct);
        uint8_t angcode = static_cast<uint8_t>((dct * angdeg) / 360.0);
        return (angcode == ct) ? 0 : angcode;
    }


    GHNav::GHNav()
    {
        init();
    }


    GHNav::~GHNav()
    {
        // nothing to do here
    }


    void GHNav::init(void)
    {
        init_scan_angs();

        // init full width/height of accumulator image for angle search
        // add 2 to prevent any out-of-bounds problems with rounding coordinates
        m_acc_fulldim = (m_match_params.acc_halfdim * 2) + 2;

        // if adjacent measurements are too far from each other
        // then they are likely not on the same surface and can be ignored
        // the threshold is distance between two measurements at max LIDAR range
        m_scan_rng_thr = m_scan_params.max_rng * tan(m_scan_params.ang_step * CONV_DEG2RAD);
        m_scan_rng_thr = m_match_params.prescale * m_scan_rng_thr;

        // init the sin and cos for the orientation search step angle
        double angradstep = m_match_params.ang_step * CONV_DEG2RAD;
        m_match_step_cos = cos(angradstep);
        m_match_step_sin = sin(angradstep);
    }


    void GHNav::preprocess_scan(
        const std::vector<double>& rscan,
        T_PREPROC& rpreproc)
    {
        std::vector<Point> vpts;
        convert_scan_to_pts(rscan, vpts, m_match_params.prescale);

        rpreproc.angcode_cts.clear();
        rpreproc.angcode_cts.resize(m_match_params.angcode_ct);

        // convert range threshold for quick integer comparison without square root
        const int dthr = static_cast<int>(m_scan_rng_thr * m_scan_rng_thr);

        for (size_t nn = 0; nn < (vpts.size() - 1); nn++)
        {
            Point pt0 = vpts[nn];
            Point pt1 = vpts[(nn + 1)];

            // the scale factor may make some points equal to one another
            // only proceed if points are not equal
            if (pt0 != pt1)
            {
                Point diffpt = pt1 - pt0;
                int rsqu = ((diffpt.x * diffpt.x) + (diffpt.y * diffpt.y));
                if (rsqu < dthr)
                {
                    // points meet the closeness criteria
                    // so create a line from pt0 to pt1
                    // and fill in angle info
                    double angdeg = xy_to_360deg(diffpt.y, diffpt.x);
                    uint8_t angcode = encode_ang(angdeg, m_match_params.angcode_ct);
                    
                    rpreproc.segments.push_back(T_PREPROC_SEGMENT{});
                    rpreproc.segments.back().angdeg = angdeg;
                    rpreproc.segments.back().angcode = angcode;

                    // generate line with real valued X,Y points between pt0 and pt1
                    // points are not equal so at least one segment point will be generated
                    int isz = max(abs(diffpt.x), abs(diffpt.y));
                    double dsz = 1.0 / static_cast<double>(isz);
                    double dzx = diffpt.x * dsz;
                    double dzy = diffpt.y * dsz;
                    Point2d zpt = pt0;
                    for (int ii = 0; ii < isz; ii++)
                    {
                        rpreproc.segments.back().lined.push_back(zpt);
                        zpt += {dzx, dzy};
                    }

                    // tally total number of occurrences of this angle code
                    // it will be used later for allocating array for Hough table
                    rpreproc.angcode_cts[angcode] += rpreproc.segments.back().lined.size();
                }
            }
        }
    }


    void GHNav::draw_preprocessed_scan(
        cv::Mat& rimg,
        cv::Point& rpt0,
        const GHNav::T_PREPROC& rpreproc,
        const int shrink)
    {
        // collect all points
        std::vector<Point> vpts;
        for (const auto& r : rpreproc.segments)
        {
            for (const auto& rr : r.lined)
            {
                vpts.push_back(rr);
            }
        }

        // get bounding box around all points
        Rect bbox = boundingRect(vpts);

        // create image same size as bounding box along with some padding
        Size imgsz = Size(
            (bbox.width / shrink) + (2 * PAD_BORDER),
            (bbox.height / shrink) + (2 * PAD_BORDER));
        rimg = Mat::zeros(imgsz, CV_8UC1);

        // shift points to fit in image and draw dot at each point
        for (const auto& r : vpts)
        {
            Point ptnew = {
                ((r.x - bbox.x) / shrink) + PAD_BORDER,
                ((r.y - bbox.y) / shrink) + PAD_BORDER };
            rimg.at<uint8_t>(ptnew) = 255;
        }

        // finally note the central point of the scan in the scan image
        rpt0 = {
            ((0 - bbox.x) / shrink) + PAD_BORDER,
            ((0 - bbox.y) / shrink) + PAD_BORDER };
    }


    void GHNav::update_match_templates(const std::vector<double>& rscan)
    {
        T_PREPROC preproc;
        m_vtemplates.clear();
        m_vtemplates.resize(m_match_params.ang_ct);
        preprocess_scan(rscan, preproc);

        // loop through all angles
        for (size_t ii = 0; ii < m_match_params.ang_ct; ii++)
        {
            // make a template for it
            // rotate the preprocessed scan by one angle step
            create_template(preproc, m_vtemplates[ii]);
            rotate_preprocessed_scan(preproc);
        }
    }


    void GHNav::perform_match(
        const std::vector<double>& rscan,
        const int a1,
        const int a2,
        cv::Point& roffset,
        double& rang)
    {
        // preprocess the input scan prior to matching
        T_PREPROC preproc;
        preprocess_scan(rscan, preproc);

        int qiimax = 0;
        double qallmax = 0.0;
        Point qallmaxpt = { 0,0 };

        // match input scan
        for (int jj = a1; jj <= a2; jj++)
        {
            Mat img_acc;
            Point qmaxpt;
            double qmax;

            // convert loop index so it will be in array bounds
            int ii = (jj + m_match_params.ang_ct + m_match_params.ang_ct) % m_match_params.ang_ct;

            // do a match with the voting points scaled down by the "div" factor
            // this scaling helps remove some of the jitter in the bin counts
            // at the expense of less resolution in the position match
            match_single_template(
                preproc,
                m_vtemplates[ii],
                img_acc, qmaxpt, qmax);

            if (qmax > qallmax)
            {
                qallmax = qmax;
                qallmaxpt = qmaxpt;
#ifdef DEMO_IMG_ACC
                img_acc.copyTo(m_img_acc);
                m_img_acc_pt = qallmaxpt;
#endif
                qiimax = ii;
            }
        }

        rang = static_cast<double>(qiimax * m_match_params.ang_step);
        roffset = qallmaxpt;
    }


    //*******************//
    // PRIVATE FUNCTIONS //
    //*******************//


    void GHNav::init_scan_angs(void)
    {
        // set array sizes before filling them in
        m_scan_angs.resize(m_scan_params.ang_ct);
        m_scan_cos_sin.resize(m_scan_params.ang_ct);

        // generate ideal scan angles
        // and lookup table for sin and cos
        double ang = m_scan_params.ang_min;
        for (size_t ii = 0; ii < m_scan_params.ang_ct; ii++)
        {
            m_scan_angs[ii] = ang;
            double ang_rad = ang * CONV_DEG2RAD;
            m_scan_cos_sin[ii] = cv::Point2d(cos(ang_rad), sin(ang_rad));
            ang += m_scan_params.ang_step;
        }
    }


    void GHNav::convert_scan_to_pts(
        const std::vector<double>& rscan,
        std::vector<cv::Point>& rvpts,
        const double resize)
    {
        rvpts.resize(rscan.size());
        for (size_t nn = 0; nn < rvpts.size(); nn++)
        {
            // convert angle and distance to (x,y)
            double mag = rscan[nn] * resize;
            int dx = static_cast<int>(round(m_scan_cos_sin[nn].x * mag));
            int dy = static_cast<int>(round(m_scan_cos_sin[nn].y * mag));
            rvpts[nn] = { dx, dy };
        }
    }


    void GHNav::rotate_preprocessed_scan(GHNav::T_PREPROC& rpreproc)
    {
        rpreproc.angcode_cts.clear();
        rpreproc.angcode_cts.resize(m_match_params.angcode_ct);

        for (auto& r : rpreproc.segments)
        {
            // rotate segment angle BACKWARDS
            // but make sure it stays in [0,360) range
            r.angdeg -= m_match_params.ang_step;
            if (r.angdeg < 0.0) r.angdeg += 360.0;

            // then convert it to angle code
            r.angcode = encode_ang(r.angdeg, m_match_params.angcode_ct);
            rpreproc.angcode_cts[r.angcode] += r.lined.size();

            // then rotate all the segment's line points
            // about (0,0) by the search angle step
            for (auto& rr : r.lined)
            {
                Point2d rnew;
                rnew.x = rr.x * (m_match_step_cos) + rr.y * (m_match_step_sin);
                rnew.y = rr.x * (-m_match_step_sin) + rr.y * (m_match_step_cos);
                rr = rnew;
            }
        }
    }


    void GHNav::create_template(
        const T_PREPROC& rpreproc,
        T_TEMPLATE& rtemplate)
    {
        // each angle code gets its own lookup table
        // create a data index pointer array the lookup tables
        std::vector<Point2d*> vppt(m_match_params.angcode_ct);

        // use preproc info to allocate template lookup tables
        // also initialize the data index pointers
        rtemplate.resize(m_match_params.angcode_ct);
        for (size_t jj = 0; jj < m_match_params.angcode_ct; jj++)
        {
            rtemplate[jj].resize(rpreproc.angcode_cts[jj]);
            vppt[jj] = rtemplate[jj].data();
        }

        // use appropriate data index pointer to fill in each lookup table
        for (const auto& r : rpreproc.segments)
        {
            for (const auto& rpt : r.lined)
            {
                // stuff point into lookup table and advance pointer
                uint8_t angcode = r.angcode;
                *(vppt[angcode]) = rpt;
                ++vppt[angcode];
            }
        }
    }


    void GHNav::match_single_template(
        const T_PREPROC& rpreproc,
        const T_TEMPLATE& rtemplate,
        cv::Mat& rimg_acc,
        cv::Point& rmaxpt,
        double& rmax)
    {
        const int acc_halfdim = m_match_params.acc_halfdim;
        const Point ctr_offset = { acc_halfdim, acc_halfdim };

        // create new vote accumulator image
        rimg_acc = Mat::zeros(m_acc_fulldim, m_acc_fulldim, CV_16U);

        // do Generalized Hough voting
        for (const auto& rseg : rpreproc.segments)
        {
            for (const auto& rlinept : rseg.lined)
            {
                for (const auto& rmatchpt : rtemplate[rseg.angcode])
                {
                    // translate the vote point and see
                    // if it falls in accumulator image
                    Point2d votept = rlinept - rmatchpt;
                    if ((abs(votept.x) < acc_halfdim) && (abs(votept.y) < acc_halfdim))
                    {
                        // tally the vote
                        // rounding is SLOW but helps eliminate the jitter that
                        // seems to be caused by excessive use of integer point structures
                        int dx = static_cast<int>(round(votept.x));
                        int dy = static_cast<int>(round(votept.y));
                        Point pt = { dx, dy };
                        Point ptacc = pt + ctr_offset;
                        rimg_acc.at<uint16_t>(ptacc)++;
                    }
                }
            }
        }

        // finally search for bin with max votes
        // apply centering offset to the point where max occurs
        Point maxpt;
        minMaxLoc(rimg_acc, nullptr, &rmax, nullptr, &maxpt);
        rmaxpt = maxpt - ctr_offset;
    }
}
