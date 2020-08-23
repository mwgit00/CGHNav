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


    uint8_t GHNav::convert_xy_to_angcode(int x, int y, uint8_t ct)
    {
        // convert X,Y to angle in degrees (0-360)
        double angdeg = atan2(y, x) * CONV_RAD2DEG;
        angdeg = (angdeg < 0.0) ? angdeg + 360.0 : angdeg;

        // convert angle to a byte code
        // at 360 the angle code is equal to ct so wrap it back
        // to angle code 0 to keep angle code in range 0 to (ct - 1)
        uint8_t angcode = static_cast<uint8_t>((ct * angdeg) / 360.0);
        if (angcode == ct)
        {
            angcode = 0;
        }

        return angcode;
    }


    void GHNav::plot_line(const cv::Point& pt0, const cv::Point& pt1, std::list<cv::Point>& rlist)
    {
        // Bresenham Line Algorithm from Wikipedia
        
        int dx = abs(pt1.x - pt0.x);
        int sx = (pt0.x < pt1.x) ? 1 : -1;
        int dy = -abs(pt1.y - pt0.y);
        int sy = (pt0.y < pt1.y) ? 1 : -1;
        int err = dx + dy;  // error value e_xy

        cv::Point pt = pt0;

        while (pt != pt1)
        {
            rlist.push_back(pt);

            int e2 = 2 * err;
            if (e2 >= dy)
            {
                // e_xy+e_x > 0
                err += dy;
                pt.x += sx;
            }
            if (e2 <= dx)
            {
                // e_xy+e_y < 0
                err += dx;
                pt.y += sy;
            }
        }

#if 0
        // pt1 can be appended here if desired
        rlist.push_back(pt1);
#endif
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
        m_acc_fulldim = (m_match_params.acc_halfdim * 2) + 1;

        double rfac = m_match_params.resize_big / m_match_params.resize;
        m_acc_halfdim_big = static_cast<int>(rfac) * m_match_params.acc_halfdim;
        m_acc_fulldim_big = ((m_acc_halfdim_big + 0) * 2) + 1;

        // if adjacent measurements are too far from each other
        // then they are likely not on the same surface and can be ignored
        // the threshold is distance between two measurements at max LIDAR range
        m_scan_rng_thr = m_scan_params.max_rng * tan(m_scan_params.ang_step * CONV_DEG2RAD);
    }


    void GHNav::preprocess_scan(
        T_PREPROC& rpreproc,
        const std::vector<double>& rscan,
        const size_t offset_index,
        const double resize)
    {
        std::vector<Point> vpts;
        convert_scan_to_pts(vpts, rscan, offset_index, resize);

        rpreproc.angcode_cts.clear();
        rpreproc.angcode_cts.resize(m_match_params.angcode_ct);

        // convert range threshold for quick integer comparison without square root
        double rng_thr_resize = m_scan_rng_thr * resize;
        const int dthr = static_cast<int>(rng_thr_resize * rng_thr_resize);

        for (size_t nn = 0; nn < (vpts.size() - 1); nn++)
        {
            Point pt0 = vpts[nn];
            Point pt1 = vpts[(nn + 1)];

            /// @TODO -- what about line that connects head to tail ???

            // the resize factor may make some points equal to one another
            // only proceed if points are not equal
            if (pt0 != pt1)
            {
                Point dpt = pt1 - pt0;
                int rsqu = ((dpt.x * dpt.x) + (dpt.y * dpt.y));
                if (rsqu < dthr)
                {
                    // points meet the closeness criteria
                    // so create a line from pt0 to pt1
                    uint8_t angcode = convert_xy_to_angcode(dpt.x, dpt.y, m_match_params.angcode_ct);
                    rpreproc.segments.push_back(T_PREPROC_SEGMENT{});
                    rpreproc.segments.back().angcode = angcode;
                    plot_line(pt0, pt1, rpreproc.segments.back().line);
                    rpreproc.angcode_cts[angcode] += rpreproc.segments.back().line.size();
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
            for (const auto& rr : r.line)
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
        m_vtemplates.clear();
        m_vtemplates.resize(m_match_params.ang_ct);

        // loop through all angles
        for (size_t ii = 0; ii < m_match_params.ang_ct; ii++)
        {
            // generate rotated representation of scan
            // and make a template for it
            T_PREPROC preproc;
            preprocess_scan(preproc, rscan, ii, m_match_params.resize);
            create_template(m_vtemplates[ii], preproc);
        }

        m_vtemplates_big.clear();
        m_vtemplates_big.resize(m_match_params.ang_ct);

        // loop through all angles
        for (size_t ii = 0; ii < m_match_params.ang_ct; ii++)
        {
            // generate rotated representation of scan
            // and make a template for it
            T_PREPROC preproc;
            preprocess_scan(preproc, rscan, ii, m_match_params.resize_big);
            create_template(m_vtemplates_big[ii], preproc);
        }
    }


    void GHNav::perform_match(
        const std::vector<double>& rscan,
        cv::Point& roffset,
        double& rang)
    {
        // preprocess the input scan (use 0 angle)
        T_PREPROC preproc;
        preprocess_scan(preproc, rscan, 0, m_match_params.resize);

        size_t qjjmax = 0U;
        double qallmax = 0.0;
        Point qallmaxpt = { 0,0 };

        // match input scan against all angle templates
        for (size_t jj = 0; jj < m_match_params.ang_ct; jj++)
        {
            Mat img_acc;
            Point qmaxpt;
            double qmax;

            match_single_template(
                m_vtemplates[jj],
                m_acc_fulldim,
                m_match_params.acc_halfdim,
                preproc, img_acc, qmaxpt, qmax);

            if (qmax > qallmax)
            {
                qallmax = qmax;
                qallmaxpt = qmaxpt;
                img_acc.copyTo(m_img_acc);
                qjjmax = jj;
            }
        }

        m_img_acc_pt = qallmaxpt;
        rang = static_cast<double>(qjjmax * m_match_params.ang_step);

        // do a single match at larger scale
        // to get better result for X,Y match offset
        T_PREPROC preproc_big;
        preprocess_scan(preproc_big, rscan, 0, m_match_params.resize_big);

        Mat img_acc_big;
        Point qbigmaxpt;
        double qbigmax;
        match_single_template(
            m_vtemplates_big[qjjmax],
            m_acc_fulldim_big,
            m_acc_halfdim_big,
            preproc_big, img_acc_big, qbigmaxpt, qbigmax);

        Size szacc = img_acc_big.size();
        Point ptctr = { szacc.width / 2, szacc.height / 2 };
        roffset = qbigmaxpt - ptctr;

        // un-rotate offset by matched orientation angle
        Point p0 = roffset;
        double rang_rad = rang * CV_PI / 180.0;
        double cos0 = cos(rang_rad);
        double sin0 = sin(rang_rad);
        roffset.x = static_cast<int>( p0.x * cos0 + p0.y * sin0);
        roffset.y = static_cast<int>(-p0.x * sin0 + p0.y * cos0);
    }


    //*******************//
    // PRIVATE FUNCTIONS //
    //*******************//


    void GHNav::init_scan_angs(void)
    {
        // flush old data
        m_scan_angs.clear();
        m_scan_cos_sin.clear();

        // generate ideal scan angles
        m_scan_angs.resize(m_scan_params.ang_ct);
        double ang = m_scan_params.ang_min;
        for (size_t ii = 0; ii < m_scan_params.ang_ct; ii++)
        {
            m_scan_angs[ii] = ang;
            ang += m_scan_params.ang_step;
        }

        // generate lookup tables for cosine and sine
        // for a range of offsets from the ideal scan angles
        double ang_offset = 0.0;
        for (size_t nn = 0; nn < m_match_params.ang_ct; nn++)
        {
            m_scan_cos_sin.push_back({});
            for (const auto& rang : m_scan_angs)
            {
                double ang_rad = (rang + ang_offset) * CONV_DEG2RAD;
                m_scan_cos_sin.back().push_back(cv::Point2d(cos(ang_rad), sin(ang_rad)));
            }
            ang_offset += m_match_params.ang_step;
        }
    }


    void GHNav::convert_scan_to_pts(
        std::vector<cv::Point>& rvpts,
        const std::vector<double>& rscan,
        const size_t offset_index,
        const double resize)
    {
        // look up the desired cos and sin table
        std::vector<Point2d>& rveccs = m_scan_cos_sin[offset_index];

        // project all measurements using ideal measurement angles
        // also determine bounds of the X,Y coordinates
        rvpts.resize(rscan.size());
        for (size_t nn = 0; nn < rvpts.size(); nn++)
        {
            double mag = rscan[nn] * resize;
            int dx = static_cast<int>((rveccs[nn].x * mag) + 0.5);
            int dy = static_cast<int>((rveccs[nn].y * mag) + 0.5);
            rvpts[nn] = { dx, dy };
        }
    }


    void GHNav::create_template(
        T_TEMPLATE& rtemplate,
        const T_PREPROC& rpreproc)
    {
        // create a data index pointer array for each angle code lookup table
        std::vector<Point*> vppt(m_match_params.angcode_ct);

        // use preproc info to allocate template lookup tables
        // also initialize data index pointers
        rtemplate.resize(m_match_params.angcode_ct);
        for (size_t jj = 0; jj < m_match_params.angcode_ct; jj++)
        {
            rtemplate[jj].resize(rpreproc.angcode_cts[jj]);
            vppt[jj] = rtemplate[jj].data();
        }

        // use appropriate data index pointer to fill in each lookup table
        for (const auto& r : rpreproc.segments)
        {
            for (const auto& rpt : r.line)
            {
                // stuff point into lookup table and advance pointer
                uint8_t angcode = r.angcode;
                *(vppt[angcode]) = rpt;
                ++vppt[angcode];
            }
        }
    }


    void GHNav::match_single_template(
        const T_TEMPLATE& rtemplate,
        const int acc_dim,
        const int acc_halfdim,
        const T_PREPROC& rpreproc,
        cv::Mat& rimg_acc,
        cv::Point& rmaxpt,
        double& rmax)
    {
        const Point ctr_offset = { acc_halfdim, acc_halfdim };

        // create new vote accumulator image
        rimg_acc = Mat::zeros(acc_dim, acc_dim, CV_16U);

        for (const auto& rseg : rpreproc.segments)
        {
            for (const auto& rlinept : rseg.line)
            {
                for (const auto& rmatchpt : rtemplate[rseg.angcode])
                {
                    // translate the vote pt and see
                    // if it falls in accumulator image
                    Point votept = rlinept - rmatchpt;
                    if ((abs(votept.x) < acc_halfdim) && (abs(votept.y) < acc_halfdim))
                    {
                        Point e = votept + ctr_offset;
                        uint16_t upix = rimg_acc.at<uint16_t>(e) + 1;
                        rimg_acc.at<uint16_t>(e) = upix;
                    }
                }
            }
        }

        // finally search for max in bin
        minMaxLoc(rimg_acc, nullptr, &rmax, nullptr, &rmaxpt);
    }
}
