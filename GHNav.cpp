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

    const int PAD_BORDER = 25;              // add some space around border of scan images

    constexpr double CONV_RAD2DEG = 180.0 / CV_PI;
    constexpr double CONV_DEG2RAD = CV_PI / 180.0;

    // cheap COTS LIDAR:  8000 samples/s, 2Hz-10Hz ???

    class cmpPtByXY
    {
    public:
        bool operator()(const cv::Point& a, const cv::Point& b) const
        {
            if (a.x == b.x)
                return a.y < b.y;
            return a.x < b.x;
        }
    };

    
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
        // pt1 can be appended here if drawing
        rlist.push_back(pt1);
#endif
    }

    
    GHNav::GHNav() :
        m_scan_ang_ct(341),                 // 340 degree scan (+1 for 0 degree sample)
        m_scan_ang_min(-170.0),             // 20 degree "blind spot" behind robot
        m_scan_ang_max(170.0),
        m_scan_ang_step(1.0),               // 1 degree between each measurement
        m_scan_max_rng(1200.0),             // 12m max range from LIDAR
        slam_loc({ 0, 0 }),
        slam_ang(0.0),
        m_search_angcode_ct(8),
        m_search_ang_ct(360),               // 360 degree search
        m_search_ang_step(1.0),             // 1 degree between each search step
        m_search_bin_decim(1),
        m_accum_img_halfdim(20),
        m_accum_bloom_k(0)
    {
        tpt0_offset.resize(m_search_ang_ct);

        init_scan_angs();

        // threshold for adjacent measurements that are too far from each other
        // they are likely not on the same surface and can be ignored
        // the threshold is distance between two measurements at max LIDAR range
        m_scan_len_thr = m_scan_max_rng * tan(m_scan_ang_step * CONV_DEG2RAD);

        // init width/height of accumulator image
        m_accum_img_fulldim = (m_accum_img_halfdim + m_accum_bloom_k) * 2 + 1;
    }
    
    
    GHNav::~GHNav()
    {
        // does nothing
    }


    void GHNav::init_scan_angs(void)
    {
        // flush old data
        m_scan_angs.clear();
        m_scan_cos_sin.clear();

        // generate ideal scan angles
        m_scan_angs.resize(m_scan_ang_ct);
        double ang = m_scan_ang_min;
        for (size_t ii = 0; ii < m_scan_ang_ct; ii++)
        {
            m_scan_angs[ii] = ang;
            ang += m_scan_ang_step;
        }

        // generate lookup tables for cosine and sine
        // for a range of offsets from the ideal scan angles
        double ang_offset = 0.0;
        for (size_t nn = 0; nn < m_search_ang_ct; nn++)
        {
            m_scan_cos_sin.push_back({});
            for (const auto& rang : m_scan_angs)
            {
                double ang_rad = (rang + ang_offset) * CONV_DEG2RAD;
                m_scan_cos_sin.back().push_back(cv::Point2d(cos(ang_rad), sin(ang_rad)));
            }
            ang_offset += m_search_ang_step;
        }
    }


    void GHNav::convert_scan_to_pts(
        std::vector<cv::Point>& rvpts,
        cv::Rect& rbbox,
        const std::vector<double>& rscan,
        const size_t offset_index,
        const double resize)
    {
        // look up the desired cos and sin table
        std::vector<Point2d>& rveccs = m_scan_cos_sin[offset_index];
        
        rvpts.resize(rscan.size());

        Point ptmin = { INT_MAX, INT_MAX };
        Point ptmax = { INT_MIN, INT_MIN };

        // project all measurements using ideal measurement angles
        // also determine bounds of the X,Y coordinates
        for (size_t nn = 0; nn < rvpts.size(); nn++)
        {
            double mag = rscan[nn] * resize;
            int dx = static_cast<int>((rveccs[nn].x * mag) + 0.5);
            int dy = static_cast<int>((rveccs[nn].y * mag) + 0.5);
            rvpts[nn] = { dx, dy };

            ptmax.x = max(ptmax.x, dx);
            ptmin.x = min(ptmin.x, dx);
            ptmax.y = max(ptmax.y, dy);
            ptmin.y = min(ptmin.y, dy);
        }

        // set bounding box around projected points
        rbbox = Rect(ptmin, ptmax);
    }


    void GHNav::preprocess_scan(
        tListPreProc& rlist,
        cv::Rect& rbbox,
        const std::vector<double>& rscan,
        const size_t offset_index,
        const double resize)
    {
        std::vector<Point> vpts;
        convert_scan_to_pts(vpts, rbbox, rscan, offset_index, resize);

        const int dthr = static_cast<int>(m_scan_len_thr * m_scan_len_thr * resize);

        for (size_t nn = 0; nn < (vpts.size() - 1); nn++)
        {
            Point pt0 = vpts[nn];
            Point pt1 = vpts[(nn + 1)];

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
                    rlist.push_back(T_PREPROC{});
                    rlist.back().angcode = convert_xy_to_angcode(dpt.x, dpt.y, m_search_angcode_ct);
                    plot_line(pt0, pt1, rlist.back().line);
                }
            }
        }
    }


    void GHNav::draw_preprocessed_scan(
        cv::Mat& rimg,
        cv::Point& rpt0,
        const GHNav::tListPreProc& rlist,
        const cv::Rect& rbbox,
        const int shrink)
    {
        // create image same size as bounding box along with some padding
        Size imgsz = Size(
            (rbbox.width / shrink) + (2 * PAD_BORDER),
            (rbbox.height / shrink) + (2 * PAD_BORDER));
        rimg = Mat::zeros(imgsz, CV_8UC1);

        for (const auto& r : rlist)
        {
            for (const auto& rr : r.line)
            {
                Point ptnew = {
                    ((rr.x - rbbox.x) / shrink) + PAD_BORDER,
                    ((rr.y - rbbox.y) / shrink) + PAD_BORDER };
                rimg.at<uint8_t>(ptnew) = 255;
            }
        }

        // finally note the sensing point in the scan image
        rpt0 = {
            ((0 - rbbox.x) / shrink) + PAD_BORDER,
            ((0 - rbbox.y) / shrink) + PAD_BORDER };
    }


    void GHNav::update_match_templates(const std::vector<double>& rscan)
    {
        m_vtemplates.clear();
        m_vtemplates.resize(m_search_ang_ct);
        for (size_t ii = 0; ii < m_search_ang_ct; ii++)
        {
            preprocess_scan(m_vtemplates[ii].list_preproc, m_vtemplates[ii].bbox, rscan, ii);
            m_vtemplates[ii].lookup.resize(m_search_angcode_ct);
            for (const auto& r : m_vtemplates[ii].list_preproc)
            {
                for (const auto& rpt : r.line)
                {
                    m_vtemplates[ii].lookup[r.angcode].push_back(rpt);
                }
            }
        }
    }


    void GHNav::perform_match(
        const std::vector<double>& rscan,
        cv::Point& roffset,
        double& rang)
    {
        tListPreProc list_preproc;
        cv::Rect bbox;

        const int BLOOM_PAD = m_accum_img_halfdim + m_accum_bloom_k;
        const Point boo = { BLOOM_PAD, BLOOM_PAD };
        
        // use 0 angle
        preprocess_scan(list_preproc, bbox, rscan, 0);

        size_t qjjmax = 0U;
        double qallmax = 0.0;
        Point qallmaxpt = { 0,0 };

        for (size_t jj = 0; jj < m_search_ang_ct; jj++)
        {
            // get new template and vote accumulator image
            T_TEMPLATE& rt = m_vtemplates[jj];
            Mat img_acc = Mat::zeros(m_accum_img_fulldim, m_accum_img_fulldim, CV_16U);

            for (const auto& rpreproc : list_preproc)
            {
                for (const auto& rlinept : rpreproc.line)
                {
                    for (const auto& rmatchpt : rt.lookup[rpreproc.angcode])
                    {
                        // translate the vote pt and see
                        // if it falls in accumulator image
                        Point votept = rlinept - rmatchpt;
                        if ((abs(votept.x) < m_accum_img_halfdim) && (abs(votept.y) < m_accum_img_halfdim))
                        {
#if 0
                            // bloom
                            for (int mm = -m_accum_bloom_k; mm <= m_accum_bloom_k; mm++)
                            {
                                for (int nn = -m_accum_bloom_k; nn <= m_accum_bloom_k; nn++)
                                {
                                    int q = m_accum_bloom_k + 1 - max(abs(nn), abs(mm));
                                    Point d = { mm, nn };
                                    Point e = votept + d + boo;
                                    uint16_t upix = img_acc.at<uint16_t>(e);
                                    upix += static_cast<uint16_t>(q);
                                    img_acc.at<uint16_t>(e) = upix;
                                }
                            }
#else
                            Point e = votept + boo;
                            uint16_t upix = img_acc.at<uint16_t>(e);
                            upix += 1;// static_cast<uint16_t>(q);
                            img_acc.at<uint16_t>(e) = upix;
#endif
                        }
                    }
                }
            }

            double qmax;
            Point qmaxpt;
            minMaxLoc(img_acc, nullptr, &qmax, nullptr, &qmaxpt);
            if (qmax > qallmax)
            {
                qallmax = qmax;
                qallmaxpt = qmaxpt;
                img_acc.copyTo(m_img_foo);
                qjjmax = jj;
            }
        }

        m_img_foo_pt = qallmaxpt;
        rang = static_cast<double>(qjjmax * m_search_ang_step);
#if 0
        // convert scan to an image (no rotation)
        draw_preprocessed_scan(m_img_scan, m_pt0_scan, sz / 2, rscan);

        Point tptq_mid = (m_img_scan.size() / 2);
        Point tptq_offset = m_pt0_scan - tptq_mid;

        // get template for running match ???

        // search for best orientation match
        // this match will also provide the translation
        // but an "un-rotation" is required after the best match is found
        // (instead of linear search, maybe the previous match could be start for this match ???)
        size_t qidmax = 0;
        double qallmax = 0.0;
        Point qptmax_offset;
        for (size_t ii = 0; ii < sz; ii++)
        {
            Mat img_match;
            double qmax = 0.0;
            Point qptmax = { 0, 0 };

            // GH ???

            double qmaxtotal = 0.0;
            if (qmaxtotal > 0.0)
            {
                qmax = qmax / qmaxtotal;
                if (qmax > qallmax)
                {
                    qallmax = qmax;
                    qidmax = ii;
                    qptmax_offset = qptmax - tptq_mid;
                }
            }
        }

        roffset = tptq_offset - tpt0_offset[qidmax] - qptmax_offset;
        rang = scan_angs_offsets[qidmax];

        // un-rotate offset by matched orientation angle
        Point p0 = roffset;
        double rang_rad = rang * CV_PI / 180.0;
        double cos0 = cos(rang_rad);
        double sin0 = sin(rang_rad);
#if 0
        roffset.x = static_cast<int>(p0.x * cos0 - p0.y * sin0);
        roffset.y = static_cast<int>(p0.x * sin0 + p0.y * cos0);
#else
        roffset.x = static_cast<int>( p0.x * cos0 + p0.y * sin0);
        roffset.y = static_cast<int>(-p0.x * sin0 + p0.y * cos0);
#endif
#endif
    }


    void GHNav::add_waypoint(GHNav::T_WAYPOINT& rwp)
    {
        m_waypoints.push_back(rwp);
    }
}
