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

#ifndef GHNAV_H_
#define GHNAV_H_

#include <vector>
#include <list>

namespace cpoz
{
    class GHNav
    {
    public:

        
        typedef struct
        {
            uint8_t angcode;
            std::list<cv::Point> line;
        } T_PREPROC;

        typedef std::list<T_PREPROC> tListPreProc;

        
        typedef struct
        {
            tListPreProc list_preproc;
            std::vector<std::list<cv::Point>> lookup;
        } T_TEMPLATE;

        
        typedef struct
        {
            cv::Point pt;               ///< best-guess location of scan
            double ang;                 ///< best-guress orientation of scan
            std::vector<double> scan;   ///< original scan data
        } T_WAYPOINT;


        static uint8_t convert_xy_to_angcode(int x, int y, uint8_t ct);
        
        static void plot_line(const cv::Point& pt0, const cv::Point& pt1, std::list<cv::Point>& rlist);
        
        GHNav();
        virtual ~GHNav();

        
        void init_scan_angs(void);

        size_t get_scan_ang_ct(void) const { return m_scan_ang_ct; }
        uint8_t get_angcode_ct(void) const { return m_search_angcode_ct; }
        
        const std::vector<double>& get_scan_angs(void) const { return m_scan_angs; }

        void convert_scan_to_pts(
            std::vector<cv::Point>& rvec,
            const std::vector<double>& rscan,
            const size_t offset_index,
            const double resize);

        void preprocess_scan(
            tListPreProc& rlist,
            const std::vector<double>& rscan,
            const size_t offset_index,
            const double resize);

        void draw_preprocessed_scan(
            cv::Mat& rimg,
            cv::Point& rpt0,
            const GHNav::tListPreProc& rlist,
            const int shrink = 1);

        void update_match_templates(const std::vector<double>& rscan);

        void perform_match(
            const std::vector<double>& rscan,
            cv::Point& roffset,
            double& rang);

        void add_waypoint(GHNav::T_WAYPOINT& rwp);

        const std::list<GHNav::T_WAYPOINT>& get_waypoints(void) const { return m_waypoints; }

    public:

        cv::Mat m_img_foo;
        cv::Point m_img_foo_pt;
        int m_accum_img_halfdim;
        int m_accum_img_fulldim;
        int m_accum_bloom_k;

        cv::Point m_pt0_template_ang_0; // center of 0 degree match template for display

        std::list<T_WAYPOINT> m_waypoints;

    public:

        size_t m_scan_ang_ct;           ///< number of angles (elements) in a LIDAR scan
        double m_scan_ang_min;          ///< negative angle from 0 (front)
        double m_scan_ang_max;          ///< positive angle from 0 (front)
        double m_scan_ang_step;         ///< step between angles in LIDAR scan
        double m_scan_max_rng;          ///< max range possible from LIDAR
        double m_scan_len_thr;

        uint8_t m_search_angcode_ct;    ///< number of angle codes to use
        size_t m_search_ang_ct;         ///< number of angles in 360 degree search
        double m_search_ang_step;       ///< angle step in 360 degree search
        double m_search_resize;         ///< resize (shrink) factor

        std::vector<double> m_scan_angs;    ///< ideal scan angles
        
        std::vector<std::vector<cv::Point2d>> m_scan_cos_sin; ///< ideal cos and sin for scan angles

        std::vector<cv::Point> tpt0_offset;         ///< center points for all templates

        std::vector<T_TEMPLATE> m_vtemplates;
    };
}

#endif // GHNAV_H_
