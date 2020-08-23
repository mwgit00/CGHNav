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

        typedef struct _T_SCAN_PARAMS_struct
        {
            size_t ang_ct;          ///< number of angles (elements) in a LIDAR scan
            double ang_step;        ///< step between angles in LIDAR scan
            double ang_min;         ///< negative angle from 0 (front)
            double ang_max;         ///< positive angle from 0 (front)
            double max_rng;         ///< max range possible from LIDAR
            _T_SCAN_PARAMS_struct() :
                ang_ct(341),        ///< 340 degree scan (-170 to 170 with 1 degree step)
                ang_step(1.0),      ///< 1 degree between each measurement
                ang_min(-170.0),    ///< for 20 degree blind spot behind robot (testing)
                ang_max(170.0),     ///< for 20 degree blind spot behind robot (testing)
                max_rng(1200.0)     ///< 12m max LIDAR range
            {}
        } T_SCAN_PARAMS;


        typedef struct _T_MATCH_PARAMS_struct
        {
            size_t ang_ct;          ///< number of angles in 360 degree search
            double ang_step;        ///< angle step in 360 degree search
            double resize;          ///< resize (shrink) factor
            double resize_big;      ///< resize (shrink) factor for big templates
            uint8_t angcode_ct;     ///< number of angle codes to use
            int acc_halfdim;        ///< half dimension of Hough accumulator bin image
            _T_MATCH_PARAMS_struct() :
                ang_ct(360),        // search through full 360 degrees
                ang_step(1.0),      // 1 degree between each search step
                resize(0.125),      // shrink factor for rotation angle match
                resize_big(0.5),    // srhink factor for final translation match
                angcode_ct(8),      // 8 angle codes is good starting point
                acc_halfdim(20)     // bigger values slow down matching process
            {}
        } T_MATCH_PARAMS;


        typedef struct
        {
            uint8_t angcode;
            std::list<cv::Point> line;
        } T_PREPROC_SEGMENT;


        typedef struct
        {
            std::vector<size_t> angcode_cts;        ///< number of occurrences of each angle code
            std::list<T_PREPROC_SEGMENT> segments;  ///< list of encoded line segments
        } T_PREPROC;

        
        typedef std::vector<std::vector<cv::Point>> T_TEMPLATE;

        static uint8_t convert_xy_to_angcode(int x, int y, uint8_t ct);
        
        static void plot_line(const cv::Point& pt0, const cv::Point& pt1, std::list<cv::Point>& rlist);
        
        
        GHNav();
        virtual ~GHNav();

        T_SCAN_PARAMS& get_scan_params(void) { return m_scan_params; }
        T_MATCH_PARAMS& get_match_params(void) { return m_match_params; }

        void init(void);

        const std::vector<double>& get_scan_angs(void) const { return m_scan_angs; }

        void preprocess_scan(
            T_PREPROC& rpreproc,
            const std::vector<double>& rscan,
            const size_t offset_index,
            const double resize);

        void draw_preprocessed_scan(
            cv::Mat& rimg,
            cv::Point& rpt0,
            const GHNav::T_PREPROC& rpreproc,
            const int shrink = 1);

        void update_match_templates(const std::vector<double>& rscan);

        void perform_match(
            const std::vector<double>& rscan,
            cv::Point& roffset,
            double& rang);

    private:

        void init_scan_angs(void);

        void convert_scan_to_pts(
            std::vector<cv::Point>& rvec,
            const std::vector<double>& rscan,
            const size_t offset_index,
            const double resize);

        void create_template(
            T_TEMPLATE& rtemplate,
            const T_PREPROC& rpreproc);

        void match_single_template(
            const T_TEMPLATE& rtemplate,
            const int acc_dim,
            const int acc_halfdim,
            const T_PREPROC& rpreproc,
            cv::Mat& rimg_acc,
            cv::Point& rmaxpt,
            double& rmax);

    public:

        cv::Mat m_img_acc;
        cv::Point m_img_acc_pt;

    private:

        T_SCAN_PARAMS m_scan_params;
        T_MATCH_PARAMS m_match_params;

        int m_acc_fulldim;      ///< accumulator bin image size calculated from match params
        int m_acc_halfdim_big;  ///< "big" accumulator bin image half-size calculated from match params
        int m_acc_fulldim_big;  ///< "big" accumulator bin image size calculated from match params

        double m_scan_rng_thr;  ///< "closeness" threshold calculated from scan params

        std::vector<double> m_scan_angs;    ///< ideal scan angles
        
        std::vector<std::vector<cv::Point2d>> m_scan_cos_sin; ///< ideal cos and sin for scan angles

        std::vector<T_TEMPLATE> m_vtemplates;
        std::vector<T_TEMPLATE> m_vtemplates_big;
    };
}

#endif // GHNAV_H_
