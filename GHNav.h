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
            double prescale;        ///< scale factor applied to scans/templates
            int ang_ct;             ///< number of angles in 360 degree search
            double ang_step;        ///< angle step in 360 degree search
            uint8_t angcode_ct;     ///< number of angle codes to use
            int acc_halfdim;        ///< half dimension of Hough accumulator bin image
            _T_MATCH_PARAMS_struct() :
                prescale(0.5),      // don't go lower than this
                ang_ct(360),        // 360 angle templates at each waypoint
                ang_step(1.0),      // 1 degree between each angle template
                angcode_ct(8),      // 8 angle codes seems sufficient
                acc_halfdim(30)     // keep within a few robot widths
            {}
        } T_MATCH_PARAMS;


        typedef struct
        {
            double angdeg;                  ///< orientation of segment in degrees
            uint8_t angcode;                ///< orientation of segment as angle code
            std::list<cv::Point2d> lined;   ///< list of points for line segment
        } T_PREPROC_SEGMENT;


        typedef struct
        {
            std::vector<size_t> angcode_cts;        ///< number of occurrences of each angle code
            std::list<T_PREPROC_SEGMENT> segments;  ///< list of encoded line segments
        } T_PREPROC;

        
        typedef std::vector<std::vector<cv::Point2d>> T_TEMPLATE;

        
        GHNav();
        virtual ~GHNav();

        T_SCAN_PARAMS& get_scan_params(void) { return m_scan_params; }
        T_MATCH_PARAMS& get_match_params(void) { return m_match_params; }

        void init(void);

        const std::vector<double>& get_scan_angs(void) const { return m_scan_angs; }

        void preprocess_scan(
            const std::vector<double>& rscan,
            T_PREPROC& rpreproc);

        void draw_preprocessed_scan(
            cv::Mat& rimg,
            cv::Point& rpt0,
            const GHNav::T_PREPROC& rpreproc,
            const int shrink);

        void update_match_templates(const std::vector<double>& rscan);

        void perform_match(
            const std::vector<double>& rscan,
            const int a1,
            const int a2,
            cv::Point& roffset,
            double& rang);

    private:

        void init_scan_angs(void);

        void convert_scan_to_pts(
            const std::vector<double>& rscan,
            std::vector<cv::Point>& rvec,
            const double resize);

        void rotate_preprocessed_scan(GHNav::T_PREPROC& rpreproc);

        void create_template(
            const T_PREPROC& rpreproc,
            T_TEMPLATE& rtemplate);

        void match_single_template(
            const T_PREPROC& rpreproc,
            const T_TEMPLATE& rtemplate,
            cv::Mat& rimg_acc,
            cv::Point& rmaxpt,
            double& rmax);

    public:

#ifdef DEMO_IMG_ACC
        // purely for testing
        cv::Mat m_img_acc;
        cv::Point m_img_acc_pt;
#endif

    private:

        T_SCAN_PARAMS m_scan_params;    ///< all params for doing fake LIDAR scan
        T_MATCH_PARAMS m_match_params;  ///< all params for Generalized Hough matching

        int m_acc_fulldim;          ///< accumulator bin image size calculated from match params

        double m_scan_rng_thr;      ///< "closeness" threshold calculated from scan params
        double m_match_step_cos;    ///< cos of search params angle step
        double m_match_step_sin;    ///< sin of search params angle step

        std::vector<double> m_scan_angs;            ///< ideal scan angles
        std::vector<cv::Point2d> m_scan_cos_sin;    ///< cos and sin for ideal scan angles
        std::vector<T_TEMPLATE> m_vtemplates;       ///< templates for orientation search
    };
}

#endif // GHNAV_H_
