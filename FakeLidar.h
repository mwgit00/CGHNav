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

#ifndef FAKE_LIDAR_H_
#define FAKE_LIDAR_H_

#include <string>
#include <vector>

namespace cpoz
{
    class FakeLidar
    {
    public:

        typedef struct
        {
            int range_dec_pt_adjust;        ///< power of 10, sets fractional digits, default 1 (0 digits)
            double jitter_range_u;          ///< indiv. range measurement jitter, default +-0.2
            double jitter_angle_deg_u;      ///< indiv. measurement angle jitter, default +-0.25deg
            double jitter_sync_deg_u;       ///< sync angle jitter, default +-0.25deg
            bool is_angle_noise_enabled;    ///< enable angle jitter for each measurement
            bool is_range_noise_enabled;    ///< enable range jitter for each measurement
        } T_PARAMS;

        FakeLidar();
        virtual ~FakeLidar();

        T_PARAMS& get_params(void) { return m_params; }
        
        void load_floorplan(const std::string& rspath);

        void set_scan_angs(const std::vector<double>& rvec) { scan_angs = rvec; }
        void set_world_pos(const cv::Point& rpt) { world_pos = rpt; }
        void set_world_ang(const double ang) { world_ang = ang; }

        void run_scan(void);        ///< simulate a scan with noise
        
        void draw_floorplan(const std::string& rspath) const;

        void draw_last_scan(
            cv::Mat& rimg,
            const cv::Scalar& rcolor) const;

        const std::vector<double>& get_last_scan(void) const { return last_scan; }

    private:

        T_PARAMS m_params;
        
        cv::Mat img_floorplan;                          ///< binary overhead image of floorplan
        std::vector<std::vector<cv::Point>> contours;   ///< "walls" extracted from floorplan img

        cv::Point world_pos;    ///< real-world position in floorplan
        double world_ang;       ///< real-world orienatation angle in floorplan (degrees)

        std::vector<double> last_scan;              ///< last result from run_scan

        std::vector<double> scan_angs;              ///< ideal scan angles in degrees
        std::vector<cv::Point2d> jitter_cos_sin;    ///< cos and sin for noisy scan angles
    };
}

#endif // FAKE_LIDAR_H_
