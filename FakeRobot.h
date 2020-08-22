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

#ifndef FAKE_ROBOT_H_
#define FAKE_ROBOT_H_

#include "opencv2/imgproc.hpp"

namespace cpoz
{
    class FakeRobot
    {
    public:
        
        FakeRobot();
        virtual ~FakeRobot();

        int get_radius(void) const { return m_radius; }
        cv::Point2d get_vLR(void) const { return m_vLR; }
        cv::Point2d get_xypos(void) const { return m_xypos; }
        
        double get_ang(void) const { return m_angdeg; }

        cv::Point2d get_ang_vec(void) const;

        void set_vLR(const cv::Point2d& rvLR) { m_vLR = rvLR; }
        void set_xypos_ang(const cv::Point2d& rxypos, const double angdeg);

        void accel(const cv::Point2d& racc, const double vmax);

        void update(void);

    private:

        int m_radius;           ///< distance from robot center to wheel contact center
        cv::Point2d m_vLR;      ///< left and right wheel velocity
        cv::Point2d m_xypos;    ///< position
        double m_angdeg;        ///< heading angle (degrees)
    };
}

#endif // FAKE_ROBOT_H_