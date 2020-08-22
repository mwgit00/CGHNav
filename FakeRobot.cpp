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

#include "FakeRobot.h"

namespace cpoz
{
    FakeRobot::FakeRobot() :
        m_radius(20)
    {
        set_xypos_ang({ 0.0, 0.0 }, 0.0);
    }

    
    FakeRobot::~FakeRobot()
    {
        // nothing to do here
    }


    cv::Point2d FakeRobot::get_ang_vec(void) const
    {
        double angrad = m_angdeg * CV_PI / 180.0;
        double c = cos(angrad);
        double s = sin(angrad);
        return { c, s };
    }

    
    void FakeRobot::set_xypos_ang(const cv::Point2d& rxypos, const double angdeg)
    {
        m_xypos = rxypos;
        m_angdeg = angdeg;
    }


    void FakeRobot::accel(const cv::Point2d& racc, const double vmax)
    {
        // don't do this if spinning in place
        if ((m_vLR.x >= 0.0) && (m_vLR.y >= 0.0))
        {
            m_vLR.x += racc.x;
            if (m_vLR.x < 0.0) m_vLR.x = 0.0;
            if (m_vLR.x > vmax) m_vLR.x = vmax;
            m_vLR.y += racc.y;
            if (m_vLR.y < 0.0) m_vLR.y = 0.0;
            if (m_vLR.y > vmax) m_vLR.y = vmax;
        }
    }

    
    void FakeRobot::update(void)
    {
        // 2-wheel differential drive equations
        
        double LRavg = (m_vLR.x + m_vLR.y) / 2.0;
        double LRdiff = (m_vLR.x - m_vLR.y);
        double sin_theta = LRdiff / (m_radius * 2.0);

        double dtheta = asin(sin_theta) * 180.0 / CV_PI;
        
        m_angdeg += dtheta;
        if (m_angdeg >= 360.0) m_angdeg -= 360;
        if (m_angdeg < 0.0) m_angdeg += 360.0;

        cv::Point2d ang_to_xy = get_ang_vec();
        
        cv::Point2d dpos = { ang_to_xy.x * LRavg, ang_to_xy.y * LRavg };
        m_xypos += dpos;
    }
}
