//
// Created by glh on 8/4/20.
//

#include "docker_detector/line_split.hpp"
#include <cmath>
LineSplit::LineSplit(uint32_t min_point_num, double_t max_gap_dis):
    min_point_num(min_point_num),
    max_gap_dis(max_gap_dis)
{
    std::cout << " line split , min_point_num: " << min_point_num << std::endl;
    std::cout << " line split , max_gap_dis: " << max_gap_dis << std::endl;
}

double LineSplit::perpendicularDistance(const Point &p, const Line &line)
{

    double d;

    return  d;
}

double LineSplit::perpendicularDistance(const Point &p, const Point &line_p1, const Point &line_p2)
{
    double d;

    Eigen::Vector3d a(line_p2.x-line_p1.x, line_p2.y-line_p1.y, 0);
    Eigen::Vector3d b(p.x-line_p1.x, p.y-line_p1.y, 0);

    Eigen::Vector3d c = a.cross(b);
    d = c.norm()/a.norm();
    return d;
}

std::vector<Line> LineSplit::LineSplitWithRDP(const std::vector<Point> &Points, double epsilon)
{
    std::vector<Line> resultlist;

    if(Points.size() < min_point_num)   //todo 应该丢弃这一段
        return resultlist;
    double max_distance = 0;
    int32_t max_i;
    int32_t gap_i = -1;

    for(int32_t i=0; i < Points.size()-1;i++) //exclude start end
    {
        double point_gap_dist = hypot(Points[i].x - Points[i+1].x, Points[i].y - Points[i+1].y);
        if(point_gap_dist > max_gap_dis)
        {
            gap_i = i;
            break; // braek , split
        }

        double d = perpendicularDistance(Points[i], Points.front(), Points.back());
        if(d > max_distance)    // distance
        {
            max_distance = d;
            max_i = i;
        }
    }
    if(gap_i != -1)
    {
        std::vector<Point> pre_part, next_part;

        pre_part.assign(Points.begin(), Points.begin() + gap_i+1);  // [begin, begin+max_i)
        next_part.assign(Points.begin() + gap_i+1, Points.end());   //[begin+max_i, end)

        std::vector<Line> resultlist1 = LineSplitWithRDP(pre_part, epsilon);
        std::vector<Line> resultlist2 = LineSplitWithRDP(next_part, epsilon);

        if ( !resultlist1.empty())
            resultlist.insert(resultlist.end(), resultlist1.begin(), resultlist1.end());
        if( !resultlist2.empty())
            resultlist.insert(resultlist.begin(), resultlist2.begin(), resultlist2.end());
    }
    // If max distance is greater than epsilon, recursively simplify
    else if(max_distance > epsilon)
    {
        std::vector<Point> pre_part, next_part;

        pre_part.assign(Points.begin(), Points.begin() + max_i+1);  // [begin, begin+max_i)
        next_part.assign(Points.begin() + max_i, Points.end());   //[begin+max_i, end)

        std::vector<Line> resultlist1 = LineSplitWithRDP(pre_part, epsilon);
        std::vector<Line> resultlist2 = LineSplitWithRDP(next_part, epsilon);

        if ( !resultlist1.empty())
            resultlist.insert(resultlist.end(), resultlist1.begin(), resultlist1.end());
        if( !resultlist2.empty())
            resultlist.insert(resultlist.begin(), resultlist2.begin(), resultlist2.end());
    }
    else
    {
        Line result;
        result.start_point = Points.front();
        result.end_point = Points.back();

        double_t diff_x = result.end_point.x - result.start_point.x;
        double_t diff_y = result.end_point.y - result.start_point.y;
        result.theta = std::atan2(diff_y, diff_x);

        result.len = hypot(result.end_point.x - result.start_point.x, result.end_point.y - result.start_point.y);
        result.pointNUm = Points.size();
        resultlist.push_back(result);
    }
    return resultlist;
}
