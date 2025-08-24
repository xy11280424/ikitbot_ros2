//
// Created by glh on 8/5/20.
//
#include "docker_detector/docker_detector.hpp"

int main(int argc, char **argv)
{
    double d;

    Point p={1,1,0};
    Point line_p1={0,0,0};
    Point line_p2={-1,-1,0};

    Eigen::Vector3d a(line_p2.x-line_p1.x, line_p2.y-line_p1.y, 0);
    Eigen::Vector3d b(p.x-line_p1.x, p.y-line_p1.y, 0);

    Eigen::Vector3d c = a.cross(b);

    cout << "Cross product:\n" << a.cross(b) << endl;
    d = c.norm()/a.norm();

    std::cout << "dist : " << d << std::endl;
    return 0;
}