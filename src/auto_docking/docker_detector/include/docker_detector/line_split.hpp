/**
 * @file        line_split.hpp
 * @brief       激光点云线段分割
 * @details  	an implementation of Ramer–Douglas–Peucker algorithm
 * @author      glh
 * @date        2020-08-04
 * @version     V1.0.0
 * @copyright   Copyright (c) 2021-2021  iscas
 */
#ifndef SRC_LINE_SPLIT_HPP
#define SRC_LINE_SPLIT_HPP

#include <iostream>
#include <vector>
#include <list>
#include <Eigen/Dense>

/**
 * @brief The Point struct
 */
struct Point
{
    double x;
    double y;
//    int index;
};
/**
 * @brief The Line struct
 */
struct Line
{
    Point start_point;
    Point end_point;
    double len;
    double theta;
    uint32_t pointNUm;
};
/**
 * @brief The LineSplit class
 */
class LineSplit
{
public:
    LineSplit(uint32_t min_point_num, double_t max_gap_dis);

    /**
     * @brief LineSplitWithRDP RDP算法
     * @param Points 激光点云
     * @param epsilon 分割距离阈值
     * @return 线段向量
     */
    std::vector<Line> LineSplitWithRDP(const std::vector<Point>& Points, double epsilon);

private:
    uint32_t min_point_num;
    double max_gap_dis;

    /**
     * @brief perpendicularDistance 计算点到直线的距离
     * @param p
     * @param line
     * @return
     */
    double perpendicularDistance(const Point& p, const Line& line);
    /**
     * @brief perpendicularDistance 计算点到直线的距离
     * @param p
     * @param line_p1
     * @param line_p2
     * @return
     */
    double perpendicularDistance(const Point& p, const Point& line_p1, const Point& line_p2);
};


#endif //SRC_LINE_SPLIT_HPP
