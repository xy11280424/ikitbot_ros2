/**
 * @file        yaml_helper.h
 * @brief       yaml文件读写工具
 * @details  	主要提供yaml文件的读写功能
 * @author      weibw
 * @date        2021-11-05
 * @version     V1.0.0
 * @copyright   Copyright (c) 2021-2021  iscas
 */
#pragma once

#include <iostream>
#include <fstream>
#include <string>
//#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include "geometry_msgs/msg/vector3.hpp"

namespace YAML {
template <>
struct convert<geometry_msgs::msg::Vector3> {
    static Node encode(const geometry_msgs::msg::Vector3 &rhs){
        Node node;
        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        return node;
    }
    static bool decode(const Node& node, geometry_msgs::msg::Vector3& rhs) {
        if(!node.IsSequence() || node.size() != 3){
            return false;
        }
        rhs.x = node[0].as<double>();
        rhs.y = node[1].as<double>();
        rhs.z = node[2].as<double>();
        return true;
    }
};
}

/**
 * @class SyncBuffer
 * @brief 主动将文件内容同步到磁盘
 */
class SyncBuffer : public std::filebuf {
public:
    /**
     * @brief 构造函数
     * @param fout 文件输出流
     * @par 示例:
     * @code
     *    ofstream fout(file_path);
     *    fout << file << flush;
     *    fout.flush();
     *    SyncBuffer x(fout);
     *    fout.close();
     * @endcode
     */
    SyncBuffer(std::ofstream& fout) {
        SyncBuffer* sbuf;
        sbuf = static_cast<SyncBuffer*>(fout.rdbuf());
        fsync(sbuf->_M_file.fd());
    }
};
/**
 * @class YamlHelper
 * @brief yaml文件读写工具类
 */
class YamlHelper {
public:
    /**
     * @brief 构造函数
     */
    YamlHelper() {}

    /**
     * @brief 析构函数
     */
    ~YamlHelper() {}

    /**
     * @brief 新建yaml文件
     * @param[in] file_name 文件名
     */
    bool newFile(const std::string& file_name) {
        std::ofstream fout(file_name);
        fout << m_yaml_content;
        fout.close();
        return loadFile(file_name);
    }

    /**
     * @brief 加载yaml文件
     * @param[in] file_name 文件名
     * @return 加载结果
     * @retval true 加载成功
     * @retval false 加载失败
     */
    bool loadFile(const std::string& file_name) {
        m_file_path = file_name;
        try {
            m_yaml_content = YAML::LoadFile(file_name);
        }
        catch (YAML::BadFile &e) {
//            ROS_ERROR_STREAM("yaml badfile: " << e.msg);
            return false;
        }
        catch (YAML::Exception &e) {
//            ROS_ERROR_STREAM("YAML::Exception: " << e.msg);
            return false;
        }

        return true;
    }

    /**
     * @brief 修改yaml文件中的参数值
     * @tparam T 输入类型
     * @param[in] param_name 参数名
     * @param[in] value 参数值
     * @return 修改结果
     * @retval true 修改成功
     * @retval false 修改失败
     * @note 使用此方法只能修改yaml文件中已经存在的参数，如果参数名不存在则修改失败
     */
    template<typename T>
    bool modifyParam(const std::string& param_name, const T& value) {
        std::ofstream fout(m_file_path);
        if(m_yaml_content[param_name]) {
            m_yaml_content[param_name] = value;
            fout << m_yaml_content << std::flush;
            fout.flush();
            SyncBuffer x(fout);
            fout.close();
        }
        else {
            //ROS_ERROR_STREAM("it is invalid or the file does not contain a tag: " << param_name);
            return false;
        }
        return true;
    }

    /**
     * @brief 读取yaml文件中的参数值
     * @tparam T 输入类型
     * @param[in] param_name 参数名
     * @param[out] value 参数值
     * @return 读取结果
     * @retval true 读取成功
     * @retval false 读取失败
     */
    template<typename T>
    bool readParam(const std::string& param_name, T& value) {
        try {
            value = m_yaml_content[param_name].as<T>();
        }
        catch (YAML::BadConversion) {
            //ROS_ERROR_STREAM("it is invalid or the file does not contain a tag: " << param_name);
            return false;
        }
        catch (YAML::Exception &e) {
            //ROS_ERROR_STREAM("YAML::Exception " << e.msg);
            return false;
        }
        return true;
    }

    /**
     * @brief 向yaml文件中添加参数
     * @tparam T 输入类型
     * @param[in] param_name 参数名
     * @param[in] value 参数值
     */
    template<typename T>
    void addParam(const std::string& param_name, const T &value) {
        std::ofstream fout(m_file_path);
        m_yaml_content[param_name] = value;
        fout << m_yaml_content<< std::flush;
        fout.flush();
        SyncBuffer x(fout);
        fout.close();
    }

    /**
     * @brief 删除yaml文件中的参数
     * @param[in] param_name 参数名
     * @return 执行结果
     * @retval true 删除成功
     * @retval false 删除失败
     */
    bool removeParam(const std::string& param_name) {
        std::ofstream fout(m_file_path);
        if(m_yaml_content[param_name]) {
            m_yaml_content.remove(param_name);
            if(m_yaml_content.size() == 0) {
                fout.close();
                std::ofstream fout(m_file_path, std::ios::out);
            }
            else {
                fout << m_yaml_content<< std::flush;
                fout.flush();
                SyncBuffer x(fout);
                fout.close();
            }
            return true;
        }
        else
            return false;
    }

    /**
     * @brief 检查yaml文件中是否存在某个参数
     * @param[in] param_name 参数名
     * @return 参数是否存在
     * @retval true 存在
     * @retval false 不存在
     */
    bool hasKey(const std::string& param_name) {
        if(m_yaml_content[param_name])
            return true;
        else
            return false;
    }

private:
    std::string         m_file_path;    ///< yaml文件完整路径
    YAML::Node          m_yaml_content; ///< yaml文件内容
};


