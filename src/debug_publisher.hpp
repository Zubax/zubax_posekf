/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <unordered_map>
#include <type_traits>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt64.h>
#include <eigen3/Eigen/Eigen>

namespace zubax_posekf
{

class DebugPublisher
{
    static constexpr unsigned QueueSize = 10;

    ros::NodeHandle node_;
    std::unordered_map<std::string, ros::Publisher> pubs_;

    template <typename Message>
    ros::Publisher& getPublisher(const std::string& name)
    {
        auto pub = pubs_.find(name);
        if (pub == pubs_.end())
        {
            (void)pubs_.insert({name, node_.advertise<Message>(name, QueueSize)});
            pub = pubs_.find(name);
            ROS_INFO("Debug publisher: New topic '%s' of type '%s'",
                     name.c_str(), ros::message_traits::DataType<Message>::value());
        }
        ROS_ASSERT(pub != pubs_.end());
        return pub->second;
    }

public:
    DebugPublisher(const std::string& ros_ns = "debug")
        : node_(ros_ns)
    { }

    template <class Message>
    void publishROSMessage(const std::string& name, const Message& msg)
    {
        getPublisher<Message>(name).publish(msg);
    }

    template <typename Derived>
    void publish(const std::string& name, const Eigen::MatrixBase<Derived>& matrix)
    {
        std_msgs::Float64MultiArray msg;

        msg.layout.dim.resize(2);
        msg.layout.dim[0].stride = static_cast<unsigned>(matrix.rows() * matrix.cols());
        msg.layout.dim[0].size   = static_cast<unsigned>(matrix.rows());
        msg.layout.dim[1].stride = static_cast<unsigned>(matrix.cols());
        msg.layout.dim[1].size   = static_cast<unsigned>(matrix.cols());
        msg.data.resize(static_cast<unsigned>(matrix.size()));

        {
            unsigned write_idx = 0;
            for (int row_idx = 0; row_idx < matrix.rows(); row_idx++)
            {
                // We can't access the matrix elements directly via coeff() in a generic way
                const auto row = matrix.row(row_idx);
                for (int col_idx = 0; col_idx < matrix.cols(); col_idx++)
                {
                    msg.data[write_idx++] = static_cast<double>(row[col_idx]);
                }
            }
        }

        publishROSMessage(name, msg);
    }

    template <typename Scalar, int Options>
    void publish(const std::string& name, const Eigen::Matrix<Scalar, 3, 1, Options>& vector)
    {
        geometry_msgs::Vector3 msg;
        msg.x = vector[0];
        msg.y = vector[1];
        msg.z = vector[2];
        publishROSMessage(name, msg);
    }

    template <typename Scalar>
    typename std::enable_if<std::is_floating_point<Scalar>::value>::type
    publish(const std::string& name, Scalar scalar)
    {
        std_msgs::Float64 msg;
        msg.data = static_cast<double>(scalar);
        publishROSMessage(name, msg);
    }

    template <typename Scalar>
    typename std::enable_if<std::is_integral<Scalar>::value>::type
    publish(const std::string& name, Scalar scalar)
    {
        std_msgs::Int64 msg;
        msg.data = static_cast<std::int64_t>(scalar);
        publishROSMessage(name, msg);
    }
};

}
