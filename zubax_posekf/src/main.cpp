/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "imu_filter.hpp"

namespace zubax_posekf
{
    class IMUFilterWrapper
    {
        IMUFilter filter_;
        ros::Subscriber sub_imu_;
        ros::Publisher pub_imu_;

        void cbImu(const sensor_msgs::Imu& msg)
        {
            (void)msg;
        }

    public:
        IMUFilterWrapper(ros::NodeHandle& node, unsigned queue_size = 10)
        {
            sub_imu_ = node.subscribe("in", queue_size, &IMUFilterWrapper::cbImu, this);
            pub_imu_ = node.advertise<sensor_msgs::Imu>("out", queue_size);
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "posekf");

    ros::NodeHandle node;

    zubax_posekf::IMUFilterWrapper wrapper(node);

    ros::spin();

    return 0;
}
