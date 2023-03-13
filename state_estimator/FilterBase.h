//
// Created by redwan on 3/12/23.
//

#ifndef CREATE3_CONTROLLER_FILTERBASE_H
#define CREATE3_CONTROLLER_FILTERBASE_H
#include <cassert>
#include <vector>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/twist.hpp>

class FilterBase{
public:
    FilterBase()
    {
        init_ = false;
    }
    virtual void update(const tf2::Transform& obs, tf2::Transform& res)
    {
        assert("Not implemented");
    }
    virtual void update(const std::vector<double>& obs, std::vector<double>& result)
    {
        assert("Not implemented");
    }

    virtual void update(const tf2::Transform& obs, const geometry_msgs::msg::Twist& cmd,  tf2::Transform& res)
    {
        assert("Not implemented");
    }
protected:
    bool init_;
};

#endif //CREATE3_CONTROLLER_FILTERBASE_H
