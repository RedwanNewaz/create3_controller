//
// Created by redwan on 11/21/22.
//

#ifndef CREATE3_CONTROLLER_PARAM_MANAGER2_H
#define CREATE3_CONTROLLER_PARAM_MANAGER2_H
#include <yaml-cpp/yaml.h>
#include <memory>
#include <iostream>
#define DEBUG(x) std::cout << x << std::endl

class param_manager2;
typedef std::shared_ptr<param_manager2> ParamPtr2;


struct Matrix2D {
    std::vector<std::vector<double>> data;
    bool operator == (const Matrix2D& other)
    {
       return other.data.size() == data.size() && other.data[0].size() == data[0].size();
    }

    friend std::ostream &operator<<(std::ostream &os, const Matrix2D &d) {
        for (int i = 0; i < d.data.size(); ++i) {
            for (int j = 0; j < d.data[0].size(); ++j) {
                os << d.data[i][j] << " ";
            }
            os << "\n";
        }

        return os;
    }
};

namespace YAML {
    template<>
    struct convert<Matrix2D> {
        static Node encode(const Matrix2D& rhs) {
            Node node;
            for(auto& d:rhs.data)
                node.push_back(d);
            return node;
        }

        static bool decode(const Node& node, Matrix2D& rhs) {
            for(auto& temp:node)
            {
                std::vector<double> cols;
                for(auto& item: temp)
                    cols.push_back(item.as<double>());
                rhs.data.push_back(cols);
            }

            return true;
        }
    };
}



class param_manager2: public std::enable_shared_from_this<param_manager2>{
public:
    param_manager2(const std::string& file)
    {
        DEBUG("loading " << file);
        config_ = YAML::LoadFile(file);
    }



    template<class T>
    T get_param(const std::string& field)
    {
        T value = config_[field].template as<T>();
        DEBUG(field << " = " << value);
        return value;
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2)
    {
        return config_[field1][field2].template as<T>();
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2, const std::string& field3)
    {
        return config_[field1][field2][field3].template as<T>();
    }

    template<class T>
    void get_obstacles(T& result)
    {
        std::cout << "[obstacles]:" << std::endl;
        Matrix2D obstacles = config_["obstacles"].as<Matrix2D>();
        std::cout << obstacles << std::endl;
        for(auto& item: obstacles.data)
        {
            result.push_back({item[0], item[1]});
        }
    }
private:
    YAML::Node config_;


};

#endif //CREATE3_CONTROLLER_PARAM_MANAGER2_H
