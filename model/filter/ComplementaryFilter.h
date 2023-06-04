//
// Created by redwan on 3/12/23.
//

#ifndef CREATE3_CONTROLLER_COMPLEMENTARYFILTER_H
#define CREATE3_CONTROLLER_COMPLEMENTARYFILTER_H
#include "FilterBase.h"


namespace model
{
    namespace filter
    {
        /// @brief The ComplementaryFilter.h class implements the complmentary filter 
        /// where it initializes the state and updates it using the filter's alpha paramater in update() method.
        class ComplementaryFilter: public FilterBase{
        public:
            explicit ComplementaryFilter(double alpha): m_alpha(alpha)
            {
                init_ = false;
            }

            void init(const std::vector<double>& X0)
            {
                X_.clear();
                std::copy(X0.begin(), X0.end(), std::back_inserter(X_));
            }
            
            void update(const std::vector<double>& obs, std::vector<double>& result) override
            {
                if(!init_)
                {
                    init(obs);
                    init_ = true;
                }
                for (int i = 0; i < obs.size(); ++i) {
                    X_[i] = m_alpha * X_[i] + (1 - m_alpha) * obs[i];
                }
                if(result.empty())
                    std::copy(X_.begin(), X_.end(),std::back_inserter(result));
                else
                    std::copy(X_.begin(), X_.end(),result.begin());
            }

            void update(const tf2::Transform& obs, tf2::Transform& res) override
            {
                // convert tf to vector
                auto origin = obs.getOrigin();
                auto theta = obs.getRotation().getAngle();
                std::vector<double> input{origin.x(), origin.y(), origin.z(), theta};
                std::vector<double> result;
                update(input, result);
                res.setOrigin(tf2::Vector3(result[0], result[1], result[2]));
                tf2::Quaternion q;
                q.setRPY(0, 0, result[3]);
                res.setRotation(q);
            }
        private:
            double m_alpha;
            std::vector<double>X_;
        };
    }
}


#endif //CREATE3_CONTROLLER_COMPLEMENTARYFILTER_H
