#ifndef SUBSCRIBER_STATISTICS_H
#define SUBSCRIBER_STATISTICS_H


#include <ros/ros.h>
#include <string>
#include <map>


namespace sim_tasks {

    class SubscriberStatistics {
        protected:
            size_t counter;
            ros::Time last_message;
            ros::Time first_message;
            size_t freq_counter;
            ros::Time mark;
        public:
            SubscriberStatistics() :counter(0),freq_counter(0) {}
            void setMark() {
                freq_counter = 0;
                mark = ros::Time::now();
            }
            void tick() {
                ros::Time t = ros::Time::now();
                if (counter==0) {
                    first_message = t;
                }
                ++freq_counter;
                ++counter;
                last_message = t;
            }
            double getFrequency(bool since_mark) const {
                if (since_mark) {
                    return freq_counter / (ros::Time::now() - mark).toSec();
                } else {
                    return counter / (ros::Time::now() - first_message).toSec();
                }
            }
            size_t getCount(bool since_mark) const {
                if (since_mark) {
                    return freq_counter ;
                } else {
                    return counter ;
                }
            }
            double timeSinceLastMessage() const {
                return (ros::Time::now() - last_message).toSec();
            }
    };

    typedef std::map<std::string,SubscriberStatistics> SubscriberStatMap;
};


#endif // SUBSCRIBER_STATISTICS_H


