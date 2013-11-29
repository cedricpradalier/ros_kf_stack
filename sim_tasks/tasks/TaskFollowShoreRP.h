#ifndef TASK_FOLLOW_SHORE_RP_H
#define TASK_FOLLOW_SHORE_RP_H

#include "radial_plan/RadialPlan.h"
#include "radial_plan/LocalPlan.h"
#include "task_manager_lib/TaskDefinition.h"
#include "sim_tasks/SimTasksEnv.h"
#include "sim_tasks_cfg/TaskFollowShoreRPConfig.h"

using namespace task_manager_lib;
using namespace sim_tasks_cfg;

namespace sim_tasks {
    class TaskFollowShoreRP : public TaskInstance<TaskFollowShoreRPConfig,SimTasksEnv>
    {

        protected:
            ros::Publisher path_pub;
            boost::shared_ptr<radial_plan::RadialPlan> RP;
            boost::shared_ptr<radial_plan::LocalPlan> LP;
        public:
            TaskFollowShoreRP(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskFollowShoreRP() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryFollowShoreRP : public TaskDefinition<TaskFollowShoreRPConfig, SimTasksEnv, TaskFollowShoreRP>
    {

        public:
            TaskFactoryFollowShoreRP(TaskEnvironmentPtr env) : 
                Parent("FollowShoreRP","Follow the shore of the lake usin radial planning",true,env) {}
            virtual ~TaskFactoryFollowShoreRP() {};
    };
};

#endif // TASK_FOLLOW_SHORE_RP_H
