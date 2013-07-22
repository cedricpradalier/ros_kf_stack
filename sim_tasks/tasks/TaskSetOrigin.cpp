#include <math.h>
#include "TaskSetOrigin.h"
#include "sim_tasks_cfg/TaskSetOriginConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace sim_tasks_cfg;
using namespace sim_tasks;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskSetOrigin::iterate()
{
    if (cfg.current) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        env->setOrigin2D(tpose);
    } else {
        geometry_msgs::Pose2D tpose;
        tpose.x = cfg.x;
        tpose.y = cfg.y;
        env->setOrigin2D(tpose);
    }
    return TaskStatus::TASK_COMPLETED;
}

DYNAMIC_TASK(TaskFactorySetOrigin);
