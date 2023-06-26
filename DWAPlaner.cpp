#include <iostream>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define DEG2RAD(a) ((a) * (M_PI / 180.0f))

// ARANGE Template (like in Python)
template <typename T>
vector<T> arange(T start, T stop, T step)
{
    vector<T> values;
    for (T i = start; i <= stop + 0.0001f; i += step)
    {
        values.push_back(i);
    }
    return values;
}

class DWA
{
public:
    // Variables
    float goal_x;
    float goal_y;
    float position_x = 0;
    float position_y = 0;
    float position_accuracy = 0.2;

    float robot_theta = 0;

    float *lidar_x;
    float *lidar_y;
    float *ranges;

    float v_ref = 0;
    float omega_ref = 0;

    float v_current = 0;
    float omega_current = 0;
    // DWA Parameters
    float a_max = 0.1;
    float v_max = 0.1;
    float omega_max = 0.3 * 3.14;
    float epsilon_max = 1 * 3.14;
    // STEP
    float dT = 0.5;

    float v_accuracy_prediction = 0.005;
    float omega_accuracy_prediction = 0.001 * 3.14;
    // Prediction time
    float prediction_time = 3.0f;
    float heading = 1.0f;
    float distance = 20.0f;
    float velocity = 1.0f;
    float obstacledistance = 0.3;

    int lidar_N = 0;

    int min_index = 0;

    bool firstinit = false;
    // minimum laser range!!!!!
    float min_range = 0.1;

    // inicialization of goal_x and goal_y
    DWA(float x, float y)
    {
        goal_x = x;
        goal_y = y;
    }

    // Take robot position x,y,theta
    void ODO_CALLBACK(const nav_msgs::Odometry::ConstPtr &msg)
    {

        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        robot_theta = 2.0f * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        ROSINFO("X=%f, Y=%f, Theta=%f", position_x, position_y, robot_theta);

        degree_repair(robot_theta);
    }
    void LASER_CALLBACK(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        lidar_N = (int)((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;

        // first initialization - locate memory for table
        if (firstinit == false)
        {
            ranges = new float[lidar_N];
            lidar_x = new float[lidar_N];
            lidar_y = new float[lidar_N];
            firstinit = true;
        }
        // Calculate obstacle global position X/Y and min range from the obstacle
        float min_value = 9999;
        for (int i = 0; i < lidar_N; i++)
        {

            ranges[i] = msg->ranges[i];
            if (~isnan(ranges[i]) && ~isinf(ranges[i]) && ranges[i] > min_range)
            {
                if (ranges[i] < min_value)
                {
                    min_index = i;
                    min_value = ranges[i];
                }

                float xl = cos(msg->angle_min + i * msg->angle_increment) * ranges[i];
                float yl = sin(msg->angle_min + i * msg->angle_increment) * ranges[i];

                // obstacle global position
                lidar_x[i] = position_x + xl * cos(robot_theta) - yl * sin(robot_theta);
                lidar_y[i] = position_y + xl * sin(robot_theta) + yl * cos(robot_theta);
            }
        }
    }
    // Destructor
    ~DWA()
    {
        delete[] ranges;
        delete[] lidar_x;
        delete[] lidar_y;
    }
    // making sure that degree will be between -pi to pi
    void degree_repair(float &robot_theta)
    {

        if (robot_theta > DEG2RAD(180))
        {
            robot_theta -= DEG2RAD(360);
        }
        else if (robot_theta < (-1.0f) * DEG2RAD(180))
        {
            robot_theta += DEG2RAD(360);
        }
    }

    // Calculating DWA
    void compute()
    {

        if (hypot(goal_x - position_x, goal_y - position_y) > position_accuracy)
        {

            float DynamicWindow[4] = {max(0.05f, v_current - a_max * dT),
                                      min(v_max, v_current + a_max * dT),
                                      max(-omega_max, omega_current - epsilon_max * dT),
                                      min(omega_max, omega_current + epsilon_max * dT)};

            auto v_range = arange<float>(DynamicWindow[0], DynamicWindow[1], v_accuracy_prediction);
            auto omega_range = arange<float>(DynamicWindow[2], DynamicWindow[3], omega_accuracy_prediction);
            auto t_prediction_range = arange<float>(0.0f, prediction_time, dT);
            int v_range_size = v_range.size();
            int omega_range_size = omega_range.size();
            int t_prediction_size = t_prediction_range.size();

            // Vector 4D :D
            vector<vector<vector<vector<float>>>> eval_trajectory(v_range_size, vector<vector<vector<float>>>(omega_range_size, vector<vector<float>>(t_prediction_size, vector<float>(3, 0.0f))));

            vector<vector<float>> eval_function(v_range_size, vector<float>(omega_range_size, 0));

            float opt_max_eval_fun = -999.0f;
            float opt_idx[2] = {0, 0};
            // ROS_INFO("compute");
            int checkcount = 0;
            int checkcountfull = 0;
            for (int i = 0; i < v_range_size; i++)
            {
                for (int j = 0; j < omega_range_size; j++)
                {
                    float eval_v = v_range[i];
                    float eval_omega = omega_range[j];
                    eval_trajectory[i][j][0][0] = position_x;
                    eval_trajectory[i][j][0][1] = position_y;
                    eval_trajectory[i][j][0][2] = robot_theta;
                    for (int k = 1; k < t_prediction_size; k++)
                    {
                        float x_prev = eval_trajectory[i][j][k - 1][0];
                        float y_prev = eval_trajectory[i][j][k - 1][1];
                        float theta_prev = eval_trajectory[i][j][k - 1][2];

                        eval_trajectory[i][j][k][0] = x_prev + dT * cos(theta_prev) * eval_v;
                        eval_trajectory[i][j][k][1] = y_prev + dT * sin(theta_prev) * eval_v;
                        eval_trajectory[i][j][k][2] = theta_prev + dT * eval_omega;
                    }

                    float robot_theta = eval_trajectory[i][j][t_prediction_size - 1][2];
                    float goaltheta = atan2(goal_y - eval_trajectory[i][j][t_prediction_size - 1][1],
                                            goal_x - eval_trajectory[i][j][t_prediction_size - 1][0]);

                    degree_repair(robot_theta);

                    float velocityReward = velocity * eval_v;
                    float headingReward = heading * (DEG2RAD(180) - abs(robot_theta - goaltheta));
                    float distanceReward = distance * (1.0f / (1.0f + hypot(goal_x - eval_trajectory[i][j][t_prediction_size - 1][0],
                                                                            goal_y - eval_trajectory[i][j][t_prediction_size - 1][1])));

                    eval_function[i][j] = headingReward + distanceReward + velocityReward;
                    bool check = false;
                    // Check if there are obstacles on the way, ignore first 2 predictions
                    for (int k = 2; k < t_prediction_size; k = k + 1)
                    {

                        if (hypot(lidar_x[min_index] - eval_trajectory[i][j][k][0], lidar_y[min_index] - eval_trajectory[i][j][k][1]) <= obstacledistance)
                        {
                            check = true;
                            checkcount++;
                        }
                    }

                    if (eval_function[i][j] > opt_max_eval_fun && check == false)
                    {
                        opt_idx[0] = i;
                        opt_idx[1] = j;
                        opt_max_eval_fun = eval_function[i][j];
                    }
                }
            }

            try
            {
                v_ref = v_range[opt_idx[0]];
                omega_ref = omega_range[opt_idx[1]];
            }
            catch (...)
            {
                v_ref = 0;
                omega_ref = 0;
            }
            v_current = v_ref;

            omega_current = omega_ref;
        }
    }
};
// Basic Main Node function
int main(int argc, char **argv)
{
    float ref_x = 0.0f;
    float ref_y = 0.0f;

    if (argc >= 2)
    {
        ref_x = atof(argv[1]);
        ref_y = atof(argv[2]);
    }

    DWA dwa(ref_x, ref_y);
    
    ros::init(argc, argv, "DWAPlaner");

    ros::NodeHandle n;

    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &DWA::LASER_CALLBACK, &dwa);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, &DWA::ODO_CALLBACK, &dwa);

    ros::Publisher ctr_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        dwa.compute();
        geometry_msgs::Twist msg;
        msg.angular.z = dwa.omega_ref;
        msg.linear.x = dwa.v_ref;
        ctr_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
