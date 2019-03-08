#include <PlannersLib/PlannersLib.hpp>


void showTime(string message, struct timeb st, struct timeb ft)
{

	float seconds, milliseconds;
	seconds = ft.time - st.time - 1;
	milliseconds = (1000 - st.millitm) + ft.millitm;
	cout << message << (milliseconds + seconds * 1000) <<  endl;
}
float getYawFromQuat(geometry_msgs::Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}
void printfTrajectory(trajectory_msgs::MultiDOFJointTrajectory traj)
{
    printf(PRINTF_YELLOW "Trajectory points: %d\n", (int)traj.points.size());
    for (unsigned int i = 0; i < traj.points.size(); i++)
    {
        double yaw = getYawFromQuat(traj.points[i].transforms[0].rotation);
        printf(PRINTF_BLUE "\t %d: [%.3f, %.3f] m,  [%.2f] grados \n", i, traj.points[i].transforms[0].translation.x, traj.points[i].transforms[0].translation.y, yaw);
    }
    printf(PRINTF_REGULAR);
}
geometry_msgs::TransformStamped getTransform(string from_frame, string to_frame,tf2_ros::Buffer *tfBuffer)
{
    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer->lookupTransform(to_frame, from_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return ret;
}