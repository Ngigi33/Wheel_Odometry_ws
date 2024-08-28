from launch import LaunchDescription
from launch_ros.actions import Node 
 
def generate_launch_description():
    ld=LaunchDescription()
     
    Serial_Node=Node(
        package="serial_pkg",
        executable="Serial_node",
    )
    Get_RPM= Node(
        package="rpm_getter",
        executable="Get_Rpm"
    )
    Distance_Calculator=Node(
        package="distance_calculator",
        executable="Distance_Calculator"
    )
    odometry_estimator_node=Node(
        package="odometry_estimator",
        executable="odometry_estimator"
    )
     
     
     
     #node_one is just a name; you can name it whatever
 
    ld.add_action(Serial_Node)
    ld.add_action(Get_RPM)
    ld.add_action(Distance_Calculator)
    ld.add_action(odometry_estimator_node)
    return ld
     
     
   