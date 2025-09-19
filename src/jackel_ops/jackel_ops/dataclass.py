"""
This module defines data structures and enumerations for the status monitor.

Classes:
    WayPoint: Represents a waypoint with a unique identifier and coordinates.
    SubTask: Represents a sub-task with an identifier, type, description, and associated data.
    Task: Represents a task assigned to a robot with details such as ID, robot assignment,
        schedule, and sub-tasks.
    WPFStatus: Represents the status of a way point follower with details like current and
        target nodes.
    DockingGoal: Represents the goal for a docking process, including dock ID and navigation flag.
    UndockingGoal: Represents the goal for an undocking process, including dock type and
        maximum undocking time.
    DockingFeedback: Represents feedback for a docking process, including status, location, and
        retry information.
"""
from dataclasses import dataclass


# @dataclass
# class WayPoint:
#     """
#     Represents a waypoint with a unique identifier and coordinates.

#     Attributes:
#         node_id (int): Unique identifier for the waypoint.
#         x (float): X-coordinate of the waypoint.
#         y (float): Y-coordinate of the waypoint.
#     """

#     node_id: int
#     x: float
#     y: float


# @dataclass
# class SubTask:
#     """
#     Represents a sub-task with an identifier, description, and associated data.

#     Attributes:
#         sub_task_id (int): The unique identifier for the sub-task.
#         type (str): The type or category of sub-task.
#         description (str): A brief description of the sub-task.
#         data (list[WayPoint] | str): The data associated with the sub-task,
#             which can either be a list of WayPoint objects or a string.
#     """

#     sub_task_id: int
#     type: int
#     description: str
#     data: list[WayPoint] | str


# @dataclass
# class Task:
#     """
#     Represents a task assigned to a robot in the system.

#     Attributes:
#         task_id (int): Unique identifier for the task.
#         assigned_robot_id (int): Identifier of the robot assigned to this task.
#         target_node_id (int): Identifier of the target node where the task is to be performed.
#         description (str): A brief description of the task.
#         crop_type (str): Type of crop associated with the task.
#         sub_tasks (list[SubTask] | SubTask): A list of sub-tasks or a single sub-task
#             associated with this task.
#     """

#     task_id: int
#     task_type: int
#     assigned_robot_id: int
#     target_node_id: int
#     job_schedule: str
#     description: str
#     crop_type: str
#     sub_tasks: SubTask


@dataclass
class WPFStatus:
    """
    WPFStatus class represents the status of a way point follower process.

    Attributes:
        status (int): The current status of the way point follower process.
        task (str): The task associated with the way point follower process.
        current_node_id (int): The ID of the current node in the way point follower.
        target_node_id (int): The ID of the target node in the way point follower.
    """

    status: int
    task: str
    current_node_id: int
    target_node_id: int


# @dataclass
# class DockingGoal:
#     """
#     DockingGoal class represents the goal for a docking process.

#     Attributes:
#         dock_id (str): The ID of the docking station.
#         navigate_to_staging_pose (bool): Flag indicating whether to navigate to the staging pose.
#     """

#     dock_id: str
#     navigate_to_staging_pose: bool


# @dataclass
# class UndockingGoal:
#     """
#     UndockingGoal class represents the goal for an undocking process.

#     Attributes:
#         dock_id (str): The ID of the docking station.
#         navigate_to_staging_pose (bool): Flag indicating whether to navigate to the staging pose.
#     """
#     dock_type: str
#     max_undocking_time: float


@dataclass
class DockingFeedback:
    """DockingStatus class represents the status of a docking process.

    Attributes:
        status (int): The current status of the docking process.
        task (str): The task associated with the docking process.
    """

    status: int
    task: str
    docking_location: str
    feedback_message: str
    docking_time: float
    num_retries: int
