from enum import Enum
"""
This module defines enumerations for various statuses and tasks related to a robotic system.

Enumerations:
    SubTaskEnum: Represents various sub-tasks that can be performed in the system.
    OnlineFlagEnum: Represents the online status of a robot.
    RobotStatusEnum: Defines various statuses for a robot's operation.
"""


class TaskEnum(Enum):
    """
    Enumeration representing different types of tasks for the system.
    Attributes:
        CHARGING_TASK (int): Represents a charging task.
        HARVESTING_TASK (int): Represents a harvesting task.
        UNLOADING_TASK (int): Represents an unloading task.
    """

    CHARGING_TASK = 0
    HARVESTING_TASK = 1
    UNLOADING_TASK = 2


class SubTaskEnum(Enum):
    """
    SubTaskEnum is an enumeration that represents various sub-tasks
    that can be performed in the system. Each sub-task is associated
    with a unique integer value.
    Attributes:
        MOVING (int): Represents the task of moving.
        HARVESTING (int): Represents the task of harvesting.
        DOCKING (int): Represents the task of docking.
        CHARGING (int): Represents the task of charging.
        LOADING (int): Represents the task of loading.
        UNLOADING (int): Represents the task of unloading.
    """

    MOVING = 1
    HARVESTING = 2
    DOCKING = 3
    CHARGING = 4
    LOADING = 5
    UNLOADING = 6
    UNDOCKING = 7


class OnlineFlagEnum(Enum):
    """
    OnlineFlagEnum is an enumeration that represents the online status of a robot.

    Attributes:
        OFFLINE (int): Represents the offline state of the robot (value: 0).
        ONLINE (int): Represents the online state of the robot (value: 1).
        EMERGENCY_STOP (int): Represents the emergency stop state of the robot (value: 10).
        ABNORMAL (int): Represents an abnormal state of the robot (value: 11).
    """

    OFFLINE = 0
    ONLINE = 1
    EMERGENCY_STOP = 10
    ABNORMAL = 11


class RobotStatusEnum(Enum):
    """
    RobotStatusEnum is an enumeration that defines various statuses for a robot's operation.

    Each status is represented by a unique integer value.

    Attributes:
        IDLE (int): The robot is idle and not performing any task.
        JOB_START (int): The robot has started a job.
        JOB_DONE (int): The robot has completed a job.
        START_MOVING (int): The robot has started moving.
        MOVING (int): The robot is currently moving.
        DESTINATION_REACHED (int): The robot has reached its destination.
        START_HARVESTING (int): The robot has started the harvesting process.
        HARVESTING (int): The robot is currently harvesting.
        DONE_HARVESTING (int): The robot has completed the harvesting process.
        START_DOCKING (int): The robot has started the docking process.
        DOCKING (int): The robot is currently docking.
        DONE_DOCKING (int): The robot has completed the docking process.
        START_LOADING (int): The robot has started the loading process.
        LOADING (int): The robot is currently loading.
        DONE_LOADING (int): The robot has completed the loading process.
        START_UNLOADING (int): The robot has started the unloading process.
        UNLOADING (int): The robot is currently unloading.
        DONE_UNLOADING (int): The robot has completed the unloading process.
        START_CHARGING (int): The robot has started the charging process.
        CHARGING (int): The robot is currently charging.
        DONE_CHARGING (int): The robot has completed the charging process.
        ERROR (int): The robot has encountered an error.
        PAUSED (int): The robot is paused.
        MAINTENANCE (int): The robot is under maintenance.
        OFFLINE (int): The robot is offline.
        EMERGENCY_STOP (int): The robot has been stopped due to an emergency.
        ABNORMAL (int): The robot is in an abnormal state.
    """

    IDLE = 0

    JOB_START = 1
    JOB_DONE = 2

    START_MOVING = 3
    MOVING = 4
    DESTINATION_REACHED = 5

    START_HARVESTING = 6
    HARVESTING = 7
    DONE_HARVESTING = 8

    START_DOCKING = 9
    DOCKING = 10
    DONE_DOCKING = 11

    START_LOADING = 12
    LOADING = 13
    DONE_LOADING = 14

    START_UNDOCKING = 15
    UNDOCKING = 16
    DONE_UNDOCKING = 17

    START_UNLOADING = 18
    UNLOADING = 19
    DONE_UNLOADING = 20

    START_CHARGING = 21
    CHARGING = 22
    DONE_CHARGING = 23

    ERROR = 94
    PAUSED = 95
    MAINTENANCE = 96
    OFFLINE = 97
    EMERGENCY_STOP = 98
    ABNORMAL = 99
