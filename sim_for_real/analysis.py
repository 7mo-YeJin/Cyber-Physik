import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json
from pathlib import Path
import rosbag

bagfile = Path("run3.bag")

lap_complete_topic = "/lap_complete"
speed_lim_topic = "/controller/constant_speed"
lookahead_topic = "/controller/lookahead_distance"
error_topic = "/controller/err"
d_min_curr_topic = "/controller/d_min_curr"
d_goal = 0.5
odom_topic = "/odometry/filtered"

topics_to_collect = set([lap_complete_topic,
                        speed_lim_topic,
                        error_topic,
                        lookahead_topic,
                        d_min_curr_topic,
                        odom_topic,
                        "/pid_drive"])


def bag_to_dfs(bagfile, topic_names):
    msgs = dict()

    cache_file = bagfile.with_suffix(".cache")
    if cache_file.is_file():
        print(f"reading {bagfile} messages from cache")
        with open(cache_file, "r") as f:
            msgs = json.load(f)
    else:
        print(f"started reading {bagfile}")

        for topic, msg, t in rosbag.Bag(bagfile).read_messages():
            if topic in topic_names:
                if topic not in msgs:
                    msgs[topic] = []

                msg_data = {"t": t.secs + 1e-9*t.nsecs}

                if msg._type == 'ackermann_msgs/AckermannDriveStamped':
                    for member in dir(msg.drive):
                        if not member.startswith('_'):
                            d = getattr(msg.drive, member)
                            if not callable(d):
                                msg_data[member] = d
                elif msg._type == 'nav_msgs/Odometry':
                    for member in dir(msg.twist.twist):
                        if not member.startswith('_'):
                            member_obj = getattr(msg.twist.twist, member)
                            for child_member in dir(member_obj):
                                if not child_member.startswith('_'):
                                    d = getattr(member_obj, child_member)
                                    if not callable(d):
                                        msg_data[f"{member}_{child_member}"] = d
                else:
                    msg_data["data"] = msg.data

                msgs[topic].append(msg_data)

        with open(cache_file, "w") as f:
            json.dump(msgs, f)

    print(f"started conversion to dataframes")

    dfs = dict()

    for topic in msgs:
        df = pd.DataFrame.from_dict(msgs[topic])
        df.set_index("t", inplace=True)
        dfs[topic] = df

    return dfs


def get_between_time(start_time, end_time, df):
    return df[(df.index > start_time) & (df.index < end_time)]


def get_last_before(time, df):
    return df[df.index < time].iloc[-1]


def rmse(df):
    return np.sqrt(np.mean(df.to_numpy()**2))


valid_laps_times = []
valid_laps_stats = pd.DataFrame()

if __name__ == "__main__":
    dfs = bag_to_dfs(bagfile, topics_to_collect)
    for i in range(len(dfs[lap_complete_topic].index)-1):
        start_time = dfs[lap_complete_topic].index[i]
        end_time = dfs[lap_complete_topic].index[i+1]
        speed_lim = get_between_time(
            start_time, end_time, dfs[speed_lim_topic])
        # Either there should be no new speed limit, or it should have stayed the same
        if len(speed_lim) == 0 or np.allclose(speed_lim, speed_lim.iloc[0]):
            valid_laps_times.append([start_time, end_time])

    lap = 0
    for start_time, end_time in valid_laps_times:
        lap += 1
        stats = dict()
        stats["lap_time [s]"] = end_time-start_time
        stats["speed_limit [m/s]"] = float(get_last_before(
            start_time, dfs[speed_lim_topic]))
        stats["lookahead distance [m]"] = float(get_last_before(
            start_time, dfs[lookahead_topic]))
        stats["RMSE (at lookahead point) [m]"] = rmse(get_between_time(
            start_time, end_time, dfs[error_topic]))
        stats["RMSE (at current pos) [m]"] = rmse(get_between_time(
            start_time, end_time, dfs[d_min_curr_topic]-d_goal))
        stats["ME (at lookahead point) [m]"] = float(np.mean(get_between_time(
            start_time, end_time, dfs[error_topic])))
        stats["ME (at current pos) [m]"] = float(np.mean(get_between_time(
            start_time, end_time, dfs[d_min_curr_topic]-d_goal)))
        stats["average speed [m/s]"] = np.mean(get_between_time(
            start_time, end_time, dfs[odom_topic]["linear_x"]))

        valid_laps_stats = valid_laps_stats.append(
            pd.DataFrame(stats, index=[lap]))

    valid_laps_stats.to_csv(bagfile.with_suffix(".csv"))

    ax = plt.figure().add_subplot(projection='3d')
    ax.scatter(valid_laps_stats["speed_limit [m/s]"],
               valid_laps_stats["lookahead distance [m]"], valid_laps_stats["RMSE (at lookahead point) [m]"])
    ax.scatter(valid_laps_stats["speed_limit [m/s]"],
               valid_laps_stats["lookahead distance [m]"], valid_laps_stats["RMSE (at current pos) [m]"])
    ax.set_xlabel("speed_limit [m/s]")
    ax.set_ylabel("lookahead distance [m]")
    ax.set_zlabel("RMSE [m]")
    plt.show()
    pass
