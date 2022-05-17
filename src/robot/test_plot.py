import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    print(plt.get_current_fig_manager())

    load_data = np.genfromtxt(
        "C:/Users/Student/Documents/RLBot_IS/trajectory-optimization/data/bot_data.csv",
        delimiter=",",
    )
    y = load_data[0]
    z = load_data[1]
    y_dot = load_data[2]
    z_dot = load_data[3]
    theta = load_data[4]
    theta_dot = load_data[5]
    boost = load_data[6]
    pitch = load_data[7]

    y_traj = load_data[8]
    z_traj = load_data[9]
    y_dot_traj = load_data[10]
    z_dot_traj = load_data[11]
    theta_traj = load_data[12]
    theta_dot_traj = load_data[13]
    boost_traj = load_data[14]
    pitch_traj = load_data[15]

    t = load_data[16]

    theta_des = load_data[17]

    a_traj_y = load_data[18]
    a_traj_z = load_data[19]
    a_lat_y = load_data[20]
    a_lat_z = load_data[21]
    a_des_y = load_data[22]
    a_des_z = load_data[23]
    boost_des = load_data[24]

    plt.figure(1)
    plt.title("Trajectory")
    plt.xlabel("y")
    plt.ylabel("z")
    plt.plot(y, z, label="Actual Trajectory")
    plt.plot(y_traj, z_traj, label="Optimal Trajectory")
    # plt.scatter(y[100], z[100], marker="x")
    # plt.scatter(y_traj[100], z_traj[100], marker="*")
    plt.legend()
    figure = plt.gcf()
    figure.set_size_inches(32, 18)
    plt.savefig("plots/trajectory.png", bbox_inches="tight")

    plt.figure(2)
    plt.title("Pitch Inputs")
    plt.xlabel("t")
    plt.ylabel("pitch acceleration input")
    plt.plot(t, pitch, label="Actual")
    plt.plot(t, pitch_traj, label="Optimal Trajectory")
    plt.legend()
    figure = plt.gcf()
    figure.set_size_inches(32, 18)
    plt.savefig("plots/pitch_inputs.png", bbox_inches="tight")

    # plt.figure()
    # plt.title("Pitch Inputs")
    # plt.xlabel("t")
    # plt.ylabel("Pitch Input")
    # plt.plot(t, theta, label="Actual")
    # plt.plot(t, theta_traj, label="Optimal Trajectory")
    # plt.legend()

    plt.figure(3)
    plt.title("Angular Velocity")
    plt.xlabel("t")
    plt.ylabel("Theta_dot")
    plt.plot(t, theta_dot, label="Actual")
    plt.plot(t, -theta_dot_traj * 180 / np.pi, label="Optimal Trajectory")
    plt.legend()
    figure = plt.gcf()
    figure.set_size_inches(32, 18)
    plt.savefig("plots/angular_vel.png", bbox_inches="tight")

    plt.figure(4)
    plt.title("Theta error")
    plt.xlabel("t")
    plt.ylabel("Theta")
    plt.plot(t, theta, label="Theta")
    plt.plot(t, theta_traj, label="Theta Trajectory")
    plt.plot(t, theta_des * 180 / np.pi, label="Theta Desired")
    plt.legend()
    figure = plt.gcf()
    figure.set_size_inches(32, 18)
    plt.savefig("plots/theta.png", bbox_inches="tight")

    plt.figure(5)
    plt.title("Boost")
    plt.xlabel("t")
    plt.ylabel("Boost")
    plt.plot(t, boost, label="Actual Boost Input")
    plt.plot(t, boost_traj, label="Optimal Trajectory")
    plt.plot(t, boost_des, label="Desired Boost Input")
    plt.legend()
    figure = plt.gcf()
    figure.set_size_inches(32, 18)
    plt.savefig("plots/boost.png", bbox_inches="tight")

    plt.figure(6)
    plt.title("Geometric Control")
    plt.xlabel("y acceleration")
    plt.ylabel("z acceleration")
    plt.plot(a_traj_y, a_traj_z, label="Optimal Trajectory Acceleration")
    plt.plot(a_lat_y, a_lat_z, label="Lateral Error k_p = 1")
    plt.plot(a_des_y, a_des_z, label="Desired acceleration (optimal + lateral error)")
    plt.legend()
    figure = plt.gcf()
    figure.set_size_inches(32, 18)
    plt.savefig("plots/accel.png", bbox_inches="tight")
