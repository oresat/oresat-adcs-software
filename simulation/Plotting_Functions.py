import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
from Basilisk.utilities import macros

def getLineColor(idx, maxNum, offset = 0, colorlib = "viridis"):
    """pick a nicer color pattern to plot 3 vector components"""
    values = list(range(0, maxNum+offset))
    colorMap = plt.get_cmap(colorlib) # select color from https://matplotlib.org/stable/users/explain/colors/colormaps.html gist_ncar, nipy_spectral, inferno, viridis
    cNorm = colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=colorMap)
    return scalarMap.to_rgba(values[idx + 1])

def plot_rw_speeds(timeData, dataOmegaRW, numRW, config, errorArray=None):
    """Plot the RW spin rates with optional error curve on right axis."""
    fig, ax1 = plt.subplots(figsize=(8,4))

    # --- Reaction wheel speeds ---
    for idx in range(numRW):
        ax1.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=getLineColor(idx, numRW, 2),
                 label=r'$\Omega_{' + str(idx) + '}$')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('RW Speed (RPM)')
    ax1.grid(True)

    # --- Optional error line ---
    if errorArray is not None:
        ax2 = ax1.twinx()  # create second y-axis
        ax2.plot(timeData, errorArray, 'r--', label='Error')
        ax2.set_ylabel('Error (deg)')
        # Add legend for error separately
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        plt.title("Reaction Wheel Speeds and Orientation Error")
    else:
        plt.legend()
        plt.title("Reaction Wheel Speeds")
    
    if config["save_plots"] == True:
        pdf_path = config["plot_basepath"] / "RW_graph.pdf"
        plt.savefig(pdf_path, dpi=300)
        print(f"Plot saved as {pdf_path}")
    
    plt.show(block=False)
    
def plot_magfield(times, TAMvalues, orbital_period, config, time_axis = "orbits"): 
    """Plot the measured three-axis magnetic field values"""
    if time_axis == "seconds":
        time_array = times
        label_name = 'Time [s]'
    elif time_axis == "orbits":
        time_array = times/orbital_period # convert times to orbital periods
        label_name = 'Orbits'
    else:
        print("Unkown time axis argument, defaulting to 'orbit'")
        time_array = times/orbital_period # convert times to orbital periods
        label_name = 'Orbits'
        
    fig, ax1 = plt.subplots(figsize=(8,4))
    axis_labels = ['B$_x$', 'B$_y$', 'B$_z$']
    for idx in range(3):
        ax1.plot(time_array, TAMvalues[:, idx],
                 color=getLineColor(idx, 3, 2, "inferno"),
                 label=axis_labels[idx])
    ax1.set_xlabel(label_name)
    ax1.set_ylabel('Magnetic Field [T]')
    ax1.grid(True)
    
    plt.legend()
    plt.title("3-Axis Magnetometer Measurements")
    
    if config["save_plots"] == True:
        pdf_path = config["plot_basepath"] / "TAM.pdf"
        plt.savefig(pdf_path, dpi=300)
        print(f"Plot saved as {pdf_path}")
    
    plt.show(block=False)
    
def plot_imu(times, imuValues, orbital_period, config, time_axis = "orbits", errorArray=None): 
    """Plot the measured IMU values"""
    if time_axis == "seconds":
        time_array = times
        label_name = 'Time [s]'
    elif time_axis == "orbits":
        time_array = times/orbital_period # convert times to orbital periods
        label_name = 'Orbits'
    else:
        print("Unkown time axis argument, defaulting to 'orbit'")
        time_array = times/orbital_period # convert times to orbital periods
        label_name = 'Orbits'
        
    fig, ax1 = plt.subplots(figsize=(8,4))
    axis_labels = [r'$\omega_x$', r'$\omega_y$', r'$\omega_z$']
    for idx in range(3):
        ax1.plot(time_array, imuValues[:, idx],
                 color=getLineColor(idx, 3, 3, "nipy_spectral"),
                 label=axis_labels[idx])
    ax1.set_xlabel(label_name)
    ax1.set_ylabel('Angular Velocity [rad/s]')
    ax1.grid(True)
    
    # --- Optional error line ---
    if errorArray is not None:
        ax2 = ax1.twinx()  # create second y-axis
        ax2.plot(time_array, errorArray, 'r--', label='Error')
        ax2.set_ylabel('Error (deg)')
        # Add legend for error separately
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        plt.title("3-Axis Satellite Body Rates and Orientation Error")
    else:
        plt.legend()
        plt.title("3-Axis Satellite Body Rates")
        
    if config["save_plots"] == True:
        pdf_path = config["plot_basepath"] / "IMU.pdf"
        plt.savefig(pdf_path, dpi=300)
        print(f"Plot saved as {pdf_path}")
    
    plt.show(block=False)