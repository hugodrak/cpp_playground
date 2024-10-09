import numpy as np
from pykalman import KalmanFilter
import matplotlib.pyplot as plt

# Time step
dt = 1.0  # assuming data is sampled every second

# State transition matrix (modeling constant velocity)
F = np.array([[1, 0, dt, 0],  # next latitude
              [0, 1, 0, dt],  # next longitude
              [0, 0, 1,  0],  # next velocity in latitude
              [0, 0, 0,  1]]) # next velocity in longitude

# Observation matrix (we can directly observe lat and long)
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])

# Initial state (latitude, longitude, velocity_lat, velocity_long)
initial_state = np.array([0, 0, 0, 0])

# Initial covariance matrix (large values mean high uncertainty)
initial_covariance = np.eye(4) * 1000


# Process noise (model uncertainties, e.g., unexpected accelerations)
process_noise = np.eye(4)
process_noise[0:2, 0:2] *= 0.02  # position noise
process_noise[2:4, 2:4] *= 0.5   # velocity noise

# Measurement noise (how noisy are the GPS readings)
measurement_noise = np.eye(2) * 2


kf = KalmanFilter(transition_matrices=F,
                  observation_matrices=H,
                  initial_state_mean=initial_state,
                  initial_state_covariance=initial_covariance,
                  transition_covariance=process_noise,
                  observation_covariance=measurement_noise)

# Simulate GPS data
true_lat = np.linspace(0, 1, 100)
true_long = np.linspace(0, 1, 100)
observed_lat = true_lat + np.random.normal(0, 0.1, 100)  # noisy observations
observed_long = true_long + np.random.normal(0, 0.1, 100)

observations = np.column_stack([observed_lat, observed_long])


state_means, state_covariances = kf.filter(observations)

# Extract velocities
velocities = state_means[:, 2:4]

# Compute speed (magnitude of velocity vector)
speeds = np.sqrt(velocities[:, 0]**2 + velocities[:, 1]**2)

# Compute heading using arctangent of the velocity components
headings = np.arctan2(velocities[:, 1], velocities[:, 0]) * 180 / np.pi


# Create a 2x2 grid of plots
fig, axs = plt.subplots(3, 2)

# Plot GPS latitude
axs[0, 0].plot(true_lat, label="RAW LAT")
axs[0, 0].plot(observed_lat, label="EST LAT")
axs[0, 0].set_title('GPS Latitude')
axs[0, 0].legend()

# Plot GPS longitude
axs[0, 1].plot(true_long, label="RAW LONG")
axs[0, 1].plot(observed_long, label="EST LONG")
axs[0, 1].set_title('GPS Longitude')
axs[0, 1].legend()

# Plot filtered latitude estimates
axs[1, 0].plot(headings, label="Estimated")
axs[1, 0].set_title('Est heading deg')
axs[1, 0].legend()
# axs[1, 0].set_ylim(0.0, 360.0)

# Plot filtered longitude estimates
axs[1, 1].plot(speeds, label="Estimated")
axs[1, 1].set_title('Est vel m/s')
axs[1, 1].legend()
# axs[1, 1].set_xlim(0, 3)

axs[2, 1].scatter(true_long, true_lat)
axs[2, 1].set_title('Gps path')

# Adjust spacing between subplots
plt.tight_layout()

# Display the plot
plt.show()
