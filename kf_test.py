import numpy as np
import matplotlib.pyplot as plt

C = 111319.44444444444
def velhead_to_latlong_old(lat, lon, velocity, heading, dt):
	# Earth's radius in meters
	R_earth = 6371000

	# Convert heading to radians
	heading_rad = np.deg2rad(heading)

	# Calculate the distance covered in meters
	distance = velocity * dt

	# Calculate change in latitude in degrees
	delta_lat = distance * np.cos(heading_rad) / R_earth
	delta_lat = np.rad2deg(delta_lat)

	# Calculate change in longitude in degrees
	# Note: Longitude change depends on latitude
	delta_lon = distance * np.sin(heading_rad) / (R_earth * np.cos(np.deg2rad(lat)))
	delta_lon = np.rad2deg(delta_lon)

	# Calculate the new position
	new_lat = lat + delta_lat
	new_lon = lon + delta_lon

	return new_lat, new_lon

def velhead_to_latlong(lat, lon, velocity, heading, dt):
	# Convert heading to radians
	heading_rad = np.deg2rad(-(heading-90)) # also compensated
	dist = velocity * dt
	dx = dist * np.cos(heading_rad)
	dy = dist * np.sin(heading_rad)

	new_lat = lat + dy / C
	new_lon = lon + dx / (C * np.cos(np.deg2rad(lat)))

	# # Calculate the distance covered in meters
	# distance = velocity * dt

	# # Calculate change in latitude in degrees
	# delta_lat = distance * np.cos(heading_rad) / R_earth
	# delta_lat = np.rad2deg(delta_lat)

	# # Calculate change in longitude in degrees
	# # Note: Longitude change depends on latitude
	# delta_lon = distance * np.sin(heading_rad) / (R_earth * np.cos(np.deg2rad(lat)))
	# delta_lon = np.rad2deg(delta_lon)

	# # Calculate the new position
	# new_lat = lat + delta_lat
	# new_lon = lon + delta_lon

	return new_lat, new_lon

class GPSKalmanFilterOld:
	def __init__(self):
		self.R_earth = 6371000  # Earth radius in meters
		self.x = np.zeros(4)  # State vector: [lat, lon, speed, heading]
		self.P = np.eye(4) * 500  # Initial uncertainty large 4x4
		self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Measurement of lat and lon 2x4
		self.R = np.array([[0.000001, 0], [0, 0.000001]])  # Measurement noise
		self.Q = np.eye(4) * 0.1  # Process noise

	def predict(self, dt):
		# Assume speed and heading remain constant over dt
		heading_rad = np.deg2rad(self.x[3])
		delta_lat = self.x[2] * dt * np.cos(heading_rad) / self.R_earth
		delta_lon = self.x[2] * dt * np.sin(heading_rad) / self.R_earth
		self.F = np.array([
			[1, 0, dt * np.cos(heading_rad) / self.R_earth, 0],
			[0, 1, dt * np.sin(heading_rad) / self.R_earth, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
		])
		self.x[0] += delta_lat
		self.x[1] += delta_lon
		self.P = self.F @ self.P @ self.F.T + self.Q

	def update(self, z, dt):
		y = z - self.H @ self.x[:2]  # Innovation (lat, lon)
		S = self.H @ self.P[:2, :2] @ self.H.T + self.R  # Innovation covariance
		K = self.P[:2, :2] @ self.H.T @ np.linalg.inv(S)  # Kalman Gain
		self.x[:2] += K @ y  # Update lat and lon
		self.P[:2, :2] -= K @ self.H @ self.P[:2, :2]

		# Recalculate speed and heading based on updated lat and lon
		if dt > 0:
			dlat = self.x[0] - self.last_lat
			dlon = self.x[1] - self.last_lon
			self.x[2] = np.sqrt(dlat**2 + dlon**2) / dt * self.R_earth  # Speed in m/s
			self.x[3] = np.rad2deg(np.arctan2(dlon, dlat))  # Heading in degrees

		self.last_lat = self.x[0]
		self.last_lon = self.x[1]

	def process(self, measurements, dt):
		self.predict(dt)
		self.update(measurements, dt)
		return self.x

class GPSKalmanFilter:
	def __init__(self):
		self.R_earth = 6371000  # Earth radius in meters
		self.x = np.zeros(4)  # State vector: [lat, lon, speed, heading]
		self.P = np.eye(4) * 500  # Initial uncertainty
		self.H = np.eye(2, 4)  # Measurement matrix (only lat, lon measured)
		#self.R = np.diag([0.000001, 0.000001])  # Measurement noise
		self.R = np.eye(2) * 0.1  # Measurement noise
		#self.R = np.eye(4) * 0.000001  # Measurement noise
		self.Q = np.eye(4) * 0.001  # Process noise
		self.last_measurement = np.zeros(2)  # Last measurement for speed calculation

	def predict(self, dt):
		if dt > 0:  # Avoid division by zero
			# Assume speed and heading remain constant over dt
			heading_rad = np.deg2rad(self.x[3])
			delta_lat = self.x[2] * dt * np.cos(heading_rad) / self.R_earth
			delta_lon = self.x[2] * dt * np.sin(heading_rad) / self.R_earth
			self.F = np.array([
				[1, 0, dt * np.cos(heading_rad) / self.R_earth, 0],
				[0, 1, dt * np.sin(heading_rad) / self.R_earth, 0],
				[0, 0, 1, 0],
				[0, 0, 0, 1]
			])
			# Update lat and lon based on speed and heading
			self.x[0] += delta_lat
			self.x[1] += delta_lon

		# Prediction step for covariance
		self.P = self.F @ self.P @ self.F.T + self.Q

	def update(self, z, dt):
		# Update using measurements
		y = z - self.H @ self.x  # Innovation
		S = self.H @ self.P @ self.H.T + self.R  # Innovation covariance
		K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman Gain
		self.x += K @ y  # Update state
		self.P = (np.eye(4) - K @ self.H) @ self.P

		# Recalculate speed and heading based on updated lat and lon
		if np.any(self.last_measurement):
			dlat = self.x[0] - self.last_measurement[0]
			dlon = self.x[1] - self.last_measurement[1]
			dy = dlat * C
			dx = dlon * C * np.cos(np.deg2rad(self.x[0]))

			self.x[2] = np.sqrt(dx**2 + dy**2) / dt  # Speed in m/s
			self.x[3] = -np.rad2deg(np.arctan2(dy, dx))+90  # Heading in degrees
			if self.x[3] < 0:
				self.x[3] += 360.0

		self.last_measurement = z.copy()

	def process(self, measurements, dt):
		self.predict(dt)
		self.update(measurements, dt)
		return self.x



for h in [0, 90, 180, 270]:
	lat, lon = velhead_to_latlong(50.0, 10.0, 10, h, 1)
	print(f"Lat: {lat}, Lon: {lon}")

# Test the Kalman filter
kf = GPSKalmanFilter()
ts = []
lats = []
lons = []
est_lats = []
est_lons = []
est_vel = []
est_heading = []
act_vel = []
act_head = []
N= 400

prev_lat, prev_lon = 57.8, 11.2
vel = 5.23
#head = 123.4
# TODO: Add noise!!
for t in range(N):
	ts.append(t)
	head = t % 360
	# if t < 100:
	# 	vel = 2
	# elif t < 200:
	# 	lat = 57.8 - t*0.03
	# 	lon = 11.2 + t*0.03
	# elif t < 300:
	# 	lat = 57.8 - t*0.03
	# 	lon = 11.2 - t*0.03
	# elif t < 400:
	# 	lat = 57.8 + t*0.03
	# 	lon = 11.2 - t*0.03

	lat, lon = velhead_to_latlong(prev_lat, prev_lon, vel, head, 1)
	lat += np.random.normal(0, 0.0001)
	lon += np.random.normal(0, 0.0001)
	prev_lon = lon
	prev_lat = lat
		
	lats.append(lat)
	lons.append(lon)
	act_vel.append(vel)
	act_head.append(head)
	measurements = np.array([lat, lon])
	est_state = kf.process(measurements, 1)
	est_lats.append(est_state[0])
	est_lons.append(est_state[1])
	est_vel.append(est_state[2])
	#heading_deg = np.rad2deg(-(est_state[3]-90))
	est_heading.append(est_state[3])


# Create a 2x2 grid of plots
fig, axs = plt.subplots(3, 2)

# Plot GPS latitude
axs[0, 0].plot(ts, lats, label="RAW LAT")
axs[0, 0].plot(ts, est_lats, label="EST LAT")
axs[0, 0].set_title('GPS Latitude')
axs[0, 0].legend()

# Plot GPS longitude
axs[0, 1].plot(ts, lons, label="RAW LONG")
axs[0, 1].plot(ts, est_lons, label="EST LONG")
axs[0, 1].set_title('GPS Longitude')
axs[0, 1].legend()

# Plot filtered latitude estimates
axs[1, 0].plot(ts, est_heading, label="Estimated")
axs[1, 0].plot(ts, act_head, label="Actual")
axs[1, 0].set_title('Est heading deg')
axs[1, 0].legend()
# axs[1, 0].set_ylim(0.0, 360.0)

# Plot filtered longitude estimates
axs[1, 1].plot(ts, est_vel, label="Estimated")
axs[1, 1].plot(ts, act_vel, label="Actual")
axs[1, 1].set_title('Est vel m/s')
axs[1, 1].legend()
# axs[1, 1].set_xlim(0, 3)

axs[2, 1].scatter(lons, lats)
axs[2, 1].set_title('Gps path')

# Adjust spacing between subplots
plt.tight_layout()

# Display the plot
plt.show()

# plt.plot(lons, lats, label="GPS lat")
# plt.scatter(lons, lats, label="GPS lon")
# plt.scatter(est_lons, est_lats, label="Filtered estimates")
# plt.scatter(est_lons, est_lats, label="Filtered estimates")
# plt.legend()
# plt.show()