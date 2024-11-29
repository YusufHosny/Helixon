import numpy as np
from scipy.spatial.transform import Rotation
from hlxon_hdf5io import *


"""
Spiral Synthetic Sensor Data Generator

Designed so that you can provide "fake" positions and orientations, or real ones from SLAM
and the script will automatically generate BNO055 and BMP390 style measurements

What it does:
    inputs: fs, positions, orientations
    (position)' is velocity
    (velocity)' is acceleration
    (orientation)' is angular velocity
    using the orientation the magnetometer data will be calculated
    and using the orietation the generated data will be transformed into the sensors coordinate space

output columns:
    - timestamp:    (double seconds) time in seconds
    - gt_posx:      (double meters) ground truth x position
    - gt_posy:      (double meters) ground truth y position
    - gt_posz:      (double meters) ground truth z position
    - gt_roll:      (double degrees) ground truth roll
    - gt_pitch:     (double degrees) ground truth pitch
    - gt_yaw:       (double degrees) ground truth yaw
    - accelx:       (double m/s^2) accelerometer x (device coords)
    - accely:       (double m/s^2) accelerometer y (device coords)
    - accelz:       (double m/s^2) accelerometer z (device coords)
    - gyrox:        (double rad/s) gyroscope x (device coords)
    - gyroy:        (double rad/s) gyroscope y (device coords)
    - gyroz:        (double rad/s) gyroscope z (device coords)
    - magnx:        (double uT) magnetometer x (device coords)
    - magny:        (double uT) magnetometer y (device coords)
    - magnz:        (double uT) magnetometer z (device coords)
    - tempbno:      (int C) bno sensor temperature
    - tempbmp:      (double C) bmp sensor temperature
    - pressure:     (double Pa) pressure

    - rssiCnt:      (int) Count of found networks
    - bssids:       (string) Mac Addresses of Networks
    - rssis:        (int dB) received signal strength intensity for found Networks

"""

# define time and samples
total_time          = 45 # sec
fs                  = 20 # Hz
number_samples_gt   = total_time*fs # ground truth sample count (positions and orientations)
N                   = number_samples_gt # for convenience
X, Y, Z             = 0, 1, 2 # for convenience

# define spiral constants
spiral_r        = 8 # m
spiral_pitch    = 4 # m
theta           = np.arctan(spiral_pitch / (2 * np.pi * spiral_r)) # radians

# define ground truth positions and orientations
t           = np.linspace(0,total_time, N) # (N, ) shape
ts          = t.reshape((-1, 1)) # timestamps but (N, 1) shape
gt_pos      = np.zeros((N, 3))
gt_orien    = np.zeros((N, 3))
K           = np.zeros_like(t)  # K is position along spiral in radians

# max vertical velocity
vv0 = 0.3
# current position example increases slowly for the first 2 seconds,
# then at a constant rate along the spiral
for i in range(1, N): 
    # define z axis value and thru spiral eq get x and y
    if t[i] < 2: # first 2 sec
        gt_pos[i, Z] = gt_pos[i-1, Z] + (t[i]-t[i-1]) * vv0*np.exp(4*(t[i]/2 - 1)) # z = v0(e^4(t/2-1)) to exponentially inc speed until ~ v0 m/s at 2sec
    else:
        gt_pos[i, Z] = gt_pos[i-1, Z] + (t[i]-t[i-1]) * vv0
    K[i] = 2*np.pi* (gt_pos[i, Z]/spiral_pitch)
    gt_pos[i, X] = spiral_r * (np.cos(K[i]) - 1)
    gt_pos[i, Y] = spiral_r * np.sin(K[i])

# orientation is tangent to the spiral top down projection
roll    = np.ones_like(ts) * theta * (180/np.pi)
roll: np.ndarray
pitch   = np.zeros_like(ts)
yaw     = (K * (180/np.pi)).reshape(-1, 1) # to degrees 
rpy     = np.concatenate((roll, pitch, yaw), axis=1)

# calculate velocities in global coordinate space using positions
vraw = np.zeros((N, 3))
for i in range(1, N):
    dx = (gt_pos[i]-gt_pos[i-1])
    dt = (t[i]-t[i-1])
    vraw[i] += dx/dt

# calculate accelerations in global coord
araw = np.zeros((N, 3))
for i in range(1, N):
    dv = (vraw[i]-vraw[i-1])
    dt = (t[i]-t[i-1])
    araw[i] += dv/dt
araw[:, Z] += 9.81 # add gravity bc real sensor has z axis flipped

# transform accelerations to device/sensor coord space
accel = np.zeros_like(gt_pos)
for i in range(N):
    rotToDevice = Rotation.from_euler('xyz', rpy[i], degrees=True)
    accel[i] += rotToDevice.apply(araw[i])

# define gyro in device coords
gyro =  np.zeros_like(gt_pos)
for i in range(1, N):
    dtheta = (rpy[i]-rpy[i-1])
    dt = (t[i]-t[i-1])
    gyro[i] = dtheta/dt

# define magn in global coord space
mx0 = mz0 = np.zeros_like(ts)
my0 = np.ones_like(ts)*40. # ~40 microtesla towards magnetic north
magnraw = np.concatenate((mx0, my0, mz0), axis=1)

# transform magnetometer data to device/sensor coord space
magn = np.zeros_like(gt_pos)
for i in range(N):
    rotToDevice = Rotation.from_euler('xyz', rpy[i], degrees=True)
    magn[i] = rotToDevice.apply([magnraw[i]])

# tempbno
tbno = np.ones_like(ts) * 25

# tempbmp
tbmp = np.ones_like(ts) * 25.77

# pressure
p0 = 101325 # Pa
alpha = 1.16e-4
ps = p0 * np.exp(-alpha*gt_pos[:, 2])

# add noise
percent_gaussian = .2 # amount of noise in %
mean_acc = 0.01177*percent_gaussian
std_acc = (0.00314)*percent_gaussian
mean_gyro = 0.001745*percent_gaussian
std_gyro = (0.003491/3)*percent_gaussian
mean_magnet = 0.3*percent_gaussian
std_magnet = (0.2/3)*percent_gaussian

# accel noise
for i in range(N):
    accel[i, X] += np.random.normal(mean_acc, std_acc)
    accel[i, Y] += np.random.normal(mean_acc, std_acc)
    accel[i, Z] += np.random.normal(mean_acc, std_acc)

# gyro noise
for i in range(N):
    gyro[i, X] += np.random.normal(mean_gyro, std_gyro)
    gyro[i, Y] += np.random.normal(mean_gyro, std_gyro)
    gyro[i, Z] += np.random.normal(mean_gyro, std_gyro)

# magno noise
for i in range(N):
    magn[i, X] += np.random.normal(mean_magnet, std_magnet)
    magn[i, Y] += np.random.normal(mean_magnet, std_magnet)
    magn[i, Z] += np.random.normal(mean_magnet, std_magnet)


data = np.concatenate( ((ts*1e6).astype(np.int64),
                accel[:, X].reshape(-1, 1),
                accel[:, Y].reshape(-1, 1),
                accel[:, Z].reshape(-1, 1),
                gyro[:, X].reshape(-1, 1),
                gyro[:, Y].reshape(-1, 1),
                gyro[:, Z].reshape(-1, 1),
                magn[:, X].reshape(-1, 1),
                magn[:, Y].reshape(-1, 1),
                magn[:, Z].reshape(-1, 1),
                roll, pitch, yaw, tbno, tbmp, ps.reshape(-1, 1)),
                    axis=1)

gtdata = np.concatenate( ((ts*1e6).astype(np.int64), \
                gt_pos[:, 0].reshape(-1, 1), 
                gt_pos[:, 1].reshape(-1, 1), 
                gt_pos[:, 2].reshape(-1, 1), 
                roll, pitch, yaw), 
                axis = 1)

# generate wifi data


# wifi data
rssi_measured_power = -40 # typical 1-meter-RSSI for WiFi routers is between -40 and -60 dBm
rssi_path_loss_exponent = 2 # typical indoor path loss exponent is between 4 and 6

X, Y, Z, MAC = 0, 1, 2, 3 # for convenience


# generates array of routers, each with x y z coordinates with corresponding MAC address
wifi_routers = [[-1.9625912635830327, -0.042755400999599624, 0.236777918073553, '9e:2a:ea:04:ac:93'],
                [-2.86743255185108, 10.230164478313519, 2.373233293431522, 'e2:8c:54:f4:47:9f'],
                [-2.6049383725563793, 10.992929854309349, 4.82598431890248, 'cc:41:11:fa:b6:f4'],
                [11.27152174407813, 10.361298068605526, 6.438432335099835, '2d:7a:ed:50:7f:a9'],
                [-0.9229859736350821, -0.8181204089944636, 8.208368772600491, '45:84:bd:7b:45:8f'],
                [10.304070899801548, 11.943232998360061, 10.297419378938036, 'cc:c5:88:bb:70:f8'],
                [10.548371853778425, -2.882304115246261, 12.740700625938995, 'de:be:c6:8f:32:26'],
                [12.843273134642441, 11.65724374052579, 14.151707456823424, 'c1:ef:d9:17:b4:54'],
                [12.615689578047608, 10.765756456022414, 16.13282069036388067, '1f:3c:d0:f0:ec:04'],
                [-1.3179787074473732, -1.2796112394304908, 18.53774228686071, 'ec:2a:13:a2:94:dc']]

def get_RSSI_MAC(position):
    wifi_data = [len(wifi_routers)]

    for router in wifi_routers:
        distance = np.sqrt(
                (router[X] - position[X]) ** 2 +
                (router[Y] - position[Y]) ** 2 +
                (router[Z] - position[Z]) ** 2
            )

        mac = router[MAC]
        wifi_data.append(mac)
        
        rssi = rssi_measured_power - 10 * rssi_path_loss_exponent * np.log10(distance)
        wifi_data.append(rssi)

    return wifi_data


wifidata = []

for i in range(N):
    
    if (i % 10 == 0): # Every 10 measurements
        wifi_row = [gtdata[i][0]]
        wifi_row.extend(get_RSSI_MAC(gt_pos[i]))
        wifidata.append(wifi_row)

wifidata = np.array(wifidata, dtype = object)


# store everything in an HDF5 file
storeAsHDF5('synthetic', data, gtdata, wifidata)