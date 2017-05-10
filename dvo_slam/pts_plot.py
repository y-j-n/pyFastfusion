import matplotlib.pyplot as plt
import time
import sys

if len(sys.argv) != 2:
    print 'Usage: %s <path_traj>'
    sys.exit(1)

path_traj = sys.argv[1]

fig = plt.figure()
ax = fig.add_subplot(111)


#filename = '../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt'
#filename = '../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/assoc_opt_traj_final.txt'
filename = path_traj

list_time = []
list_tx = []
list_ty = []
list_tz = []
list_qx = []
list_qy = []
list_qz = []
list_qw = []
with open(filename) as f:
    for l in f:
        if len(l.split()) != 8:
            continue

        time, tx, ty, tz, qx, qy, qz, qw = l.split()
        if time.startswith('#'):
            continue
        else:
            list_time.append(time)
            list_tx.append(tx)
            list_ty.append(ty)
            list_tz.append(tz)
            list_qx.append(qx)
            list_qy.append(qy)
            list_qz.append(qz)
            list_qw.append(qw)

#x_points = xrange(0,9)
#y_points = xrange(0,9)
#p = ax.plot(x_points, y_points, 'b')

ax.plot(list_time, list_tx, 'r')
ax.plot(list_time, list_ty, 'g')
ax.plot(list_time, list_tz, 'b')


ax.set_xlabel('x-points')
ax.set_ylabel('y-points')
ax.set_title(f)
#fig.show()
plt.show()

#time.sleep(120)

