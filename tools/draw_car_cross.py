import matplotlib.pyplot as plt
import numpy as np
import subprocess
import sys
import glob
import matplotlib.image as mpimg
import matplotlib.animation as animation

COTHERS = ["#E69F00", "#56B4E9", "#009E73", "#D55E00", "#CC79A7", "#8C79A7", "#AC79A7"]
CPOMDP = '#0072B2'
colors = COTHERS + [CPOMDP]

car_pos = []
car_vel = []
ped_pos = []
ped_id = []
ped_goal = []
obstacles = []
laser_range=10
car_colli=[]
car_acc=[]
rounds=[]
colli=[]
dis_trav=[]

def loadData(filename):
	#car_pos = []
	#car_vel = []
	#ped_pos = []
	#ped_id = []
	#ped_goal = []
	ped_pos_one_time = []
	ped_id_one_time = []
	ped_goal_one_time = []
	ped_num = -1
	#obstacles = []
	#laser_range=10

	file_handler = open(filename,'r')
	content = file_handler.read().splitlines()
	time_step = 0
	num_ped_visited = 0
	round_end=False
	for line in content:
		line_split = line.split(' ')
		if "dist_trav" in line:
			car_pos.append([float(line_split[7][1:-1]), float(line_split[8][0:-1])])
			car_vel.append(float(line_split[12]))
			if round_end==True:
				dis_trav.append(float(line_split[10]))
				round_end=False

		elif "pedestrians" in line:
			ped_pos_one_time = []
			ped_id_one_time = []
			ped_goal_one_time = []
			num_ped_visited = 0
			ped_num = int(line_split[0])
		elif "dist2car" in line:
			ped_pos_one_time.append([float(line_split[17][1:-1]), float(line_split[18][0:-1])])
			ped_id_one_time.append(int(line_split[15]))
			ped_goal_one_time.append(int(line_split[22]))
			num_ped_visited = num_ped_visited + 1
		elif "obstacle" in line:
			obstacle = []
			for i in range(1, len(line_split),2):
				obstacle.append([float(line_split[i]), float(line_split[i+1])])
			obstacles.append(obstacle)
		elif "LASER_RANGE" in line:
			laser_range = float(line_split[1])
		elif "- Reward =" in line:
			reward=float(line_split[3])
			if (reward < -200):
				car_colli.append(1)
			else:
				car_colli.append(0)
		elif "collision=1:" in line:
			colli.append(1)
		elif "- Action = " in line:
			if (float(line_split[3])==0):
				car_acc.append(0)
			elif (float(line_split[3])==1):
				car_acc.append(1)
			elif (float(line_split[3])==2):
				car_acc.append(-1)
		elif "final_state" in line:
			rounds.append(1)
			round_end=True
			#num_rounds = num_rounds + 1;
		else:
			pass;
		if (num_ped_visited == ped_num):
			ped_num = -1
			ped_pos.append(ped_pos_one_time)
			ped_id.append(ped_id_one_time)
			ped_goal.append(ped_goal_one_time)

	return car_pos, car_vel, ped_pos, ped_id, ped_goal, obstacles, laser_range, dis_trav


#x = np.linspace(0, 2 * np.pi, 120)
#y = np.linspace(0, 2 * np.pi, 100).reshape(-1, 1)

#def f(x, y):
#    return np.sin(x) + np.cos(y)

#im = plt.imshow(f(x, y), animated=True)

#def updatefig(*args):
#	im=mpimg.imread('step'+'.png')
#	return im

def drawFigures(car_pos, car_vel, ped_pos, ped_id, ped_goal, obstacles, laser_range):
	time_length = len(car_vel)
	#print ped_goal
	#plt.savefig('step', bbox_inches='tight')
	#ani = animation.FuncAnimation(fig, updatefig, interval=1000, blit=True)
	#plt.show()
	for time_step in range(0, time_length):
		# draw vehicle
		car_color='green'
		if time_step<len(car_colli):
			coll=car_colli[time_step]
			#print coll
			if coll==1 and time_step < time_length-1:
				car_color='red'

		rect1=plt.Rectangle((car_pos[time_step][0],car_pos[time_step][1]-0.6),2.2, 1.2, 
			fill=True, color=car_color)
		circle1=plt.Circle((car_pos[time_step][0],car_pos[time_step][1]),laser_range, 
			fill=True, alpha=0.05, color='teal',ls='dashed')
		circle2=plt.Circle((car_pos[time_step][0],car_pos[time_step][1]),0.1, 
			fill=True, color='grey')
		ax = plt.gca()
		ax.add_artist(rect1)
		ax.add_artist(circle1)
		ax.add_artist(circle2)
		#print ped_goal
		# draw pedestrians
		# for ped in ped_pos[time_step]:
		# 	circle1=plt.Circle((ped[0],ped[1]),0.3, fill=True, color='k')
		# 	ax = plt.gca()
		# 	ax.add_artist(circle1)
		for i in range(0,len(ped_pos[time_step])):
			circle1=plt.Circle((ped_pos[time_step][i][0],ped_pos[time_step][i][1]),0.3, fill=True, color=colors[ped_goal[time_step][i]])
			ax = plt.gca()
			ax.add_artist(circle1)

		# draw obstacles
		for obstacle in obstacles:
			obstacle.append(obstacle[0]) #repeat the first point to create a 'closed loop'
			xs, ys = zip(*obstacle) #create lists of x and y values
			plt.plot(xs,ys)

		plt.ylim([0,15])
		plt.xlim([3,23])
		#im = plt.imshow(frame, cmap=plt.cm.gray)
		#fig.canvas.draw()
		#fig.clf()
		#plt.savefig('step', bbox_inches='tight')
		#im=mpimg.imread('step'+'.png')
		plt.savefig('step%d'%(time_step), bbox_inches='tight')
		plt.close()
		#img=mpimg.imread('step%d'%(time_step)+'.png')
		#im = plt.imshow(img, animated=True)
	#plt.close()

fig = plt.figure()
ax = plt.gca()
plt.ylim([0,15])
plt.xlim([3,23])
#plt.ylim([35,50])
#plt.xlim([30, 50])
#time_text = plt.axes().text(0.02, 0.95, '', transform=plt.axes().transAxes)

def init():
	ax.clear()
	return []

def drawFigures1(time_step):
	time_length = len(car_vel)
	#print ped_goal
	#plt.savefig('step', bbox_inches='tight')
	#ani = animation.FuncAnimation(fig, updatefig, interval=1000, blit=True)
	#plt.show()
	ax.clear()

	if time_step < time_length:
		# draw vehicle
		#print coll
		car_color='green'
		bar_color='orange'
		if time_step<len(car_colli):
			coll=car_colli[time_step]
			if coll==1 and time_step < time_length-1:
				car_color='red'

		if time_step>=0 and time_step<time_length and time_step< len(car_vel)-1 and time_step<len(car_acc):
			if car_acc[time_step]==-1 and car_vel[time_step]==car_vel[time_step+1] and car_vel[time_step]!=0:
				bar_color='maroon'

		patches = []
		rect1=plt.Rectangle((car_pos[time_step][0],car_pos[time_step][1]-0.6),2.2, 1.2, 
			fill=True, color=car_color)
		#rect1=plt.Rectangle((car_pos[time_step][0]-0.6,car_pos[time_step][1]-2.2),1.2, 2.2, 
		#	fill=True, color=car_color)
		circle1=plt.Circle((car_pos[time_step][0],car_pos[time_step][1]),laser_range,
		 fill=True, alpha=0.2, color='teal',ls='dashed')
		circle2=plt.Circle((car_pos[time_step][0],car_pos[time_step][1]),0.1, 
			fill=True, color='grey')
		patches.append(ax.add_patch(rect1))
		patches.append(ax.add_patch(circle1))
		patches.append(ax.add_patch(circle2))
		for i in range(0,len(ped_pos[time_step])):
			circle1=plt.Circle((ped_pos[time_step][i][0],ped_pos[time_step][i][1]),0.3, fill=True, color=colors[ped_goal[time_step][i]])
			patches.append(ax.add_patch(circle1))
		speed_rect=plt.Rectangle((5.2,1.1),1, car_vel[time_step], fill=True, alpha=0.5, color='red')
		patches.append(ax.add_patch(speed_rect))
		acc_rect=plt.Rectangle((4,1.1),1, 0+car_acc[min(time_step,len(car_acc)-1)], 
			fill=True, alpha=0.5, color=bar_color)
		patches.append(ax.add_patch(acc_rect))
		#time_text.set_text('Car vel='+str(float(car_vel[time_step][0])))
		#time_text.set_text('Car vel=')
	return patches

def printStatistics():
	print "colli_len=%d, acc_len=%d, vel_len=%d, num_rounds=%d" % (len(colli),len(car_acc),len(car_vel), len(rounds))

	colli_count = len(colli) #np.count_nonzero(car_colli==1)
	acc_count = (car_acc).count(1) #np.count_nonzero(car_acc==1)
	dec_count = (car_acc).count(-1) #np.count_nonzero(car_acc==-1)
	num_steps = len(car_acc)

	print "dist_len=%f, total_dist=%f" % (len(dis_trav),sum(dis_trav))

	ave_dist= sum(dis_trav) / float(len(dis_trav))

	smoothness= float(sum(dis_trav))/float(acc_count+dec_count)
	colliRate = float(colli_count)/float(sum(dis_trav))
	print "colli_count=%d, acc_count=%d, dec_count=%d, num_rounds=%d" % (colli_count,acc_count,dec_count,len(rounds))
	print "Collision rate = %.5f" % colliRate

	print "Smoothness = %.5f" % (smoothness)
	print "Average distance = %.5f" % ave_dist


Writer = animation.writers['avconv'] #ffmpeg
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

if __name__ == '__main__' :

	file_name = 'trial'
	if len(sys.argv) > 1:
		file_name = sys.argv[1]

	print 'Loading data...'
	car_pos, car_vel, ped_pos, ped_id, ped_goal, obstacles, laser_range, dis_trav = loadData(file_name)

	#print car_vel
	printStatistics()

	show_anim=True
	if show_anim==True:
		#drawFigures(car_pos, car_vel, ped_pos, ped_id, ped_goal, obstacles, laser_range)
		print 'Generating animation...'
		anim = animation.FuncAnimation(fig, drawFigures1, init_func=init, frames=len(car_vel), 
			interval=30, blit=True, repeat=False)
		plt.show()

		print 'Saving '+'output.avi with '+str(len(car_vel))+' frames...'
		anim.save('output.avi', writer=writer)
		#subprocess.call(['ffmpeg', '-i', 'step%d.png', 'output.avi'])
		print 'Converting to '+file_name+'.avi'+'...'
		#subprocess.call(['ffmpeg', '-i', 'output.avi', '-vf', 'setpts=8.333*PTS', file_name+'.avi'])
		subprocess.call(['ffmpeg', '-i', 'output.avi', '-vf', 'setpts=5.0*PTS', file_name+'.avi'])
		subprocess.call(['rm', 'output.avi'])
		#for fl in glob.glob("step*.png"):
		#	subprocess.call(['rm', fl])
