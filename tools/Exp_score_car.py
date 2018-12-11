import subprocess
from random import *
import gc
import numpy
import sys
import os.path
import glob
import numpy
import sys
sys.path.append("./")
import scipy.stats as stats
from scipy.stats import binom

#from bayes_opt import BayesianOptimization


import subprocess, shlex
from threading import Timer

def run(cmd, timeout_sec, output):
  proc = subprocess.Popen(shlex.split(cmd), stdout=output, 
    stderr=output)
  kill_proc = lambda p: p.kill()
  timer = Timer(timeout_sec, kill_proc, [proc])
  try:
    timer.start()
    stdout,stderr = proc.communicate()
  finally:
    timer.cancel()

class CarDriveData:
	def __init__(self):
		self.finished_rounds=[]
		self.laser_range=10
		self.dis_trav=[]
		self.num_rounds=0
		self.total_reward=[]
		self.total_nondis_reward=[]
		self.success_count=0
		self.collision_count=0
		self.total_step=0
		self.total_search_depth=0
		self.default_count=0
		self.expansion_count=0
		self.policy_size=0
		self.expansion_time=0
		self.total_time=0
		self.init_lb=0
		self.init_ub=0
		self.final_lb=0
		self.final_ub=0
		self.acc_count=0;
		self.dec_count=[];
		self.num_finishedrounds=0;
		self.num_trial=[]
		self.expanded_nodes=[]
		self.tree_nodes=[]


	def ClearData(self):
		self.finished_rounds=[]
		self.laser_range=10
		self.dis_trav=[]
		self.num_rounds=0
		self.total_reward=[]
		self.total_nondis_reward=[]
		self.success_count=0
		self.collision_count=0
		self.total_step=0
		self.total_search_depth=0
		self.default_count=0
		self.expansion_count=0
		self.policy_size=0
		self.expansion_time=0
		self.total_time=0
		self.init_lb=0
		self.init_ub=0
		self.final_lb=0
		self.final_ub=0
		self.acc_count=0;
		self.dec_count=[];
		self.num_finishedrounds=0;
		self.num_trial=[]
		self.expanded_nodes=[]
		self.tree_nodes=[]
		gc.collect()

	def remove_redundant(self, line):
		line = line.replace("(", "")
		line = line.replace(")", "")
		line = line.replace(",", "")
                #print line
		return line

	def loadData(self,filename):
		file_handler = open(filename,'r')
		content = file_handler.read().splitlines()
		round_end=False
		values=[]
		succ=False
		coll=False
		round_dec_count=0

		for line in content:
			line_split = line.split(' ')
			if "Initial state: " in line:
				self.num_rounds+=1
			elif "Completed" in line:
				self.num_finishedrounds+=1
				self.finished_rounds.append(1)
			elif "final_state" in line:
				round_end=True
				self.dec_count.append(round_dec_count)
			elif "goal_reached=1" in line:
				self.success_count+=1
				succ=True
			elif "collision=1" in line:
				self.collision_count+=1
				coll=True
			elif "act=" in line:
				# self.total_step+=1
				if (float(line_split[1])==1):
					self.acc_count+=1	
				elif (float(line_split[1])==2):
					round_dec_count+=1
			elif "Total discounted reward =" in line:
				self.total_reward.append(float(line_split[4]))
			elif "Total undiscounted reward =" in line:
				self.total_nondis_reward.append(float(line_split[4]))
				values.append(float(line_split[4]))
			elif "- Action =" in line:
				self.total_step+=1
				self.actions=line_split[3]
			elif line.startswith("Trials:"):
				if float(line_split[6])>0:
					self.num_trial.append(int(line_split[6]))
				self.total_search_depth+=int(line_split[8])
			elif "Execute default" in line:
				self.default_count+=1
			elif line.startswith("# nodes: expanded"):
				self.expansion_count+=int(line_split[8])
				self.policy_size+=int(line_split[12])
				self.expanded_nodes.append(int(line_split[8]))
				self.tree_nodes.append(int(line_split[10]))
			elif line.startswith("Time (CPU s)"):
				self.expansion_time+=float(line_split[13])
				self.total_time+=float(line_split[17])
			elif line.startswith("Initial bounds:"):
				self.init_lb+=float(self.remove_redundant(line_split[2]))
				self.init_ub+=float(self.remove_redundant(line_split[3]))
			elif line.startswith("Final bounds:"):
				self.final_lb+=float(self.remove_redundant(line_split[2]))
				self.final_ub+=float(self.remove_redundant(line_split[3]))
				#num_rounds = num_rounds + 1;
			elif "dist_trav" in line:
				if round_end==True:
					round_end=False
					self.dis_trav.append(float(line_split[10]))
			else:
				pass;

		return values, succ, coll

	def PrintData(self):
		print '==============================================Statistics=============================================='
		print args
		print '==============================================Statistics=============================================='
		print '#Rounds:'
		print self.num_rounds
		print '#Finished rounds:'
		print self.num_finishedrounds
		print 'Ave reward: discountred / non-discounted:'
		print '%.3f (%.3f)/ %.3f (%.3f)' % (sum(self.total_reward)/float(self.num_finishedrounds), 
			stats.sem(self.total_reward,axis=None, ddof=0),
			sum(self.total_nondis_reward)/float(self.num_finishedrounds),
			stats.sem(self.total_nondis_reward,axis=None, ddof=0))
		print 'Success %:' 
		print float(self.success_count)/float(self.num_finishedrounds),
		print str(binom.std(self.num_finishedrounds, float(self.success_count)/float(self.num_finishedrounds), loc=0)/float(self.num_finishedrounds))
		print 'collision % per round'
		print float(self.collision_count)/float(self.num_finishedrounds),
		print str(binom.std(self.num_finishedrounds, float(self.collision_count)/float(self.num_finishedrounds), loc=0)/float(self.num_finishedrounds))
		print 'collision % per step'
		print float(self.collision_count)/float(self.total_step),
		print str(binom.std(self.total_step, float(self.collision_count)/float(self.total_step), loc=0)/float(self.total_step))
		print 'collision % per meter:'
                if(sum(self.dis_trav) > 0):
		  print float(self.collision_count)/float(sum(self.dis_trav)),
		  print str(binom.std(sum(self.dis_trav), float(self.collision_count)/float(sum(self.dis_trav)), loc=0)/float(sum(self.dis_trav)))
		  print 'Ave distance travelled per round:'
		  print '%.3f' % (numpy.mean(numpy.array(self.dis_trav))),
		  print str(stats.sem(self.dis_trav))
		print 'smoothness:'
		print float(sum(self.dec_count))/self.num_finishedrounds,
		print str(stats.sem(self.dec_count))
		print 'Total steps per round:'
		print float(self.total_step)/float(self.num_rounds)
		print 'Min / Max num of trials:'
		print str(min(self.num_trial))+'/'+str(max(self.num_trial))+' ('+str(numpy.std(numpy.array(self.num_trial),ddof=1))+')'
		print 'Default move count: '
		print float(self.default_count/self.num_rounds)
		print 'Ave tree nodes / Ave expanded nodes / Ave policy sizes: '
		print '%.3f %.3f / %.3f / %.3f ' % (float(sum(self.tree_nodes))/float(self.total_step),
			stats.sem(self.tree_nodes,axis=None, ddof=0),
			float(self.expansion_count)/float(self.total_step), 
			float(self.policy_size)/float(self.total_step))
		print 'Max expanded nodes: '
		print '%d' % (max(self.expanded_nodes))
		print 'Ave expansion time: '
		print '%.3f' % (self.expansion_time/float(self.total_step))
		print 'Ave total time: '
		print '%.3f' % (self.total_time/float(self.total_step))
		print 'Initial bounds: '
		print '( %.3f , %.3f )' % (self.init_lb/float(self.total_step),
			self.init_ub/float(self.total_step))	
		print 'Final bounds: '
		print '( %.3f , %.3f )' % (self.final_lb/float(self.total_step),
			self.final_ub/float(self.total_step))	


	def SaveData(self, args, rand, folder):
		filename=folder+'/Search_record'+str(rand)+'.txt'
		with open(filename, "a") as output:
			output.write('==============================================Statistics==============================================\n')
			output.write( args+'\n')
			output.write('==============================================Statistics==============================================\n')
			output.write( '#Rounds:'+'\n')
			output.write( str(self.num_rounds) +'\n')
			output.write( '#Finished rounds:'+'\n')
			output.write( str(self.num_finishedrounds)+'\n')
			output.write( 'Ave reward: discountred / non-discounted:'+'\n')
			output.write( '%.3f (%.3f) / %.3f (%.3f)' % (sum(self.total_reward)/float(self.num_finishedrounds), 
				stats.sem(self.total_reward,axis=None, ddof=0),
				sum(self.total_nondis_reward)/float(self.num_finishedrounds),
				stats.sem(self.total_nondis_reward,axis=None, ddof=0))+'\n')
			output.write( 'Success %:' +'\n')
			output.write( str(float(self.success_count)/float(self.num_finishedrounds))+' ')
			output.write( str(binom.std(self.num_finishedrounds, float(self.success_count)/float(self.num_finishedrounds), loc=0)/float(self.num_finishedrounds)) + '\n')
			output.write('collision % per round' + '\n')
			output.write(str(float(self.collision_count)/float(self.num_finishedrounds)) + ' ')
			output.write( str(binom.std(self.num_finishedrounds, float(self.collision_count)/float(self.num_finishedrounds), loc=0)/float(self.num_finishedrounds)) + '\n')
			output.write( 'collision % per step' + '\n')
			output.write( str(float(self.collision_count)/float(self.total_step)) + ' ' )
			output.write( str(binom.std(self.total_step, float(self.collision_count)/float(self.total_step), loc=0)/float(self.total_step)) + '\n')
			output.write( 'collision % per meter:' + '\n')
                        if(sum(self.dis_trav) > 0):
			  output.write( str(float(self.collision_count)/float(sum(self.dis_trav))) + ' ')
			  output.write( str(binom.std(sum(self.dis_trav), float(self.collision_count)/float(sum(self.dis_trav)), loc=0)/float(sum(self.dis_trav))) + '\n' )
		
			#output.write( 'collision %'+'\n')
			#output.write( str(float(self.collision_count)/float(self.num_finishedrounds))+'\n')
			output.write( 'Ave distance travelled per round:'+'\n')
			output.write( '%.3f' % (sum(self.dis_trav)/self.num_finishedrounds)+' ')
			output.write( str(stats.sem(self.dis_trav)) + '\n')
			output.write( 'smoothness:'+'\n')
                        if(sum(self.dec_count) > 0):
			  output.write( str(float(sum(self.dis_trav))/float(sum(self.dec_count)))+'\n')
			output.write('dec count:' + '\n')
			output.write( str(float(sum(self.dec_count))/self.num_finishedrounds) + ' ')
			output.write( str(stats.sem(self.dec_count)) + '\n')
			output.write( 'Total steps per round:'+'\n')
			output.write( str(float(self.total_step)/float(self.num_rounds))+'\n')
			output.write( 'Max search depth per step:'+'\n')
			output.write( str(float(self.total_search_depth)/float(self.total_step))+'\n')
			output.write(  'Min / Max num of trials:'+'\n')
			output.write(  str(min(self.num_trial))+'/'+str(max(self.num_trial))+'\n')
			output.write( 'Default move count: '+'\n')
			output.write( str(float(self.default_count/self.num_rounds))+'\n')
			output.write( 'Ave tree nodes / Ave expanded nodes / Ave policy sizes: '+'\n')
			output.write( '%.3f / %.3f / %.3f ' % (float(sum(self.tree_nodes))/float(self.total_step),
				float(self.expansion_count)/float(self.total_step), 
				float(self.policy_size)/float(self.total_step))+'\n')
			output.write('Max expanded nodes: ' + '\n')
			output.write( '%d' % (max(self.expanded_nodes)) + '\n')
			output.write( 'Ave expansion time: '+'\n')
			output.write( '%.3f' % (self.expansion_time/float(self.total_step))+'\n')
			output.write( 'Ave total time: '+'\n')
			output.write( '%.3f' % (self.total_time/float(self.total_step))+'\n')
			output.write( 'Initial bounds: '+'\n')
			output.write( '( %.3f , %.3f )' % (self.init_lb/float(self.total_step),
				self.init_ub/float(self.total_step))+'\n')
			output.write( 'Final bounds: '+'\n')
			output.write( '( %.3f , %.3f )' % (self.final_lb/float(self.total_step),
				self.final_ub/float(self.total_step))+'\n')


def LoadHistory(filename,value_map, numrun_map):
	file_handler = open(filename,'r')
	content = file_handler.read().splitlines()
	for line in content:
		line_split = line.split(' ')
		if "key" in line:
			prune=float(line_split[1])
			econst=float(line_split[2])
			value=float(line_split[3])
			num_runs=int(line_split[4])
			current_key= str(prune)+' '+ str(econst)
			if current_key not in value_map.keys():
				value_map[current_key]=value
				numrun_map[current_key]=num_runs
			else:
				old_run=numrun_map[current_key]
				old_value=value_map[current_key]
				value_map[current_key]=float(value*num_runs)/float(old_run+num_runs)+float(old_value*old_run)/float(old_run+num_runs)
				numrun_map[current_key]=old_run+num_runs
		else:
			pass;
	file_handler.close()
	return value_map, numrun_map

def SaveHistory(filename, key, value, num_runs):
	with open(filename, "a") as output:
		output.write('key '+key+' '+str(value)+' '+str(num_runs)+'\n');

if __name__ == '__main__' :

	file_flag='trial_'

	folder='./'
	remove_incomplete=False
	if len(sys.argv)<=1:
		print 'args: folder, fileflag, remove_incomplete'
		raw_input('press any key to continue...')
	if len(sys.argv) > 1:
		folder = sys.argv[1]
	if len(sys.argv) > 2:
		file_flag = sys.argv[2]
	if len(sys.argv) > 3:
		remove_incomplete = bool(sys.argv[3])
	
	data=CarDriveData()
	root = os.path.dirname(os.path.realpath(__file__))+'/'
	args=''

	print "checking "+root+folder+'/'+file_flag+'*'+'...'
	if remove_incomplete ==True:
		print 'Removing incompelte runs...'
		raw_input('press any key to comfirm removal (flag: Reach terminal) ...')
		subprocess.call('cd '+root+'/'+folder+"; grep -L --null 'Completed' ./"+file_flag+"* | xargs -0 rm", shell=True)
	if os.path.isdir(folder):
		existing_files=glob.glob(root+folder+'/'+file_flag+'*')
		#print existing_files
		current_value=0
		for existing_file in existing_files:
                        #print existing_file
			value_records, succ, col=data.loadData(existing_file)
			current_value+=sum(value_records)
			#data.num_finishedrounds+=len(value_records)
			#print 'Found datafile ' + existing_file + ' with '\
			#+ str(len(value_records))+' data' 
		print str(data.num_rounds) + ' existing data found, '+ str(data.num_finishedrounds) +' finished'
		if data.num_rounds>0:
			print 'Current value '+str(current_value/float(data.num_finishedrounds))
			data.PrintData()
			global_rand=randint(0, 10000)
			data.SaveData(args,global_rand, folder)
			data.ClearData()

	

	




