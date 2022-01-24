import roslaunch
import rospy
import subprocess
import os
import numpy
import itertools
import copy
import smtplib
from string import Template
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from timeit import default_timer as timer


def createRunFolder():
	# Creates empty folder
	i = 1
	while True:
		path = os.path.join(result_folder_path, '') + "Run_" + str(i)
		if not os.path.exists(path):
				os.makedirs(path + "/")
				return path + "/"
		i = i+1

def createFolder(i, repeat, j):
		path = runFolderPath + str(i) + "_repeat_" + str(repeat) + "/seq_" + str(j)
		if not os.path.exists(path):
				os.makedirs(path + "/")
				return path + "/"


def runLaunch(roslaunch_file, path,  cnt, repeat, kitti_item):

	roslaunch_args = roslaunch_file[0][1]
	print "\n\nSet: " + str(cnt) +  "  |  Repeat: " + str(repeat) + "  |  Sequence: " +  str(kitti_item["seq"]) +  "  |  ", roslaunch_args[:], "\n\n"

	rospy.sleep(2)

	# Run launch file
	launch = roslaunch.scriptapi.ROSLaunch()
	launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
	time_start = timer()
	launch.start()
	launch.spin()
	time_end = timer()
	print("Stopping launch")
	launch.shutdown()
	# for i in range (10):
	#    launch.parent.shutdown()
	#    rospy.sleep(5)

	# Run validation Test
	path_kitti_item = path + ("%02d" % kitti_item["seq"])
	cmd1 = "cp ../evaluation/g2o_output/g2o_sol.txt " + path
	cmd2 = "cp ../evaluation/g2o_output/g2o_sol_all.txt " + path
	cmd3 = "cp ../evaluation/g2o_output/gps_lib.pos " + path

	#Save params to file
	with open(path_kitti_item + '_params' + '.txt', 'w') as f:
		for item in roslaunch_args:
				f.write("%s\n" % item)

	subprocess.call(cmd1, shell=True)
	subprocess.call(cmd2, shell=True)
	subprocess.call(cmd3, shell=True)

	# Total execution time
	execution_time = (time_end - time_start) # Time in seconds
	f = open(path_kitti_item + "_total_execution_time.txt","w+")
	f.write("Total time: " + str(execution_time / 60) + " minutes")
	f.close()

if __name__ == '__main__':
    
    # Update evaluation script
	subprocess.call("jupyter nbconvert --to python evaluationTokyo.ipynb", shell=True, cwd="../evaluation")
    
	########################### Parameters  ##############################

	# Path to save results
	result_folder_path = "/home/kcwian/Pulpit/result_g2o"

	# Selected Kitti sequences
	kitti_sequences = [
    					{"seq" : 1}
	]
	# How many times repeat single parameters
	repeat_num = 1

	# If True, start rosbag_player and shutdowns all nodes when it finishes
	use_rosbag_player = True

	# Time for single run in sec - duration of rosbag / play speed.
	# Doesnt matter if use_rosbag_player = True
	rosbag_rate = 0.1

	# Changing parameters
	var_args = [
				
                {"windowSize" : numpy.arange(20,30,10)}
	]

	# Const parameters
	# It is possible to change only parameters that are defined outside "include launch" in main launch file
	# Parameter's value must be set using "default=" instead of "value=" in launch file
	const_args = [
                
                '../launch/raw_gnss_rtklib.launch',
                'verbose:=false',
                #'windowSize:=20',
                'maxIterations:=50',
                'maxIterationsEnd:=10',
                'optimizeBiasesAgain:=true',
                'optimizeBiasesAgainEnd:=true',
				'paramLaserInform:=10',
	]

	###############################################################################

	if not var_args:
		var_args.append({"fake_parameter" : [1]})

	rospy.init_node('optim_evaluation', anonymous=True)
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	runFolderPath = createRunFolder()
	print(runFolderPath)

	tries_num = 0
	for params_set in var_args:
		combinations = list(itertools.product(*params_set.values()))
		tries_num += len(combinations)

	sequences_num = len(kitti_sequences)

	print("\n\n%d Sequences x %d Combinations" % (sequences_num, tries_num))

	for repeat in range(1, repeat_num + 1):
		i = 1
		for params_set in var_args:
			combinations = list(itertools.product(*params_set.values()))
			for cb in combinations:
				# Restores default args
				roslaunch_args = const_args[:]
				# Assign values to parameter names
				for idx, param in enumerate(params_set.keys()):
					roslaunch_args.append(param + ":=" +  str(cb[idx]))

				roslaunch_args_copy = roslaunch_args[:]

				# Repeat this for all Kitti sequences
				for item in kitti_sequences:
					folderPath = createFolder(i, repeat, item["seq"])
					roslaunch_args = roslaunch_args_copy[:]
					roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(roslaunch_args)[0], roslaunch_args[1:])]
					runLaunch(roslaunch_file, folderPath, i, repeat, item)

					# Call evaluation
					f = open(folderPath + "results.txt", "w")
					subprocess.call("./env/bin/python -m IPython evaluationTokyo.py", stdout=f, shell=True, cwd="../evaluation")			
					# Moving results
					subprocess.call("cp ./results/evo_ape_* " + folderPath, shell=True, cwd="../evaluation")
					subprocess.call("cp ./results/gt_ecef.tum " + folderPath, shell=True, cwd="../evaluation")
					subprocess.call("cp ./results/gt_utm.tum " + folderPath, shell=True, cwd="../evaluation")
					subprocess.call("cp ./results/gps_ecef.tum " + folderPath, shell=True, cwd="../evaluation")
					subprocess.call("cp ./results/g2o_ecef.tum " + folderPath, shell=True, cwd="../evaluation")
					subprocess.call("cp ./results/g2o_all_ecef.tum " + folderPath, shell=True, cwd="../evaluation")
					subprocess.call("cp ./results/map.html " + folderPath, shell=True, cwd="../evaluation")

				i += 1

	print("Script Finished")