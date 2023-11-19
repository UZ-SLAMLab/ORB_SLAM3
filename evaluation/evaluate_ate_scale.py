# Modified by Raul Mur-Artal
# Automatically compute the optimal scale factor for monocular VO/SLAM.

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements: 
# sudo apt-get install python-argparse

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
import numpy
import argparse
import associate

def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    """


    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh

    rotmodel = rot*model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += numpy.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
        normi = numpy.linalg.norm(model_zerocentered[:,column])
        norms += normi*normi

    s = float(dots/norms)    
    
    transGT = data.mean(1) - s*rot * model.mean(1)
    trans = data.mean(1) - s * rot * model.mean(1)

    model_alignedGT = s*rot * model + transGT
    model_aligned = s * rot * model + trans

    alignment_errorGT = model_alignedGT - data
    alignment_error = model_aligned - data

    trans_errorGT = numpy.sqrt(numpy.sum(numpy.multiply(alignment_errorGT,alignment_errorGT),0)).A[0]
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,transGT,trans_errorGT,trans,trans_error, s

def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)
            

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    first_file = '/home/justmohsen/Documents/SLAM2/Datasets/EuRoc/MH01/mav0/state_groundtruth_estimate0/data.csv'
    second_file = '/home/justmohsen/Documents/SLAM2/ORB_SLAM3/f_dataset-MH01_mono.txt'
    # print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)
    is_verbose = True
    # print scale eror and RMSE absolute translational error in meters after alignment with and without scale correction
    is_verbose2 = True
    # save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)
    is_save_associations = True
    # save aligned second trajectory to disk (format: stamp2 x2 y2 z2)
    is_save = True
    # plot the first and the aligned second trajectory to an image (format: png)
    is_plot = True
    # maximally allowed time difference for matching entries (default: 10000000 ns)
    max_time_difference_ns = 20000000
    # time offset added to the timestamps of the second file (default: 0.0)
    time_offset = 0
    # scaling factor for the second trajectory (default: 1.0)
    scale_factor = 1.0
    poses_associations_output_file = 'poses_associations.txt'
    time_associations_output_file = 'time_associations.txt'
    plot_results_output_file = 'results_output.png'

    first_list = associate.read_file_list(first_file, False)
    second_list = associate.read_file_list(second_file, False)

    matches = associate.associate(first_list, second_list,float(time_offset),float(max_time_difference_ns))    
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")
    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = numpy.matrix([[float(value)*float(scale_factor) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    dictionary_items = second_list.items()
    sorted_second_list = sorted(dictionary_items)

    second_xyz_full = numpy.matrix([[float(value)*float(scale_factor) for value in sorted_second_list[i][1][0:3]] for i in range(len(sorted_second_list))]).transpose() # sorted_second_list.keys()]).transpose()
    rot,transGT,trans_errorGT,trans,trans_error, scale = align(second_xyz,first_xyz)
    
    second_xyz_aligned = scale * rot * second_xyz + trans
    second_xyz_notscaled = rot * second_xyz + trans
    second_xyz_notscaled_full = rot * second_xyz_full + trans
    first_stamps = list(first_list.keys())
    first_stamps.sort()
    first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
    
    second_stamps = list(second_list.keys())
    second_stamps.sort()
    second_xyz_full = numpy.matrix([[float(value)*float(scale_factor) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    second_xyz_full_aligned = scale * rot * second_xyz_full + trans
    
    if is_verbose:
        print(f"compared_pose_pairs %d pairs"%(len(trans_error)))
        print(f"absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)))
        print(f"absolute_translational_error.mean %f m"%numpy.mean(trans_error))
        print(f"absolute_translational_error.median %f m"%numpy.median(trans_error))
        print(f"absolute_translational_error.std %f m"%numpy.std(trans_error))
        print(f"absolute_translational_error.min %f m"%numpy.min(trans_error))
        print(f"absolute_translational_error.max %f m"%numpy.max(trans_error))
        print(f"max idx: %i" %numpy.argmax(trans_error))
    else:
        # print "%f, %f " % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)),  scale)
        # print "%f,%f" % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)),  scale)
        print("%f,%f,%f" % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)), scale, numpy.sqrt(numpy.dot(trans_errorGT,trans_errorGT) / len(trans_errorGT))))
        # print "%f" % len(trans_error)
    if is_verbose2:
        print(f"compared_pose_pairs %d pairs"%(len(trans_error)))
        print(f"absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)))
        print(f"absolute_translational_errorGT.rmse %f m"%numpy.sqrt(numpy.dot(trans_errorGT,trans_errorGT) / len(trans_errorGT)))

    if is_save_associations:
        file = open(poses_associations_output_file,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
        file.close()
        
    if is_save:
        file = open(time_associations_output_file,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_notscaled_full.transpose().A)]))
        file.close()

    if is_plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.pylab as pylab
        from matplotlib.patches import Ellipse
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","ground truth")
        plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")
        label="difference"
        for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_notscaled.transpose().A):
            ax.plot([x1,x2],[y1,y2],'-',color="violet",label=label, alpha=0.5, linewidth=0.05)
            label=""
            
        ax.legend()
            
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.axis('equal')
        plt.savefig(plot_results_output_file,format="png")
