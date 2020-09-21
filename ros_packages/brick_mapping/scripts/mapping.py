#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from matplotlib import lines
import numpy as np
import math
from math import cos, sin, tan
import sys, copy

# from scipy.cluster.vq import kmeans
from sklearn.mixture import GaussianMixture
from sklearn.decomposition import PCA
from matplotlib.patches import Ellipse
from matplotlib.legend_handler import HandlerLine2D
from matplotlib.patches import FancyArrowPatch

from brick_mapping.srv import GetMappedArenaObjects, GetMappedArenaObjectsResponse
import brick_mapping.msg
import brick_mapping.srv

from mbzirc_msgs.msg import ObjectWithTypeArray, ObjectWithType

from mbzirc_msgs.msg import BrickMap
from std_srvs.srv import TriggerResponse, Trigger
from brick_mapping.msg import MappedArenaObjects
from catkin_pkg import rospack

PLOT_MAP = False
# PLOT_MAP = True

RED_LEN = 0.3
GREEN_LEN = 0.6
BLUE_LEN = 1.2

RED = 1
GREEN = 2
BLUE = 3
WALL = 5
GROUND_PLACING = 66

colors = [ "cyan", "magenta", "yellow", "salmon", "purple", "olive", "tan", "grey", "orange"]

TYPED_COLOR = {RED:'r', GREEN:'g', BLUE:'b', WALL:'y', WALL:'m'}
TYPED_LENGTH = {RED:RED_LEN, GREEN:GREEN_LEN, BLUE:BLUE_LEN}

global brick_subs_
brick_subs_ = None


class AnnotationHandler(HandlerLine2D):

    def __init__(self, ms, *args, **kwargs):
        self.ms = ms
        HandlerLine2D.__init__(self, *args, **kwargs)
        
    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize,
                       trans):
        xdata, xdata_marker = self.get_xdata(legend, xdescent, ydescent,
                                             width, height, fontsize)
        ydata = ((height - ydescent) / 2.) * np.ones(xdata.shape, float)
        legline = FancyArrowPatch(posA=(xdata[0], ydata[0]),
                                  posB=(xdata[-1], ydata[-1]),
                                  mutation_scale=self.ms,
                                  **orig_handle.arrowprops)
        legline.set_transform(trans)
        return legline,


def test_func(x, a, b):
    return a * x + b


def draw_ellipse(position, covariance, ax=None, **kwargs):
    """Draw an ellipse with a given position and covariance"""
    ax = ax or plt.gca()
    
    # Convert covariance to principal axes
    if covariance.shape == (2, 2):
        U, s, Vt = np.linalg.svd(covariance)
        angle = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
        width, height = 2 * np.sqrt(s)
    else:
        angle = 0
        width, height = 2 * np.sqrt(covariance)
    
    # Draw the Ellipse
    for nsig in range(1, 4):
        ax.add_patch(Ellipse(position, nsig * width, nsig * height,
                             angle, **kwargs))


def draw_vector(v0, v1, ax=None, color='k'):
    ax = ax or plt.gca()
    arrowprops = dict(arrowstyle='->',
                    linewidth=2,
                    shrinkA=0, shrinkB=0, color=color)
    print("draw vector from ", v0, " to ", v1)
    
    # return plt.arrow(v0[0],v0[1] , v1[0]-v0[0], v1[1]-v0[1],color = color)
    return ax.annotate('', v1, v0, arrowprops=arrowprops)


class BrickMapper:

    def __init__(self):
        global PLOT_MAP
        
        rospy.init_node('brick_mapping')
        param_prefix_namespace = "brick_mapping/";
        rospy.loginfo("[BrickMapper] starting brick mapper")
        
        rospy.Service("~get_wall_brick_positions", GetMappedArenaObjects, self.mapped_positions_service_callback)
        rospy.Service("~reset_map", Trigger, self.reset_map_service_callback)
        rospy.Service("~plot_map", Trigger, self.plot_map_service_callback)
        
        plotmap_param = rospy.get_param('~plotmap')
        PLOT_MAP = bool(plotmap_param)
        if PLOT_MAP:
            rospy.logwarn("[BrickMapper] will plot the map at the end")
        
        self.max_diff_yaw_same_wall = rospy.get_param('~max_diff_yaw_same_wall')
        self.max_diff_xy_same_wall = rospy.get_param('~max_diff_xy_same_wall')
        # self.num_minimal_n_corrections = rospy.get_param('~num_minimal_n_corrections')
        self.num_minimal_n_corrections_bricks = rospy.get_param('~num_minimal_n_corrections_bricks')
        self.num_minimal_n_corrections_wall = rospy.get_param('~num_minimal_n_corrections_wall')
        self.final_wall_min_length = rospy.get_param('~final_wall_min_length')
        self.bricks_place_length = rospy.get_param('~bricks_place_length')
        
        self.wall_max_median_distance = rospy.get_param('~wall_max_median_distance')
        self.brick_max_median_distance = rospy.get_param('~brick_max_median_distance')
        self.ground_robot_max_median_distance = rospy.get_param('~ground_robot_max_median_distance')
        self.optimize_median_till_distance = rospy.get_param('~optimize_median_till_distance')
        self.optimize_median_till_count = rospy.get_param('~optimize_median_till_count')
        self.robot_name_list = rospy.get_param('~main/robot_name_list')
        self.robot_name = rospy.get_param('~robot_name')
        self.max_distance_intersection_from_wall_center = rospy.get_param('~max_distance_intersection_from_wall_center')
        self.object_detection_topic = rospy.get_param('~object_detection_topic')
        self.connected_wall_min_dist_yaw = rospy.get_param('~connected_wall_min_dist_yaw')
        
        rospy.loginfo("[BrickMapper] self.robot_name_list " + str(self.robot_name_list))
        rospy.loginfo("[BrickMapper] self.robot_name " + self.robot_name)
        self.measurements_from_other_robots = {}
        
        self.this_robot_index = 0
        for i in range(len(self.robot_name_list)):
            if self.robot_name_list[i] == self.robot_name:
                self.this_robot_index = i
                break
        
        if self.robot_name not in self.robot_name_list:
            rospy.logerr("[BrickMapper] this robot name %s is not in robot list names %s" % (self.robot_name, str(self.robot_name_list)))
            self.robot_name_list.append(self.robot_name)
            
        global brick_subs_
        brick_subs_ = {}
        for idx in range(len(self.robot_name_list)):
            brick_map_topic = "/" + self.robot_name_list[idx] + "/" + self.object_detection_topic
            rospy.loginfo("subscribing " + brick_map_topic)
            brick_subs_[self.robot_name_list[idx]] = rospy.Subscriber(brick_map_topic, BrickMap, self.object_detection_callback, callback_args=[brick_map_topic])
            self.measurements_from_other_robots[brick_map_topic] = 0    
        
        self.reset_map()

    def reset_map(self):
        rospy.loginfo("[BrickMapper]: reseting map")
        self.typed_detections = {RED:[], GREEN:[], BLUE:[], WALL:[], GROUND_PLACING:[]}
        self.wall_mean = None
        self.wall_mean_num = 0
        self.brick_mean = None
        self.brick_mean_num = 0
        self.mapped_objects = None
        self.mapping = True
    
    def mapped_positions_service_callback(self, req):
        rospy.loginfo("[BrickMapper]: mapped_positions_service_callback")
        """
        callback for mapped positions
        """
        
        rospy.loginfo("[BrickMapper]: unregister brick subscriber")
        
        self.mapping = False 
        # self.brick_subs_ = None
        
        response = GetMappedArenaObjectsResponse()
        mapped_objects = self.process_detections()
        rospy.loginfo("[BrickMapper]: type mapped_objects " + str(type(mapped_objects)))
        rospy.loginfo("[BrickMapper]: type response.mapped_objects " + str(type(response.mapped_objects)))
        response.mapped_objects = MappedArenaObjects()
        response.mapped_objects.wall1 = mapped_objects.wall1
        response.mapped_objects.wall2 = mapped_objects.wall2
        response.mapped_objects.wall3 = mapped_objects.wall3
        response.mapped_objects.wall4 = mapped_objects.wall4
        response.mapped_objects.bricks_red = mapped_objects.bricks_red
        response.mapped_objects.bricks_green = mapped_objects.bricks_green
        response.mapped_objects.bricks_blue = mapped_objects.bricks_blue
        response.mapped_objects.ugv_place = mapped_objects.ugv_place
        response.success = True
        response.message = "returning the mapped wall and object lines positions"
        rospy.loginfo("[BrickMapper]:response")
        rospy.loginfo(response)
       
        return response
    
    def reset_map_service_callback(self, req):
        rospy.loginfo("[BrickMapper]: reset_map_service_callback")
        self.reset_map()
        
        res = TriggerResponse()
        res.success = True
        res.message = "map reseted"
        return res
    
    def plot_map_service_callback(self, req):
        rospy.loginfo("[BrickMapper]: plot_map_service_callback")
        
        idx_wall = 0
        for object in self.mapped_objects:
            if object.type == RED:
                plt.plot([object.x], [object.y], 'r*') 
            if object.type == GREEN:
                plt.plot([object.x], [object.y], 'g*') 
            if object.type == BLUE:
                plt.plot([object.x], [object.y], 'b*') 
            if object.type == WALL:
                plt.plot([object.x], [object.y], "o" , color='k', mfc=colors[idx_wall % len(colors)])
                idx_wall += 1
        
        plt.axis('equal')
        # plt.axis('square')
        # plt.gca().set_aspect('equal', adjustable='box')
        plt.show();
        res = TriggerResponse()
        res.success = True
        res.message = "ploted"
        return res
    
# BrickMapper::callbackBrickPlaced() //{ 
    
    def process_detections(self):
        """method for processing all mapped detections"""
        
        rospy.loginfo("[BrickMapper]: process_detections begin %s" % (str(rospy.get_time())))
        # concatenate all objects create median
        all_bjects = []
        all_wall = []
        median_objects = []
        median_wall = []
        median_ground_robot = [] 
        
        mean_wall = []
        mean_objects = []
        
        for det_type in self.typed_detections:
            if det_type in [RED, GREEN, BLUE]:
                all_bjects += self.typed_detections[det_type]
            if det_type is WALL:
                all_wall += self.typed_detections[det_type]
                
        rospy.loginfo("[BrickMapper]: have all walls at time %s" % (str(rospy.get_time())))
        
        # get all objects median and average        
        if len(all_bjects) > 0:
            rospy.loginfo("all_bjects len %d at time %s" % (len(all_bjects), str(rospy.get_time())))
            all_objects_arr = np.array(all_bjects)
            median_objects = self.calc_medians(all_bjects)
            # median_objects = np.median(all_objects_arr[:, 0:2], axis=0)
            mean_objects = np.mean(all_objects_arr[:, 0:2], axis=0)
            # kmeans_clusters, distortion = kmeans(all_objects_arr[:, 0:2],2)
            gmm = GaussianMixture(n_components=2, covariance_type='diag').fit(all_objects_arr[:, 0:2])
            
            # remove objects that are not within 6m from the cluster centers 
            max_dist_point_from_cluester = 6
            indexes_to_remove = []
            rospy.loginfo('all_objects_arr shape before ' + str(all_objects_arr.shape))
            for i in range(all_objects_arr.shape[0]):
                
                xo = all_objects_arr[i, 0]
                yo = all_objects_arr[i, 1]
                
                dist1 = math.sqrt((gmm.means_[0][0] - xo) ** 2 + (gmm.means_[0][1] - yo) ** 2)
                dist2 = math.sqrt((gmm.means_[1][0] - xo) ** 2 + (gmm.means_[1][1] - yo) ** 2)
                if dist1 > max_dist_point_from_cluester and dist2 > max_dist_point_from_cluester:
                    # remove the point
                    indexes_to_remove.append(i)
            
            outliers = all_objects_arr[indexes_to_remove, :]
            #
            # rospy.loginfo("outliers "+str(outliers))
            # rospy.loginfo("indexes_to_remove "+str(indexes_to_remove))
            all_objects_arr = np.delete(all_objects_arr, indexes_to_remove, 0)
            rospy.loginfo('all_objects_arr shape after ' + str(all_objects_arr.shape))
            
            # just used to get mean :-))))
            gmm = GaussianMixture(n_components=2, covariance_type='diag').fit(all_objects_arr[:, 0:2])
            labels = gmm.predict(all_objects_arr[:, 0:2])
            rospy.loginfo('gmm.means_ ' + str(gmm.means_))
            rospy.loginfo('gmm.means_.size ' + str(gmm.means_.size))
            rospy.loginfo('gmm.covariances_ ' + str(gmm.covariances_))
            
            first_indexes = []
            second_indexes = []
            for i in range(labels.size):
                if labels[i] == 0:
                    first_indexes.append(i)
                else:
                    second_indexes.append(i)

            # run PCAs to distinguish UGV and UAV grasping places                    
            pca1 = PCA(n_components=2)
            pca1.fit(all_objects_arr[first_indexes, 0:2])
            rospy.loginfo("1 pca.explained_variance_ " + str(pca1.explained_variance_))
            
            pca2 = PCA(n_components=2)
            pca2.fit(all_objects_arr[second_indexes, 0:2])
            rospy.loginfo("2 pca.explained_variance_ " + str(pca2.explained_variance_))
            
            min1 = min(pca1.explained_variance_) 
            min2 = min(pca2.explained_variance_)
                        
            rospy.loginfo('min1 ' + str(min1))
            rospy.loginfo('min2 ' + str(min2))
            
            init_point_bricks = gmm.means_[0]  # where to initialize median filter
            
            colorPCA1 = 'cyan'
            colorPCA2 = 'black'
            if min2 > min1:
                init_point_bricks = gmm.means_[1]  # where to initialize median filter
                rospy.loginfo('min2 > min1 ' + str(min2))
                colorPCA1 = 'black'
                colorPCA2 = 'cyan'
            else:
                rospy.loginfo('min1 > min2 ' + str(min1))
                
            median_objects = init_point_bricks
            
            rospy.loginfo("init_point_bricks " + str(init_point_bricks))
            
            ugv_component_plot = None
            uav_component_plot = None
            if PLOT_MAP:
                wid = 13.0
                inc_size = 0
                minx = min([p[0] for p in all_bjects] + [p[0] for p in all_wall]) - inc_size
                maxx = max([p[0] for p in all_bjects] + [p[0] for p in all_wall]) + inc_size
                miny = min([p[1] for p in all_bjects] + [p[1] for p in all_wall]) - inc_size 
                maxy = max([p[1] for p in all_bjects] + [p[1] for p in all_wall]) + inc_size
                hei = ((maxy - miny) * wid) / (maxx - minx) 
                fig1 = plt.figure(1, figsize=(wid, hei))
                ax = fig1.add_subplot(111)
                ax.set_xlim([minx, maxx])
                ax.set_ylim([miny, maxy])
                plt.axes().set_aspect('equal')
                point_hei = hei * 72 
                x1, x2, y1, y2 = plt.axis()
                yrange = y2 - y1
                plt.xlabel('x[m]')
                plt.ylabel('y[m]')
                
                brick_linewid = (0.2 * (point_hei / yrange)) * 0.8  # 0.2 equals to brick width
                wall_linewid = (0.3 * (point_hei / yrange)) * 0.8  # 0.3 equals to wall width
                
                plt.scatter(outliers[:, 0], outliers[:, 1], c='purple', s=100, zorder=2)
                
                for length, vector in zip(pca1.explained_variance_, pca1.components_):
                    v = vector * 3 * np.sqrt(length)
                    uav_component_plot = draw_vector(pca1.mean_, pca1.mean_ + v, color=colorPCA1)
                    print("uav_component_plot", type(uav_component_plot), uav_component_plot)
                for length, vector in zip(pca2.explained_variance_, pca2.components_):
                    v = vector * 3 * np.sqrt(length)
                    ugv_component_plot = draw_vector(pca2.mean_, pca2.mean_ + v, color=colorPCA2)
                    print("ugv_component_plot", type(ugv_component_plot), ugv_component_plot)
                
                if min2 > min1:
                    tmp = ugv_component_plot
                    ugv_component_plot = uav_component_plot
                    uav_component_plot = tmp
    
                # plt.plot([init_point_bricks[0]], [init_point_bricks[1]], 'o', color='orange', ms=10)
        
        rospy.loginfo("all_bjects time %s" % (str(rospy.get_time())))
            
        red_bricks_plot = None
        if len(self.typed_detections.get(RED)) > 0:
            reds = np.array(self.typed_detections[RED])
            mean_reds = np.mean(reds, axis=0)
          
            if PLOT_MAP:
                for red in reds:
                    s1 = [red[0] - RED_LEN / 2.0 * cos(red[3]) , red[1] - RED_LEN / 2.0 * sin(red[3])]
                    s2 = [red[0] + RED_LEN / 2.0 * cos(red[3]) , red[1] + RED_LEN / 2.0 * sin(red[3])]
                    red_bricks_plot = plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], 'r-', linewidth=brick_linewid)
                # plt.plot(reds[:, 0], reds[:, 1], 'r.')
                # plt.plot([mean_reds[0]], [mean_reds[1]], 'ro', mfc="k")
        else:
            rospy.logerr("no red bricks detected")
        
        green_bricks_plot = None
        if len(self.typed_detections.get(GREEN)) > 0:
            greens = np.array(self.typed_detections[GREEN])
            mean_greens = np.mean(greens, axis=0)
            if PLOT_MAP:
                for green in greens:
                    s1 = [green[0] - GREEN_LEN / 2.0 * cos(green[3]) , green[1] - GREEN_LEN / 2.0 * sin(green[3])]
                    s2 = [green[0] + GREEN_LEN / 2.0 * cos(green[3]) , green[1] + GREEN_LEN / 2.0 * sin(green[3])]
                    green_bricks_plot = plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], 'g-', linewidth=brick_linewid)
                # plt.plot(greens[:, 0], greens[:, 1], 'g.')
                # plt.plot([mean_greens[0]], [mean_greens[1]], 'go', mfc="k")
        else:
            rospy.logerr("no green bricks detected")
            
        blue_bricks_plot = None
        if len(self.typed_detections.get(BLUE)) > 0:
            blues = np.array(self.typed_detections[BLUE])
            mean_blues = np.mean(blues, axis=0)
            if PLOT_MAP:
                for blue in blues:
                    s1 = [blue[0] - BLUE_LEN / 2.0 * cos(blue[3]) , blue[1] - BLUE_LEN / 2.0 * sin(blue[3])]
                    s2 = [blue[0] + BLUE_LEN / 2.0 * cos(blue[3]) , blue[1] + BLUE_LEN / 2.0 * sin(blue[3])]
                    blue_bricks_plot = plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], 'b-', linewidth=brick_linewid)
                # plt.plot(blues[:, 0], blues[:, 1], 'b.')
                # plt.plot([mean_blues[0]], [mean_blues[1]], 'bo', mfc="k")
        else:
            rospy.logerr("no blue bricks detected")
            
        if len(self.typed_detections.get(GROUND_PLACING)) > 0:
            grounds = np.array(self.typed_detections[GROUND_PLACING])
            median_ground_robot = np.median(grounds[:, 0:2], axis=0)
            
            if PLOT_MAP:
                plt.plot(grounds[:, 0], grounds[:, 1], 'c.')
                plt.plot([median_ground_robot[0]], [median_ground_robot[1]], 'co', mfc="k")
        else:
            rospy.logerr("no ground robot placing area detected")
        
        # get wall median wall
        walls_plot = None
        if len(self.typed_detections.get(WALL)) > 0:
            walls = np.array(self.typed_detections[WALL])
            median_wall = self.calc_medians(self.typed_detections[WALL])
            mean_wall = np.mean(walls, axis=0)
            
            if PLOT_MAP:
                for wall in walls:
                    s1 = [wall[0] - wall[4] / 2.0 * cos(wall[3]) , wall[1] - wall[4] / 2.0 * sin(wall[3])]
                    s2 = [wall[0] + wall[4] / 2.0 * cos(wall[3]) , wall[1] + wall[4] / 2.0 * sin(wall[3])]
                    walls_plot = plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], 'y-', linewidth=wall_linewid)
                    
                # plt.plot(walls[:,0], walls[:,1], 'y.')
                # plt.plot([median_wall[0]], [median_wall[1]], 'yo', mfc="k")
                # plt.plot([median_wall[0]], [median_wall[1]], 'y*', mfc="k")
                # plt.plot([median_wall[0]], [median_wall[1]], 'k*')
        else:
            rospy.logerr("no wall detected")
        
        # filter out brick not within 8m of median
        rospy.loginfo("[BrickMapper]: median_filter begin %s" % (str(rospy.get_time())))
        self.median_filter(wall_max_median_distance=self.wall_max_median_distance, brick_max_median_distance=self.brick_max_median_distance
                            , ground_robot_max_median_distance=self.ground_robot_max_median_distance, median_wall=median_wall
                            , median_bricks=median_objects, median_ground_robot=median_ground_robot)
        rospy.loginfo("[BrickMapper]: median_filter end %s" % (str(rospy.get_time())))
        
        if len(self.typed_detections.get(WALL)) == 0:
            rospy.logerr("no walls")
        else:
            rospy.loginfo("have walls")
            
        if PLOT_MAP:
            # finish old plot first
            legend_plots = []
            legend_plots_label = []
            handler_map = {}
            if red_bricks_plot is not None:
                legend_plots.append(red_bricks_plot[0])
                legend_plots_label.append("red bricks detections")
            if green_bricks_plot is not None:
                legend_plots.append(green_bricks_plot[0])
                legend_plots_label.append("green bricks detections")
            if blue_bricks_plot is not None:
                legend_plots.append(blue_bricks_plot[0])
                legend_plots_label.append("blue bricks detections")
            if walls_plot is not None:
                legend_plots.append(walls_plot[0])
                legend_plots_label.append("wall detections")
            if ugv_component_plot is not None:
                legend_plots.append(ugv_component_plot)
                legend_plots_label.append("UGV PCA component variances")
                handler_map = {type(ugv_component_plot) : AnnotationHandler(5)}
            if uav_component_plot is not None:
                legend_plots.append(uav_component_plot)
                legend_plots_label.append("UAV PCA component variances")
                handler_map = {type(uav_component_plot) : AnnotationHandler(5)}
              
            ax.legend(legend_plots, legend_plots_label, handler_map=handler_map)
            
            plt.xlabel('x[m]')
            plt.ylabel('y[m]')
            plt.axis('equal')
            plt.savefig('topological_map_wall_bricks_detections.png')

            # start new plot
            # fig2, ax2 = plt.subplots()
            wid = 5.0
            inc_size = 0
            all_bjects_filtered = []
            all_wall_filtered = []
            for det_type in self.typed_detections:
                if det_type in [RED, GREEN, BLUE]:
                    all_bjects_filtered += self.typed_detections[det_type]
                if det_type is WALL:
                    all_wall_filtered += self.typed_detections[det_type]
            minx = min([p[0] for p in all_bjects_filtered] + [p[0] for p in all_wall_filtered]) - inc_size
            maxx = max([p[0] for p in all_bjects_filtered] + [p[0] for p in all_wall_filtered]) + inc_size
            miny = min([p[1] for p in all_bjects_filtered] + [p[1] for p in all_wall_filtered]) - inc_size 
            maxy = max([p[1] for p in all_bjects_filtered] + [p[1] for p in all_wall_filtered]) + inc_size
            hei = ((maxy - miny) * wid) / (maxx - minx) 
            fig2 = plt.figure(2, figsize=(wid, hei))
            ax2 = fig2.add_subplot(111)
            ax2.set_xlim([minx, maxx])
            ax2.set_ylim([miny, maxy])
            plt.axes().set_aspect('equal')
            
            plt.xlabel('x[m]')
            plt.ylabel('y[m]')
            
        # get mapped object - bricks line centers and individual walls center
        mapped_objects = MappedArenaObjects()
        mapped_objects.bricks_red.type = 0
        mapped_objects.bricks_green.type = 0
        mapped_objects.bricks_blue.type = 0
        mapped_objects.ugv_place.type = 0
        
        all_bricks = []
        if len(self.typed_detections.get(RED)) > 1:
            all_bricks += self.typed_detections.get(RED)
        if len(self.typed_detections.get(GREEN)) > 1:
            all_bricks += self.typed_detections.get(GREEN)
        if len(self.typed_detections.get(BLUE)) > 1:
            all_bricks += self.typed_detections.get(BLUE)
        
        all_bricks_yaw = 0
        all_bricks_mean = []
        if len(all_bricks) > 1 :
            all_bricks_arr = np.array(all_bricks)
            all_bricks_yaw = np.median(all_bricks_arr[:, 3], axis=0)
            all_bricks_mean = np.mean(all_bricks_arr[:, 0:2], axis=0)
        
            # all_bricks_yaw += math.pi#for testing switch
            if PLOT_MAP:
                # plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], 'k--')
                #plt.quiver(all_bricks_mean[0], all_bricks_mean[1], 4 * math.cos(all_bricks_yaw), 4 * math.sin(all_bricks_yaw), width=0.03 , color='k')
                pass
            
        #fill the detections of bricks from meand x y and median yaw of all bricks
        brick_line1_red = None
        if len(self.typed_detections.get(RED)) > 1:
            red_bricks, plot_handlers = self.fit_mean_detections(RED, all_bricks_yaw)
            brick_line1_red, brick_line2_red = plot_handlers
            bricks_red = self.get_typed_object_list([red_bricks], RED)
            mapped_objects.bricks_red = bricks_red[0]
            mapped_objects.bricks_red.yaw = all_bricks_yaw
        
        brick_line1_green = None
        if len(self.typed_detections.get(GREEN)) > 1:
            green_bricks, plot_handlers = self.fit_mean_detections(GREEN, all_bricks_yaw)
            brick_line1_green, brick_line2_green = plot_handlers
            bricks_green = self.get_typed_object_list([green_bricks], GREEN)
            mapped_objects.bricks_green = bricks_green[0]
            mapped_objects.bricks_green.yaw = all_bricks_yaw
        
        brick_line1_blue = None
        if len(self.typed_detections.get(BLUE)) > 1:
            blue_bricks, plot_handlers = self.fit_mean_detections(BLUE, all_bricks_yaw)
            brick_line1_blue, brick_line2_blue = plot_handlers
            bricks_blue = self.get_typed_object_list([blue_bricks], BLUE)
            mapped_objects.bricks_blue = bricks_blue[0] 
            mapped_objects.bricks_blue.yaw = all_bricks_yaw
            
        if len(self.typed_detections.get(GROUND_PLACING)) > 1:
            ugv_places = np.array(self.typed_detections.get(GROUND_PLACING))
            ugv_places_med = np.median(ugv_places, axis=0)
            ugv_place = self.get_typed_object_list([ugv_places_med], GROUND_PLACING)
            mapped_objects.ugv_place = ugv_place[0] 
            
        #recognize wall pattern 
        walls, wall_plot_handlers = self.find_walls()
           
        wall_detections_handler, wall_intersections_hanler,wall_center_handler,wall_detected_handler = wall_plot_handlers
        
        rospy.loginfo("[BrickMapper]: get_typed_object_list begin %s" % (str(rospy.get_time())))
        walls = self.get_typed_object_list(walls, WALL)
        rospy.loginfo("[BrickMapper]: get_typed_object_list begin %s" % (str(rospy.get_time())))
        rospy.loginfo("[BrickMapper]: walls:")
        rospy.loginfo(str(walls))
        
        mapped_objects.wall1.type = 0
        mapped_objects.wall2.type = 0
        mapped_objects.wall3.type = 0
        mapped_objects.wall4.type = 0
        
        distance_now = 0
        distance_alternative = 0
        if len(walls) >= 1:
            rospy.loginfo("[BrickMapper]: walls[0] %s" % (str(walls[0])))
            mapped_objects.wall1 = walls[0]
            rospy.loginfo("[BrickMapper]: mapped_objects.wall1 %s" % (str(mapped_objects.wall1)))
            if len(all_bricks_mean) > 0:
                curs_now = [all_bricks_mean[0] - (self.bricks_place_length / 2.0) * cos(all_bricks_yaw) , all_bricks_mean[1] - (self.bricks_place_length / 2.0) * sin(all_bricks_yaw)]
                curs_alt = [all_bricks_mean[0] + (self.bricks_place_length / 2.0) * cos(all_bricks_yaw) , all_bricks_mean[1] + (self.bricks_place_length / 2.0) * sin(all_bricks_yaw)]
                wall = [walls[0].x, walls[0].y]
                print("wall", str(wall))
                print("curs_now", str(curs_now))
                print("curs_alt", str(curs_alt))
                distance_now += self.dist_xy(wall, curs_now)
                distance_alternative += self.dist_xy(wall, curs_alt)
                
        if len(walls) >= 2:
            rospy.loginfo("[BrickMapper]: walls[1] %s" % (str(walls[1])))
            mapped_objects.wall2 = walls[1]
            rospy.loginfo("[BrickMapper]: mapped_objects.wall2 %s" % (str(mapped_objects.wall2)))
        if len(walls) >= 3:
            rospy.loginfo("[BrickMapper]: walls[2] %s" % (str(walls[2])))
            mapped_objects.wall3 = walls[2]
            rospy.loginfo("[BrickMapper]: mapped_objects.wall3 %s" % (str(mapped_objects.wall3)))
            if len(all_bricks_mean) > 0:
                curs_now = [all_bricks_mean[0] + (self.bricks_place_length / 2.0) * cos(all_bricks_yaw) , all_bricks_mean[1] + (self.bricks_place_length / 2.0) * sin(all_bricks_yaw)]
                curs_alt = [all_bricks_mean[0] - (self.bricks_place_length / 2.0) * cos(all_bricks_yaw) , all_bricks_mean[1] - (self.bricks_place_length / 2.0) * sin(all_bricks_yaw)]
                wall = [walls[2].x, walls[2].y]
                print("wall", str(wall))
                print("curs_now", str(curs_now))
                print("curs_alt", str(curs_alt))
                distance_now += self.dist_xy(wall, curs_now)
                distance_alternative += self.dist_xy(wall, curs_alt)
        if len(walls) >= 4:
            rospy.loginfo("[BrickMapper]: walls[3] %s" % (str(walls[3])))
            mapped_objects.wall4 = walls[3]
            rospy.loginfo("[BrickMapper]: mapped_objects.wall4 %s" % (str(mapped_objects.wall4)))
        
        if distance_alternative < distance_now:
            rospy.logwarn("!!!!! it is better to change the yaw of grasping area")
            rospy.logwarn("distance_now %s distance_alternative %s" % (distance_now, distance_alternative))
            mapped_objects.bricks_red.yaw += math.pi
            mapped_objects.bricks_green.yaw += math.pi
            mapped_objects.bricks_blue.yaw += math.pi
            if PLOT_MAP:
                # plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], 'k--')
                plt.quiver(all_bricks_mean[0], all_bricks_mean[1], 4 * math.cos(all_bricks_yaw + np.pi), 4 * math.sin(all_bricks_yaw + np.pi), width=0.03 , color='g')
             
        else:
            rospy.logwarn("!!!!! current brick yaw of grasping area is OK")
            rospy.logwarn("distance_now %s distance_alternative %s" % (distance_now, distance_alternative))
            
        rospy.loginfo("[BrickMapper]: mapped_objects:")
        rospy.loginfo(str(mapped_objects))
        rospy.loginfo("[BrickMapper]: process_detections end %s" % (str(rospy.get_time())))
        
        rospy.loginfo("[BrickMapper]: PLOT_MAP val is %s" % (str(PLOT_MAP)))               
        if PLOT_MAP:
            
            legend_plots = []
            legend_plots_label = []
            if red_bricks_plot is not None:
                legend_plots.append(red_bricks_plot[0])
                legend_plots_label.append("red bricks detections")
                
            if green_bricks_plot is not None:
                legend_plots.append(green_bricks_plot[0])
                legend_plots_label.append("green bricks detections")
                
            if blue_bricks_plot is not None:
                legend_plots.append(blue_bricks_plot[0])
                legend_plots_label.append("blue bricks detections")
                
            if brick_line1_red is not None:
                legend_plots.append((brick_line1_red, brick_line2_red))
                legend_plots_label.append("red brick area")
                
            if brick_line1_green is not None:
                legend_plots.append((brick_line1_green, brick_line2_green))
                legend_plots_label.append("green brick area")
                
            if brick_line1_blue is not None:
                legend_plots.append((brick_line1_blue, brick_line2_blue))
                legend_plots_label.append("blue brick area")
            
            if  wall_detections_handler is not None:
                legend_plots.append(wall_detections_handler[0])
                legend_plots_label.append("wall detections")
                
            if wall_intersections_hanler is not None:
                legend_plots.append(wall_intersections_hanler[0])
                legend_plots_label.append("wall line intersections")
                
            if wall_center_handler is not None:
                legend_plots.append(wall_center_handler[0])
                legend_plots_label.append("wall centers")
            
            if wall_detected_handler is not None:
                legend_plots.append(wall_detected_handler[0])
                legend_plots_label.append("mapped walls")
                
            print("legend_plots",legend_plots)
            print("legend_plots_label",legend_plots_label)
            ax2.legend(legend_plots, legend_plots_label)
            # plt.axis('square')
            plt.xlabel('x[m]')
            plt.ylabel('y[m]')
            plt.axis('equal')
            plt.savefig('topological_map_wall_bricks_uav_only.png')
            plt.show()
        
        return mapped_objects
    
# }
    
    def get_typed_object_list(self, object_arr, type):
        # rospy.loginfo("get_typed_object_list " + str(type))
        mapped_objects = []
        for single in object_arr:
            # rospy.loginfo("single " + str(single))     
            obj = ObjectWithType()
            obj.stamp = rospy.get_rostime()
            obj.type = type
            obj.x = single[0]
            obj.y = single[1]
            obj.z = single[2]
            obj.yaw = single[3]
            obj.len = single[4]
            mapped_objects.append(obj)
        return mapped_objects
    
    def object_detection_callback(self, data, cb_args):
        """
        callback method for detections from map in brick estimation
        """
    
        if self.mapping:
            brick_map_topic = cb_args[0]
            rospy.loginfo("brick_map_topic " + str(brick_map_topic))
            objects = data.bricks
            # for type in self.typed_detections:
            # clearing wall in each callback
            self.typed_detections[WALL] = []
            
            for object in objects:
                # if object.type == WALL:
                #    print("num corrections:",object.n_corrections," for type")
                
                min_num_corrections = self.num_minimal_n_corrections_bricks
                if object.type == WALL:
                    min_num_corrections = self.num_minimal_n_corrections_wall
                
                if object.n_corrections >= min_num_corrections:
                    
                    if self.typed_detections.get(object.type) is None:
                        rospy.loginfo("[BrickMapper]: unknown robot type %d" % (object.type))
                        self.typed_detections[object.type] = []
                    
                    self.typed_detections[object.type].append([object.x, object.y, object.z, self.normalize_yaw(object.yaw), object.len, object.n_corrections + 1])
                    self.measurements_from_other_robots[brick_map_topic] += 1
                    if self.measurements_from_other_robots[brick_map_topic] % 10 == 0:
                        rospy.loginfo("[BrickMapper]: has %d detections from drone %s" % (self.measurements_from_other_robots[brick_map_topic], brick_map_topic))
                    if len(self.typed_detections[object.type]) % 10 == 0:
                        rospy.loginfo("[BrickMapper]: has %d detections of brick type %d" % (len(self.typed_detections[object.type]), object.type))
            
    def dist_yaw_line(self, yaw1, yaw2):
        """distance in yaw"""        
        
        dist_yaw = yaw1 - yaw2 
        while dist_yaw < -math.pi / 2.0:
            dist_yaw += math.pi
        while dist_yaw > math.pi / 2.0: 
            dist_yaw -= math.pi
        
        return math.fabs(dist_yaw)
    
    def dist_xy(self, arr1, arr2):
        """distance in xy plane"""
        # print("dist", arr1, arr2)
        dist_xy = math.sqrt((arr1[0] - arr2[0]) ** 2 + (arr1[1] - arr2[1]) ** 2)
        return dist_xy
        
    def normalize_yaw(self, yaw):
        """normalize between math.pi/2 and -math.pi/2 because of simetricity of brick and wall"""
        normalized_yaw = yaw
        while normalized_yaw > math.pi / 2.0:
            normalized_yaw -= math.pi
            # print("normalize from", yaw, "to", normalized_yaw)
        while normalized_yaw <= -math.pi / 2.0:
            normalized_yaw += math.pi
            # print("normalize from", yaw, "to", normalized_yaw)     
        return normalized_yaw
                            
    def find_walls(self):
        """find individual walls in the detections"""
        
        rospy.loginfo("[BrickMapper]: find_walls")
        plot_handlers= []
        
        # cluster the walls only based on xy distance and yaw distance sequentially
        different_walls_detects = []
        different_walls_averages = []
        if len(self.typed_detections.get(WALL)) == 0:
            rospy.logerr("[BrickMapper]: no detection of type wall")
            return []
        else:
            wall_detections = self.typed_detections[WALL]
            rospy.loginfo("[BrickMapper]: len(wall_detections) " + str(len(wall_detections)))
            for idx_detects in range(len(wall_detections)):
                min_idx_walls = -1

                for idx_walls in range(len(different_walls_detects)):
                    min_xy = sys.float_info.max
                    min_yaw = sys.float_info.max
                    min_idx = -1
                    for ids_walls_det in range(len(different_walls_detects[idx_walls])):
        
                        dist_xy = self.dist_xy(different_walls_detects[idx_walls][ids_walls_det], wall_detections[idx_detects])
                        dist_yaw = self.dist_yaw_line(different_walls_detects[idx_walls][ids_walls_det][3], wall_detections[idx_detects][3])
                        
                        if dist_xy < min_xy and dist_yaw < min_yaw:
                            min_idx = ids_walls_det
                            min_yaw = dist_yaw
                            min_xy = dist_xy
                        
                    if min_idx >= 0 and min_xy < self.max_diff_xy_same_wall and min_yaw < self.max_diff_yaw_same_wall:
                        min_idx_walls = idx_walls

                if min_idx_walls >= 0:
                    different_walls_detects[min_idx_walls].append(wall_detections[idx_detects])
                    # rospy.loginfo("add wall "+str(wall_detections[idx_detects]) + "among walls " + str(different_walls_detects[min_idx_walls])+ " min_yaw "+str(min_yaw)+ " min_xy "+str(min_xy))
                else:
                    different_walls_detects.append([wall_detections[idx_detects]])
                    # rospy.loginfo("add wall niw different wall "+str(wall_detections[idx_detects]))
        
        wall_detections_handler = None
        if PLOT_MAP:
            fig = plt.gcf()
            width, hei = fig.get_size_inches()
            point_hei = hei * 72 
            x1, x2, y1, y2 = plt.axis()
            yrange = y2 - y1
            wall_linewid = (0.3 * (point_hei / yrange)) * 0.8  # 0.3 equals to wall width
            #plot_handlers.append()
            #wall_detections_handler = [None]*len(different_walls_detects)
            for w_idx in range(len(different_walls_detects)):
                da = np.array(different_walls_detects[w_idx])
                #plt.plot(da[:, 0], da[:, 1], '.', lw=0.5, color=colors[w_idx % len(colors)])
                for wall in different_walls_detects[w_idx]:
                    s1 = [wall[0] - wall[4] / 2.0 * cos(wall[3]) , wall[1] - wall[4] / 2.0 * sin(wall[3])]
                    s2 = [wall[0] + wall[4] / 2.0 * cos(wall[3]) , wall[1] + wall[4] / 2.0 * sin(wall[3])]
                    #color=colors[w_idx % len(colors)]
                    wall_detections_handler = plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], '-', color='y',linewidth=wall_linewid)
                    # plt.text(wall[0] - 2.0, wall[1], "idx " + str(w_idx))
            
        plot_handlers.append(wall_detections_handler)    
        # plt.show()
        rospy.loginfo("[BrickMapper]: initiall num different_walls_detects %d" % (len(different_walls_detects)))
        
        #merge clusters with close averages
        close_averages = True
        while close_averages:
            close_averages = False    
            # average wall centers based on their paired detections
            for idx_walls in range(len(different_walls_detects)):
                different_walls_averages.append([0, 0, 0, 0, 0])
                for idx_one_wall in range(len(different_walls_detects[idx_walls])):
                    # add start,middle,end
                    len_w = different_walls_detects[idx_walls][idx_one_wall][4]
                    yaw = different_walls_detects[idx_walls][idx_one_wall][3]
                    z = different_walls_detects[idx_walls][idx_one_wall][2]
                    
                    x1 = different_walls_detects[idx_walls][idx_one_wall][0] - len_w / 2.0 * math.cos(yaw)
                    x2 = different_walls_detects[idx_walls][idx_one_wall][0]
                    x3 = different_walls_detects[idx_walls][idx_one_wall][0] + len_w / 2.0 * math.cos(yaw)
                    y1 = different_walls_detects[idx_walls][idx_one_wall][1] - len_w / 2.0 * math.sin(yaw)
                    y2 = different_walls_detects[idx_walls][idx_one_wall][1]
                    y3 = different_walls_detects[idx_walls][idx_one_wall][1] + len_w / 2.0 * math.sin(yaw)
                    
                    different_walls_averages[idx_walls][0] += x1 + x2 + x3
                    different_walls_averages[idx_walls][1] += y1 + y2 + y3
                    different_walls_averages[idx_walls][2] += z * 3
                    different_walls_averages[idx_walls][3] += yaw * 3 
            
                # divide by len*3 = added start-middle-end 
                for idx_one_wall_part in range(len(different_walls_averages[idx_walls])):
                    different_walls_averages[idx_walls][idx_one_wall_part] /= float(len(different_walls_detects[idx_walls]) * 3)
            
            # check distance of wall averages and connect those if close to each other
            connected_some_walls = False
            for idx_walls_1 in range(len(different_walls_detects)):
                for idx_walls_2 in range(idx_walls_1 + 1, len(different_walls_detects)):
                    wall1_avg = different_walls_averages[idx_walls_1]
                    wall2_avg = different_walls_averages[idx_walls_2]
                    
                    dist_xy = self.dist_xy(wall1_avg, wall2_avg)
                    dist_yaw = self.dist_yaw_line(wall1_avg[3], wall2_avg[3])
                    if dist_xy < self.max_diff_xy_same_wall and dist_yaw < self.max_diff_yaw_same_wall:
                        rospy.loginfo("[BrickMapper]: num bef different_walls_detects %d" % (len(different_walls_detects)))
                        rospy.loginfo("[BrickMapper]: we should joint those walls")
                        rospy.loginfo("[BrickMapper]: add wall id %d into %d" % (idx_walls_2, idx_walls_1))
                        rospy.loginfo("[BrickMapper]: len wall before %d" % (len(different_walls_detects[idx_walls_1])))
                        rospy.loginfo("[BrickMapper]: len wall addition %d" % (len(different_walls_detects[idx_walls_2])))
                        
                        different_walls_detects[idx_walls_1] += copy.deepcopy(different_walls_detects[idx_walls_2])  # append idx_walls_2 to idx_walls_1
                        different_walls_detects[idx_walls_2] = copy.deepcopy(different_walls_detects[len(different_walls_detects) - 1])  # put last to idx_walls_2
                        del different_walls_detects[len(different_walls_detects) - 1]
                        rospy.loginfo("[BrickMapper]: len wall after %d" % (len(different_walls_detects[idx_walls_1])))
                        close_averages = True
                        connected_some_walls = True
                        rospy.loginfo("[BrickMapper]: num aft different_walls_detects %d" % (len(different_walls_detects)))
                        break
                        
                if connected_some_walls:
                    break        
        
        # find out the length of the filtered walls and plot the individual walls        
        for w_idx in range(len(different_walls_detects)):
            max_distance_average = -sys.float_info.max
            for wall in different_walls_detects[w_idx]:   
                s1 = [wall[0] - wall[4] / 2.0 * cos(wall[3]) , wall[1] - wall[4] / 2.0 * sin(wall[3])]
                s2 = [wall[0] + wall[4] / 2.0 * cos(wall[3]) , wall[1] + wall[4] / 2.0 * sin(wall[3])]
                dist1 = self.dist_xy(s1, different_walls_averages[w_idx])
                dist2 = self.dist_xy(s2, different_walls_averages[w_idx])
                if dist1 > max_distance_average:
                    max_distance_average = dist1
                if dist2 > max_distance_average:
                    max_distance_average = dist2
 
            if PLOT_MAP:
                da = np.array(different_walls_detects[w_idx])
                #plt.plot(da[:, 0], da[:, 1], '.', lw=0.5, color=colors[w_idx % len(colors)])
                
            wall_yaw = different_walls_averages[w_idx][3]
            wall_length = 2.0 * max_distance_average
            different_walls_averages[w_idx][4] = wall_length
        
        rospy.loginfo("[BrickMapper]: different_walls_averages num " + str(len(different_walls_averages)) + " are " + str(different_walls_averages))
        
        for idx in range(len(different_walls_averages) - 1, -1, -1):
            if different_walls_averages[idx][4] < self.final_wall_min_length:
                rospy.logwarn("[BrickMapper]: remove short wall %d with length %f" % (idx, different_walls_averages[idx][4]))
                del different_walls_averages[idx];
            else:
                if PLOT_MAP:
                    len_half = different_walls_averages[idx][4] / 2.0
                    wall_yaw = different_walls_averages[idx][3]
                    """
                    plt.plot([different_walls_averages[idx][0] - len_half * cos(wall_yaw), different_walls_averages[idx][0] + len_half * cos(wall_yaw)],
                             [different_walls_averages[idx][1] - len_half * sin(wall_yaw), different_walls_averages[idx][1] + len_half * sin(wall_yaw)],
                              "-" , color='k')
                    plt.plot([different_walls_averages[idx][0] - len_half * cos(wall_yaw), different_walls_averages[idx][0] + len_half * cos(wall_yaw)],
                             [different_walls_averages[idx][1] - len_half * sin(wall_yaw), different_walls_averages[idx][1] + len_half * sin(wall_yaw)],
                              "--" , color=colors[idx % len(colors)])
                    """
                    
        # calculate wall_intersections between walls
        wall_intersections = {}
        wall_intersections_hanler = None
        for w_idx1 in range(len(different_walls_averages)):
            centr1 = different_walls_averages[w_idx1]
            a1, b1 = self.line_params_from_pos_ang(different_walls_averages[w_idx1])
            rospy.loginfo("[BrickMapper]: w_idx1 %s" % (str(w_idx1)))
            for w_idx2 in range(w_idx1 + 1, len(different_walls_averages)):
                rospy.loginfo("[BrickMapper]: w_idx2 %s" % (str(w_idx2)))
                centr2 = different_walls_averages[w_idx2]
                a2, b2 = self.line_params_from_pos_ang(different_walls_averages[w_idx2])
                
                ix, iy = self.line_intersect(a1, b1, a2, b2)
                
                if ix is None or iy is None:
                    rospy.logwarn("[BrickMapper]: no intersection!!!!! between %s %s" % (str(ix), str(iy)))
                    continue
                
                rospy.loginfo("centr1 " + str(centr1))
                rospy.loginfo("centr2 " + str(centr2))
                yaw_line1 = math.atan2(centr1[1] - iy, centr1[0] - ix)
                yaw_line2 = math.atan2(centr2[1] - iy, centr2[0] - ix)
                dist_yaw = self.dist_yaw_line(yaw_line1, yaw_line2)
                
                dist1 = math.sqrt((ix - centr1[0]) ** 2 + (iy - centr1[1]) ** 2)
                dist2 = math.sqrt((ix - centr2[0]) ** 2 + (iy - centr2[1]) ** 2)
                
                # distance of intersection must be within max_distance_intersection_from_wall_center and dist_yaw within connected_wall_min_dist_yaw
                if dist1 < self.max_distance_intersection_from_wall_center and dist2 < self.max_distance_intersection_from_wall_center and dist_yaw > self.connected_wall_min_dist_yaw:
                    if wall_intersections.get(w_idx1) is None:
                        wall_intersections[w_idx1] = {}
                    if wall_intersections.get(w_idx2) is None:
                        wall_intersections[w_idx2] = {}
                    dist_sum_intersections = dist1 + dist2
                    wall_intersections[w_idx1][w_idx2] = (ix, iy , dist_sum_intersections, dist_yaw)
                    wall_intersections[w_idx2][w_idx1] = (ix, iy , dist_sum_intersections, dist_yaw)
                    
                    rospy.loginfo("[BrickMapper]: add intersection between " + str(w_idx1) + " and " + str(w_idx2))
                    
                    if PLOT_MAP:
                        wall_intersections_hanler = plt.plot([ix], [iy], "." , color='red', mfc='y')
                    plot_handlers.append
                    if len(wall_intersections[w_idx1]) > 2 or len(wall_intersections[w_idx1]) > 2:
                        rospy.logerr("[BrickMapper]: there is wall with more wall_intersections than 2, wall  " + str(w_idx1) + " and " + str(w_idx2))
        
        rospy.loginfo("[BrickMapper]: wall_intersections num " + str(len(wall_intersections)) + " are:")
        
        for w_id in wall_intersections:
            rospy.loginfo("[BrickMapper]: wall " + str(w_id) + " intersections: " + str(wall_intersections[w_id]))
        
        wall_intersections_backup = copy.deepcopy(wall_intersections)
        plot_handlers.append(wall_intersections_hanler)
        # intersection only kolmost
        
        # order the walls based on wall_intersections - first find the nonconnected wall
        ordered_wall_indexes = []
        last_wall_idx = -1
        if len(wall_intersections) > 0:
            
            # remove longest distance intersections
            no_wall_with_one_intersection = True
            while no_wall_with_one_intersection:
                
                # find wall that has only one intersection == the first wall
                for w_key in wall_intersections:
                    if len(wall_intersections[w_key]) == 1:
                        ordered_wall_indexes.append(w_key)
                        last_wall_idx = w_key
                        rospy.loginfo("[BrickMapper]: first wall is " + str(last_wall_idx) + " at idx " + str(w_key) + " wall pos " + str(different_walls_averages[last_wall_idx]))
                        rospy.loginfo("[BrickMapper]: add wall " + str(len(ordered_wall_indexes)) + " idx " + str(last_wall_idx) + " wall pos " + str(different_walls_averages[last_wall_idx]))
                        no_wall_with_one_intersection = False
                        break
                    
                # still could not decide which wall is the first == to much intersections == remove the longest intersections
                if last_wall_idx == -1:
                    rospy.loginfo("[BrickMapper]: there is no wall with only one intersection - will remove the longest intersection")
                    max_len_intersection = -sys.float_info.max
                    max_w_key1 = 0
                    max_w_key2 = 0
                    for w_key1 in wall_intersections:
                        for w_key2 in wall_intersections[w_key1]:
                            if wall_intersections[w_key1][w_key2][2] > max_len_intersection:
                                max_len_intersection = wall_intersections[w_key1][w_key2][2]
                                max_w_key1 = w_key1
                                max_w_key2 = w_key2

                    wall_intersections[max_w_key1].pop(max_w_key2)
                    wall_intersections[max_w_key2].pop(max_w_key1)
                    rospy.loginfo("[BrickMapper]: removing intersection between " + str(max_w_key1) + " and " + str(max_w_key2) + " with maximal distance " + str(max_len_intersection))
            
        elif len(different_walls_averages) == 1:
            ordered_wall_indexes = [0]
        
        # order the walls based on wall_intersections - than go along the chain
        filling_walls = True
        counter = 0
        while len(wall_intersections) > 0 and filling_walls and last_wall_idx != -1 and counter < 10:
            counter += 1
            rospy.loginfo("[BrickMapper]: counter " + str(counter))
            rospy.loginfo("[BrickMapper]: last_wall_idx " + str(last_wall_idx))
            keys = wall_intersections[last_wall_idx].keys()

            # remove the keys that are already in ordered_wall_indexes
            for  keyidx in range(len(keys) - 1, -1, -1): 
                if keys[keyidx] in ordered_wall_indexes:
                    rospy.loginfo("[BrickMapper]: remove wall intersection with wall already in the " + str(keyidx))
                    del keys[keyidx]
                    break

            # if there is more than one intersections with other walls, than remove the longest
            if len(keys) > 1:
                rospy.loginfo("[BrickMapper]: there is more than one remaining intersection on wall, remove the longest")
                has_more_than_one_left = True
                
                while has_more_than_one_left:
                    has_more_than_one_left = False
                    max_dist_keyidx = -1
                    max_dist_intersection = -sys.float_info.max
                    
                    for  keyidx in range(len(keys) - 1, -1, -1): 
                        key = keys[keyidx]
                        dist_intersection = wall_intersections[last_wall_idx][key][2]
                        if dist_intersection > max_dist_intersection :
                            max_dist_intersection = dist_intersection
                            max_dist_keyidx = keyidx
                    del keys[max_dist_keyidx]
                    rospy.loginfo("[BrickMapper]: remove wall intersection with wall " + str(max_dist_keyidx) + "with longest distance to intersect " + str(max_dist_intersection))
                    if len(keys) > 1:
                        has_more_than_one_left = True
                            
            # end if there is no remaining keys
            if len(keys) == 0:
                filling_walls = False
                break                
                    
            # there is only one remaining key
            new_wall_idx = keys[0]
            if wall_intersections[last_wall_idx].get(new_wall_idx) is None:
                rospy.logerr("[BrickMapper] there is no intersection with wall of such type!!!!!")
                last_wall_idx = -1
                filling_walls = False
                break
            
            last_wall_idx = new_wall_idx
            ordered_wall_indexes.append(last_wall_idx)
            rospy.loginfo("[BrickMapper]: add wall index " + str(last_wall_idx) + "to ordered wall indexes" + str(len(ordered_wall_indexes)))
            if len(wall_intersections[last_wall_idx]) == 1:
                rospy.loginfo("[BrickMapper]: last wall is " + str(last_wall_idx))
                filling_walls = False
                break
                    
        # fill out the wall string based on the intersection, print the ordered walls (and plot in case)
        ordered_walls_averages = []
        if last_wall_idx != -1:
            for idx in range(len(ordered_wall_indexes)):
                iw = ordered_wall_indexes[idx]
                rospy.loginfo("[BrickMapper]: ordered wall " + str(iw) + " at pos " + str(different_walls_averages[iw]))
                ordered_walls_averages.append(different_walls_averages[iw])
                
                # set yaw based on intersections
                if len(ordered_walls_averages) <= 1:
                    # set yaw from next intersection to middle for first wall
                    if idx + 1 < len(ordered_wall_indexes):
                        next_iw = ordered_wall_indexes[idx + 1];
                        ix = wall_intersections[iw][next_iw][0]
                        iy = wall_intersections[iw][next_iw][1] 
                        ordered_walls_averages[-1][3] = math.atan2(different_walls_averages[iw][1] - iy, different_walls_averages[iw][0] - ix)
                        
                        # set center based on intersections
                        # s1 is first point
                        s1 = [ordered_walls_averages[-1][0] + (ordered_walls_averages[-1][4] / 2.0) * cos(ordered_walls_averages[-1][3]), ordered_walls_averages[-1][1] + (ordered_walls_averages[-1][4] / 2.0) * sin(ordered_walls_averages[-1][3])]
                        length = self.dist_xy(s1, wall_intersections[iw][next_iw])
                        ordered_walls_averages[-1][0] = ix + (length / 2.0) * cos(ordered_walls_averages[-1][3])
                        ordered_walls_averages[-1][1] = iy + (length / 2.0) * sin(ordered_walls_averages[-1][3])
                        ordered_walls_averages[-1][4] = length
                else:
                    # set yaw from middle to previous intersection to 
                    last_iw = ordered_wall_indexes[idx - 1];
                    ix = wall_intersections[iw][last_iw][0]
                    iy = wall_intersections[iw][last_iw][1] 
                    ordered_walls_averages[-1][3] = math.atan2(iy - different_walls_averages[iw][1], ix - different_walls_averages[iw][0])
                    
                    if idx + 1 < len(ordered_wall_indexes):
                        # set center based on two intersections
                        
                        next_iw = ordered_wall_indexes[idx + 1];
                        ix_next = wall_intersections[iw][next_iw][0]
                        iy_next = wall_intersections[iw][next_iw][1] 
                        length = self.dist_xy(wall_intersections[iw][last_iw], wall_intersections[iw][next_iw])
                        ordered_walls_averages[-1][0] = ix - (length / 2.0) * cos(ordered_walls_averages[-1][3])
                        ordered_walls_averages[-1][1] = iy - (length / 2.0) * sin(ordered_walls_averages[-1][3])
                        ordered_walls_averages[-1][4] = length
                        
                    else:
                        # set last center based on one intersections
                        s2 = [ordered_walls_averages[-1][0] - (ordered_walls_averages[-1][4] / 2.0) * cos(ordered_walls_averages[-1][3]), ordered_walls_averages[-1][1] - (ordered_walls_averages[-1][4] / 2.0) * sin(ordered_walls_averages[-1][3])]
                        length = self.dist_xy(s2, wall_intersections[iw][last_iw])
                        ordered_walls_averages[-1][0] = ix - (length / 2.0) * cos(ordered_walls_averages[-1][3])
                        ordered_walls_averages[-1][1] = iy - (length / 2.0) * sin(ordered_walls_averages[-1][3])
                        ordered_walls_averages[-1][4] = length
                        
        else:
            rospy.logerr("[BrickMapper]: no first wall found, filling unordered walls")
            ordered_walls_averages = different_walls_averages 
        
        wall_center_handler = None
        wall_detected_handler = None
        if PLOT_MAP:
            # pass
            for idx in range(len(ordered_walls_averages)):
                plt.text(ordered_walls_averages[idx][0] + 0.5, ordered_walls_averages[idx][1], "wall " + str(idx))
                #plt.quiver(ordered_walls_averages[idx][0], ordered_walls_averages[idx][1],
                #                    math.cos(ordered_walls_averages[idx][3]), math.sin(ordered_walls_averages[idx][3]), width=0.003)
                obj = ordered_walls_averages[idx]
                s1 = [obj[0] - obj[4] / 2.0 * cos(obj[3]) , obj[1] - obj[4] / 2.0 * sin(obj[3])]
                s2 = [obj[0] + obj[4] / 2.0 * cos(obj[3]) , obj[1] + obj[4] / 2.0 * sin(obj[3])]
                #wall_center_handler = plt.plot([ordered_walls_averages[idx][0]], [ordered_walls_averages[idx][1]], '.k', lw=0.5)
                wall_detected_handler = plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], '--k', linewidth=wall_linewid/8.0)
                
        plot_handlers.append(wall_center_handler)
        plot_handlers.append(wall_detected_handler)
        # set the yaw of walls based on intersections
        
        if len(ordered_walls_averages) != 4:
            rospy.logerr("[BrickMapper]: there is only %d walls instead of 4" % (len(ordered_walls_averages)))
    
        return ordered_walls_averages, plot_handlers
    
    def line_params_from_pos_ang(self, object):
        """get a and b params of line from position and angle"""
        yaw = object[3]
        a = math.tan(yaw)
        b = object[1] - a * object[0]
        return a, b    
    
    def  line_intersect(self, a1, b1, a2, b2):
        """get intersetion of lines from a, b params"""
        if math.fabs(a1 - a2) < 0.001 :
            rospy.loginfo("[BrickMapper]: too close to intersection")
            return None, None

        x = (b2 - b1) / (a1 - a2)
        y = a1 * x + b1
        return x, y
    
    def calc_medians(self, object_list):
   
        med = []
        if len(object_list) > 0:
            objects_with_corrections = []
            for idx in range(len(object_list)):
                addition = object_list[idx][-1] * [object_list[idx][0:2]]
                objects_with_corrections += addition
                
            med = np.median(objects_with_corrections, axis=0)
        
        return med    
        
    def median_filter(self, wall_max_median_distance, brick_max_median_distance, ground_robot_max_median_distance , median_wall , median_bricks, median_ground_robot):
        """filter wall and objects by distance from median"""
        
        median_moved = True
        num_iter = 0
        new_typed_detections = copy.deepcopy(self.typed_detections)
        old_median_wall, old_median_bricks, old_median_ground_robot = median_wall, median_bricks, median_ground_robot
        while median_moved:
            
            new_typed_detections = copy.deepcopy(self.typed_detections)
            
            for det_type in new_typed_detections:
                
                if det_type in [RED, GREEN, BLUE] and len(list(median_bricks)) > 0:
                    for idx in range(len(new_typed_detections[det_type]) - 1, -1, -1): 
                        # dist = math.sqrt((new_typed_detections[det_type][idx][0] - median_bricks[0]) ** 2 + (new_typed_detections[det_type][idx][1] - median_bricks[1]) ** 2)
                        dist = self.dist_xy(new_typed_detections[det_type][idx], median_bricks)
                        # math.sqrt((new_typed_detections[det_type][idx][0] - median_bricks[0]) ** 2 + (new_typed_detections[det_type][idx][1] - median_bricks[1]) ** 2)
                        if dist > brick_max_median_distance:
                            del new_typed_detections[det_type][idx]
                    
                # print("median_wall", median_wall)
                if det_type is WALL and len(list(median_wall)) > 0:
                    for idx in range(len(new_typed_detections[det_type]) - 1, -1, -1): 
                        # dist = math.sqrt((new_typed_detections[det_type][idx][0] - median_wall[0]) ** 2 + (new_typed_detections[det_type][idx][1] - median_wall[1]) ** 2)
                        dist = self.dist_xy(new_typed_detections[det_type][idx], median_wall)
                        if dist > wall_max_median_distance:
                            del new_typed_detections[det_type][idx]
                            
                if det_type is GROUND_PLACING and len(list(median_wall)) > 0:
                    for idx in range(len(new_typed_detections[det_type]) - 1, -1, -1): 
                        # dist = math.sqrt((new_typed_detections[det_type][idx][0] - median_wall[0]) ** 2 + (new_typed_detections[det_type][idx][1] - median_wall[1]) ** 2)
                        dist = self.dist_xy(new_typed_detections[det_type][idx], median_ground_robot)
                        if dist > ground_robot_max_median_distance:
                            del new_typed_detections[det_type][idx]
                        
            all_bricks = new_typed_detections[RED] + new_typed_detections[GREEN] + new_typed_detections[BLUE]
            new_median_bricks = self.calc_medians(all_bricks)
            new_median_wall = self.calc_medians(new_typed_detections[WALL])
            new_median_ground_robot = self.calc_medians(new_typed_detections[GROUND_PLACING])
            
            dist_median_w = 0
            if len(median_wall) > 0:
                dist_median_w = self.dist_xy(new_median_wall, median_wall)
                rospy.loginfo("[BrickMapper]: median_wall %s, new_median_wall %s" % (str(median_wall), str(new_median_wall)))
                rospy.loginfo("[BrickMapper]: dist_median_w %s" % (str(dist_median_w)))
            
            dist_median_b = 0    
            if len(median_bricks) > 0:
                dist_median_b = self.dist_xy(new_median_bricks, median_bricks)
                rospy.loginfo("[BrickMapper]: median_bricks %s, new_median_bricks %s" % (str(median_bricks), str(new_median_bricks)))
                rospy.loginfo("[BrickMapper]: dist_median_b %s" % (str(dist_median_b)))
            
            dist_median_g = 0    
            if len(median_ground_robot) > 0:
                dist_median_g = self.dist_xy(new_median_ground_robot, median_ground_robot)
                rospy.loginfo("[BrickMapper]: median_ground_robot %s, new_median_ground_robot %s" % (str(median_ground_robot), str(new_median_ground_robot)))
                rospy.loginfo("[BrickMapper]: dist_median_g %s" % (str(dist_median_g)))
            
            num_iter += 1
            if num_iter >= self.optimize_median_till_count:
                median_moved = False
                break
            if dist_median_b <= self.optimize_median_till_distance and dist_median_w <= self.optimize_median_till_distance and dist_median_g <= self.brick_max_median_distance:
                median_moved = False
                break
            
            median_wall = new_median_wall
            median_bricks = new_median_bricks
            median_ground_robot = new_median_ground_robot
        
        self.typed_detections = new_typed_detections
    
    def fit_mean_detections(self, det_type, all_bricks_yaw):
        if len(self.typed_detections[det_type]) <= 1:
            return None
        objects = np.array(self.typed_detections[det_type])
        mean_objects = np.mean(objects, axis=0)
        yaw = self.normalize_yaw(all_bricks_yaw)  # np.mean(objects[:, 3], axis=0)
        
        # print("fit_mean_detections yaw is", yaw)
         
        # find max distance of object = len
        max_distance = -sys.float_info.max
        for obj in objects:
            s1 = [obj[0] - TYPED_LENGTH[det_type] / 2.0 * cos(obj[3]) , obj[1] - TYPED_LENGTH[det_type] / 2.0 * sin(obj[3])]
            s2 = [obj[0] + TYPED_LENGTH[det_type] / 2.0 * cos(obj[3]) , obj[1] + TYPED_LENGTH[det_type] / 2.0 * sin(obj[3])]
            dist1 = self.dist_xy(s1, mean_objects)
            dist2 = self.dist_xy(s2, mean_objects)
            if dist1 > max_distance:
                max_distance = dist1
            if dist2 > max_distance:
                max_distance = dist2
            
        brick_len = max_distance * 2.0
        typed_objects = [mean_objects[0], mean_objects[1], 0.2, yaw, brick_len]
        
        brick_line1 = None
        brick_line2 = None
        if PLOT_MAP:
            middlex = mean_objects[0]
            middley = mean_objects[1]
            fig = plt.gcf()
            width, hei = fig.get_size_inches()
            point_hei = hei * 72 
            x1, x2, y1, y2 = plt.axis()
            yrange = y2 - y1
            linewid = (0.3 * (point_hei / yrange)) * 0.8  # 0.3 equals to wall width
            if det_type in [RED, GREEN, BLUE]:
                linewid = (0.2 * (point_hei / yrange)) * 0.8  # 0.2 equals to brick width
            
            for obj in objects:
                s1 = [obj[0] - TYPED_LENGTH[det_type] / 2.0 * cos(obj[3]) , obj[1] - TYPED_LENGTH[det_type] / 2.0 * sin(obj[3])]
                s2 = [obj[0] + TYPED_LENGTH[det_type] / 2.0 * cos(obj[3]) , obj[1] + TYPED_LENGTH[det_type] / 2.0 * sin(obj[3])]
                plt.plot([s1[0] , s2[0]], [s1[1] , s2[1]], TYPED_COLOR[det_type] + '-', linewidth=linewid)
                
            # plt.plot(objects[:, 0], objects[:, 1], TYPED_COLOR[det_type] + '.', lw=0.5)
                
            minmaxx = [middlex - (self.bricks_place_length / 2.0) * cos(yaw), middlex + (self.bricks_place_length / 2.0) * cos(yaw)]
            minmaxy = [middley - (self.bricks_place_length / 2.0) * sin(yaw), middley + (self.bricks_place_length / 2.0) * sin(yaw)]
            plt.plot(minmaxx, minmaxy, '-' ,color='cyan', linewidth=linewid / 2.0)
            plt.plot(minmaxx, minmaxy, '--',color = TYPED_COLOR[det_type], linewidth=linewid / 4.0,dashes=(2.5, 5))
            
            brick_line1 = lines.Line2D([], [], linewidth=linewid / 2.0, linestyle="--", color='cyan')
            brick_line2 = lines.Line2D([], [], linewidth=linewid / 4.0, linestyle="-",dashes=(2.5, 5),color = TYPED_COLOR[det_type])

            arrow_size = 2
            x_dirs = arrow_size * math.cos(yaw)
            y_dirs = arrow_size * math.sin(yaw)
            # plt.quiver([middlex] , [middley], [x_dirs], [y_dirs], color=TYPED_COLOR[det_type])
            
        return typed_objects, (brick_line1, brick_line2)
    
    def fit_line_detections(self, det_type):
        """fit lines to detections"""
        
        if len(self.typed_detections[det_type]) <= 1:
            return None
        
        objects = np.array(self.typed_detections[det_type])
        params, params_covariance = optimize.curve_fit(test_func, objects[:, 0], objects[:, 1])
        yaw = math.atan(params[0])
        minx = min(objects[:, 0]) - math.cos(yaw) * TYPED_LENGTH[det_type] / 2.0
        maxx = max(objects[:, 0]) + math.cos(yaw) * TYPED_LENGTH[det_type] / 2.0
        middlex = (minx + maxx) / 2.0
        middley = test_func(middlex, params[0], params[1])
        
        typed_objects = [middlex, middley, 0.2, yaw]
        
        if PLOT_MAP:
            plt.plot(objects[:, 0], objects[:, 1], TYPED_COLOR[det_type] + '.')
            minmaxx = np.array([minx, maxx])
            plt.plot(minmaxx, test_func(minmaxx, params[0], params[1]), TYPED_COLOR[det_type])
            arrow_size = 2
            x_dirs = arrow_size * math.cos(yaw)
            y_dirs = arrow_size * math.sin(yaw)
            plt.quiver([middlex] , [middley], [x_dirs], [y_dirs], color=TYPED_COLOR[det_type])
            
        return typed_objects
    
    def run(self):
        """spin the wheel"""

        rospy.spin()


if __name__ == '__main__':
    brick_mapper = BrickMapper()
    brick_mapper.run()
