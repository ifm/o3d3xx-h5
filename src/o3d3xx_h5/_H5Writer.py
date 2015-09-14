#
# Copyright (C) 2015 Love Park Robotics, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distribted on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""
A ROS node for capturing o3d3xx-ros camera data to HDF5.

"""
import os
import threading
import rospy
import h5py
import cv2
import simplejson
import numpy as np
from o3d3xx_h5 import mk_type
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from o3d3xx.srv import Dump


class H5Writer(object):

    def __init__(self):
        """
        Initializes the output HDF5 file
        """
        rospy.init_node('h5_writer')
        #----------------------
        # Get our runtime params from parameter server
        #----------------------
        self.outdir_ = rospy.get_param("~outdir")
        self.outfile_ = rospy.get_param("~outfile")
        self.open_mode_ = rospy.get_param("~open_mode", "w-")
        self.compression_ = rospy.get_param("~compression", 9)
        self.circular_buffer_sz_ = \
          rospy.get_param("~circular_buffer_sz", 999999999)

        #----------------------
        # Initialize our output file
        #----------------------
        try:
            if not os.path.isdir(self.outdir_):
                rospy.loginfo("Making output directory: %s" % self.outdir_)
                os.makedirs(self.outdir_)

        except OSError, ose:
            rospy.logwarn("Failed to make directory: %s" % self.outdir_)
            raise

        try:
            rospy.loginfo("Opening (%s): %s/%s" % \
                          (self.open_mode_, self.outdir_, self.outfile_))
            self.h5_ = h5py.File("%s/%s" % (self.outdir_, self.outfile_),
                                self.open_mode_)
        except IOError, ioe:
            rospy.logwarn("Failed to open: %s/%s" % \
                           (self.outdir_, self.outfile_))
            rospy.logwarn("Attempted open with mode: %s" % self.open_mode_)
            raise

        # create group names based on the topics we will subscribe to
        self.depth_grp_str_ = rospy.resolve_name("/depth")
        self.amplitude_grp_str_ = rospy.resolve_name("/amplitude")
        self.confidence_grp_str_ = rospy.resolve_name("/confidence")
        self.xyzi_grp_str_ = rospy.resolve_name("/xyzi_image")
        self.xyzi_grp_str_ = self.xyzi_grp_str_.replace("_image", "")

        # create or fetch the groups
        try:
            self.depth_grp_ = self.h5_[self.depth_grp_str_]
        except KeyError:
            rospy.loginfo("Creating group: %s" % self.depth_grp_str_)
            self.depth_grp_ = self.h5_.create_group(self.depth_grp_str_)

        try:
            self.amplitude_grp_ = self.h5_[self.amplitude_grp_str_]
        except KeyError:
            rospy.loginfo("Creating group: %s" % self.amplitude_grp_str_)
            self.amplitude_grp_ = \
                self.h5_.create_group(self.amplitude_grp_str_)

        try:
            self.confidence_grp_ = self.h5_[self.confidence_grp_str_]
        except KeyError:
            rospy.loginfo("Creating group: %s" % self.confidence_grp_str_)
            self.confidence_grp_ = \
                self.h5_.create_group(self.confidence_grp_str_)

        try:
            self.xyzi_grp_ = self.h5_[self.xyzi_grp_str_]
        except KeyError:
            rospy.loginfo("Creating group: %s" % self.xyzi_grp_str_)
            self.xyzi_grp_ = \
              self.h5_.create_group(self.xyzi_grp_str_)

        #----------------------
        # Create some counters for correlating different image types to
        # eachother. I was originally using the ROS topic's `seq` number
        # but this becomes problematic in real-world usage.
        #----------------------
        self.count_ = {
                       "depth": 0,
                       "depth_lock": threading.Lock(),
                       "confidence": 0,
                       "confidence_lock": threading.Lock(),
                       "amplitude": 0,
                       "amplitude_lock": threading.Lock(),
                       "xyzi": 0,
                       "xyzi_lock": threading.Lock()
                      }

        self.ds_name_ = {
                          "depth": self.depth_grp_str_,
                          "confidence": self.confidence_grp_str_,
                          "amplitude": self.amplitude_grp_str_,
                          "xyzi": self.xyzi_grp_str_,
                        }

        # initialize counters (mostly applies for when opening in 'a' mode)
        try:
            self.count_["depth"] = \
                int(sorted(self.depth_grp_.keys())[-1])+1
            self.count_["depth"] %= self.circular_buffer_sz_

        except:
            self.count_["depth"] = 0

        try:
            self.count_["amplitude"] = \
                int(sorted(self.amplitude_grp_.keys())[-1])+1
            self.count_["amplitude"] %= self.circular_buffer_sz_
        except:
            self.count_["amplitude"] = 0

        try:
            self.count_["confidence"] = \
                int(sorted(self.confidence_grp_.keys())[-1])+1
            self.count_["confidence"] %= self.circular_buffer_sz_
        except:
            self.count_["confidence"] = 0

        try:
            self.count_["xyzi"] = \
                int(sorted(self.xyzi_grp_.keys())[-1])+1
            self.count_["xyzi"] %= self.circular_buffer_sz_
        except:
            self.count_["xyzi"] = 0

        for t in ["depth", "amplitude", "confidence", "xyzi"]:
            rospy.loginfo("Initial %s data: %09d" % \
                          (t, self.count_[t]))

        #----------------------
        # Set group attributes by calling the `Dump` service on the
        # camera and parsing the JSON
        #----------------------

        self.set_group_attrs()

        #----------------------
        # Subscribed topics
        #----------------------
        self.bridge_ = CvBridge()

        rospy.loginfo("Initializing subscribers...")
        self.depth_sub_ = \
          rospy.Subscriber("/depth", Image, self.image_cb,
                           queue_size=None,
                           callback_args="depth")

        self.confidence_sub_ = \
          rospy.Subscriber("/confidence", Image, self.image_cb,
                           queue_size=None,
                           callback_args="confidence")

        self.amplitude_sub_ = \
          rospy.Subscriber("/amplitude", Image, self.image_cb,
                           queue_size=None,
                           callback_args="amplitude")

        self.xyzi_sub_ = \
          rospy.Subscriber("/xyzi_image", Image, self.image_cb,
                           queue_size=None,
                           callback_args="xyzi")

    def set_group_attrs(self):
        """
        Set attributes on the HDF5 objects based on the
        camera configuration.
        """
        rospy.loginfo("Calling `Dump` service...")
        rospy.wait_for_service("/Dump")
        dump_srv = rospy.ServiceProxy("/Dump", Dump)
        resp = dump_srv()
        json = simplejson.loads(resp.config)

        # get the "o3d3xx" and "camera" groups
        camera_grp = self.depth_grp_.parent
        o3d3xx_grp = camera_grp.parent

        # set the "o3d3xx" attributes
        o3d3xx_grp.attrs["o3d3xx.libo3d3xx"] = json["o3d3xx"]["libo3d3xx"]
        o3d3xx_grp.attrs["o3d3xx.Date"] = json["o3d3xx"]["Date"]

        for k in json["o3d3xx"]["HWInfo"].keys():
            key = "o3d3xx.HWInfo.%s" % k
            o3d3xx_grp.attrs[key] = \
              mk_type(key)(json["o3d3xx"]["HWInfo"][k])

        for k in json["o3d3xx"]["SWVersion"].keys():
            key = "o3d3xx.SWVersion.%s" % k
            o3d3xx_grp.attrs[key] = \
              mk_type(key)(json["o3d3xx"]["SWVersion"][k])

        # set the "camera" attributes
        for k in json["o3d3xx"]["Device"].keys():
            key = "o3d3xx.Device.%s" % k
            camera_grp.attrs[key] = \
              mk_type(key)(json["o3d3xx"]["Device"][k])

        for k in json["o3d3xx"]["Net"].keys():
            key = "o3d3xx.Net.%s" % k
            camera_grp.attrs[key] = \
              mk_type(key)(json["o3d3xx"]["Net"][k])

        active_app = int(json["o3d3xx"]["Device"]["ActiveApplication"])
        app_json = json["o3d3xx"]["Apps"][active_app - 1]
        for k in app_json.keys():
            if k in ['PcicTcpResultSchema',
                     'PcicEipResultSchema',
                     'LogicGraph',
                     'Imager']:
                continue
            key = "o3d3xx.App.%s" % k
            camera_grp.attrs[key] = mk_type(key)(app_json[k])

        imager_json = app_json["Imager"]
        for k in imager_json.keys():
            if ((k == 'SpatialFilter') or
                (k == 'TemporalFilter')):
                filter_json = imager_json[k]
                for fk in filter_json.keys():
                    key = "o3d3xx.App.Imager.%s.%s" % (k, fk)
                    camera_grp.attrs[key] = \
                      mk_type(key)(filter_json[fk])
            else:
                key = "o3d3xx.App.Imager.%s" % k
                camera_grp.attrs[key] = \
                  mk_type(key)(imager_json[k])

        return

    def image_cb(self, data, *args):
        """
        Callback to write image data to the HDF5 file
        """
        try:
            cv_img = self.bridge_.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            rospy.logwarn("Failed to deserialize image message: %s" % \
                          str(e))
            return

        im_type = args[0]
        with self.count_["%s_lock" % im_type]:
            ds_name = "%s/%09d" % \
              (self.ds_name_[im_type],
               self.count_[im_type] % self.circular_buffer_sz_)
            self.count_[im_type] += 1

        if ds_name in self.h5_:
            dset = self.h5_[ds_name]
            dset[...] = np.asarray(cv_img)
        else:
            dset = self.h5_.create_dataset(ds_name, data=np.asarray(cv_img),
                                           compression=self.compression_)

        dset.attrs['seq'] = data.header.seq
        dset.attrs['frame_id'] = data.header.frame_id
        dset.attrs['stamp.secs'] = data.header.stamp.secs
        dset.attrs['stamp.nsecs'] = data.header.stamp.nsecs
        dset.attrs['step'] = data.step

    def run(self):
        rospy.loginfo("Spinning...")
        rospy.spin()
