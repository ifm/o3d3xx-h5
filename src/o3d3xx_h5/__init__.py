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
o3d3xx_h5
=========

o3d3xx_h5 provides a means to manage o3d3xx-ros camera data by utilizing HDF5
as a backend data store.

"""

def my_bool(v):
    v_ = str(v.upper())
    if ((v_ == "FALSE") or (v_ == "NO") or (v_ == "0")):
        return False

    return bool(v)

MK_TYPE = \
  {
    "o3d3xx.Device.ActiveApplication": int,
    "o3d3xx.Device.PcicEipEnabled": my_bool,
    "o3d3xx.Device.PcicTcpPort": int,
    "o3d3xx.Device.PcicProtocolVersion": int,
    "o3d3xx.Device.IOLogicType": int,
    "o3d3xx.Device.IODebouncing": my_bool,
    "o3d3xx.Device.IOExternApplicationSwitch": int,
    "o3d3xx.Device.SessionTimeout": int,
    "o3d3xx.Device.ServiceReportPassedBuffer": int,
    "o3d3xx.Device.ServiceReportFailedBuffer": int,
    "o3d3xx.Device.ExtrinsicCalibTransX": float,
    "o3d3xx.Device.ExtrinsicCalibTransY": float,
    "o3d3xx.Device.ExtrinsicCalibTransZ": float,
    "o3d3xx.Device.ExtrinsicCalibRotX": float,
    "o3d3xx.Device.ExtrinsicCalibRotY": float,
    "o3d3xx.Device.ExtrinsicCalibRotZ": float,
    "o3d3xx.Device.IPAddressConfig": int,
    "o3d3xx.Device.PasswordActivated": my_bool,
    "o3d3xx.Device.OperatingMode": int,
    "o3d3xx.Net.UseDHCP": my_bool,
    "o3d3xx.App.TriggerMode": int,
    "o3d3xx.App.PcicTcpResultOutputEnabled": my_bool,
    "o3d3xx.App.Index": int,
    "o3d3xx.App.Imager.Channel": int,
    "o3d3xx.App.Imager.ClippingBottom": int,
    "o3d3xx.App.Imager.ClippingTop": int,
    "o3d3xx.App.Imager.CliipingLeft": int,
    "o3d3xx.App.Imager.ClippingRight": int,
    "o3d3xx.App.Imager.ContinuousAutoExposure": my_bool,
    "o3d3xx.App.Imager.EnableAmplitudeCorrection": my_bool,
    "o3d3xx.App.Imager.EnableFilterAmplitudeImage": my_bool,
    "o3d3xx.App.Imager.EnableFilterDistanceImage": my_bool,
    "o3d3xx.App.Imager.EnableRectificationAmplitudeImage": my_bool,
    "o3d3xx.App.Imager.EnableRectificationDistanceImage": my_bool,
    "o3d3xx.App.Imager.ExposureTime": int,
    "o3d3xx.App.Imager.ExposureTimeRatio": int,
    "o3d3xx.App.Imager.FrameRate": int,
    "o3d3xx.App.Imager.MinimumAmplitude": int,
    "o3d3xx.App.Imager.ReduceMotionArtifacts": my_bool,
    "o3d3xx.App.Imager.SpatialFilterType": int,
    "o3d3xx.App.Imager.SymmetryThreshold": int,
    "o3d3xx.App.Imager.TemporalFilterType": int,
    "o3d3xx.App.Imager.ThreeFreqMax2FLineDistPercentage": int,
    "o3d3xx.App.Imager.ThreeFreqMax3FLineDistPercentage": int,
    "o3d3xx.App.Imager.TwoFreqMaxLineDistPercentage": int,
    "o3d3xx.App.Imager.SpatialFilter.Type": int,
    "o3d3xx.App.Imager.TemporalFilter.Type": int,
  }

def mk_type(prop):
    return MK_TYPE.get(prop, str)

from o3d3xx_h5._H5Writer import H5Writer
