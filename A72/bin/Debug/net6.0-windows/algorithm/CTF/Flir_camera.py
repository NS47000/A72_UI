# coding=utf-8
# =============================================================================
# Copyright (c) 2001-2021 FLIR Systems, Inc. All Rights Reserved.
#
# This software is the confidential and proprietary information of FLIR
# Integrated Imaging Solutions, Inc. ("Confidential Information"). You
# shall not disclose such Confidential Information and shall use it only in
# accordance with the terms of the license agreement you entered into
# with FLIR Integrated Imaging Solutions, Inc. (FLIR).
#
# FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
# SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
# SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
# THIS SOFTWARE OR ITS DERIVATIVES.
# =============================================================================
#
#  Exposure_QuickSpin.py shows how to customize image exposure time
#  using the QuickSpin API. QuickSpin is a subset of the Spinnaker library
#  that allows for simpler node access and control.
#
#  This example prepares the camera, sets a new exposure time, and restores
#  the camera to its default state. Ensuring custom values fall within an
#  acceptable range is also touched on. Retrieving and setting information
#  is the only portion of the example that differs from Exposure.
#
#  A much wider range of topics is covered in the full Spinnaker examples than
#  in the QuickSpin ones. There are only enough QuickSpin examples to
#  demonstrate node access and to get started with the API; please see full
#  Spinnaker examples for further or specific knowledge on a topic.

import PySpin
import sys

NUM_IMAGES = 1  # number of images to save

def configure_custom_image_settings(nodemap):
    """
    Configures a number of settings on the camera including offsets  X and Y, width,
    height, and pixel format. These settings must be applied before BeginAcquisition()
    is called; otherwise, they will be read only. Also, it is important to note that
    settings are applied immediately. This means if you plan to reduce the width and
    move the x offset accordingly, you need to apply such changes in the appropriate order.

    :param nodemap: GenICam nodemap.
    :type nodemap: INodeMap
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    print('\n*** CONFIGURING CUSTOM IMAGE SETTINGS *** \n')

    try:
        result = True

        # Apply mono 8 pixel format
        #
        # *** NOTES ***
        # Enumeration nodes are slightly more complicated to set than other
        # nodes. This is because setting an enumeration node requires working
        # with two nodes instead of the usual one.
        #
        # As such, there are a number of steps to setting an enumeration node:
        # retrieve the enumeration node from the nodemap, retrieve the desired
        # entry node from the enumeration node, retrieve the integer value from
        # the entry node, and set the new value of the enumeration node with
        # the integer value from the entry node.
        #
        # Retrieve the enumeration node from the nodemap
        node_pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
        if PySpin.IsAvailable(node_pixel_format) and PySpin.IsWritable(node_pixel_format):

            # Retrieve the desired entry node from the enumeration node
            node_pixel_format_mono10 = PySpin.CEnumEntryPtr(node_pixel_format.GetEntryByName('Mono10'))
            if PySpin.IsAvailable(node_pixel_format_mono10) and PySpin.IsReadable(node_pixel_format_mono10):

                # Retrieve the integer value from the entry node
                pixel_format_mono8 = node_pixel_format_mono10.GetValue()

                # Set integer as new value for enumeration node
                node_pixel_format.SetIntValue(pixel_format_mono8)

                print('Pixel format set to %s...' % node_pixel_format.GetCurrentEntry().GetSymbolic())

            else:
                print('Pixel format mono 8 not available...')

        else:
            print('Pixel format not available...')

        # Apply minimum to offset X
        #
        # *** NOTES ***
        # Numeric nodes have both a minimum and maximum. A minimum is retrieved
        # with the method GetMin(). Sometimes it can be important to check
        # minimums to ensure that your desired value is within range.
        node_offset_x = PySpin.CIntegerPtr(nodemap.GetNode('OffsetX'))
        if PySpin.IsAvailable(node_offset_x) and PySpin.IsWritable(node_offset_x):

            node_offset_x.SetValue(node_offset_x.GetMin())
            print('Offset X set to %i...' % node_offset_x.GetMin())
            
        else:
            print('Offset X not available...')

        # Apply minimum to offset Y
        #
        # *** NOTES ***
        # It is often desirable to check the increment as well. The increment
        # is a number of which a desired value must be a multiple of. Certain
        # nodes, such as those corresponding to offsets X and Y, have an
        # increment of 1, which basically means that any value within range
        # is appropriate. The increment is retrieved with the method GetInc().
        node_offset_y = PySpin.CIntegerPtr(nodemap.GetNode('OffsetY'))
        if PySpin.IsAvailable(node_offset_y) and PySpin.IsWritable(node_offset_y):

            node_offset_y.SetValue(node_offset_y.GetMin())
            print('Offset Y set to %i...' % node_offset_y.GetMin())

        else:
            print('Offset Y not available...')

        # Set maximum width
        #
        # *** NOTES ***
        # Other nodes, such as those corresponding to image width and height,
        # might have an increment other than 1. In these cases, it can be
        # important to check that the desired value is a multiple of the
        # increment. However, as these values are being set to the maximum,
        # there is no reason to check against the increment.
        node_width = PySpin.CIntegerPtr(nodemap.GetNode('Width'))
        if PySpin.IsAvailable(node_width) and PySpin.IsWritable(node_width):

            width_to_set = node_width.GetMax()
            node_width.SetValue(width_to_set)
            print('Width set to %i...' % node_width.GetValue())
            
        else:
            print('Width not available...')

        # Set maximum height
        #
        # *** NOTES ***
        # A maximum is retrieved with the method GetMax(). A node's minimum and
        # maximum should always be a multiple of its increment.
        node_height = PySpin.CIntegerPtr(nodemap.GetNode('Height'))
        if PySpin.IsAvailable(node_height) and PySpin.IsWritable(node_height):

            height_to_set = node_height.GetMax()
            node_height.SetValue(height_to_set)
            print('Height set to %i...' % node_height.GetValue())

        else:
            print('Height not available...')

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result

def configure_exposure(cam,exptime):
    """
     This function configures a custom exposure time. Automatic exposure is turned
     off in order to allow for the customization, and then the custom setting is
     applied.

     :param cam: Camera to configure exposure for.
     :type cam: CameraPtr
     :return: True if successful, False otherwise.
     :rtype: bool
    """

    print('*** CONFIGURING EXPOSURE ***\n')

    try:
        result = True

        # Turn off automatic exposure mode
        #
        # *** NOTES ***
        # Automatic exposure prevents the manual configuration of exposure
        # times and needs to be turned off for this example. Enumerations
        # representing entry nodes have been added to QuickSpin. This allows
        # for the much easier setting of enumeration nodes to new values.
        #
        # The naming convention of QuickSpin enums is the name of the
        # enumeration node followed by an underscore and the symbolic of
        # the entry node. Selecting "Off" on the "ExposureAuto" node is
        # thus named "ExposureAuto_Off".
        #
        # *** LATER ***
        # Exposure time can be set automatically or manually as needed. This
        # example turns automatic exposure off to set it manually and back
        # on to return the camera to its default state.

        if cam.ExposureAuto.GetAccessMode() != PySpin.RW:
            print('Unable to disable automatic exposure. Aborting...')
            return False

        cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        print('Automatic exposure disabled...')

        # Set exposure time manually; exposure time recorded in microseconds
        #
        # *** NOTES ***
        # Notice that the node is checked for availability and writability
        # prior to the setting of the node. In QuickSpin, availability and
        # writability are ensured by checking the access mode.
        #
        # Further, it is ensured that the desired exposure time does not exceed
        # the maximum. Exposure time is counted in microseconds - this can be
        # found out either by retrieving the unit with the GetUnit() method or
        # by checking SpinView.
        # Retrieve node (Enumeration node in this case)



        if cam.ExposureTime.GetAccessMode() != PySpin.RW:
            print('Unable to set exposure time. Aborting...')
            return False

        # Ensure desired exposure time does not exceed the maximum
        exposure_time_to_set = exptime
        exposure_time_to_set = min(cam.ExposureTime.GetMax(), exposure_time_to_set)
        cam.ExposureTime.SetValue(exposure_time_to_set)
        print('Shutter time set to %s us...\n' % exposure_time_to_set)

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result

def configure_gain_gamma(cam,gain):
    result=True
    try:
        # Retrieve node (Enumeration node in this case)
        
        cam.GainAuto.SetValue(PySpin.GainAuto_Off)
        print('Automatic exposure disabled...')
        cam.Gain.SetValue(gain)
        #cam.GammaEnable.SetValue(0)
        #cam.Gamma.SetValue(1)
        
        
        print("========set gain success=============")
    
    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False
    

    return result
    

def reset_exposure(cam):
    """
    This function returns the camera to a normal state by re-enabling automatic exposure.

    :param cam: Camera to reset exposure on.
    :type cam: CameraPtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Turn automatic exposure back on
        #
        # *** NOTES ***
        # Automatic exposure is turned on in order to return the camera to its
        # default state.

        if cam.ExposureAuto.GetAccessMode() != PySpin.RW:
            print('Unable to enable automatic exposure (node retrieval). Non-fatal error...')
            return False

        cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Continuous)

        print('Automatic exposure enabled...')

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result


def print_device_info(cam):
    """
    This function prints the device information of the camera from the transport
    layer; please see NodeMapInfo example for more in-depth comments on printing
    device information from the nodemap.

    :param cam: Camera to get device information from.
    :type cam: CameraPtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """

    print('*** DEVICE INFORMATION ***\n')

    try:
        result = True
        nodemap = cam.GetTLDeviceNodeMap()

        node_device_information = PySpin.CCategoryPtr(nodemap.GetNode('DeviceInformation'))

        if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
            features = node_device_information.GetFeatures()
            for feature in features:
                node_feature = PySpin.CValuePtr(feature)
                print('%s: %s' % (node_feature.GetName(),
                                  node_feature.ToString() if PySpin.IsReadable(node_feature) else 'Node not readable'))

        else:
            print('Device control information not available.')

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex.message)
        return False

    return result


def acquire_images(cam,image_name):
    """
    This function acquires and saves 10 images from a device; please see
    Acquisition example for more in-depth comments on the acquisition of images.

    :param cam: Camera to acquire images from.
    :type cam: CameraPtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    print('*** IMAGE ACQUISITION ***')

    try:
        result = True

        # Set acquisition mode to continuous
        if cam.AcquisitionMode.GetAccessMode() != PySpin.RW:
            print('Unable to set acquisition mode to continuous. Aborting...')
            return False

        cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
        print('Acquisition mode set to continuous...')

        # Begin acquiring images
        cam.BeginAcquisition()

        print('Acquiring images...')

        # Get device serial number for filename
        device_serial_number = ''
        if cam.TLDevice.DeviceSerialNumber is not None and cam.TLDevice.DeviceSerialNumber.GetAccessMode() == PySpin.RO:
            device_serial_number = cam.TLDevice.DeviceSerialNumber.GetValue()

            print('Device serial number retrieved as %s...' % device_serial_number)

        # Get the value of exposure time to set an appropriate timeout for GetNextImage
        print("debug1")
        timeout = 0
        if cam.ExposureTime.GetAccessMode() == PySpin.RW or cam.ExposureTime.GetAccessMode() == PySpin.RO:
            # The exposure time is retrieved in µs so it needs to be converted to ms to keep consistency with the unit being used in GetNextImage
            timeout = (int)(cam.ExposureTime.GetValue() / 1000 + 2000)
        else:
            print ('Unable to get exposure time. Aborting...')
            return False

        # Retrieve, convert, and save images
        print("debug2")
        i=image_name
        try:
            # Retrieve next received image and ensure image completion
            # By default, GetNextImage will block indefinitely until an image arrives.
            # In this example, the timeout value is set to [exposure time + 1000]ms to ensure that an image has enough time to arrive under normal conditions
            image_result = cam.GetNextImage(timeout)
            print("debug3")
            if image_result.IsIncomplete():
                print('Image incomplete with image status %d...' % image_result.GetImageStatus())

            else:
                # Print image information
                width = image_result.GetWidth()
                height = image_result.GetHeight()
                print('Grabbed Image %s, width = %d, height = %d' % (i, width, height))

                # Convert image to Mono8
                image_converted = image_result.Convert(PySpin.PixelFormat_Mono8)

                # Create a unique filename
                filename = i

                # Save image
                image_converted.Save(filename)

                print('Image saved at %s' % filename)

            # Release image
            image_result.Release()

        except PySpin.SpinnakerException as ex:
            print('Error: %s' % ex)
            result = False

        # End acquisition
        cam.EndAcquisition()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result


def run_single_camera(cam,image_name,exp_time,gain):
    """
     This function acts as the body of the example; please see NodeMapInfo_QuickSpin example for more
     in-depth comments on setting up cameras.

     :param cam: Camera to run example on.
     :type cam: CameraPtr
     :return: True if successful, False otherwise.
     :rtype: bool
    """
    try:
        # Initialize camera
        cam.Init()

        # Print device info
        result = print_device_info(cam)

        # Configure exposure
        if not configure_exposure(cam,exp_time):
            return False
        #Mason for gamma adjust
        if not configure_gain_gamma(cam,gain):
            return False
        # Retrieve GenICam nodemap
        #nodemap = cam.GetNodeMap()

        # Configure custom image settings
        #if not configure_custom_image_settings(nodemap):
        #    return False
        # Acquire images
        result &= acquire_images(cam,image_name)

        # Reset exposure
        #result &= reset_exposure(cam)

        # Deinitialize camera
        cam.DeInit()

        return result

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False


def getpicture(image_name,exp_time,gain):
    """
    Example entry point; please see Enumeration_QuickSpin example for more
    in-depth comments on preparing and cleaning up the system.

    :return: True if successful, False otherwise.
    :rtype: bool
    """
    result = True

    # Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()

    # Get current library version
    version = system.GetLibraryVersion()
    print('Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

    # Retrieve list of cameras from the system
    cam_list = system.GetCameras()

    num_cameras = cam_list.GetSize()

    print('Number of cameras detected: %d' % num_cameras)

    # Finish if there are no cameras
    if num_cameras == 0:
        # Clear camera list before releasing system
        cam_list.Clear()

        # Release system instance
        system.ReleaseInstance()

        print('Not enough cameras!')
        input('Done! Press Enter to exit...')
        return False

    # Run example on each camera
    for i, cam in enumerate(cam_list):

        print('Running example for camera %d...' % i)

        result &= run_single_camera(cam,image_name,exp_time,gain)
        print('Camera %d example complete... \n' % i)

    # Release reference to camera
    # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
    # cleaned up when going out of scope.
    # The usage of del is preferred to assigning the variable to None.
    del cam

    # Clear camera list before releasing system
    cam_list.Clear()

    # Release system instance
    system.ReleaseInstance()

    
    return result

if __name__ == '__main__':
    getpicture("masontry.PNG",66666,0)
        
