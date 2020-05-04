"""
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
*/

///////////////////////////////////////////////////////////////////////////////
//
// setup the simulation in rviz
//
///////////////////////////////////////////////////////////////////////////////
"""
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

server = None
menu_handler = MenuHandler()
br = None
counter = 0

def frameCallback( msg ):
    global counter, br
    br.sendTransform( (0, 0, counter*.001), (0, 0, 0, 1.0), rospy.Time.now(), "base_link", "P1" )
    br.sendTransform( (0, 0, counter*-.001), (0, 0, 0, 1.0), rospy.Time.now(), "base_link", "P2" )
    counter += 1
    print(counter)

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def makeChessPieceMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "chess_piece"
    int_marker.description = "Chess Piece\n(2D Move + Alignment)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    control.markers.append( makeBox(int_marker) )
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, processFeedback)

    # set different callback for POSE_UPDATE feedback
    #server.setCallback(int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )

def makeMovingMarkerP1(position):
    int_marker = Marker()
    int_marker.header.frame_id = "P1"
    int_marker.pose.position = position
    int_marker.scale = 1
    int_marker.ns= "P1"
    int_marker.id= 1

    int_marker.description = "Player 1"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append( makeBox(int_marker) )
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

def makeMovingMarkerP2(position):
    int_marker = Marker()
    int_marker.header.frame_id = "P2"
    int_marker.pose.position = position
    int_marker.scale = 1
    int_marker.ns = "P2"
    int_marker.id = 2


    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append( makeBox(int_marker) )
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

if __name__=="__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    
    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)
    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert( "First Entry", callback=processFeedback )
    menu_handler.insert( "Second Entry", callback=processFeedback )
    sub_menu_handle = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )
   
    position = Point( 3, -6, 0)
    makeMovingMarkerP1( position )

    position = Point( 3, -6, 0)
    makeMovingMarkerP2( position )

    server.applyChanges()

    rospy.spin()
