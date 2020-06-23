#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
#    Copyright (C) 2013 by Jerome Maye                                     #
#    jerome.maye@mavt.ethz.ch                                              #
#                                                                          #
#    All rights reserved.                                                  #
#                                                                          #
#    Redistribution and use in source and binary forms, with or without    #
#    modification, are permitted provided that the following conditions    #
#    are met:                                                              #
#                                                                          #
#    1. Redistributions of source code must retain the above copyright     #
#       notice, this list of conditions and the following disclaimer.      #
#                                                                          #
#    2. Redistributions in binary form must reproduce the above copyright  #
#       notice, this list of conditions and the following disclaimer in    #
#       the documentation and/or other materials provided with the         #
#       distribution.                                                      #
#                                                                          #
#    3. The name of the copyright holders may be used to endorse or        #
#       promote products derived from this software without specific       #
#       prior written permission.                                          #
#                                                                          #
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     #
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        #
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  #
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  #
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      #
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    #
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN     #
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       #
#    POSSIBILITY OF SUCH DAMAGE.                                           #
############################################################################

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import sys
import rospy
import socket
from subprocess import Popen, PIPE

import time

import re

from mbot_diagnostics import Status
from mbot_diagnostics_ros import DiagnosticUpdater, GenericDiagnostic

NAME = 'ntp_monitor'

def ntp_monitor(namespace, offset=500, self_offset=500, diag_hostname = None, error_offset = 5000000):
    rospy.init_node(NAME, anonymous=True)
    diag_updater = DiagnosticUpdater(
        name=namespace + 'ntp',
        display_name=diag_hostname + ' NTP',
    )

    hostname = socket.gethostname()
    if diag_hostname is None:
        diag_hostname = hostname

    ntp_hostname = rospy.get_param('~reference_host', 'ntp.ubuntu.com')
    offset = rospy.get_param('~offset_tolerance', 500)
    error_offset = rospy.get_param('~error_offset_tolerance', 5000000)

    stat = DiagnosticStatus()
    stat.level = 0
    stat.name = "NTP Offset"
    stat.message = "OK"
    stat.hardware_id = hostname
    stat.values = []
    stat_diagnostic = GenericDiagnostic('/offset')
    stat_diagnostic.add_to_updater(diag_updater)

#    self_stat = DiagnosticStatus()
#    self_stat.level = DiagnosticStatus.OK
#    self_stat.name = "NTP self-offset for "+ diag_hostname
#    self_stat.message = "OK"
#    self_stat.hardware_id = hostname
#    self_stat.values = []

    while not rospy.is_shutdown():
        for st,host,off in [(stat,ntp_hostname,offset)]:
            try:
                p = Popen(["ntpdate", "-q", host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
                res = p.wait()
                (o,e) = p.communicate()
            except OSError, (errno, msg):
                if errno == 4:
                    break #ctrl-c interrupt
                else:
                    raise
            if (res == 0):
                measured_offset = float(re.search("offset (.*),", o).group(1))*1000000
                st.level = DiagnosticStatus.OK
                st.message = "OK"
                st.values = [ KeyValue("Offset (us)", str(measured_offset)),
                              KeyValue("Offset tolerance (us)", str(off)),
                              KeyValue("Offset tolerance (us) for Error", str(error_offset)) ]

                if (abs(measured_offset) > off):
                    st.level = DiagnosticStatus.WARN
                    st.message = "NTP offset too high"
                if (abs(measured_offset) > error_offset):
                    st.level = DiagnosticStatus.ERROR
                    st.message = "NTP offset too high"

            else:
                # Warning (not error), since this is a non-critical failure.
                st.level = DiagnosticStatus.WARN
                st.message = "Error running ntpdate (returned %d)" % res
                st.values = [ KeyValue("Offset (us)", "N/A"),
                              KeyValue("Offset tolerance (us)", str(off)),
                              KeyValue("Offset tolerance (us) for Error", str(error_offset)),
                              KeyValue("Output", o),
                              KeyValue("Errors", e) ]


        # Convert from ROS diagnostics to mbot_diagnostics for publishing.
        stat_diagnostic.set_status(Status(stat.level), stat.message)
        for diag_val in stat.values:
            stat_diagnostic.set_metric(diag_val.key, diag_val.value)

        time.sleep(1)

def ntp_monitor_main(argv=sys.argv):
    import optparse
    parser = optparse.OptionParser(usage="usage: ntp_monitor --diag-hostname=com-X []")
    parser.add_option("--offset-tolerance", dest="offset_tol",
                      action="store", default=500,
                      help="Offset from NTP host", metavar="OFFSET-TOL")
    parser.add_option("--error-offset-tolerance", dest="error_offset_tol",
                      action="store", default=5000000,
                      help="Offset from NTP host. Above this is error", metavar="OFFSET-TOL")
    parser.add_option("--self_offset-tolerance", dest="self_offset_tol",
                      action="store", default=500,
                      help="Offset from self", metavar="SELF_OFFSET-TOL")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'com-1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default='com-?')
    options, args = parser.parse_args(rospy.myargv())

#    if (len(args) != 2):
#        parser.error("Invalid arguments. Must have HOSTNAME [args]. %s" % args)


    try:
        offset = int(options.offset_tol)
        self_offset = int(options.self_offset_tol)
        error_offset = int(options.error_offset_tol)
    except:
        parser.error("Offsets must be numbers")

    namespace = rospy.get_namespace() or hostname

    ntp_monitor(namespace, offset, self_offset, options.diag_hostname, error_offset)


if __name__ == "__main__":
    try:
        ntp_monitor_main(rospy.myargv())
    except KeyboardInterrupt: pass
    except SystemExit: pass
    except:
        import traceback
        traceback.print_exc()
