
/*******************************************************************************
#             uvccapture: USB UVC Video Class Snapshot Software                #
#This package work with the Logitech UVC based webcams with the mjpeg feature  #
#.                                                                             #
# 	Orginally Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard       #
#       Modifications Copyright (C) 2006  Gabriel A. Devenyi                   #
#                               (C) 2010  Alexandru Csete                      #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <jpeglib.h>
#include <time.h>
#include <linux/videodev2.h>

#include "v4l2uvc.h"

static const char version[] = VERSION;
int run = 1;

int main (int argc, char *argv[])
{
    char *videodevice = "/dev/video0";
    char *outputfile = "snap.jpg";
    char  thisfile[200]; /* used as filename buffer in multi-file seq. */
    char *post_capture_command[3];
    int format = V4L2_PIX_FMT_MJPEG;
    int grabmethod = 1;
    int width = 320;
    int height = 240;
    int brightness = 0, contrast = 0, saturation = 0, gain = 0;
    int num = -1; /* number of images to capture */
    int verbose = 0;
    int delay = 0;
    int skip = 0;
    int quality = 95;
    int post_capture_command_wait = 0;
    int multifile = 0;   /* flag indicating that we save to a multi-file sequence */
    int i = 0;

    time_t ref_time;
    struct vdIn *videoIn;
    FILE *file;

    //Reset all camera controls
    if (verbose >= 1)
        fprintf (stderr, "Resetting camera settings\n");


    ref_time = time (NULL);

    while (run) {
        if (verbose >= 2)
            fprintf (stderr, "Grabbing frame\n");
        if (uvcGrab (videoIn) < 0) {
            fprintf (stderr, "Error grabbing\n");
            close_v4l2 (videoIn);
            free (videoIn);
            exit (1);
        }

        if (skip > 0) { skip--; continue; }

        if ((difftime (time (NULL), ref_time) > delay) || delay == 0) {
            if (multifile == 1) {
                sprintf (thisfile, outputfile, i);
                i++;
                if (verbose >= 1)
                    fprintf (stderr, "Saving image to: %s\n", thisfile);
                file = fopen (thisfile, "wb");
            }
            else {
                if (verbose >= 1)
                    fprintf (stderr, "Saving image to: %s\n", outputfile);
                file = fopen (outputfile, "wb");
            }
            
            if (file != NULL) {
                switch (videoIn->formatIn) {
                case V4L2_PIX_FMT_YUYV:
                    compress_yuyv_to_jpeg (videoIn, file, quality);
                    break;
                default:
                    fwrite (videoIn->tmpbuffer, videoIn->buf.bytesused + DHT_SIZE, 1,
                            file);
                    break;
                }
                fclose (file);
                videoIn->getPict = 0;
            }
            if (post_capture_command[0]) {
                if (verbose >= 1)
                    fprintf (stderr, "Executing '%s %s'\n", post_capture_command[0],
                             post_capture_command[1]);
                if (spawn (post_capture_command, post_capture_command_wait, verbose)) {
                    fprintf (stderr, "Command exited with error\n");
                    close_v4l2 (videoIn);
                    free (videoIn);
                    exit (1);
                }
            }

            ref_time = time (NULL);
        }
        if ((delay == 0) || (num == i))
            break;
    }
    close_v4l2 (videoIn);
    free (videoIn);

    return 0;
}

