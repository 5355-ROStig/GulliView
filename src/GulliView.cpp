/*********************************************************************
 * This file is distributed as part of the C++ port of the APRIL tags
 * library. The code is licensed under GPLv2.
 *
 * Original author: Edwin Olson <ebolson@umich.edu>
 * C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
 * ----------------------- Modified ---------------------------------e
 * Code modified for project in Vision Based Localization for
 * Autonomous Vehicles at Chalmers University, Goteborg, Sweden
 * Modification Authors:
 * Copyright (c) 2013-2014 Andrew Soderberg-Rivkin <sandrew@student.chalmers.se>
 * Copyright (c) 2013-2014 Sanjana Hangal <sanjana@student.chalmers.se>
 * Copyright (c) 2014 Thomas Petig <petig@chalmers.se>
 ********************************************************************/

#include "AprilTypes.h"
#include "TagFamily.h"

#include <ctime>
#include <iostream>
#include <cstdio>
#include <getopt.h>
#include <cstring>
#include <cstdlib>
#include <string>
#include <csignal>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <boost/chrono/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <boost/chrono/process_cpu_clocks.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/chrono/floor.hpp>
#include <boost/chrono/round.hpp>

#include "CameraUtil.h"

#include  <chrono>

#include "apriltag/apriltag.h"

#define DEFAULT_TAG_FAMILY "tag36h11"
#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_PORT "2121"
using namespace std;
using boost::asio::ip::udp;
using boost::posix_time::ptime;
using boost::posix_time::time_duration;


sig_atomic_t sig_stop = 0;

void signal_handler(int param) {
    sig_stop = 1;
}

typedef struct __attribute__ ((packed)) DetectionMessage {
    uint32_t id;
    uint32_t x;
    uint32_t y;
    uint32_t camera_id;
} DetectionMessage;

typedef struct __attribute__ ((packed)) Message {
    uint32_t type;
    uint32_t subtype;
    uint32_t seq;
    uint64_t time_msec;
    uint64_t UNUSED;
    uint32_t length;
    DetectionMessage detections[14];
} Message;

typedef struct GulliViewOptions {
    GulliViewOptions() :
            family_str(DEFAULT_TAG_FAMILY),
            error_fraction(1),
            device_num(0),
            focal_length(500),
            tag_size(0.1905),
            frame_width(0),
            frame_height(0),
            /* Changed to False so that text comes out correctly. */
            /* Issues with detection when set to False */
            mirror_display(false),
            no_gui(false),
            // *ADDED: Default value for IP address and port number to server
            ip(DEFAULT_IP),
            broadcast(false),
            port(DEFAULT_PORT) {
    }

    std::string family_str;
    double error_fraction;
    int device_num;
    double focal_length;
    double tag_size;
    int frame_width;
    int frame_height;
    bool mirror_display;
    bool no_gui;
    // *ADDED: Variables for storing IP address and port number to server
    std::string ip;
    bool broadcast;
    std::string port;
} GulliViewOptions;


void print_usage(const char *tool_name, FILE *output = stderr) {

    //TagDetectorParams p;
    GulliViewOptions o;

    fprintf(output, "\
Usage: %s [OPTIONS]\n\
GulliView Program used for tag detection on Autonomous Vehicles. Options:\n\
 -h              Show this help message.\n\
 -f FAMILY       Look for the given tag family (default \"%s\")\n\
 -d DEVICE       Set camera device number (default %d)\n\
 -z SIZE         Set the tag size in meters (default %f)\n\
 -W WIDTH        Set the camera image width in pixels\n\
 -H HEIGHT       Set the camera image height in pixels\n\
 -M              Toggle display mirroring\n\
 -n              No gui\n\n\
 -V              Server IP-address\n\
 -B              Enable broadcast (use when broadcast IP is given for -V flag)\n\
 -N              Server Port number (Default: 2121)\n",
            tool_name,
            /* Options removed that are not needed */
            /* Can be added later for further functionality */
            //p.sigma,
            //p.segSigma,
            //p.thetaThresh,
            //p.magThresh,
            //p.adaptiveThresholdValue,
            //p.adaptiveThresholdRadius,
            DEFAULT_TAG_FAMILY,
            //o.error_fraction,
            o.device_num,
            //o.focal_length,
            o.tag_size);


    fprintf(output, "Known tag families:");

    for (std::string& t : TagFamily::getFamilyNames()) {
        fprintf(output, " %s", t.c_str());
    }
    fprintf(output, "\n");
    /* Old Options removed can be re-added if they are needed. Default values set for now:
     * -D              Use decimation for segmentation stage.\n\
     * -S SIGMA        Set the original image sigma value (default %.2f).\n\
     * -s SEGSIGMA     Set the segmentation sigma value (default %.2f).\n\
     * -a THETATHRESH  Set the theta threshold for clustering (default %.1f).\n\
     * -m MAGTHRESH    Set the magnitude threshold for clustering (default %.1f).\n\
     * -V VALUE        Set adaptive threshold value for new quad algo (default %f).\n\
     * -N RADIUS       Set adaptive threshold radius for new quad algo (default %d).\n\
     * -b              Refine bad quads using template tracker.\n\
     * -r              Refine all quads using template tracker.\n\
     * -n              Use the new quad detection algorithm.\n\
     * -e FRACTION     Set error detection fraction (default %f)\n\
     * -F FLENGTH      Set the camera's focal length in pixels (default %f)\n\
     */
}

GulliViewOptions parse_options(int argc, char **argv) {
    GulliViewOptions opts;
    const char *options_str = "hDS:s:a:m:V:BN:brnf:e:d:F:z:W:H:M";
    int c;
    while ((c = getopt(argc, argv, options_str)) != -1) {
        switch (c) {
            // Reminder: add new options to 'options_str' above and print_usage()!
            case 'h':
                print_usage(argv[0], stdout);
                exit(0);
                break;
                //case 'D': opts.params.segDecimate = true; break;
                //case 'S': opts.params.sigma = atof(optarg); break;
                //case 's': opts.params.segSigma = atof(optarg); break;
                //case 'a': opts.params.thetaThresh = atof(optarg); break;
                //case 'm': opts.params.magThresh = atof(optarg); break;
                //case 'V': opts.params.adaptiveThresholdValue = atof(optarg); break;
                //case 'N': opts.params.adaptiveThresholdRadius = atoi(optarg); break;
                //case 'b': opts.params.refineBad = true; break;
                //case 'r': opts.params.refineQuads = true; break;
                //case 'n': opts.params.newQuadAlgorithm = true; break;
            case 'f':
                opts.family_str = optarg;
                break;
                //case 'e': opts.error_fraction = atof(optarg); break;
            case 'd':
                opts.device_num = atoi(optarg);
                break;
                //case 'F': opts.focal_length = atof(optarg); break;
            case 'z':
                opts.tag_size = atof(optarg);
                break;
            case 'W':
                opts.frame_width = atoi(optarg);
                break;
            case 'H':
                opts.frame_height = atoi(optarg);
                break;
            case 'M':
                opts.mirror_display = !opts.mirror_display;
                break;
            case 'n':
                opts.no_gui = 1;
                break;
                // *ADDED: Flags for providing IP address and port number to server
            case 'V' :
                opts.ip = optarg;
                break;
            case 'B' :
                opts.broadcast = !opts.broadcast;
                break;
            case 'N' :
                opts.port = optarg;
                break;
            default:
                fprintf(stderr, "\n");
                print_usage(argv[0], stderr);
                exit(1);
        }
    }
    // opts.params.adaptiveThresholdRadius += (opts.params.adaptiveThresholdRadius + 1) % 2;
    return opts;
}

cv::Mat perspectiveTransform(int camera) {
    /* Setting pts for this camera,
   going throght the camreas in order from the back wall and towards the door ---------------------------------- */
    at::Point source_points_pts[4];
    at::Point dest_points_pts[4];


    if (camera == 3) {
        /* Camera 3 (1:4) */
        source_points_pts[0] = at::Point(132.93, 85.90);
        source_points_pts[1] = at::Point(661.55, 82.38);
        source_points_pts[3] = at::Point(99.88, 421.53);
        source_points_pts[2] = at::Point(703.45, 420.75);
        dest_points_pts[0] =  at::Point(1.00, 1.00);
        dest_points_pts[1] =  at::Point(4.00, 1.00);
        dest_points_pts[3] =  at::Point(1.00, 2.77);
        dest_points_pts[2] =  at::Point(4.00, 2.82);
    } else if (camera == 0) {
        /* Camera 0 (2:4) */
        source_points_pts[0] = at::Point(672.00, 418.80);
        source_points_pts[1] = at::Point(115.75, 431.65);
        source_points_pts[3] = at::Point(681.00, 30.69);
        source_points_pts[2] = at::Point(86.90, 36.00);
        dest_points_pts[0] =  at::Point(1.00, 2.77);
        dest_points_pts[1] =  at::Point(4.00, 2.82);
        dest_points_pts[3] =  at::Point(1.00, 4.83);
        dest_points_pts[2] =  at::Point(4.00, 4.86);
    } else if (camera == 2) {
        /* Camera 2 (3:4) */
        source_points_pts[0] = at::Point(102.75, 17.36);
        source_points_pts[1] = at::Point(672.40, 18.45);
        source_points_pts[3] = at::Point(99.22, 428.17);
        source_points_pts[2] = at::Point(681.79, 423.80);
        dest_points_pts[0] =  at::Point(1.00, 4.83);
        dest_points_pts[1] =  at::Point(4.00, 4.86);
        dest_points_pts[3] =  at::Point(1.00, 6.97);
        dest_points_pts[2] =  at::Point(4.00, 6.965);
    } else if (camera == 1) {
        /* Camera 1 (4:4) */
        source_points_pts[0] = at::Point(690.96, 429.82);
        source_points_pts[1] = at::Point(93.40, 431.02);
        source_points_pts[3] = at::Point(667.82, 40.81);
        source_points_pts[2] = at::Point(119.83, 37.85);
        dest_points_pts[0] =  at::Point(1.00, 6.93);
        dest_points_pts[1] =  at::Point(4.00, 6.965);
        dest_points_pts[3] =  at::Point(1.00, 9.00);
        dest_points_pts[2] =  at::Point(4.00, 9.00);
    } else {
        throw std::runtime_error("Unknown camera ID");
    }

    return getPerspectiveTransform(source_points_pts, dest_points_pts);
}

int main(int argc, char **argv) {
    // Doing graceful shutdown, prevents Linux USB system from crashing
    signal(SIGINT, signal_handler);

    GulliViewOptions opts = parse_options(argc, argv);
    at::Mat pts = perspectiveTransform(opts.device_num);

    // Multiply by two due to each camera having two /devs
    cv::VideoCapture vc(opts.device_num * 2, cv::CAP_V4L2);
    vc.set(cv::CAP_PROP_BUFFERSIZE, 2);

    if (opts.frame_width && opts.frame_height) {
        // Use uvcdynctrl to figure this out dynamically at some point?
        vc.set(cv::CAP_PROP_FRAME_WIDTH, opts.frame_width);
        vc.set(cv::CAP_PROP_FRAME_HEIGHT, opts.frame_height);
    }

    std::cout << "Set camera to resolution: "
              << vc.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << vc.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";

    cv::Mat frame, gray;

    /* Optical Center of video capturing frame with X and Y coordinates */
    cv::Point2d opticalCenter(frame.cols * 0.5, frame.rows * 0.5);

    vc >> frame;
    if (frame.empty()) {
        std::cerr << "no frames!\n";
        exit(1);
    }

    std::string win = "GulliViewer";
    if (not opts.no_gui) {
        cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    }

    TagFamily family(opts.family_str);

    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family.at_family);
    detector->quad_decimate = 1.0f;
    detector->quad_sigma = 0.6f;

    boost::asio::io_service io_service;
    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), opts.ip, opts.port);
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    udp::socket socket(io_service);
    socket.open(udp::v4());

    if (opts.broadcast) {
        socket.set_option(boost::asio::socket_base::broadcast(true));
    }

    cv::Mat k1 = (cv::Mat1d(3, 3) << 927.42805517, 0.0, 401.59811614, 0, 850.04900153, 225.08468986, 0, 0, 1);
    cv::Mat d1 = (cv::Mat1d(1, 5) << 0.24592604, -1.97913584, -0.01938124, 0.00740747, 2.37610561);
    cv::Mat opt1 = cv::getOptimalNewCameraMatrix(k1, d1, frame.size(), 0);

    uint32_t seq = 0;
    while (true) {
        ptime fetch_start = boost::posix_time::microsec_clock::universal_time();
        vc >> frame;

        cv::Mat ret;
        cv::undistort(frame, ret, k1, d1, opt1);
        frame = ret.clone();

        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        if (frame.empty()) {
            break;
        }

        image_u8_t im = {
                gray.cols,
                gray.rows,
                gray.cols,
                gray.data
        };

        zarray_t *detections = apriltag_detector_detect(detector, &im);

        if (zarray_size(detections) == 0) {
            string idToText = "---Nothing Detected---";
            putText(frame, idToText,
                    cv::Point(30, 30),
                    cv::FONT_HERSHEY_PLAIN,
                    1.5, cv::Scalar(180, 250, 0), 1, cv::LINE_AA);
        } else {
            // Get time of frame/detection----------------
            //show = family.superimposeDetections(frame, detections); //-- Used to actually
            //superimpose tag image in video

            double s = opts.tag_size;
            double ss = 0.5 * s;
            /* sz changed to negative value to flip cube */
            double sz = -s;

            enum {
                npoints = 8, nedges = 12
            };

            /* Cube flipped by changing sz to negative */
            cv::Point3d src[npoints] = {
                    cv::Point3d(-ss, -ss, 0),
                    cv::Point3d(ss, -ss, 0),
                    cv::Point3d(ss, ss, 0),
                    cv::Point3d(-ss, ss, 0),
                    cv::Point3d(-ss, -ss, sz),
                    cv::Point3d(ss, -ss, sz),
                    cv::Point3d(ss, ss, sz),
                    cv::Point3d(-ss, ss, sz),
            };

            /* Possible edges of the box created. Come back to THIS*/
            int edges[nedges][2] = {
                    {0, 1},
                    {1, 2},
                    {2, 3},
                    {3, 0},

                    /* Comment out two matrices below for 2D box */
                    {4, 5},
                    {5, 6},
                    {6, 7},
                    {7, 4},

                    {0, 4},
                    {1, 5},
                    {2, 6},
                    {3, 7}
            };

            cv::Point2d dst[npoints];

            double f = opts.focal_length;
            /* Optical centers, possible 2D config*/
            double K[9] = {
                    f, 0, opticalCenter.x,
                    0, f, opticalCenter.y,
                    0, 0, 1
            };

            cv::Mat_<cv::Point3d> srcmat(npoints, 1, src);
            cv::Mat_<cv::Point2d> dstmat(npoints, 1, dst);

            cv::Mat_<double> Kmat(3, 3, K);

            cv::Mat_<double> distCoeffs = cv::Mat_<double>::zeros(4, 1);

            static ptime epoch(boost::gregorian::date(1970, 1, 1));
            uint64_t msecs = (fetch_start - epoch).total_milliseconds();

            Message buf {
                htobe32(1) /* type */ ,
                htobe32(2) /* subtype */,
                htobe32(seq) /* seq */,
                htobe64(msecs) /* time_msec */
            };

            size_t index = 0;

            std::vector<at::Point> rawDetections(zarray_size(detections));
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                rawDetections[i] = at::Point(dd->c[0], dd->c[1]);
            }
            std::vector<at::Point> newDetections(zarray_size(detections));
            perspectiveTransform(rawDetections, newDetections, pts);

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);

                at::Point cxy = rawDetections[i];

                if (dd->id == 0) {
                    putText(frame, "0,0",
                            cv::Point(cxy.x, cxy.y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1.0, cv::Scalar(0, 0, 250), 2, cv::LINE_AA);
                    std::cout << "0_x: " << cxy.x << " 0_y: " << cxy.y << "\n";
                    // New X-Axis detected
                } else if (dd->id == 1) {
                    putText(frame, "X Axis",
                            cv::Point(cxy.x, cxy.y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1.0, cv::Scalar(0, 0, 250), 2, cv::LINE_AA);
                    std::cout << "X_x: " << cxy.x << " X_y: " << cxy.y << "\n";
                    // New Y-Axis detected
                } else if (dd->id == 2) {
                    putText(frame, "Y Axis",
                            cv::Point(cxy.x, cxy.y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1.0, cv::Scalar(0, 0, 250), 2, cv::LINE_AA);
                    std::cout << "Y_x: " << cxy.x << " Y_y: " << cxy.y << "\n";
                    // Quad Angle used for perspective transform
                } else if (dd->id == 3) {
                    putText(frame, "Quad Axis",
                            cv::Point(cxy.x, cxy.y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1.0, cv::Scalar(0, 0, 250), 2, cv::LINE_AA);
                    std::cout << "Quad_x: " << cxy.x << " Quad_y: " << cxy.y << "\n";
                } else {
                    // Print out Tag ID in center of Tag
                    putText(frame, std::to_string(dd->id),
                            cv::Point(cxy.x, cxy.y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1.0, cv::Scalar(0, 250, 0), 2, cv::LINE_AA);

                    int32_t x_coord = (int32_t) (newDetections[i].x * 1000.0);
                    int32_t y_coord = (int32_t) (newDetections[i].y * 1000.0);

                    buf.detections[index++] = {
                            htobe32(dd->id) /* id */,
                            htobe32(x_coord) /* x */,
                            htobe32(y_coord) /* y */,
                            htobe32(opts.device_num) /* camera_id */
                    };

                    std::cout << "camera: " << opts.device_num << " tag: " << dd->id << " x: " << x_coord << " y: "
                              << y_coord << std::endl;
                }

                cv::Mat_<double> homography(dd->H->nrows, dd->H->ncols, dd->H->data);

                cv::Mat_<double> M =
                        CameraUtil::homographyToPose(f, f, s,
                                                     homography,
                                                     false);

                cv::Mat_<double> R = M.rowRange(0, 3).colRange(0, 3);
                cv::Mat t = M.rowRange(0, 3).col(3);
                cv::Mat r;
                cv::Rodrigues(R, r);
                cv::projectPoints(srcmat, r, t, Kmat, distCoeffs, dstmat);

                /* Used to draw lines on video image */
                for (auto &edge: edges) {
                    cv::line(frame,
                             dstmat(edge[0], 0),
                             dstmat(edge[1], 0),
                             CV_RGB(255, 0, 0),
                             1, cv::LINE_AA);
                }
            }

            buf.length = htobe32(index);
            socket.send_to(boost::asio::buffer((uint8_t*) &buf, 256), receiver_endpoint);
            ++seq;
        }

        if (opts.mirror_display)
            cv::flip(frame, frame, 1);

        if (not opts.no_gui) {
            cv::imshow(win, frame);
            cv::waitKey(1);
        }

        apriltag_detections_destroy(detections);

        if (sig_stop)
            break;
    }

    apriltag_detector_destroy(detector);

    return 0;
}
