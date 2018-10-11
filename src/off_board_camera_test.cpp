#include "ros/ros.h"
#include "fwmav_of_exp/MSG_NodeTime.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Image.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <iostream>
#include <fstream>
#include <ctype.h>
#include <math.h>

#define S_TIME      0.1

#define ALPHA       0.00003

#define PI          3.141592

#define O_WIDTH     800
#define O_HEIGHT    600

#define	WIDTH       80
#define HEIGHT	    60

#define WIDTH_H     80
#define HEIGHT_H    12

#define WIDTH_V     16
#define HEIGHT_V    60

#define HEIGHT_H_O  24
#define WIDTH_V_O   32

using namespace cv;
using namespace std;

// ---------------------- //
// -- Grobal Variables -- //
// ---------------------- //

unsigned char Img_data[O_WIDTH*O_HEIGHT*3];

// ----------------------- //
// -- General Functions -- //
// ----------------------- //

// Enter the general functions here...

// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //

void msgCallback_img(const sensor_msgs::Image::ConstPtr& Img){
    for(int i = 0; i<O_WIDTH*O_HEIGHT*3; i++){
        Img_data[i] = Img->data[i];
    }
}

// ------------------- //
// -- Main Function -- //
// ------------------- //

int main (int argc, char **argv){
    ros::init(argc, argv, "off_board_camer_test");
    ros::NodeHandle nh, nh_mavros, nh_image;

    ros::Publisher oa_of_pub = nh.advertise<fwmav_of_exp::MSG_NodeTime>("node_time",100);
    ros::Subscriber oa_of_sub_image = nh_image.subscribe("fwmav/camera/image_raw", 10, msgCallback_img);

    ros::Rate loop_rate(1/S_TIME);

    ofstream file_image_data("image_data.txt");

    Mat mat_arrow_h(Size(WIDTH_H*100, HEIGHT_H*100),CV_8UC1,255);
    Mat mat_arrow_v(Size(WIDTH_V*100, HEIGHT_V*100),CV_8UC1,255);
    Mat mat_original;
    Mat mat_grey;
    Mat mat_resized;

    char keypressed;

    uchar arr_gray_prev[WIDTH][HEIGHT];
    uchar arr_gray_curr[WIDTH][HEIGHT];

    uchar arr_gray_prev_h[WIDTH_H][HEIGHT_H];
    uchar arr_gray_curr_h[WIDTH_H][HEIGHT_H];

    uchar arr_gray_prev_v[WIDTH_V][HEIGHT_V];
    uchar arr_gray_curr_v[WIDTH_V][HEIGHT_V];

    double Ix_h[WIDTH_H-1][HEIGHT_H-1];
    double Iy_h[WIDTH_H-1][HEIGHT_H-1];
    double It_h[WIDTH_H-1][HEIGHT_H-1];

    double u_h[WIDTH_H][HEIGHT_H];
    double v_h[WIDTH_H][HEIGHT_H];

    double mu_u_h[WIDTH_H-2][HEIGHT_H-2];
    double mu_v_h[WIDTH_H-2][HEIGHT_H-2];

    CvPoint p1_h[WIDTH_H][HEIGHT_H];
    CvPoint p2_h[WIDTH_H][HEIGHT_H];

    double Ix_v[WIDTH_V-1][HEIGHT_V-1];
    double Iy_v[WIDTH_H-1][HEIGHT_V-1];
    double It_v[WIDTH_V-1][HEIGHT_V-1];

    double u_v[WIDTH_V][HEIGHT_V];
    double v_v[WIDTH_V][HEIGHT_V];

    double mu_u_v[WIDTH_V-1][HEIGHT_V-1];
    double mu_v_v[WIDTH_V-1][HEIGHT_V-1];

    CvPoint p1_v[WIDTH_V][HEIGHT_V];
    CvPoint p2_v[WIDTH_V][HEIGHT_V];

    for(int i=0; i<(WIDTH_H-1); i++){
        for(int j=0; j<(HEIGHT_H-1); j++){
            p1_h[i][j] = cvPoint(i,j);
        }
    }

    for(int i=0; i<(WIDTH_V-1); i++){
        for(int j=0; j<(HEIGHT_V-1); j++){
            p1_v[i][j] = cvPoint(i,j);
        }
    }

    for(int i=0; i<(WIDTH_H-1); i++){
        for(int j=0; j<(HEIGHT_H-1); j++){
            p1_h[i][j].x = (p1_h[i][j].x*100)+100;
            p1_h[i][j].y = (p1_h[i][j].y*100)+100;
        }
    }

    for(int i=0; i<(WIDTH_V-1); i++){
        for(int j=0; j<(HEIGHT_V-1); j++){
            p1_v[i][j].x = (p1_v[i][j].x*100)+100;
            p1_v[i][j].y = (p1_v[i][j].y*100)+100;
        }
    }

    double count = 0;
	while (ros::ok()){
        mat_arrow_h.setTo(255);
        mat_arrow_v.setTo(255);
        mat_original = Mat(O_HEIGHT, O_WIDTH, CV_8UC3, &Img_data);

        cvtColor(mat_original, mat_grey, CV_RGB2GRAY);
		resize(mat_grey, mat_resized, Size(WIDTH, HEIGHT));

        // ------------------------- //
		// -- Save Previous Image -- //
		// ------------------------- //

		for(int i=0; i<(WIDTH_H); i++){
			for(int j=0; j<(HEIGHT_H); j++){
				arr_gray_prev_h[i][j] = arr_gray_curr_h[i][j];
			}
		}

		for(int i=0; i<(WIDTH_V); i++){
			for(int j=0; j<(HEIGHT_V); j++){
				arr_gray_prev_v[i][j] = arr_gray_curr_v[i][j];
			}
		}

		// -------------------- //
		// -- Save New Image -- //
		// -------------------- //

        for(int i=0; i<(WIDTH); i++){
            for(int j=0; j<(HEIGHT); j++){
                arr_gray_curr[i][j] = mat_resized.at<uchar>(j,i);
            }
        }

        // ------------------- //
		// -- Image Cutting -- //
		// ------------------- //

		for(int i=0; i<WIDTH_H; i++){
            for(int j=0; j<HEIGHT_H; j++){
                arr_gray_curr_h[i][j] = arr_gray_curr[i][HEIGHT_H_O+j];
            }
        }

        for(int i=0; i<WIDTH_V; i++){
            for(int j=0; j<HEIGHT_V; j++){
                arr_gray_curr_v[i][j] = arr_gray_curr[WIDTH_V_O+i][j];
            }
        }

		// ----------------------------------------- //
		// -- Horizental Optical Flow Calculation -- //
        // ----------------------------------------- //

		for(int i=0; i<(WIDTH_H-1); i++){
			for(int j=0; j<(HEIGHT_H-1); j++){
				Ix_h[i][j] = (arr_gray_prev_h[i+1][j] - arr_gray_prev_h[i][j] + arr_gray_prev_h[i+1][j+1] - arr_gray_prev_h[i][j+1] + arr_gray_curr_h[i+1][j] - arr_gray_curr_h[i][j] + arr_gray_curr_h[i+1][j+1] - arr_gray_curr_h[i][j+1])/4;
				Iy_h[i][j] = (arr_gray_prev_h[i][j+1] - arr_gray_prev_h[i][j] + arr_gray_prev_h[i+1][j+1] - arr_gray_prev_h[i+1][j] + arr_gray_curr_h[i][j+1] - arr_gray_curr_h[i][j] + arr_gray_curr_h[i+1][j+1] - arr_gray_curr_h[i+1][j])/4;
				It_h[i][j] = (arr_gray_curr_h[i][j] - arr_gray_prev_h[i][j] + arr_gray_curr_h[i+1][j] - arr_gray_prev_h[i+1][j] + arr_gray_curr_h[i][j+1] - arr_gray_prev_h[i][j+1] + arr_gray_curr_h[i+1][j+1] - arr_gray_prev_h[i+1][j+1])/4;
			}
		}

		for(int i=0; i<(WIDTH_H-2); i++){
		    for(int j=0; j<(HEIGHT_H-2); j++){
                mu_u_h[i][j] = (u_h[i][j+1] + u_h[i+1][j] + u_h[i+2][j+1] + u_h[i+1][j+2])/6 + (u_h[i][j] + u_h[i][j+2] + u_h[i+2][j] + u_h[i+2][j+2])/12;
                mu_v_h[i][j] = (v_h[i][j+1] + v_h[i+1][j] + v_h[i+2][j+1] + v_h[i+1][j+2])/6 + (v_h[i][j] + v_h[i][j+2] + v_h[i+2][j] + v_h[i+2][j+2])/12;
		    }
		}

		for(int i=0; i<(WIDTH_H-2); i++){
		    for(int j=0; j<(HEIGHT_H-2); j++){
                u_h[i+1][j+1] = mu_u_h[i][j] - Ix_h[i][j]*((Ix_h[i][j]*mu_u_h[i][j] + Iy_h[i][j]*mu_v_h[i][j] + It_h[i][j])/(ALPHA*ALPHA + Ix_h[i][j]*Ix_h[i][j] + Iy_h[i][j]*Iy_h[i][j]));
                v_h[i+1][j+1] = mu_v_h[i][j] - Iy_h[i][j]*((Ix_h[i][j]*mu_u_h[i][j] + Iy_h[i][j]*mu_v_h[i][j] + It_h[i][j])/(ALPHA*ALPHA + Ix_h[i][j]*Ix_h[i][j] + Iy_h[i][j]*Iy_h[i][j]));
		    }
		}

		// --------------------------------------- //
		// -- Vertical Optical Flow Calculation -- //
        // --------------------------------------- //

		for(int i=0; i<(WIDTH_V-1); i++){
			for(int j=0; j<(HEIGHT_V-1); j++){
				Ix_v[i][j] = (arr_gray_prev_v[i+1][j] - arr_gray_prev_v[i][j] + arr_gray_prev_v[i+1][j+1] - arr_gray_prev_v[i][j+1] + arr_gray_curr_v[i+1][j] - arr_gray_curr_v[i][j] + arr_gray_curr_v[i+1][j+1] - arr_gray_curr_v[i][j+1])/4;
				Iy_v[i][j] = (arr_gray_prev_v[i][j+1] - arr_gray_prev_v[i][j] + arr_gray_prev_v[i+1][j+1] - arr_gray_prev_v[i+1][j] + arr_gray_curr_v[i][j+1] - arr_gray_curr_v[i][j] + arr_gray_curr_v[i+1][j+1] - arr_gray_curr_v[i+1][j])/4;
				It_v[i][j] = (arr_gray_curr_v[i][j] - arr_gray_prev_v[i][j] + arr_gray_curr_v[i+1][j] - arr_gray_prev_v[i+1][j] + arr_gray_curr_v[i][j+1] - arr_gray_prev_v[i][j+1] + arr_gray_curr_v[i+1][j+1] - arr_gray_prev_v[i+1][j+1])/4;
			}
		}

		for(int i=0; i<(WIDTH_V-2); i++){
		    for(int j=0; j<(HEIGHT_V-2); j++){
                mu_u_v[i][j] = (u_v[i][j+1] + u_v[i+1][j] + u_v[i+2][j+1] + u_v[i+1][j+2])/6 + (u_v[i][j] + u_v[i][j+2] + u_v[i+2][j] + u_v[i+2][j+2])/12;
                mu_v_v[i][j] = (v_v[i][j+1] + v_v[i+1][j] + v_v[i+2][j+1] + v_v[i+1][j+2])/6 + (v_v[i][j] + v_v[i][j+2] + v_v[i+2][j] + v_v[i+2][j+2])/12;
		    }
		}

		for(int i=0; i<(WIDTH_V-2); i++){
		    for(int j=0; j<(HEIGHT_V-2); j++){
                u_v[i+1][j+1] = mu_u_v[i][j] - Ix_v[i][j]*((Ix_v[i][j]*mu_u_v[i][j] + Iy_v[i][j]*mu_v_v[i][j] + It_v[i][j])/(ALPHA*ALPHA + Ix_v[i][j]*Ix_v[i][j] + Iy_v[i][j]*Iy_v[i][j]));
                v_v[i+1][j+1] = mu_v_v[i][j] - Iy_v[i][j]*((Ix_v[i][j]*mu_u_v[i][j] + Iy_v[i][j]*mu_v_v[i][j] + It_v[i][j])/(ALPHA*ALPHA + Ix_v[i][j]*Ix_v[i][j] + Iy_v[i][j]*Iy_v[i][j]));
		    }
		}

        // ------------- //
        // -- Display -- //
        // ------------- //

        for(int i=0; i<(WIDTH_H-1); i++){
            for(int j=0; j<(HEIGHT_H-1); j++){
                p2_h[i][j].x = p1_h[i][j].x+(int)(u_h[i][j]*20);
                p2_h[i][j].y = p1_h[i][j].y+(int)(v_h[i][j]*20);
            }
        }

		for(int i=0; i<(WIDTH_V-1); i++){
		    for(int j=0; j<(HEIGHT_V-1); j++){
                p2_v[i][j].x = p1_v[i][j].x+(int)(u_v[i][j]*20);
                p2_v[i][j].y = p1_v[i][j].y+(int)(v_v[i][j]*20);
		    }
		}

        for(int i=0; i<(WIDTH_H-1); i++){
            for(int j=0; j<(HEIGHT_H-1); j++){
                arrowedLine(mat_arrow_h,p1_h[i][j],p2_h[i][j],0,3,CV_AA,0,1);
            }
        }

		for(int i=0; i<(WIDTH_V-1); i++){
		    for(int j=0; j<(HEIGHT_V-1); j++){
                arrowedLine(mat_arrow_v,p1_v[i][j],p2_v[i][j],0,3,CV_AA,0,1);
		    }
		}

		namedWindow("Image_original",WINDOW_NORMAL);
		imshow("Image_original",mat_original);
        //namedWindow("Image_grey",WINDOW_NORMAL);
		//imshow("Image_grey",mat_grey);
        namedWindow("Image_resized",WINDOW_NORMAL);
		imshow("Image_resized",mat_resized);
		namedWindow("Optical_flow_h",WINDOW_NORMAL);
		imshow("Optical_flow_h",mat_arrow_h);
		namedWindow("Optical_flow_v",WINDOW_NORMAL);
		imshow("Optical_flow_v",mat_arrow_v);

		// --------------- //
        // -- Data Save -- //
		// --------------- //

        //// Image Data Save
        file_image_data << count << ", ";
        for(int i=0; i<(WIDTH); i++){
		    for(int j=0; j<(HEIGHT); j++){
                file_image_data << (int)arr_gray_curr[i][j] << ", ";
		    }
		}
        file_image_data << endl;

		keypressed = (char)waitKey(10);
		if(keypressed == 27)
			break;

        fwmav_of_exp::MSG_NodeTime msg_node_time;

		msg_node_time.data = count;

		oa_of_pub.publish(msg_node_time);

        ROS_INFO(" ");
        ROS_INFO("-------------------------------");
		ROS_INFO("Send msg = %f", count);
		ROS_INFO("-------------------------------");

        ros::spinOnce();
        loop_rate.sleep();
		count = count + S_TIME;
	}

	file_image_data.close();

	return 0;
}
