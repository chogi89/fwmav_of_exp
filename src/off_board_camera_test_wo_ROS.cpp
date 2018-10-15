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
#include <iomanip>
#include <stdlib.h>
#include <unistd.h>

#include "opencv2/imgproc/imgproc_c.h"


#define S_TIME      0.1

#define ALPHA       0.00003

#define PI          3.141592

#define O_WIDTH     800
#define O_HEIGHT    600

#define	WIDTH       80
#define HEIGHT	    60

#define WIDTH_H     80
#define HEIGHT_H    12

#define HEIGHT_H_O  24
#define WIDTH_V_O   32

#define D_SET       0.1

#define RL_P_GAIN   0.01
#define RL_D_GAIN   0.01
#define EPS_P_GAIN  1000
#define ETA_P_GAIN  0.8
#define ETA_D_GAIN  0

#define SIGMA_C_ETA 2.5
#define SIGMA_C_RL  2.5

#define SIGMA_M_ETA 20
#define SIGMA_M_RL  20

#define CO_FRQ_RL   0.5
#define CO_FRQ_ETA  5

using namespace cv;
using namespace std;

// ---------------------- //
// -- Grobal Variables -- //
// ---------------------- //

unsigned char Img_data[O_WIDTH*O_HEIGHT*3];

// ----------------------- //
// -- General Functions -- //
// ----------------------- //

double LPF(double y_p, double x_n, double tau);
double Saturation(double val, double sat);
double Sign(double val);
double PD_controller(double &error_c, double &error_p, double P_gain, double D_gain);
double Sigmoid_fnc(double sigma_m, double sigma_c, double val, int sgn);

// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //

// Enter the callback functions here...

// ------------------- //
// -- Main Function -- //
// ------------------- //

int main (int argc, char **argv){
    Mat mat_arrow_h(Size(WIDTH_H*100, HEIGHT_H*100),CV_8UC1,255);
    Mat mat_original;
    Mat mat_grey;
    Mat mat_resized;

    VideoCapture cap;

    char keypressed;

    // ----------------------------- //
    // -- Optical Flow Estimation -- //
    // ----------------------------- //

    uchar arr_gray_curr[WIDTH][HEIGHT];
    uchar arr_gray_prev_h[WIDTH_H][HEIGHT_H];
    uchar arr_gray_curr_h[WIDTH_H][HEIGHT_H];

    double Ix_h[WIDTH_H-1][HEIGHT_H-1];
    double Iy_h[WIDTH_H-1][HEIGHT_H-1];
    double It_h[WIDTH_H-1][HEIGHT_H-1];

    double u_h[WIDTH_H][HEIGHT_H];
    double v_h[WIDTH_H][HEIGHT_H];

    double mu_u_h[WIDTH_H-2][HEIGHT_H-2];
    double mu_v_h[WIDTH_H-2][HEIGHT_H-2];

    CvPoint p1_h[WIDTH_H][HEIGHT_H];
    CvPoint p2_h[WIDTH_H][HEIGHT_H];

    double r_x_h[WIDTH_H][HEIGHT_H];
    double r_y_h[WIDTH_H][HEIGHT_H];
    double r_h[WIDTH_H][HEIGHT_H];

    double eta_h[WIDTH_H][HEIGHT_H];

    // -------------------------------------- //
    // -- Obstacle Avoidance Guidance Rule -- //
    // -------------------------------------- //

    double OFright = 0;
    double OFleft = 0;

    double of_rl_e = 0;
    double of_rl_e_f = 0;
    double of_rl_e_f_p = 0;
    double of_rl_ctrl_input = 0;

    double eta_h_r = 0;
    double eta_h_l = 0;
    double eta_h_r_f = 0;
    double eta_h_l_f = 0;
    double eta_h_sum = 0;
    double eta_h_sum_f = 0;
    double eta_h_sum_f_p = 0;
    double eta_h_e = 0;
    double eta_h_ctrl_input = 0;
    double eta_h_ctrl_signed_input = 0;

    double sigmoid_eta = 0;
    double sigmoid_rl = 0;

    double d_vel_lx = 0;    // Desired linear velocity to x direction
    double d_vel_ly = 0;    // Desired linear velocity to y direction
    double d_vel_lz = 0;    // Desired linear velocity to z direction
    double d_vel_az = 0;    // Desired angular velocity to z direction(yaw angle)

    cap.open(-1);
	if(!cap.isOpened()){
		cout << "***Could not initialize capturing...***\n";
		return -1;
	}

    for(int i=0; i<(WIDTH_H-1); i++){
        for(int j=0; j<(HEIGHT_H-1); j++){
            p1_h[i][j] = cvPoint(i,j);
        }
    }

    for(int i=0; i<(WIDTH_H-1); i++){
        for(int j=0; j<(HEIGHT_H-1); j++){
            p1_h[i][j].x = (p1_h[i][j].x*100)+100;
            p1_h[i][j].y = (p1_h[i][j].y*100)+100;
        }
    }

    for(int i=0; i<(WIDTH_H); i++){
        for(int j=0; j<(HEIGHT_H); j++){
            r_x_h[i][j] = i-WIDTH_H/2+0.5;
            r_y_h[i][j] = j-HEIGHT_H/2+0.5;
            r_h[i][j] = sqrt(r_x_h[i][j]*r_x_h[i][j] + r_y_h[i][j]*r_y_h[i][j]);
        }
    }

    double count = 0;
	while (1){
        mat_arrow_h.setTo(255);
        mat_original = Mat(O_HEIGHT, O_WIDTH, CV_8UC3, &Img_data);

        cap >> mat_original;
        if(mat_original.empty()){
            break;
        }

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

		// ---------------------------------------- //
		// -- Horizental Optical Flow Estimation -- //
        // ---------------------------------------- //

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

        // -------------------------------------- //
        // -- Obstacle Avoidance Guidance Rule -- //
        // -------------------------------------- //

        OFright = 0;
        OFleft = 0;

        for (int i=0; i<((WIDTH_H/2)-2); i++){
            for(int j=0; j<(HEIGHT_H-2); j++){
                OFleft = OFleft + sqrt(u_h[i][j]*u_h[i][j]);
            }
        }
        for (int i=((WIDTH_H/2)); i<(WIDTH_H-2); i++){
            for(int j=0; j<(HEIGHT_H-2); j++){
                OFright = OFright + sqrt(u_h[i][j]*u_h[i][j]);
            }
        }

        for(int i=0; i<(WIDTH_H); i++){
		    for(int j=0; j<(HEIGHT_H); j++){
                eta_h[i][j] = (r_x_h[i][j]*u_h[i][j] + r_y_h[i][j]*v_h[i][j])/((r_h[i][j])*(r_h[i][j]));
		    }
		}

        eta_h_l = 0;
        eta_h_r = 0;
        for (int i=0; i<((WIDTH_H/2)); i++){
            for(int j=0; j<(HEIGHT_H); j++){
                eta_h_l = eta_h_l + eta_h[i][j];
            }
        }
        for (int i=((WIDTH_H/2)); i<(WIDTH_H); i++){
            for(int j=0; j<(HEIGHT_H); j++){
                eta_h_r = eta_h_r + eta_h[i][j];
            }
        }

        of_rl_e = OFright - OFleft;
        of_rl_e_f = LPF(of_rl_e_f, of_rl_e, CO_FRQ_RL);
        of_rl_ctrl_input = PD_controller(of_rl_e_f, of_rl_e_f_p, RL_P_GAIN, RL_D_GAIN)*S_TIME;

        eta_h_sum = eta_h_r + eta_h_l;
        eta_h_sum_f = LPF(eta_h_sum_f, eta_h_sum, CO_FRQ_ETA);
        eta_h_ctrl_input = PD_controller(eta_h_sum_f, eta_h_sum_f_p, ETA_P_GAIN, ETA_D_GAIN)*S_TIME;
        eta_h_e = eta_h_r - eta_h_l;
        eta_h_ctrl_signed_input = Sign(eta_h_e) * eta_h_ctrl_input;

        sigmoid_eta = Sigmoid_fnc(SIGMA_M_ETA,SIGMA_C_ETA,eta_h_sum_f,-1);
        sigmoid_rl = Sigmoid_fnc(SIGMA_M_RL,SIGMA_C_RL,eta_h_sum_f,1);

        d_vel_lx = D_SET;
        d_vel_ly = 0;
        d_vel_lz = 0;
        d_vel_az = (sigmoid_eta * eta_h_ctrl_signed_input) + (sigmoid_rl * of_rl_ctrl_input);

        // ------------- //
        // -- Display -- //
        // ------------- //

        for(int i=0; i<(WIDTH_H-1); i++){
            for(int j=0; j<(HEIGHT_H-1); j++){
                p2_h[i][j].x = p1_h[i][j].x+(int)(u_h[i][j]*20);
                p2_h[i][j].y = p1_h[i][j].y+(int)(v_h[i][j]*20);
            }
        }

        for(int i=0; i<(WIDTH_H-1); i++){
            for(int j=0; j<(HEIGHT_H-1); j++){
                arrowedLine(mat_arrow_h,p1_h[i][j],p2_h[i][j],0,3,CV_AA,0,1);
            }
        }
        cout << "---------------------------" << endl;
        cout << eta_h_ctrl_signed_input << endl;
        cout << of_rl_ctrl_input << endl;
        cout << d_vel_az <<endl;

		namedWindow("Image_original",WINDOW_NORMAL);
		imshow("Image_original",mat_original);
        namedWindow("Image_resized",WINDOW_NORMAL);
		imshow("Image_resized",mat_resized);
		namedWindow("Optical_flow_h",WINDOW_NORMAL);
		imshow("Optical_flow_h",mat_arrow_h);

		keypressed = (char)waitKey(10);
		if(keypressed == 27)
			break;
        sleep(0.1);
	}

	return 0;
}

double LPF(double y_p, double x_n, double tau){
    double y_f = 0;
    y_f = tau/(tau+S_TIME)*y_p + S_TIME/(tau+S_TIME)*x_n;
    return y_f;
}

double Saturation(double val, double sat){
    if(val>sat){
        val = sat;
    }
    if(val<(-1*sat)){
        val = -1*sat;
    }
    return val;
}

double Sign(double val){
    return (val/abs(val));
}

double PD_controller(double &error_c, double &error_p, double P_gain, double D_gain){
    double P_term = P_gain * error_c;
    double D_term = D_gain * (error_c - error_p)/S_TIME;
    error_p = error_c;
    return (P_term + D_term);
}

double Sigmoid_fnc(double sigma_m, double sigma_c, double val, int sgn){
    double sigmoid = 1/(1+exp(sgn*sigma_m*(val-sigma_c)));
    return sigmoid;
}
