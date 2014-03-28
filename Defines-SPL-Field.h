#ifndef DEFINES_H
#define DEFINES_H

#define M2CM 100
#define CM2M 0.01

#define WIDTH 320
#define HEIGHT 240
enum CAMERAPOSITION{TOP_LEFT=0,TOP_MIDDLE,TOP_RIGHT,BOTTOM_LEFT,BOTTOM_MIDDLE,BOTTOM_RIGHT,POSITION_NUMBER};
enum ObjectId{UNKNOWN_OBJECT,BLUE_ROBOT,RED_ROBOT,BLACK_ROBOT,WHITE_ROBOT,ORANGE_BALL,GREEN_FIELD,REFEREE,NumberObjectID};
enum ObjectGuess{BALL_FIELD,ROBOT,REFEREE_GUESS};
#define SCALE 0.7
#define LINE_WIDTH_PIX 10 * SCALE
#define FIELD_BORDER_M 0.7
#define FIELD_WIDTH_M 9.0
#define FIELD_HEIGHT_M 6.0
#define CIRCLE_RADIUS_CM 75
#define FIELD_WIDTH (FIELD_WIDTH_M*M2CM*SCALE)
#define FIELD_HEIGHT (FIELD_HEIGHT_M*M2CM*SCALE)
#define FIELD_BORDER (FIELD_BORDER_M*M2CM*SCALE)
#define TOTAL_FIELD_WIDTH ((FIELD_WIDTH+FIELD_BORDER*2))
#define TOTAL_FIELD_HEIGHT ((FIELD_HEIGHT+FIELD_BORDER*2))
#define CENTER_X (FIELD_BORDER+FIELD_WIDTH/2)
#define	CENTER_Y (FIELD_BORDER+FIELD_HEIGHT/2)

#define CIRCLE_RADIUS_PIX (75 * SCALE)
#define ROBOT_SIZE_PIX 10
#define ROBOT_WIDTH_PIX 3
#define BALL_SIZE_PIX 5
#define BALL_WIDTH_PIX BALL_SIZE_PIX


#define ROBOT_WIDTH_M 0.25
#define ROBOT_DEPTH_M 0.08
#define PRINT_DEBUG cout<< "FILE : " << __FILE__ <<"LINE: " << __LINE__ << endl;

#define RED_H   0
#define RED_S   100
#define RED_V   50

#define BLUE_H   230
#define BLUE_S   100
#define BLUE_V   50

#define GREEN_S   120
#define GREEN_H   180
#define GREEN_V   50

#define ORANGE_H   30
#define ORANGE_S   100
#define ORANGE_V   50

#define HUE_THRESHOLD 40
#define UPPER_S 100
#define LOWER_S 0
#define UPPER_L 100
#define LOWER_L 0

#define TORSO_MIN_HEIGHT 0.3
#define TORSO_MAX_HEIGHT 0.5
#define MAX_POINTCLOUD_DIST 6
#define ON_THE_FIELD_THRESHOLD 0.20

#define BALL_CLUSTER_THRESHOLD 0.08
#define ROBOT_CLUSTER_THRESHOLD 0.2

#define BALL_MIN_POINT_NUM 8
#define BALL_MAX_POINT_NUM 100
#define ROBOT_MIN_POINT_NUM 10
#define ROBOT_MAX_POINT_NUM 40000
#define REFEREE_MIN_POINT_NUM 200

#define FOV_V 45
#define FOV_H 58

#define FIELD_ACTUAL_BORDER_Y_M 1
#define FIELD_ACTUAL_BORDER_X_M 0.3

#define In_THE_FIELD_Y_THRESHOLD (FIELD_WIDTH_M/2 /*+ FIELD_ACTUAL_BORDER_Y_M*/)
#define In_THE_FIELD_X_THRESHOLD (FIELD_HEIGHT_M/2.0 /*+ FIELD_ACTUAL_BORDER_X_M*/)

#define CAM_MIN_HEIGHT 0.8
#define MIN_COLOR_PIXELS 3
#endif // DEFINES_H

