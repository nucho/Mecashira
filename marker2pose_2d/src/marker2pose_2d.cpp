#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <termios.h>
#include <term.h>
#include <curses.h>

#include <math.h>

static struct termios initial_settings, new_settings;
static int peek_character = -1;

void init_keyboard()
{
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &initial_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

int kbhit()
{
    char ch;
    int nread;

    if(peek_character != -1)
        return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);

    if(nread == 1) {
        peek_character = ch;
        return 1;
    }
    return 0;
}


int readch()
{
    char ch;

    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0, &ch, 1);
    return ch;
}   



int main(int argc, char** argv){
	double offset_x,offset_y,offset_th;

  offset_x=0;
  offset_y=0;
  offset_th=0;

  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  tf::TransformListener listener;
  tf::TransformBroadcaster br;

  ros::Rate rate(10.0);

	int ch=0;
 
	init_keyboard();
	while (node.ok()){
		tf::StampedTransform transform;
    try
    {
			listener.lookupTransform("/ar_marker", "/usb_cam",  ros::Time(0), transform);

			double now_x = transform.getOrigin().x();
			double now_y = transform.getOrigin().y();
			double now_th = tf::getYaw(transform.getRotation());

			if(kbhit())
			{
				ch = readch();
				ROS_INFO("you hit %X : %c",ch,ch);

				offset_x = now_x * cos(-now_th) - now_y * sin(-now_th);
				offset_y = now_x * sin(-now_th) + now_y * cos(-now_th);
				offset_th= now_th;
			}

			double diff_th = -(now_th-offset_th);

			double rot_x = now_x * cos(-now_th) - now_y * sin(-now_th) - offset_x;
			double rot_y = now_x * sin(-now_th) + now_y * cos(-now_th) - offset_y;

			double rot_x2 = rot_x * cos(offset_th) - rot_y * sin(offset_th);
			double rot_y2 = rot_x * sin(offset_th) + rot_y * cos(offset_th);

			ROS_INFO("x:%f\ty=%f\tyaw:%f", rot_x2, rot_y2,diff_th*180/M_PI);
        
			transform.setOrigin( tf::Vector3(rot_x2, rot_y2, 0.0) );
 			transform.setRotation( tf::Quaternion(diff_th, 0, 0) );
        
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ar_marker", "robot_position"));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    rate.sleep();
    
    if(ch==0x03){
        close_keyboard();
        break;
    }
  }
  return 0;
};

