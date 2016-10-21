///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, Code laboratories, Inc.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

// Config parameters
#include <duo3d_driver/Duo3DConfig.h>

// Include DUO API header file
#include "DUOLib.h"

using namespace std;
using namespace cv;

#define NODE_NAME   "duo3d"

namespace duo3d_driver
{
// topic items
enum { LEFT, RIGHT, ITEM_COUNT };
const vector<string> prefix =
{
    "left", "right"
};

// parameter names
const vector<string> topic_param_name =
{
    prefix[LEFT] + "_topic",
    prefix[RIGHT] + "_topic",
};
const vector<string> cam_info_topic_param_name =
{
    prefix[LEFT] + "_cam_info_topic",
    prefix[RIGHT] + "_cam_info_topic",
};
const vector<string> frame_id_param_name =
{
    prefix[LEFT] + "_frame_id",
    prefix[RIGHT] + "_frame_id",
};

// parameter default values
vector<string> topic_name =
{
    prefix[LEFT] + "/image_raw",
    prefix[RIGHT] + "/image_raw",
};
vector<string> cam_info_topic_name =
{
    prefix[LEFT] + "/camera_info",
    prefix[RIGHT] + "/camera_info",
};
vector<string> frame_id_name =
{
    string(NODE_NAME) + "/camera_frame",      // LEFT
    string(NODE_NAME) + "/camera_frame",      // RIGHT
};

// DUO3DDriver class
class DUO3DDriver
{
    DUOInstance _duoInstance;

    ros::NodeHandle _nh;
    // Dynamic reconfigure server
    dynamic_reconfigure::Server<Duo3DConfig> _server;

    // Camera Parameters
    float _frame_rate;
    vector<int> _image_size;

    double _start_time;
    uint32_t _frame_num;

    // Image publishers
    image_transport::Publisher _pub_image[ITEM_COUNT];
    // Camera info publishers
    ros::Publisher _pub_cam_info[ITEM_COUNT];
    // Camera info messages
    sensor_msgs::CameraInfo _msg_cam_info[ITEM_COUNT];

    // Gyroscope offset calibration
    int _num_samples;
    double _gyro_offset[3];

public:
    DUO3DDriver()
        : _duoInstance(NULL),
          _nh(),
          _frame_rate(30),
          _image_size({640, 480})
	{
        getParams();

        image_transport::ImageTransport itrans(_nh);
        for(int i = 0; i < topic_name.size(); i++)
            _pub_image[i] = itrans.advertise(topic_name[i], 16);
        for(int i = 0; i < cam_info_topic_name.size(); i++)
            _pub_cam_info[i] = _nh.advertise<sensor_msgs::CameraInfo>(cam_info_topic_name[i], 1);
    }
    ~DUO3DDriver()
    {
        closeDuo();
    }

    void run()
    {
        try
        {
            if(!openDuo()) return;

            fillCameraInfo();

            _server.setCallback(boost::bind(&DUO3DDriver::dynamicCallback, this, _1, _2));

            _frame_num = 0;   // reset frame number
            _num_samples = 0;

            if(!StartDUO(_duoInstance,
                            [](const PDUOFrame pFrame, void *pUserData)
                            {
                                if(!ros::isShuttingDown())
                                    ((DUO3DDriver*)pUserData)->duoCallback(pFrame);
                            }, this))
            {
                ROS_ERROR("Could not start DUO camera");
                return;
            }
            ros::spin();
        }
        catch(...)
        {
            ros::shutdown();
        }
    }
protected:
    int width() { return _image_size[0]; }
    int height() { return _image_size[1]; }
    double fps() { return _frame_rate; }
    Vec3b HSV2RGB(float hue, float sat, float val)
    {
        float x, y, z;

        if(hue == 1) hue = 0;
        else         hue *= 6;

        int i = static_cast<int>(floorf(hue));
        float f = hue - i;
        float p = val * (1 - sat);
        float q = val * (1 - (sat * f));
        float t = val * (1 - (sat * (1 - f)));

        switch(i)
        {
            case 0: x = val; y = t; z = p; break;
            case 1: x = q; y = val; z = p; break;
            case 2: x = p; y = val; z = t; break;
            case 3: x = p; y = q; z = val; break;
            case 4: x = t; y = p; z = val; break;
            case 5: x = val; y = p; z = q; break;
        }
        return Vec3b((uchar)(z * 255), (uchar)(y * 255), (uchar)(x * 255));
    }
    void getParams()
    {
        ros::NodeHandle nh("~");
        nh.getParam("frame_rate", _frame_rate);
        nh.getParam("image_size", _image_size);

        for(int i = 0; i < topic_param_name.size(); i++)
            nh.getParam(topic_param_name[i], topic_name[i]);
        for(int i = 0; i < cam_info_topic_param_name.size(); i++)
            nh.getParam(cam_info_topic_param_name[i], cam_info_topic_name[i]);
        for(int i = 0; i < frame_id_param_name.size(); i++)
            nh.getParam(frame_id_param_name[i], frame_id_name[i]);
    }

    void dynamicCallback(Duo3DConfig &config, uint32_t level)
    {
        // Set DUO parameters
        if(_duoInstance)
        {
            SetDUOGain(_duoInstance, config.gain);
            SetDUOExposure(_duoInstance, config.exposure);
            SetDUOAutoExposure(_duoInstance, config.auto_exposure);
            SetDUOCameraSwap(_duoInstance, config.camera_swap);
            SetDUOHFlip(_duoInstance, config.horizontal_flip);
            SetDUOVFlip(_duoInstance, config.vertical_flip);
            SetDUOLedPWM(_duoInstance, config.led);
            SetDUOIMURange(_duoInstance, config.accel_range, config.gyro_range);
            SetDUOIMURate(_duoInstance, config.imu_rate);
        }
    }

    void duoCallback(const PDUOFrame pFrame)
    {
        // Set the start time
        if(_frame_num++ == 0) _start_time = ros::Time::now().toSec();

        Size size(pFrame->width, pFrame->height);

        // Create Mat for left and right images
        Mat left(size, CV_8UC1, pFrame->leftData);
        Mat right(size, CV_8UC1, pFrame->rightData);

        for(int i = 0; i < ITEM_COUNT; i++)
        {
            std_msgs::Header header;
            header.stamp = ros::Time(_start_time + (double)pFrame->timeStamp / 10000.0);
            header.frame_id = frame_id_name[i];

            if((i == LEFT) && (_pub_image[i].getNumSubscribers() > 0))
            {
                _pub_image[i].publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, left).toImageMsg());
                _msg_cam_info[i].header = header;
                _pub_cam_info[i].publish(_msg_cam_info[i]);
            }
            if((i == RIGHT) && (_pub_image[i].getNumSubscribers() > 0))
            {
                _pub_image[i].publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, right).toImageMsg());
                _msg_cam_info[i].header = header;
                _pub_cam_info[i].publish(_msg_cam_info[i]);
            }
        }
    }

    bool fillCameraInfo()
    {
        if(!_duoInstance) return false;
        DUO_STEREO stereo;
        if(!GetDUOStereoParameters(_duoInstance, &stereo))
        {
            ROS_ERROR("Could not get DUO camera calibration data");
            return false;
        }
        for(int i = 0; i < ITEM_COUNT; i++)
        {
            _msg_cam_info[i].width = width();
            _msg_cam_info[i].height = height();
            _msg_cam_info[i].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            _msg_cam_info[i].D.resize(5, 0.0);
            _msg_cam_info[i].K.fill(0.0);
            _msg_cam_info[i].K[0] = stereo.P1[0]; // fx
            _msg_cam_info[i].K[2] = stereo.P1[2]; // cx
            _msg_cam_info[i].K[4] = stereo.P1[4]; // fy
            _msg_cam_info[i].K[6] = stereo.P1[6]; // cy
            _msg_cam_info[i].K[8] = 1.0;
            _msg_cam_info[i].R.fill(0.0);
            _msg_cam_info[i].P.fill(0.0);
            _msg_cam_info[i].P[0] = stereo.P1[0]; // fx
            _msg_cam_info[i].P[2] = stereo.P1[2]; // cx
            _msg_cam_info[i].P[5] = stereo.P1[5]; // fy
            _msg_cam_info[i].P[6] = stereo.P1[6]; // cy
            _msg_cam_info[i].P[10] = 1.0;
            if(i == RIGHT)
                _msg_cam_info[i].P[3] = stereo.P2[3] / 1000.0;  // (fx * baseline) / 1000
        }
        return true;
    }

    bool openDuo()
    {
        // Find the optimal sensor binning parameters for given (width, height)
        // This maximizes sensor imaging area for given resolution
        int binning = DUO_BIN_NONE;
        if(width() <= 752/4)        binning += DUO_BIN_HORIZONTAL4;
        else if(width() <= 752/2)   binning += DUO_BIN_HORIZONTAL2;
        if(height() <= 480/4)       binning += DUO_BIN_VERTICAL4;
        else if(height() <= 480/2)  binning += DUO_BIN_VERTICAL2;

        DUOResolutionInfo ri;
        if(!EnumerateDUOResolutions(&ri, 1, width(), height(), binning, fps()))
        {
            ROS_ERROR("Invalid DUO camera resolution");
            return false;
        }
        ROS_INFO("DUO resolution [%d x %d] @ %f fps", width(), height(), fps());

        if(!OpenDUO(&_duoInstance))
        {
            ROS_ERROR("Could not open DUO library");
            return false;
        }
        // Set the image size
        if(!SetDUOResolutionInfo(_duoInstance, ri))
        {
            ROS_ERROR("Invalid image size");
            return false;
        }
        return true;
    }
    void closeDuo()
    {
        if(_duoInstance)
        {
            StopDUO(_duoInstance);
            CloseDUO(_duoInstance);
            _duoInstance = NULL;
        }
    }
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    duo3d_driver::DUO3DDriver duo;
    duo.run();
	return 0;
}
