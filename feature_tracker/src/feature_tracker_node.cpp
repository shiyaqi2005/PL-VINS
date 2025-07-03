#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

//补充库
#include <mutex>
#include <stdio.h>
#include <queue>
#include <vector>
#include <thread>


#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

std::vector<cv_bridge::CvImageConstPtr> image_ptr_vector;

ros::Publisher pub_img,pub_match;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;

double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

void sync_process()
{
    while(1)
    {
        m_buf.lock();
        if (!img0_buf.empty() && !img1_buf.empty())
        {
            sensor_msgs::ImageConstPtr img_msg0 = img0_buf.front();
            img0_buf.pop();
            sensor_msgs::ImageConstPtr img_msg1 = img1_buf.front();
            img1_buf.pop();
        }
        m_buf.unlock();

        if(first_image_flag)
        {
            first_image_flag = false;
            first_image_time = img0_msg->header.stamp.toSec();
        }

        // frequency control, 如果图像频率低于一个值
        if (round(1.0 * pub_count / (img0_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            // reset the frequency control
            if (abs(1.0 * pub_count / (img0_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
            {
                first_image_time = img0_msg->header.stamp.toSec();
                pub_count = 0;
            }
        }
        else
            PUB_THIS_FRAME = false;

        cv_bridge::CvImageConstPtr ptr0 = cv_bridge::toCvCopy(img0_msg, sensor_msgs::image_encodings::MONO8);
        image_ptr_vector.push_back(ptr0);
        cv_bridge::CvImageConstPtr ptr1 = cv_bridge::toCvCopy(img1_msg, sensor_msgs::image_encodings::MONO8);
        image_ptr_vector.push_back(ptr1);

        cv::Mat show_img = ptr0->image;

        TicToc t_r;
        frame_cnt++;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {

            cv_bridge::CvImageConstPtr ptr = image_ptr_vector.at(i)

            ROS_DEBUG("processing camera %d", i);
            if (i != 1 || !STEREO_TRACK)     // 单目 或者 非双目,   #### readImage： 这个函数里 做了很多很多事, 提取特征 ####
                trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)));   // rowRange(i,j) 取图像的i～j行
            else
            {
                if (EQUALIZE)      // 对图像进行直方图均衡化处理
                {
                    std::cout <<" EQUALIZE " << std::endl;
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                }
                else
                    trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }

        image_ptr_vector.clear(); 

    #if SHOW_UNDISTORTION
    //        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
            trackerData[i].showUndistortion();
    #endif
        }

        // 双目特征点匹配
        if ( PUB_THIS_FRAME && STEREO_TRACK && trackerData[0].cur_pts.size() > 0)
        {
            pub_count++;
            r_status.clear();
            r_err.clear();
            TicToc t_o;
            cv::calcOpticalFlowPyrLK(trackerData[0].cur_img, trackerData[1].cur_img, trackerData[0].cur_pts, trackerData[1].cur_pts, r_status, r_err, cv::Size(21, 21), 3);
            ROS_DEBUG("spatial optical flow costs: %fms", t_o.toc());
            vector<cv::Point2f> ll, rr;
            vector<int> idx;
            for (unsigned int i = 0; i < r_status.size(); i++)
            {
                if (!inBorder(trackerData[1].cur_pts[i]))
                    r_status[i] = 0;

                if (r_status[i])
                {
                    idx.push_back(i);

                    Eigen::Vector3d tmp_p;
                    trackerData[0].m_camera->liftProjective(Eigen::Vector2d(trackerData[0].cur_pts[i].x, trackerData[0].cur_pts[i].y), tmp_p);
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    ll.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));

                    trackerData[1].m_camera->liftProjective(Eigen::Vector2d(trackerData[1].cur_pts[i].x, trackerData[1].cur_pts[i].y), tmp_p);
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    rr.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
                }
            }
            if (ll.size() >= 8)
            {
                vector<uchar> status;
                TicToc t_f;
                cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
                ROS_DEBUG("find f cost: %f", t_f.toc());
                int r_cnt = 0;
                for (unsigned int i = 0; i < status.size(); i++)
                {
                    if (status[i] == 0)
                        r_status[idx[i]] = 0;
                    r_cnt += r_status[idx[i]];
                }
            }
        }

        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j++)
                if (j != 1 || !STEREO_TRACK)                   // 单目 或者 非双目
                    completed |= trackerData[j].updateID(i);   // 更新检测到的特征的id, 如果一个特征是新的，那就赋予新的id，如果一个特征已经有了id，那就不处理
            if (!completed)
                break;
        }

        if (PUB_THIS_FRAME)
        {
            pub_count++;
            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;   //  feature id
            sensor_msgs::ChannelFloat32 u_of_point;    //  u
            sensor_msgs::ChannelFloat32 v_of_point;    //  v

            feature_points->header = img0_msg->header;
            feature_points->header.frame_id = "world";

            vector<set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                if (i != 1 || !STEREO_TRACK)  // 单目
                {
                    auto un_pts = trackerData[i].undistortedPoints();
                    auto &cur_pts = trackerData[i].cur_pts;
                    auto &ids = trackerData[i].ids;
                    for (unsigned int j = 0; j < ids.size(); j++)
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        ROS_ASSERT(inBorder(cur_pts[j]));
                    }
                }
                else if (STEREO_TRACK)
                {
                    auto r_un_pts = trackerData[1].undistortedPoints();
                    auto &ids = trackerData[0].ids;
                    for (unsigned int j = 0; j < ids.size(); j++)
                    {
                        if (r_status[j])
                        {
                            int p_id = ids[j];
                            hash_ids[i].insert(p_id);
                            geometry_msgs::Point32 p;
                            p.x = r_un_pts[j].x;
                            p.y = r_un_pts[j].y;
                            p.z = 1;

                            feature_points->points.push_back(p);
                            id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        }
                    }
                }
            }
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
            pub_img.publish(feature_points);

        }
        sum_time += t_r.toc();
        mean_time = sum_time/frame_cnt;
    //    ROS_INFO("whole point feature tracker processing costs: %f", t_r.toc());
        ROS_INFO("whole point feature tracker processing costs: %f", mean_time);

    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]); // 读相机外参

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    //ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    ros::Subscriber sub_img = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);


    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

    //处理图像的函数
    std::thread sync_thread{sync_process};


    ros::spin();
    return 0;
}
