#pragma once
#include <vector>
#include <string>
#include <stdio.h>

#include "FullSystem/HessianBlocks.h"
#include "util/NumType.h"
#include "util/MinimalImage.h"
#include "map"

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2_ros/static_transform_broadcaster.h>



#define POSE_SCALING_FACTOR 10.0
#define POINTS_SCALING_FACTOR 500.0

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;

    namespace IOWrap
    {
        class RosPublisher : public Output3DWrapper {
            public:
                inline RosPublisher(ros::Publisher &pub){
                    pc_pub = pub;
                    //outputFile  = fopen("...","w"); //temporary
                }

                virtual ~RosPublisher() override {
                    fclose (outputFile);
                }

                virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override {
                    static tf2_ros::StaticTransformBroadcaster br;

                    geometry_msgs::TransformStamped camPose_w;
                    camPose_w.header.stamp = ros::Time::now();
                    camPose_w.header.frame_id = "world";
                    camPose_w.child_frame_id = "camPose";

                    auto t = (frame->camToWorld.translation());
                    camPose_w.transform.translation.x = t.x();
                    camPose_w.transform.translation.y = t.z();
                    camPose_w.transform.translation.z = -t.y();
                
                    // need to check 
                    auto q = frame->camToWorld.unit_quaternion();
                    camPose_w.transform.rotation.x = q.x();
                    camPose_w.transform.rotation.y = -q.z();
                    camPose_w.transform.rotation.z = -q.y();
                    camPose_w.transform.rotation.w = q.w();

                    br.sendTransform(camPose_w);

                    //fprintf(outputFile,"%f,%f,%f\n",t.x(),t.z(),-t.y());
                }


                // still need to fix the pointCloud publisher
                // void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override {
                //     if(final){
                //         //id = fh->shell->id;
                //         float fx = HCalib->fxl();
                //         float fy = HCalib->fyl();
                //         float cx = HCalib->cxl();
                //         float cy = HCalib->cyl();
                //         //width = wG[0];
                //         //height = hG[0];
                //         float fxi = 1/fx;
                //         float fyi = 1/fy;
                //         float cxi = -cx / fx;
                //         float cyi = -cy / fy;
                //         int count = 0;
                //         for (FrameHessian* fh : frames){
                //             auto camToWorld = fh->shell->camToWorld.matrix().cast<float>();

                //             // sensor_msgs::PointCloud2 pc_msg;
                //             // pc_msg.header.frame_id = "world";
                //             // pc_msg.is_bigendian = false;
                //             // pc_msg.height=1;
                //             // pc_msg.width = fh->pointHessians.size();
                //             // pc_msg.point_step = 12;
                //             // pc_msg.is_dense = true;
                //             //pc_msg.row_step = pc_msg.width * pc_msg.point_step;

                //             // pc_msg.fields.resize(3);
                //             // pc_msg.fields[0].name = "x";
                //             // pc_msg.fields[0].offset = 0;
                //             // pc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
                //             // pc_msg.fields[0].count = 1;

                //             // pc_msg.fields[11].name = "y";
                //             // pc_msg.fields[1].offset = 4;
                //             // pc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
                //             // pc_msg.fields[1].count = 1;

                //             // pc_msg.fields[2].name = "z";
                //             // pc_msg.fields[2].offset = 8;
                //             // pc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
                //             // pc_msg.fields[2].count = 1;


                //             // pc_msg.data.resize(12*pc_msg.width);
                            
                //             int offset = 0;
                //             for (PointHessian* ph : fh->pointHessians){

                //                 float depth = 1.0f/(ph->idepth_scaled);                           
                //                 float x = (ph->u * fxi + cxi) * depth;
                //                 float y = (ph->v * fyi + cyi) * depth;
                //                 float z = ((1+2*fxi)*(rand()/(float)RAND_MAX-0.5f)) * depth ;
                //                 Eigen::Vector4f p_cam(x,y,z,1.0); // column vector
                //                 Eigen::Vector4f p_world = camToWorld*p_cam;
                                
                //                 x = p_world(1);
                //                 y = p_world(2);
                //                 z = p_world(3);

                //                 // float x = static_cast<float>(i)+0.7;
                //                 // float y = x;
                //                 // float z = x*x+y*y;
                //                 printf("%f,%f,%f\n",x,y,z);
                //                 fprintf (outputFile,"%f,%f,%f\n",x,y,z);

                //                 //float *castPtr_x = reinterpret_cast<float*>(&x);
                //                 //float *castPtr_y = reinterpret_cast<float*>(&y);
                //                 //float *castPtr_z = reinterpret_cast<float*>(&z);

                //                 // for (int pos=0; pos<4;pos++){
                //                 //     pc_msg.data[offset+pos] = castPtr_x[pos];
                //                 //     pc_msg.data[offset+4+pos] = castPtr_y[pos];
                //                 //     pc_msg.data[offset+8+pos] = castPtr_z[pos];
                //                 // }
                //                 // offset+=12;
                //             }
                //             //pc_pub.publish(pc_msg);
                //         }
                //     }     
                // return;
                // }


            private:
                ros::Publisher pc_pub;
                FILE * outputFile;
        };
    }
}