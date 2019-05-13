/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>

//#include <boost/log/core.hpp>

#include "SolARModuleOpengl_traits.h"
#include "SolARModuleCeres_traits.h"
#include "xpcf/xpcf.h"
#include "api/display/I3DPointsViewer.h"
#include "api/solver/map/IBundler.h"
#include "core/Log.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENGL;
using namespace SolAR::MODULES::CERES;
namespace xpcf = org::bcom::xpcf;


///@brief: Bundle problem loader struct:
/// Loads:
///   a) load 2D points.
///   b) load 3D points and visibility (to estbalih 2D/3D correspondances).
///   c) load camera extrinsics.
///   d) load camera intrinsics.
///Shows : all the loaded parameters.
///
struct SolARBALoader{
    std::vector<std::vector<SRef<Keypoint>>>m_points2d;
    std::vector<Transform3Df>m_poses;
    std::vector<SRef<CloudPoint>>m_points3d;
    std::vector<SRef<Image>> m_views;
    std::vector<SRef<DescriptorBuffer>> m_descriptors;
    CamCalibration  m_intrinsic;
    CamDistortion   m_distorsion;

    bool load2DPoints(const std::string & path_measures) {
        int N;
        std::ifstream ox(path_measures);
        if (!ox.is_open()) {
            std::cerr << " can't read measurements file from: " << path_measures << std::endl;
            return false;
        }
        else {
            std::cout<<" LOADING 2D POINTS: ";
            ox >> N;
            m_points2d.resize(N);
            for (int i = 0; i < N; ++i) {
                std::cout<<i<<" ";
                std::string path_measure;
                ox >> path_measure;
                std::ifstream ox(path_measure);
                if (!ox.is_open()) {
                    std::cerr << " can't find observation file from: " << path_measure << std::endl;
                    return false;
                }
                else {
                    int kp_no;
                    ox >> kp_no;
                    m_points2d[i].resize(kp_no);
                    for (int j = 0; j < kp_no; ++j) {
                        float x,y;
                        ox >>x;
                        ox >>y;
                        m_points2d[i][j] = xpcf::utils::make_shared<Keypoint>(x,y,0.0,0.0,0.0,0.0,0);
                    }
                }
            }
            std::cout<<" done"<<std::endl;
            return true;
        }
    }
    bool load3DPoints(const std::string & path_obs) {
        std::ifstream ox(path_obs);
        if (!ox.is_open()) {
            std::cerr << "can't find cloud from: " << path_obs << std::endl;
            return false;
        }
        else{
            std::cout<<"LOADING 3D POINTS: ";
            int obs_no;
            ox >> obs_no;
            m_points3d.resize(obs_no);
            for (int i = 0; i < obs_no; ++i) {
                double x,y,z;
                ox >> x;
                ox >> y;
                ox >> z;

                std::map<unsigned int, unsigned int> visibility_temp;

               m_points3d[i] = xpcf::utils::make_shared<CloudPoint>(x, y, z,0.0,0.0,0.0,0.0,visibility_temp);
               int viz_no; ox >> viz_no;
               for(int j = 0; j < viz_no; ++j) {
                   int idxView,idxLoc;
                    ox >>idxView;
                    ox >>idxLoc;
                    m_points3d[i]->getVisibility()[idxView] = idxLoc;
                }
            }
            std::cout<<" done"<<std::endl;
        }
        return true;

    }

    bool loadIntrinsic(const std::string&path_calib){
        std::cout<<"loading intrinsics: ";
        std::ifstream ox(path_calib);
        if(ox.is_open()){
           for(int i = 0; i < 3; ++i){
                for(int j = 0; j < 3; ++j){
                    ox>>m_intrinsic(i,j);
                }
            }
           std::cout<<"done"<<std::endl;
        }
        else{
            LOG_INFO("can't read calirabtion file from", path_calib);
           return false;
        }
        return true;
    }
    bool loadDistorsions(const std::string&path_dist) {
        std::ifstream ox(path_dist);
        if (!ox.is_open()) {
            LOG_INFO("can't read distorsion file from", path_dist);
            return false;
        }
        else {
            std::cout<<"loading intrinsic: ";
                for (int i = 0; i < 5; ++i) {
                    ox >> m_distorsion[i];
            }
                std::cout<<" done"<<std::endl;
        }
        return true;
    }

    bool loadExtrinsics(const std::string & path_poses) {
        std::ifstream ox(path_poses);
        if (!ox.is_open()) {
            std::cerr << "can't find poses file from: " << path_poses << std::endl;
            return false;
        }
        else{
            std::cout<<" loading poses: ";
            int N;
            ox >> N;
            m_poses.resize(N);
            for (unsigned int i = 0; i < N; ++i) {
                for (int ii = 0; ii < 3; ++ii) {
                    for (int jj = 0; jj < 4; ++jj) {
                        ox >> m_poses[i](ii, jj);
                    }
                }
                m_poses[i](3,0) = 0.0;m_poses[i](3,1) = 0.0;m_poses[i](3,2) = 0.0;m_poses[i](3,3) = 1.0;
            }
        }
        std::cout<<" done"<<std::endl;
        return true;
    }

    void showExtrinsics()const {
        int idx = 0;
        for (const auto &p : m_poses) {
            std::cout << " EXTRINSIC: " << idx << std::endl;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 4; ++j) {
                    std::cout << p(i, j) << " ";
                }
                std::cout << std::endl;
            }
            ++idx;
            std::cout << std::endl;
        }
    }

    void show3DPoints()const {
        int idx = 0;
        std::cout << "<3D POINTS>: " << std::endl;
        std::cout<<"    ->size: "<<m_points3d.size()<<std::endl;
        for (unsigned int i = 0; i < m_points3d.size(); ++i){
            std::cout << "p: " << m_points3d[i]->getX() << " " <<  m_points3d[i]->getY() << " " <<  m_points3d[i]->getZ() << "  ";

            std::map<unsigned int, unsigned int> visibility = m_points3d[i]->getVisibility();
            int idxFrame = 0;
            for (std::map<unsigned int, unsigned int>::iterator it = visibility.begin(); it != visibility.end(); ++it){
                std::cout<<it->first<<" "<<it->second<<" ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void show2Dpoints()const {
        std::cout << "<2D POINTS>: " << std::endl;
        for (int i = 0; i < m_points2d.size(); ++i) {
            std::cout << "	<2D POINTS from view: " << i << ">:" << std::endl;
            for (int j = 0; j < m_points2d[i].size(); ++j) {
                std::cout << m_points2d[i][j]->getX() << " " <<  m_points2d[i][j]->getY() << std::endl;
            }
        }
    }

    void  showIntrinsics()const {
        std::cout << "<INTRINSIC>: " << std::endl;
            for (int ii = 0; ii < 3; ++ii) {
                for (int jj = 0; jj < 3; ++jj) {
                    std::cout << m_intrinsic(ii, jj) << " ";
                }
                std::cout << std::endl;
            }
    }

    void  showDistorsions()const {
        std::cout << "<DISTORSION>: " << std::endl;
            for (int ii = 0; ii < 5; ++ii) {
                    std::cout << m_distorsion[ii] << " ";
            }
            std::cout << std::endl;
    }

};

void project3Dpoints(const Transform3Df pose,const CamCalibration calib,const CamDistortion dist,const std::vector<SRef<CloudPoint>>& cloud,std::vector<SRef<Point2Df>>& point2D){

    //first step :  from world coordinates to camera coordinates
    Transform3Df invPose;
    invPose=pose.inverse();
    point2D.clear();
#if (_WIN64) || (_WIN32)
        Vector3f pointInCamRef;
#else
        Vector4f pointInCamRef;
#endif
        float k1 = dist[0];
        float k2 = dist[1];
        float p1 = dist[2];
        float p2 = dist[3];
        float k3 = dist[4];
    for (auto cld = cloud.begin();cld!=cloud.end();++cld){
#if (_WIN64) || (_WIN32)
        Vector3f point((*cld)->getX(), (*cld)->getY(), (*cld)->getZ());
#else
        Vector4f point((*cld)->getX(), (*cld)->getY(), (*cld)->getZ(),1);
#endif
        pointInCamRef=invPose*point;
        if(pointInCamRef(2)>0){
            float x=pointInCamRef(0)/pointInCamRef(2);
            float y=pointInCamRef(1)/pointInCamRef(2);

            float r2 = x * x + y * y;
            float r4 = r2 * r2;
            float r6 = r4 * r2;
            float r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
            float xx = x * r_coeff + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            float yy = y * r_coeff + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);

            float u = calib(0,0)*xx +calib(0,2);
            float v = calib(1,1)*yy +calib(1,2);

            SRef<Point2Df> p2d=xpcf::utils::make_shared<Point2Df> (u,v);

            point2D.push_back(p2d);
        }
        else
            point2D.push_back(xpcf::utils::make_shared<Point2Df> (0,0));
    }
}


std::pair<double,int> getReprojectionError(SRef<Keyframe> keyFrame,SolARBALoader *ba,bool fromCloud=true){
    double r;
    Transform3Df pose=keyFrame->getPose();
    std::vector<SRef<CloudPoint>> cloud;
    std::vector<int>  indices;
    std::vector<SRef<Point2Df>>  point2D;

    if(fromCloud){
        auto pointCloud=ba->m_points3d;
        auto visibility=keyFrame->getVisibleMapPoints();
        for(auto cp:pointCloud){
            std::map<unsigned int, unsigned int> vis=cp->getVisibility();
            auto itr_v = vis.find( keyFrame->m_idx);
            if(itr_v!=vis.end()){
                cloud.push_back(cp);
                indices.push_back(itr_v->second);
                int count=0;
                for(std::map<unsigned int, SRef<CloudPoint>>::iterator v=visibility.begin();v!=visibility.end();++v){
                    if(v->second==cp){
                        count++;
                    }
                }
                if(count==0){
//                    std::cout << "cp not found \n";
                }

            }
        }
    }
    else    {
        auto visibility=keyFrame->getVisibleMapPoints();
        for(std::map<unsigned int, SRef<CloudPoint>>::iterator v=visibility.begin();v!=visibility.end();++v){
            auto ind=v->first;
            auto cp=v->second;
            indices.push_back(ind);
            cloud.push_back(cp);
        }
    }
    project3Dpoints(pose,ba->m_intrinsic,ba->m_distorsion,cloud,point2D);
    auto keypoints=keyFrame->getKeypoints();
    r=0;
    int i;
    for(i=0;i<point2D.size();++i){

        SRef<Keypoint> kp = keypoints[indices[i]];
        SRef<Point2Df> p2d = point2D[i];

        double dx=p2d->getX()-kp->getX();
        double dy=p2d->getY()-kp->getY();

        r+=dx*dx+dy*dy;
    }
    return std::make_pair(r,point2D.size());

}

std::pair<double,int> getReprojectionErrorFull(const std::vector<SRef<Keyframe>>& keyFrames,SolARBALoader *ba,std::vector<int>& selectedKeyframes){

    double r=0;
    int i=0;
    for(auto idx:selectedKeyframes){
        for(auto kf:keyFrames){
               if(kf->m_idx==idx){
                    std::pair<double,int> res=getReprojectionError(kf,ba);
                    r+=res.first;
                    i+=res.second;
               }
        }
    }
    return std::make_pair(0.5*r,i);
}


int run_bundle(std::string & scene){
    LOG_ADD_LOG_TO_CONSOLE();
    SolARBALoader *ba = new SolARBALoader();
    const std::string path_poses        = "./" + scene + "Bundle/" + scene + "Poses.txt";
    const std::string path_points3d     = "./" + scene + "Bundle/" + scene + "Pts3D.txt";;
    const std::string path_points2d     = "./" + scene + "Bundle/" + scene + "Pts2D.txt";
    const std::string path_calibration  = "./" + scene + "Bundle/" + scene + "Calibration.txt";
    const std::string path_distorison   = "./" + scene + "Bundle/" + scene + "Distorsion.txt";

    LOG_INFO("-<SolAR BA PROBLEM LOADING>-");
    ba->load3DPoints(path_points3d);
    ba->load2DPoints(path_points2d);
    ba->loadExtrinsics(path_poses);
    ba->loadIntrinsic(path_calibration);
    ba->loadDistorsions(path_distorison);


 //// uncomment code below to show ba problem.
 //   ba->show3DPoints();
 //   ba->show2Dpoints();
 //   ba->showExtrinsics();
 //   ba->showIntrinsics();
 //   ba->showDistorsions();

    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
    if(xpcfComponentManager->load("conf_Ceres.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file bundle_config.xml")
        return -1;
    }

    LOG_INFO("-<SolAR3DPointsViewerOpengl LOADING>-");
    auto viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    LOG_INFO("-<SolARBundlerCeres LOADING>-");
    auto bundler =xpcfComponentManager->create<SolARBundlerCeres>()->bindTo<api::solver::map::IBundler>();


    std::vector<SRef<Keyframe>>keyframes;
    std::vector<int>selectedKeyframes = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};  // !!!! Caution : specify the selection in ascending order

    keyframes.resize(ba->m_poses.size());
    ba->m_descriptors.resize(keyframes.size());
    ba->m_views.resize(keyframes.size());

    for(unsigned int i = 0; i < keyframes.size(); ++i){
       keyframes[i] = xpcf::utils::make_shared<Keyframe>(ba->m_points2d[i],
                                                         ba->m_descriptors[i],
                                                         ba->m_views[i],
                                                         ba->m_poses[i]);
    }



    {
       std::set<int> kfset(selectedKeyframes.begin(),selectedKeyframes.end());
       int countPoints=0;
        for(auto pc : ba->m_points3d){
            auto vis=pc->getVisibility();
            for(auto v:vis){
                int kf=v.first;
                if(kfset.find(kf)!=kfset.end()){
                    countPoints++;
                }
            }
        }
    std::cout << "nb ot points : " << countPoints << "\n";
//    getchar();
    }

    std::vector<double> errorBefore;
    for (auto kf:keyframes){
            std::pair<double,int> res=getReprojectionError(kf,ba);
            errorBefore.push_back(sqrt(res.first/res.second));
    }
    std::pair<double,int> resb=getReprojectionErrorFull(keyframes,ba,selectedKeyframes);


    std::vector<SRef<CloudPoint>>cloud, cloud_ba;
    std::vector<Transform3Df>poses , poses_ba;
    for(unsigned i = 0; i < keyframes.size(); ++i){
        Transform3Df tt = keyframes[i]->getPose();
        poses.push_back(tt);
    }

   // copy input cloud for display purpose
   for (auto c:ba->m_points3d) {
         double reprj_err = c->getReprojError();
         std::map<unsigned int, unsigned int>visibility = c->getVisibility();
         double x = c->getX();
         double y = c->getY();
         double z = c->getZ();
         cloud.push_back(xpcf::utils::make_shared<CloudPoint>(x, y, z,0.0,0.0,0.0,reprj_err,visibility));
    }


   double reproj_errorFinal  = 0.f;
   reproj_errorFinal = bundler->solve(keyframes,
                                      ba->m_points3d,
                                      ba->m_intrinsic,
                                      ba->m_distorsion,
                                      selectedKeyframes);

   LOG_INFO("reprojection error final: {}",reproj_errorFinal);


   std::vector<double> errorAfter;
   for (auto kf:keyframes){
       std::pair<double,int> res=getReprojectionError(kf,ba);
       errorAfter.push_back(sqrt(res.first/res.second));
   }
   std::pair<double,int> resa=getReprojectionErrorFull(keyframes,ba,selectedKeyframes);

   for(int i=0;i<errorBefore.size();++i){
       std::cout << "kf : " << i << " reproj error before :" << errorBefore[i]<< " reproj error after :" << errorAfter[i] << "\n";
   }

   std::cout << "\nfull reproj error before :" << resb.first<< " full reproj error after :" << resa.first << "\n";
   std::cout << "\nfull reproj error before :" << resb.first/resb.second<< " full reproj error after :" << resa.first/resa.second << "\n";

   for(auto & p: keyframes){
       Transform3Df tt = p->getPose();
       poses_ba.push_back(tt);
   }

   LOG_INFO(" pose # 0 : {}",poses_ba[0].matrix());
   LOG_INFO(" pose # 1 : {}",poses_ba[1].matrix());
    cloud_ba = ba->m_points3d;

    std::vector<Transform3Df>framePoses;
    Transform3Df pp = Transform3Df::Identity();
    while(true){
        if (viewer3DPoints->display(cloud,
                                    pp,
                                    poses,
                                    framePoses,
                                    cloud_ba,
                                    poses_ba) == FrameworkReturnCode::_STOP){
                return 0;
            }
        }


    delete ba;
    return 0;
}
int main(int argc, char ** argv){
    std::string scene_name = "room15";
    run_bundle(scene_name);
    return 0;
}



