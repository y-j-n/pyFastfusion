/*
 * viewerfusion_main.cpp
 *
 *  Created on: Jun 23, 2013
 *      Author: steinbrf
 */


#include <auxiliary/multivector.h>

#include <opencv2/opencv.hpp>
#include <omp.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <Eigen/Geometry>
#include <auxiliary/debug.hpp>

//#include <fusionGPU/texturefusion.hpp>
//#include <fusionGPU/geometryfusion_single_soa.hpp>
//#include <fusionGPU/geometryfusion_single_aos.hpp>
//#include <fusionGPU/geometryfusion_dynamic_aos.hpp>
//#include <fusionGPU/geometryfusion_dynamic_multiscale.hpp>
//#include <fusionGPU/geometryfusion_mipmap.hpp>
#include <fusion/geometryfusion_mipmap_cpu.hpp>

#include "onlinefusionviewer.hpp"
#include "onlinefusionviewer_if.hpp"
#include <qapplication.h>

#include <tclap/CmdLine.h>
#include <fusion/mesh.hpp>

#include <boost/optional.hpp>



#define BOXDELTA 0.001
//#define VOLUMERADIUS 1.5
#define VOLUMERADIUS 1.4
#define USE_ORIGINAL_VOLUME 1

#include <fusion/definitions.h>


#include <deque>
#include <list>

//CameraInfo kinectPoseFromSophus(Sophus::SE3 pos){
//	CameraInfo result;
//	cv::Mat intrinsic = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	//Kinect Intrinsic Parameters
//	//Kinect
//	intrinsic.at<double>(0,0) = intrinsic.at<double>(1,1) = 525.0;
//	intrinsic.at<double>(0,2) = 319.5;
//	intrinsic.at<double>(1,2) = 239.5;
//	//DLR
////	intrinsic.at<double>(0,0) = intrinsic.at<double>(1,1) = 551.85425;
////	intrinsic.at<double>(0,2) = 376.0;
////	intrinsic.at<double>(1,2) = 240.0;
//
//	result.setIntrinsic(intrinsic);
//	Eigen::Matrix3d rotation = pos.rotation_matrix();
//	cv::Mat rotation2 = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) for(int j=0;j<3;j++) rotation2.at<double>(i,j) = rotation(i,j);
//	result.setRotation(rotation2);
//	Eigen::Vector3d translation = pos.translation();
//	cv::Mat translation2 = cv::Mat::zeros(3,1,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) translation2.at<double>(i,0) = translation(i);
//	result.setTranslation(translation2);
//	return result;
//}

//CameraInfo kinectPoseFromSophus(Sophus::SE3 pos,float fx, float fy, float cx, float cy){
//	CameraInfo result;
//	cv::Mat intrinsic = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	//Kinect Intrinsic Parameters
//	intrinsic.at<double>(0,0) = fx;
//	intrinsic.at<double>(1,1) = fy;
//	intrinsic.at<double>(0,2) = cx;
//	intrinsic.at<double>(1,2) = cy;
//
//	result.setIntrinsic(intrinsic);
//	Eigen::Matrix3d rotation = pos.rotation_matrix();
//	cv::Mat rotation2 = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) for(int j=0;j<3;j++) rotation2.at<double>(i,j) = rotation(i,j);
//	result.setRotation(rotation2);
//	Eigen::Vector3d translation = pos.translation();
//	cv::Mat translation2 = cv::Mat::zeros(3,1,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) translation2.at<double>(i,0) = translation(i);
//	result.setTranslation(translation2);
//	return result;
//}

CameraInfo kinectPoseFromEigen(std::pair<Eigen::Matrix3d,Eigen::Vector3d> pos,float fx, float fy, float cx, float cy){
	CameraInfo result;
	cv::Mat intrinsic = cv::Mat::eye(3,3,cv::DataType<double>::type);
	//Kinect Intrinsic Parameters
	intrinsic.at<double>(0,0) = fx;
	intrinsic.at<double>(1,1) = fy;
	intrinsic.at<double>(0,2) = cx;
	intrinsic.at<double>(1,2) = cy;

	result.setIntrinsic(intrinsic);
	Eigen::Matrix3d rotation = pos.first;
	cv::Mat rotation2 = cv::Mat::eye(3,3,cv::DataType<double>::type);
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) rotation2.at<double>(i,j) = rotation(i,j);
	result.setRotation(rotation2);
	Eigen::Vector3d translation = pos.second;
	cv::Mat translation2 = cv::Mat::zeros(3,1,cv::DataType<double>::type);
	for(int i=0;i<3;i++) translation2.at<double>(i,0) = translation(i);
	result.setTranslation(translation2);
	return result;
}


void filterimage(cv::Mat &image)
{
	cv::Mat input = image.clone();
	for(int x=1;x<image.cols-1;x++){
		for(int y=1;y<image.rows-1;y++){
			if(std::isfinite(input.at<float>(y,x))){
				float sum = 0.0f; float count = 0.0f;
				for(int dx=-1;dx<=1;dx++){
					for(int dy=-1;dy<=1;dy++){
						if(std::isfinite(input.at<float>(y+dy,x+dx)) && fabs(input.at<float>(y,x)-input.at<float>(y+dy,x+dx))<0.1f){
							sum += input.at<float>(y+dy,x+dx); count += 1.0f;
						}
					}
				}
				image.at<float>(y,x) = sum/count;
			}
		}
	}
}


#include <list>

int main_orig(int argc, char *argv[])
{
//--------------------------
	fprintf(stderr,"\nSize of a MCNSplit: %li",sizeof(MCNSplit));
	fprintf(stderr,"\nSize of MCNCompact: %li",sizeof(MCNCompact));

	stb::doublevector_soa<int,float> testvector;
	fprintf(stderr,"\nFilling");
	for(size_t i=0;i<18;i++){
		testvector.push_back(i,float(i));
		fprintf(stderr," [%li %li]",i,testvector.capacity());
	}
	fprintf(stderr,"\nChanging");
	for(size_t i=0;i<18;i++){
		fprintf(stderr," (%i %f)",testvector[i].first,testvector[i].second);
		testvector[i].second = -testvector[i].second;
	}
	fprintf(stderr,"\nPrinting");
	for(size_t i=0;i<18;i++){
		fprintf(stderr," (%i %f)",testvector[i].first,testvector[i].second);
	}
	fprintf(stderr,"\nPrinting with iterator");
	for(stb::doublevector_soa<int,float>::iterator it=testvector.begin();it!=testvector.end();it++){
//		fprintf(stderr," (%i %f)",(*it).first,(*it).second);
		fprintf(stderr," (%i %f)",it->first,it->second);
	}
	fprintf(stderr,"\nEmptying");
	for(size_t i=0;i<18;i++){
		testvector.pop_back();
		fprintf(stderr," [%li %li]",18-i,testvector.capacity());
	}


	std::list<unsigned int> source;
	source.push_back(0);
	source.push_back(1);
	source.push_back(2);
	source.push_back(3);
	source.push_back(4);
	std::list<unsigned int> target;
	target.push_back(5);
	target.push_back(6);
	target.push_back(7);
	target.push_back(8);
	target.push_back(9);

	std::list<unsigned int>::iterator it = source.begin();
	it++; it++;
	target.splice(target.end(),source,it++);
	fprintf(stderr,"\nTarget1: ");
	for(std::list<unsigned int>::iterator j=target.begin();j!=target.end();j++) fprintf(stderr," %i",*j);
	target.splice(target.end(),source,it++);
	fprintf(stderr,"\nTarget2: ");
	for(std::list<unsigned int>::iterator j=target.begin();j!=target.end();j++) fprintf(stderr," %i",*j);

	fprintf(stderr,"\nSize of MeshCell: %li, Size of MeshCellNeighborhood: %li",
			sizeof(MeshCell),sizeof(MCNSplit));
	fprintf(stderr,"\nSize of MeshSeparate*: %li, MeshInterleaved*: %li, 4*sidetype: %li, 2*8*volumetype: %li",
			sizeof(MeshSeparate*),sizeof(MeshInterleaved*),4*sizeof(sidetype),2*8*sizeof(volumetype));
	fprintf(stderr,"\nSize of MeshCellNeighborhood::CellList: %li",sizeof(MCNSplit::CellList));



	std::string meshname = "";
	bool noViewer = true;
	bool bufferImages = false;
	bool useColor = true;
	bool volumeColor = true;


	bool useLoopClosures = false;

	unsigned int startimage = 0;
	unsigned int endimage = 1000000;
	unsigned int imageStep = 1;

	float maxCamDistance = MAXCAMDISTANCE;
	float scale = DEFAULT_SCALE;
	float threshold = DEFAULT_SCALE;

	float imageDepthScale = DEPTHSCALE;

	std::string intrinsics = "";

//	bool threadMeshing = true;  //@@@ orig
	bool threadMeshing = false;

	bool threadFusion = false;


	bool threadImageReading = false;
	bool performIncrementalMeshing = true;

	int depthConstistencyChecks = 0;

  TCLAP::CmdLine cmdLine("onlinefusion");

  TCLAP::ValueArg<std::string> loadMeshArg("l","loadmesh","Loads this mesh",false,meshname,"string");
  TCLAP::SwitchArg threadMeshingArg("","thread-meshing","Thread the Meshing inside the Fusion",threadMeshing);
  TCLAP::SwitchArg threadFusionArg("","thread-fusion","Thread the Fusion inside the Viewer",threadFusion);
  TCLAP::SwitchArg threadImageReadingArg("","thread-image","Thread reading the Images from Hard Disk",threadImageReading);
  TCLAP::SwitchArg viewerArg("v","viewer","Show a Viewer after Fusion",!noViewer);
  TCLAP::SwitchArg bufferArg("b","buffer","Buffer all Images",bufferImages);
  TCLAP::SwitchArg useLoopClosuresArg("c","loopclosures","Read Multiple Trajectories and perform Loop Closures",useLoopClosures);
  TCLAP::SwitchArg performIncrementalMeshingArg("","incremental-meshing","Perform incremental Meshing",performIncrementalMeshing);
  TCLAP::ValueArg<int> startimageArg("s","startimage","Number of the Start Image",false,startimage,"int");
  TCLAP::ValueArg<int> endimageArg("e","endimage","Number of the End Image",false,endimage,"int");
  TCLAP::ValueArg<int> imageStepArg("k","imagestep","Use every kth step",false,imageStep,"int");
  TCLAP::ValueArg<int> depthConsistencyChecksArg("","consistency-checks","Number of Depth Consistency Checks",false,depthConstistencyChecks,"int");
  TCLAP::ValueArg<float> maxCamDistanceArg("","max-camera-distance","Maximum Camera Distance to Surface",false,maxCamDistance,"float");
  TCLAP::ValueArg<float> scaleArg("","scale","Size of the Voxel",false,scale,"float");
  TCLAP::ValueArg<float> thresholdArg("","threshold","Threshold",false,threshold,"float");
  TCLAP::ValueArg<float> imageDepthScaleArg("","imagescale","Image Depth Scale",false,imageDepthScale,"float");
  TCLAP::ValueArg<std::string> intrinsicsArg("","intrinsics","File with Camera Matrix",false,intrinsics,"string");

  TCLAP::UnlabeledMultiArg<std::string> associationfilenamesArg("filenames", "The File Names",false,"string");


  cmdLine.add(loadMeshArg);
  cmdLine.add(threadMeshingArg);
  cmdLine.add(threadFusionArg);
  cmdLine.add(threadImageReadingArg);
  cmdLine.add(viewerArg);
  cmdLine.add(bufferArg);
  cmdLine.add(useLoopClosuresArg);
  cmdLine.add(performIncrementalMeshingArg);
  cmdLine.add(startimageArg);
  cmdLine.add(endimageArg);
  cmdLine.add(imageStepArg);
  cmdLine.add(depthConsistencyChecksArg);
  cmdLine.add(maxCamDistanceArg);
  cmdLine.add(scaleArg);
  cmdLine.add(thresholdArg);
  cmdLine.add(imageDepthScaleArg);
  cmdLine.add(intrinsicsArg);

  cmdLine.add(associationfilenamesArg);
  cmdLine.parse(argc,argv);

  meshname = loadMeshArg.getValue();
  threadMeshing = threadMeshingArg.getValue();
  threadFusion = threadFusionArg.getValue();
  threadImageReading = threadImageReadingArg.getValue();
  noViewer = !viewerArg.getValue();
  bufferImages = bufferArg.getValue();
  useLoopClosures = useLoopClosuresArg.getValue();
  performIncrementalMeshing = performIncrementalMeshingArg.getValue();
  startimage = startimageArg.getValue();
  endimage = endimageArg.getValue();
  imageStep = imageStepArg.getValue();
  if(imageStep < 1) imageStep = 1;
  depthConstistencyChecks = depthConsistencyChecksArg.getValue();
  maxCamDistance = maxCamDistanceArg.getValue();
  scale = scaleArg.getValue();
  threshold = thresholdArg.getValue();
  imageDepthScale = imageDepthScaleArg.getValue();
  intrinsics = intrinsicsArg.getValue();

  float fx = 525.0f;
  float fy = 525.0f;
  float cx = 319.5f;
  float cy = 239.5f;

  if (intrinsics != "") {
  	fprintf(stderr,"\nReading intrinsic camera parameters from %s",intrinsics.c_str());
  	std::fstream intrinsicsfile;
  	intrinsicsfile.open(intrinsics.c_str(),std::ios::in);
  	if(!intrinsicsfile.is_open()){
  		fprintf(stderr,"\nERROR: Could not open File %s for reading!",intrinsics.c_str());
  	} else {
  		float temp;
  		fx = fy = cx = cy = -1.0f;
  		intrinsicsfile >> fx;
  		intrinsicsfile >> temp;
  		intrinsicsfile >> cx;
  		intrinsicsfile >> temp;
  		intrinsicsfile >> fy;
  		intrinsicsfile >> cy;
  		intrinsicsfile.close();
  		fprintf(stderr,"\nCamera Intrinsics from File: %f %f %f %f",fx,fy,cx,cy);
  	}
  }

  if(threadMeshing) fprintf(stderr,"\nMeshing will run in a separate Thread");
  else              fprintf(stderr,"\nMeshing will NOT run in a separate Thread");
  if(threadFusion)  fprintf(stderr,"\nFusion will run in a separate Thread");
  if(threadImageReading) fprintf(stderr,"\nImage Reading will run in a separate Thread");



	CameraInfo startpos;

	std::string defaultname = "../test/rgbd_dataset_freiburg3_long_office_household/associationVICOM.txt";

	std::string tempfileprefix = "debug/";

	std::vector<std::string> associationfilenames = associationfilenamesArg.getValue();
	std::vector<std::string> prefices;

	if(!associationfilenames.size()){
		associationfilenames.push_back(defaultname);
	}


	for(unsigned int i=0;i<associationfilenames.size();i++){
		prefices.push_back(associationfilenames[i].substr(0,associationfilenames[i].find_last_of('/')+1));
	}



//	std::vector<Sophus::SE3> posesSophus;
	std::vector<std::pair<Eigen::Matrix3d,Eigen::Vector3d> > poses_from_assfile;
	std::vector<std::vector<std::string> > depthNames;
	std::vector<std::vector<std::string> > rgbNames;
	std::vector<std::vector<CameraInfo> > poses;

	if(!useLoopClosures){
		fprintf(stderr,"\nBuilding a single Trajectory...");
		poses.push_back(std::vector<CameraInfo>());
		depthNames.push_back(std::vector<std::string>());
		rgbNames.push_back(std::vector<std::string>());
		std::vector<CameraInfo> &trajectory = poses.back();
		std::vector<std::string> &depthNamesLast = depthNames.back();
		std::vector<std::string> &rgbNamesLast = rgbNames.back();
		unsigned int assfilestartindex = 0;
		for(unsigned int f=0;f<associationfilenames.size();f++){
			std::string prefix = prefices[f];
			std::fstream associationfile; float junkstamp;
			std::string depthname; std::string rgbname;
			float q1, q2, q3, q4, translation1, translation2, translation3;
			associationfile.open(associationfilenames[f].c_str(),std::ios::in);
			if(!associationfile.is_open()){
				fprintf(stderr,"\nERROR: Could not open File %s",associationfilenames[f].c_str());
			}else{
				fprintf(stderr,"\nReading Association File");
				while(!associationfile.eof()){
					std::string temp("");
					getline(associationfile,temp);
					std::stringstream stream(temp);
					stream >> junkstamp;
					stream >> translation1; stream >> translation2; stream >> translation3;
					stream >> q1; stream >> q2; stream >> q3; stream >> q4;
					stream >> junkstamp;
					stream >> depthname;
					if(temp!=""){
//						posesSophus.push_back(Sophus::SE3(Eigen::Quaterniond(q4,q1,q2,q3),Eigen::Vector3d(translation1,translation2,translation3)));
						poses_from_assfile.push_back(std::pair<Eigen::Matrix3d,Eigen::Vector3d>(Eigen::Quaterniond(q4,q1,q2,q3).toRotationMatrix(),Eigen::Vector3d(translation1,translation2,translation3)));
						depthNamesLast.push_back(depthname);
						if(useColor){
							stream >> junkstamp; stream >> rgbname; rgbNamesLast.push_back(rgbname);
						}
					}
				}
				fprintf(stderr,"\nRead %i Poses from Association File Nr %i : %s .",
						(int)depthNamesLast.size()-assfilestartindex,f,associationfilenames[f].c_str());
			}

			for(unsigned int i=assfilestartindex;i<poses_from_assfile.size();i++){
//				trajectory.push_back(kinectPoseFromSophus(posesSophus[i]));
//				trajectory.push_back(kinectPoseFromSophus(posesSophus[i],fx,fy,cx,cy));
				trajectory.push_back(kinectPoseFromEigen(poses_from_assfile[i],fx,fy,cx,cy));
				trajectory.back().setExtrinsic(startpos.getExtrinsic()*trajectory.back().getExtrinsic());
				depthNamesLast[i] = prefix + depthNamesLast[i];
				if(useColor) rgbNamesLast[i] = prefix + rgbNamesLast[i];
			}
			startpos = trajectory.back();
			assfilestartindex = depthNamesLast.size();
//			posesSophus.clear();
			poses_from_assfile.clear();
		}
	}
	else{
		fprintf(stderr,"\nReading in multiple Trajectories");
		std::vector<std::string> trajectoryNames;
		std::fstream metafile;
		if(!associationfilenames.size()){
			fprintf(stderr,"\nERROR: No association File!");
			return 1;
		}
		metafile.open(associationfilenames[0].c_str(),std::ios::in);
		if(!metafile.is_open()){
			fprintf(stderr,"\nERROR: Could not open Trajectory Metafile \"%s\"",associationfilenames[0].c_str());
			return 1;
		}
		while(!metafile.eof()){
			std::string filename;
			metafile >> filename;
			if(filename!=""){
//				fprintf(stderr,"\n%s",filename.c_str());
				trajectoryNames.push_back(prefices[0] + filename);
			}
		}
		metafile.close();
		fprintf(stderr,"\nRead in %li Trajectories from Trajectory Metafile",trajectoryNames.size());
		for(size_t i=0;i<trajectoryNames.size();i++){
//			posesSophus.clear();
			poses_from_assfile.clear();
			poses.push_back(std::vector<CameraInfo>());
			depthNames.push_back(std::vector<std::string>());
			rgbNames.push_back(std::vector<std::string>());
			std::vector<CameraInfo> &trajectory = poses.back();
			std::vector<std::string> &depthNamesLast = depthNames.back();
			std::vector<std::string> &rgbNamesLast = rgbNames.back();
			std::string prefix = prefices[0];
			std::fstream associationfile; float junkstamp;
			std::string depthname; std::string rgbname;
			float q1, q2, q3, q4, translation1, translation2, translation3;
			associationfile.open(trajectoryNames[i].c_str(),std::ios::in);
			if(!associationfile.is_open()){
				fprintf(stderr,"\nERROR: Could not open File %s",trajectoryNames[i].c_str());
			}else{
//				fprintf(stderr,"\nReading Association File \"%s\"",trajectoryNames[i].c_str());
				while(!associationfile.eof()){
					std::string temp("");
					getline(associationfile,temp);
					std::stringstream stream(temp);
					stream >> junkstamp;
					stream >> translation1; stream >> translation2; stream >> translation3;
					stream >> q1; stream >> q2; stream >> q3; stream >> q4;
					stream >> junkstamp;
					stream >> depthname;
					if(temp!=""){
//						posesSophus.push_back(Sophus::SE3(Eigen::Quaterniond(q4,q1,q2,q3),Eigen::Vector3d(translation1,translation2,translation3)));
						poses_from_assfile.push_back(std::pair<Eigen::Matrix3d,Eigen::Vector3d>(Eigen::Quaterniond(q4,q1,q2,q3).toRotationMatrix(),Eigen::Vector3d(translation1,translation2,translation3)));
						depthNamesLast.push_back(depthname);
						if(useColor){
							stream >> junkstamp; stream >> rgbname; rgbNamesLast.push_back(rgbname);
						}
					}
				}
//				fprintf(stderr,"\nRead %i Poses from Trajectory File Nr %li : %s .",
//						(int)depthNames[i].size(),i,trajectoryNames[i].c_str());
			}

			for(unsigned int i=0;i<poses_from_assfile.size();i++){
//				trajectory.push_back(kinectPoseFromSophus(posesSophus[i]));
//				trajectory.push_back(kinectPoseFromSophus(posesSophus[i],fx,fy,cx,cy));
				trajectory.push_back(kinectPoseFromEigen(poses_from_assfile[i],fx,fy,cx,cy));
				trajectory.back().setExtrinsic(startpos.getExtrinsic()*trajectory.back().getExtrinsic());
				depthNamesLast[i] = prefix + depthNamesLast[i];
				if(useColor) rgbNamesLast[i] = prefix + rgbNamesLast[i];
			}
		}
	}

	if(!useLoopClosures){
		fprintf(stderr,"\nRead %i Poses und Depth Images and %i RGB Images from %i Association Files",
				(int)depthNames.size(), (int)rgbNames.size(), (int)associationfilenames.size());
	}
	else{
		fprintf(stderr,"\nRead %li Trajectories",depthNames.size());
		//Check
		for(size_t i=1;i<poses.size();i++){
			if(poses[i].size()<poses[i-1].size()){
				fprintf(stderr,"\nERROR: Trajectory %li is shorter than No %li",i,i-1);
			}
			else{
				for(size_t j=0;j<poses[i-1].size();j++){
					if(depthNames[i][j]!=depthNames[i-1][j] || rgbNames[i][j]!=rgbNames[i-1][j]){
						fprintf(stderr,"\nERROR: Images %li at [%li / %li] inconsistent: [%s %s] vs. [%s %s]",
								j,i,i-1,depthNames[i][j].c_str(),depthNames[i-1][j].c_str(),
								rgbNames[i][j].c_str(),rgbNames[i-1][j].c_str());
					}
				}
			}
		}
	}


	if(startimage >= depthNames.front().size()) startimage = depthNames.front().size()-1;
	if(endimage >= depthNames.back().size()) endimage = depthNames.back().size()-1;



//	fprintf(stderr,"\nCreating Mipmapping GPU Octree");
//	Fusion_AoS *fusion = new FusionMipMap(0,0,0,DEFAULT_SCALE,DISTANCETHRESHOLD,0,volumeColor);

	fprintf(stderr,"\nCreating Mipmapping CPU Octree");
	FusionMipMapCPU *fusion = new FusionMipMapCPU(0,0,0,scale,threshold,0,volumeColor);

	fusion->setThreadMeshing(threadMeshing);
	fusion->setDepthChecks(depthConstistencyChecks);
	fusion->setIncrementalMeshing(performIncrementalMeshing);

	fprintf(stderr,"\nCreating Viewer\n");

//	OnlineFusionViewerManipulated viewer(false);
//--------------------------
#if 0
#warning will run without QtApp-dependency
    // ok: removed the QtApp dependency.
    // works for both in terminal and ctypes
    {
        std::vector<std::vector<CameraInfo> > _poses = poses;
        std::vector<std::vector<std::string> > _depthNames = depthNames;
        std::vector<std::vector<std::string> > _rgbNames = rgbNames;
        FusionMipMapCPU* _fusion = fusion;
        float _imageDepthScale = imageDepthScale;
        float _maxCamDistance = maxCamDistance;
        long int _currentFrame = (long int)startimage-1;
        long int _currentTrajectory;
        //long int _firstFrame = (long int)startimage;
        long int _nextStopFrame = endimage;

        bool _newMesh = false;
        bool _fusionActive = true;
        bool _fusionAlive = true;
        bool _threadImageReading = false;

        if(_poses.size()==1) {
            _currentTrajectory = _poses.size()-1;
        }

        if (1) {
            fprintf(stderr, "@@@ %s(): calling the fusion loop in this main thread...\n", __FUNCTION__);
//legathy
//            OnlineFusionViewerInterface::fusionWrapperIf(_depthNames,_rgbNames,_poses,
//                FusionParameter(_fusion,_imageDepthScale,_maxCamDistance,_threadImageReading,_nextStopFrame),
//                &_currentFrame,&_currentTrajectory,&_newMesh,&_fusionActive,&_fusionAlive);
            FusionWrapperIfData *pFusionWrapperIfData = OnlineFusionViewerInterface::fusionWrapperIfInit(&_depthNames,&_rgbNames,&_poses,
					FusionParameter(_fusion,_imageDepthScale,_maxCamDistance,_threadImageReading,_nextStopFrame),
					&_currentFrame,&_currentTrajectory,&_newMesh,&_fusionActive,&_fusionAlive);
            OnlineFusionViewerInterface::fusionWrapperIf(
                pFusionWrapperIfData,
                //_depthNames,_rgbNames,_poses,
                //FusionParameter(_fusion,_imageDepthScale,_maxCamDistance,_threadImageReading,_nextStopFrame),
                &_currentFrame,
                &_currentTrajectory,
                //&_newMesh,
                &_fusionActive,
                &_fusionAlive);
        } else {
//legathy code
//            boost::thread *_fusionThread = new boost::thread(OnlineFusionViewerInterface::fusionWrapperIf,_depthNames,_rgbNames,_poses,
//                FusionParameter(_fusion,_imageDepthScale,_maxCamDistance,_threadImageReading,_nextStopFrame),
//                &_currentFrame,&_currentTrajectory,&_newMesh,&_fusionActive,&_fusionAlive);
            FusionWrapperIfData *pFusionWrapperIfData = OnlineFusionViewerInterface::fusionWrapperIfInit(&_depthNames,&_rgbNames,&_poses,
					FusionParameter(_fusion,_imageDepthScale,_maxCamDistance,_threadImageReading,_nextStopFrame),
					&_currentFrame,&_currentTrajectory,&_newMesh,&_fusionActive,&_fusionAlive);
            boost::thread *_fusionThread = new boost::thread(OnlineFusionViewerInterface::fusionWrapperIf,
                  pFusionWrapperIfData,
                  //_depthNames,_rgbNames,_poses,
                  //FusionParameter(_fusion,_imageDepthScale,_maxCamDistance,_threadImageReading,_nextStopFrame),
                  &_currentFrame,
                  &_currentTrajectory,
                  //&_newMesh,
                  &_fusionActive,
                  &_fusionAlive);
            fprintf(stderr, "@@@ %s(): spawned _fusionThread %p and sleeping forever...\n", __FUNCTION__, _fusionThread);
            sleep(99999999);
        }
    }
#endif
#if 0
    //crash expected
    //QWidget: Must construct a QApplication before a QPaintDevice
    {
        fprintf(stderr, "@@@ %s(): testing OnlineFusionViewerInterface !!\n", __FUNCTION__);
        OnlineFusionViewerInterface *viewerpointer = new OnlineFusionViewerInterface(false);
        fprintf(stderr, "@@@ %s(): testing OnlineFusionViewerInterface0000000 !!\n", __FUNCTION__);
        OnlineFusionViewerInterface &viewer = *viewerpointer;

        fprintf(stderr,"\nSetting Viewer Parameters");
        viewer._fusion = fusion;
        viewer.setWindowTitle("Fusion Volume");
        viewer._poses = poses;
        viewer._depthNames = depthNames;
        viewer._rgbNames = rgbNames;
        viewer._threadFusion = threadFusion;
        viewer._threadImageReading = threadImageReading;
        viewer.show();
        viewer._imageDepthScale = imageDepthScale;
        viewer._maxCamDistance = maxCamDistance;
        viewer._firstFrame = (long int)startimage;
        viewer._currentFrame = (long int)startimage-1;
        fprintf(stderr,"\nSet Viewer Frame to %li",(long int)viewer._currentFrame);
        viewer._nextStopFrame = endimage;

        //crash expected
        //QWidget: Must construct a QApplication before a QPaintDevice
        viewer.updateSlotIf();


        fprintf(stderr,"\nDeleting Viewer");
        delete viewerpointer;
    }
#endif
#if 0
#warning will run in original mode as QtApp
    // @@@ original
    {
        QApplication application(argc,argv);
        OnlineFusionViewerManipulated *viewerpointer = new OnlineFusionViewerManipulated(false);
        OnlineFusionViewerManipulated &viewer = *viewerpointer;

        fprintf(stderr,"\nSetting Viewer Parameters");
        viewer._fusion = fusion;
        viewer.setWindowTitle("Fusion Volume");
        viewer._poses = poses;
        viewer._depthNames = depthNames;
        viewer._rgbNames = rgbNames;
        viewer._threadFusion = threadFusion;
        viewer._threadImageReading = threadImageReading;
        viewer.show();
        viewer._imageDepthScale = imageDepthScale;
        viewer._maxCamDistance = maxCamDistance;
        viewer._firstFrame = (long int)startimage;
        viewer._currentFrame = (long int)startimage-1;
        fprintf(stderr,"\nSet Viewer Frame to %li",(long int)viewer._currentFrame);
        viewer._nextStopFrame = endimage;
        fprintf(stderr,"\nStarting Qt-Application");
        application.exec();

        fprintf(stderr,"\nDeleting Viewer");
        delete viewerpointer;
    }
#endif

	fprintf(stderr,"\nProgram %s exiting.\n\n", argv[0]);
//	fprintf(stderr,"\nPress Enter exit the Program");
//	char input[256];
//	fprintf(stderr,"%s",fgets(input,256,stdin));
	return 0;
}

typedef struct {
    std::vector<std::vector<CameraInfo> > *_poses;
    std::vector<std::vector<std::string> > *_depthNames;
    std::vector<std::vector<std::string> > *_rgbNames;
    FusionMipMapCPU* _fusion;
    float _imageDepthScale;
    float _maxCamDistance;
    long int _currentFrame;
    long int _currentTrajectory;
    //long int _firstFrame = (long int)startimage;
    long int _nextStopFrame;

    bool _newMesh;
    bool _fusionActive;
    bool _fusionAlive;
    bool _threadImageReading;

    FusionWrapperIfData *pFusionWrapperIfData;
    std::vector<std::string> *depthLast;
    std::vector<std::string> *rgbLast;
    std::vector<CameraInfo> *pLast;

    SensorData *pSensorData;
    SensorData *pSensorDataModel; //use depthinfo of ff model
} lib_data;
lib_data *g_pdata = NULL;

extern "C" void lib_init(char *filename_associate) {
    printf("%s(): init g_pdata ...\n", __FUNCTION__);
    lib_data *pdata = (lib_data *)malloc(sizeof(lib_data));
    pdata->pSensorData = NULL;
    pdata->pSensorDataModel = NULL;

// debug:
//    filename_associate = (char *)"../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/associate.txt";
//    filename_associate = (char *)"../data-fastfusion-tum/rgbd_dataset--monitor200/associate.txt";
//    filename_associate = (char *)"../data-fastfusion-tum/rgbd_dataset-c2-600/associate.txt";
//    filename_associate = (char *)"../data-fastfusion-tum/rgbd_dataset-c3-600/associate.txt";
//    filename_associate = (char *)"../data-fastfusion-tum/rgbd_dataset-c4-600/associate.txt";
//    filename_associate = (char *)"../data-fastfusion-tum/rgbd_dataset--ambassador/associate.txt";

    printf("@@@ %s(): hardcoding argv ...\n", __FUNCTION__);
    char *argv[] = { (char *)"onlinefusion",
                     filename_associate,
                     (char *)"--thread-fusion",
                     NULL };
    int argc = sizeof(argv)/sizeof(argv[0])-1;
    printf("@@@ %s(): argc: %d\n", __FUNCTION__, argc);

    for (int i = 0; i < argc; i++) {
        printf("@@@ %s(): argv[%d]: %s\n", __FUNCTION__, i, argv[i]);
    }

    // this block was taken from the header part of main_orig()
    // assign initialized variables to pdata->*
    {
    //--------------------------
        fprintf(stderr,"\nSize of a MCNSplit: %li",sizeof(MCNSplit));
        fprintf(stderr,"\nSize of MCNCompact: %li",sizeof(MCNCompact));

        stb::doublevector_soa<int,float> testvector;
        fprintf(stderr,"\nFilling");
        for(size_t i=0;i<18;i++){
            testvector.push_back(i,float(i));
            fprintf(stderr," [%li %li]",i,testvector.capacity());
        }
        fprintf(stderr,"\nChanging");
        for(size_t i=0;i<18;i++){
            fprintf(stderr," (%i %f)",testvector[i].first,testvector[i].second);
            testvector[i].second = -testvector[i].second;
        }
        fprintf(stderr,"\nPrinting");
        for(size_t i=0;i<18;i++){
            fprintf(stderr," (%i %f)",testvector[i].first,testvector[i].second);
        }
        fprintf(stderr,"\nPrinting with iterator");
        for(stb::doublevector_soa<int,float>::iterator it=testvector.begin();it!=testvector.end();it++){
    //		fprintf(stderr," (%i %f)",(*it).first,(*it).second);
            fprintf(stderr," (%i %f)",it->first,it->second);
        }
        fprintf(stderr,"\nEmptying");
        for(size_t i=0;i<18;i++){
            testvector.pop_back();
            fprintf(stderr," [%li %li]",18-i,testvector.capacity());
        }


        std::list<unsigned int> source;
        source.push_back(0);
        source.push_back(1);
        source.push_back(2);
        source.push_back(3);
        source.push_back(4);
        std::list<unsigned int> target;
        target.push_back(5);
        target.push_back(6);
        target.push_back(7);
        target.push_back(8);
        target.push_back(9);

        std::list<unsigned int>::iterator it = source.begin();
        it++; it++;
        target.splice(target.end(),source,it++);
        fprintf(stderr,"\nTarget1: ");
        for(std::list<unsigned int>::iterator j=target.begin();j!=target.end();j++) fprintf(stderr," %i",*j);
        target.splice(target.end(),source,it++);
        fprintf(stderr,"\nTarget2: ");
        for(std::list<unsigned int>::iterator j=target.begin();j!=target.end();j++) fprintf(stderr," %i",*j);

        fprintf(stderr,"\nSize of MeshCell: %li, Size of MeshCellNeighborhood: %li",
                sizeof(MeshCell),sizeof(MCNSplit));
        fprintf(stderr,"\nSize of MeshSeparate*: %li, MeshInterleaved*: %li, 4*sidetype: %li, 2*8*volumetype: %li",
                sizeof(MeshSeparate*),sizeof(MeshInterleaved*),4*sizeof(sidetype),2*8*sizeof(volumetype));
        fprintf(stderr,"\nSize of MeshCellNeighborhood::CellList: %li",sizeof(MCNSplit::CellList));



        std::string meshname = "";
        bool noViewer = true;
        bool bufferImages = false;
        bool useColor = true;
        bool volumeColor = true;


        bool useLoopClosures = false;

        unsigned int startimage = 0;
        unsigned int endimage = 1000000;
        unsigned int imageStep = 1;

        float maxCamDistance = MAXCAMDISTANCE;
        float scale = DEFAULT_SCALE;
        float threshold = DEFAULT_SCALE;

        float imageDepthScale = DEPTHSCALE;

        std::string intrinsics = "";

    //	bool threadMeshing = true;  //@@@ orig
        bool threadMeshing = false;

        bool threadFusion = false;


        bool threadImageReading = false;
        bool performIncrementalMeshing = true;

        int depthConstistencyChecks = 0;

      TCLAP::CmdLine cmdLine("onlinefusion");

      TCLAP::ValueArg<std::string> loadMeshArg("l","loadmesh","Loads this mesh",false,meshname,"string");
      TCLAP::SwitchArg threadMeshingArg("","thread-meshing","Thread the Meshing inside the Fusion",threadMeshing);
      TCLAP::SwitchArg threadFusionArg("","thread-fusion","Thread the Fusion inside the Viewer",threadFusion);
      TCLAP::SwitchArg threadImageReadingArg("","thread-image","Thread reading the Images from Hard Disk",threadImageReading);
      TCLAP::SwitchArg viewerArg("v","viewer","Show a Viewer after Fusion",!noViewer);
      TCLAP::SwitchArg bufferArg("b","buffer","Buffer all Images",bufferImages);
      TCLAP::SwitchArg useLoopClosuresArg("c","loopclosures","Read Multiple Trajectories and perform Loop Closures",useLoopClosures);
      TCLAP::SwitchArg performIncrementalMeshingArg("","incremental-meshing","Perform incremental Meshing",performIncrementalMeshing);
      TCLAP::ValueArg<int> startimageArg("s","startimage","Number of the Start Image",false,startimage,"int");
      TCLAP::ValueArg<int> endimageArg("e","endimage","Number of the End Image",false,endimage,"int");
      TCLAP::ValueArg<int> imageStepArg("k","imagestep","Use every kth step",false,imageStep,"int");
      TCLAP::ValueArg<int> depthConsistencyChecksArg("","consistency-checks","Number of Depth Consistency Checks",false,depthConstistencyChecks,"int");
      TCLAP::ValueArg<float> maxCamDistanceArg("","max-camera-distance","Maximum Camera Distance to Surface",false,maxCamDistance,"float");
      TCLAP::ValueArg<float> scaleArg("","scale","Size of the Voxel",false,scale,"float");
      TCLAP::ValueArg<float> thresholdArg("","threshold","Threshold",false,threshold,"float");
      TCLAP::ValueArg<float> imageDepthScaleArg("","imagescale","Image Depth Scale",false,imageDepthScale,"float");
      TCLAP::ValueArg<std::string> intrinsicsArg("","intrinsics","File with Camera Matrix",false,intrinsics,"string");

      TCLAP::UnlabeledMultiArg<std::string> associationfilenamesArg("filenames", "The File Names",false,"string");


      cmdLine.add(loadMeshArg);
      cmdLine.add(threadMeshingArg);
      cmdLine.add(threadFusionArg);
      cmdLine.add(threadImageReadingArg);
      cmdLine.add(viewerArg);
      cmdLine.add(bufferArg);
      cmdLine.add(useLoopClosuresArg);
      cmdLine.add(performIncrementalMeshingArg);
      cmdLine.add(startimageArg);
      cmdLine.add(endimageArg);
      cmdLine.add(imageStepArg);
      cmdLine.add(depthConsistencyChecksArg);
      cmdLine.add(maxCamDistanceArg);
      cmdLine.add(scaleArg);
      cmdLine.add(thresholdArg);
      cmdLine.add(imageDepthScaleArg);
      cmdLine.add(intrinsicsArg);

      cmdLine.add(associationfilenamesArg);
      cmdLine.parse(argc,argv);

      meshname = loadMeshArg.getValue();
      threadMeshing = threadMeshingArg.getValue();
      threadFusion = threadFusionArg.getValue();
      threadImageReading = threadImageReadingArg.getValue();
      noViewer = !viewerArg.getValue();
      bufferImages = bufferArg.getValue();
      useLoopClosures = useLoopClosuresArg.getValue();
      performIncrementalMeshing = performIncrementalMeshingArg.getValue();
      startimage = startimageArg.getValue();
      endimage = endimageArg.getValue();
      imageStep = imageStepArg.getValue();
      if(imageStep < 1) imageStep = 1;
      depthConstistencyChecks = depthConsistencyChecksArg.getValue();
      maxCamDistance = maxCamDistanceArg.getValue();
      scale = scaleArg.getValue();
      threshold = thresholdArg.getValue();
      imageDepthScale = imageDepthScaleArg.getValue();
      intrinsics = intrinsicsArg.getValue();

      float fx = 525.0f;
      float fy = 525.0f;
      float cx = 319.5f;
      float cy = 239.5f;

      if (intrinsics != "") {
        fprintf(stderr,"\nReading intrinsic camera parameters from %s",intrinsics.c_str());
        std::fstream intrinsicsfile;
        intrinsicsfile.open(intrinsics.c_str(),std::ios::in);
        if(!intrinsicsfile.is_open()){
            fprintf(stderr,"\nERROR: Could not open File %s for reading!",intrinsics.c_str());
        } else {
            float temp;
            fx = fy = cx = cy = -1.0f;
            intrinsicsfile >> fx;
            intrinsicsfile >> temp;
            intrinsicsfile >> cx;
            intrinsicsfile >> temp;
            intrinsicsfile >> fy;
            intrinsicsfile >> cy;
            intrinsicsfile.close();
            fprintf(stderr,"\nCamera Intrinsics from File: %f %f %f %f",fx,fy,cx,cy);
        }
      }

      if(threadMeshing) fprintf(stderr,"\nMeshing will run in a separate Thread");
      else              fprintf(stderr,"\nMeshing will NOT run in a separate Thread");
      if(threadFusion)  fprintf(stderr,"\nFusion will run in a separate Thread");
      if(threadImageReading) fprintf(stderr,"\nImage Reading will run in a separate Thread");



        CameraInfo startpos;

        std::string defaultname = "../test/rgbd_dataset_freiburg3_long_office_household/associationVICOM.txt";

        std::string tempfileprefix = "debug/";

        std::vector<std::string> associationfilenames = associationfilenamesArg.getValue();
        std::vector<std::string> prefices;

        if(!associationfilenames.size()){
            associationfilenames.push_back(defaultname);
        }


        for(unsigned int i=0;i<associationfilenames.size();i++){
            prefices.push_back(associationfilenames[i].substr(0,associationfilenames[i].find_last_of('/')+1));
        }



    //	std::vector<Sophus::SE3> posesSophus;
        std::vector<std::pair<Eigen::Matrix3d,Eigen::Vector3d> > poses_from_assfile;
        //std::vector<std::vector<std::string> > depthNames;
        //std::vector<std::vector<std::string> > rgbNames;
        //std::vector<std::vector<CameraInfo> > poses;

        //http://stackoverflow.com/questions/9286390/c-pointer-and-reference-with-new-keyword-when-instantiating
        std::vector<std::vector<std::string> > *p_depthNames =
            new std::vector<std::vector<std::string> >();
        std::vector<std::vector<std::string> > &depthNames = *p_depthNames;
        std::vector<std::vector<std::string> > *p_rgbNames =
            new std::vector<std::vector<std::string> >();
        std::vector<std::vector<std::string> > &rgbNames = *p_rgbNames;
        std::vector<std::vector<CameraInfo> > *p_poses =
            new std::vector<std::vector<CameraInfo> >();
        std::vector<std::vector<CameraInfo> > &poses = *p_poses;

        if(!useLoopClosures){
            fprintf(stderr,"\nBuilding a single Trajectory...");
            poses.push_back(std::vector<CameraInfo>());
            depthNames.push_back(std::vector<std::string>());
            rgbNames.push_back(std::vector<std::string>());
            std::vector<CameraInfo> &trajectory = poses.back();
            std::vector<std::string> &depthNamesLast = depthNames.back();
            std::vector<std::string> &rgbNamesLast = rgbNames.back();
            unsigned int assfilestartindex = 0;
            for(unsigned int f=0;f<associationfilenames.size();f++){
                std::string prefix = prefices[f];
                std::fstream associationfile; float junkstamp;
                std::string depthname; std::string rgbname;
                float q1, q2, q3, q4, translation1, translation2, translation3;
                associationfile.open(associationfilenames[f].c_str(),std::ios::in);
                if(!associationfile.is_open()){
                    fprintf(stderr,"\nERROR: Could not open File %s",associationfilenames[f].c_str());
                }else{
                    fprintf(stderr,"\nReading Association File");
                    while(!associationfile.eof()){
                        std::string temp("");
                        getline(associationfile,temp);
                        std::stringstream stream(temp);
                        stream >> junkstamp;
                        stream >> translation1; stream >> translation2; stream >> translation3;
                        stream >> q1; stream >> q2; stream >> q3; stream >> q4;
                        stream >> junkstamp;
                        stream >> depthname;
                        if(temp!=""){
    //						posesSophus.push_back(Sophus::SE3(Eigen::Quaterniond(q4,q1,q2,q3),Eigen::Vector3d(translation1,translation2,translation3)));
                            poses_from_assfile.push_back(std::pair<Eigen::Matrix3d,Eigen::Vector3d>(Eigen::Quaterniond(q4,q1,q2,q3).toRotationMatrix(),Eigen::Vector3d(translation1,translation2,translation3)));
                            depthNamesLast.push_back(depthname);
                            if(useColor){
                                stream >> junkstamp; stream >> rgbname; rgbNamesLast.push_back(rgbname);
                            }
                        }
                    }
                    fprintf(stderr,"\nRead %i Poses from Association File Nr %i : %s .",
                            (int)depthNamesLast.size()-assfilestartindex,f,associationfilenames[f].c_str());
                }

                for(unsigned int i=assfilestartindex;i<poses_from_assfile.size();i++){
    //				trajectory.push_back(kinectPoseFromSophus(posesSophus[i]));
    //				trajectory.push_back(kinectPoseFromSophus(posesSophus[i],fx,fy,cx,cy));
                    trajectory.push_back(kinectPoseFromEigen(poses_from_assfile[i],fx,fy,cx,cy));
                    trajectory.back().setExtrinsic(startpos.getExtrinsic()*trajectory.back().getExtrinsic());
                    depthNamesLast[i] = prefix + depthNamesLast[i];
                    if(useColor) rgbNamesLast[i] = prefix + rgbNamesLast[i];
                }
                startpos = trajectory.back();
                assfilestartindex = depthNamesLast.size();
    //			posesSophus.clear();
                poses_from_assfile.clear();
            }
        }
        else{
            fprintf(stderr,"\nReading in multiple Trajectories");
            std::vector<std::string> trajectoryNames;
            std::fstream metafile;
            if(!associationfilenames.size()){
                fprintf(stderr,"\nERROR: No association File!");
                //return 1;
                return;
            }
            metafile.open(associationfilenames[0].c_str(),std::ios::in);
            if(!metafile.is_open()){
                fprintf(stderr,"\nERROR: Could not open Trajectory Metafile \"%s\"",associationfilenames[0].c_str());
                //return 1;
                return;
            }
            while(!metafile.eof()){
                std::string filename;
                metafile >> filename;
                if(filename!=""){
    //				fprintf(stderr,"\n%s",filename.c_str());
                    trajectoryNames.push_back(prefices[0] + filename);
                }
            }
            metafile.close();
            fprintf(stderr,"\nRead in %li Trajectories from Trajectory Metafile",trajectoryNames.size());
            for(size_t i=0;i<trajectoryNames.size();i++){
    //			posesSophus.clear();
                poses_from_assfile.clear();
                poses.push_back(std::vector<CameraInfo>());
                depthNames.push_back(std::vector<std::string>());
                rgbNames.push_back(std::vector<std::string>());
                std::vector<CameraInfo> &trajectory = poses.back();
                std::vector<std::string> &depthNamesLast = depthNames.back();
                std::vector<std::string> &rgbNamesLast = rgbNames.back();
                std::string prefix = prefices[0];
                std::fstream associationfile; float junkstamp;
                std::string depthname; std::string rgbname;
                float q1, q2, q3, q4, translation1, translation2, translation3;
                associationfile.open(trajectoryNames[i].c_str(),std::ios::in);
                if(!associationfile.is_open()){
                    fprintf(stderr,"\nERROR: Could not open File %s",trajectoryNames[i].c_str());
                }else{
    //				fprintf(stderr,"\nReading Association File \"%s\"",trajectoryNames[i].c_str());
                    while(!associationfile.eof()){
                        std::string temp("");
                        getline(associationfile,temp);
                        std::stringstream stream(temp);
                        stream >> junkstamp;
                        stream >> translation1; stream >> translation2; stream >> translation3;
                        stream >> q1; stream >> q2; stream >> q3; stream >> q4;
                        stream >> junkstamp;
                        stream >> depthname;
                        if(temp!=""){
    //						posesSophus.push_back(Sophus::SE3(Eigen::Quaterniond(q4,q1,q2,q3),Eigen::Vector3d(translation1,translation2,translation3)));
                            poses_from_assfile.push_back(std::pair<Eigen::Matrix3d,Eigen::Vector3d>(Eigen::Quaterniond(q4,q1,q2,q3).toRotationMatrix(),Eigen::Vector3d(translation1,translation2,translation3)));
                            depthNamesLast.push_back(depthname);
                            if(useColor){
                                stream >> junkstamp; stream >> rgbname; rgbNamesLast.push_back(rgbname);
                            }
                        }
                    }
    //				fprintf(stderr,"\nRead %i Poses from Trajectory File Nr %li : %s .",
    //						(int)depthNames[i].size(),i,trajectoryNames[i].c_str());
                }

                for(unsigned int i=0;i<poses_from_assfile.size();i++){
    //				trajectory.push_back(kinectPoseFromSophus(posesSophus[i]));
    //				trajectory.push_back(kinectPoseFromSophus(posesSophus[i],fx,fy,cx,cy));
                    trajectory.push_back(kinectPoseFromEigen(poses_from_assfile[i],fx,fy,cx,cy));
                    trajectory.back().setExtrinsic(startpos.getExtrinsic()*trajectory.back().getExtrinsic());
                    depthNamesLast[i] = prefix + depthNamesLast[i];
                    if(useColor) rgbNamesLast[i] = prefix + rgbNamesLast[i];
                }
            }
        }

        if(!useLoopClosures){
            fprintf(stderr,"\nRead %i Poses und Depth Images and %i RGB Images from %i Association Files",
                    (int)depthNames.size(), (int)rgbNames.size(), (int)associationfilenames.size());
        }
        else{
            fprintf(stderr,"\nRead %li Trajectories",depthNames.size());
            //Check
            for(size_t i=1;i<poses.size();i++){
                if(poses[i].size()<poses[i-1].size()){
                    fprintf(stderr,"\nERROR: Trajectory %li is shorter than No %li",i,i-1);
                }
                else{
                    for(size_t j=0;j<poses[i-1].size();j++){
                        if(depthNames[i][j]!=depthNames[i-1][j] || rgbNames[i][j]!=rgbNames[i-1][j]){
                            fprintf(stderr,"\nERROR: Images %li at [%li / %li] inconsistent: [%s %s] vs. [%s %s]",
                                    j,i,i-1,depthNames[i][j].c_str(),depthNames[i-1][j].c_str(),
                                    rgbNames[i][j].c_str(),rgbNames[i-1][j].c_str());
                        }
                    }
                }
            }
        }


        if(startimage >= depthNames.front().size()) startimage = depthNames.front().size()-1;
        if(endimage >= depthNames.back().size()) endimage = depthNames.back().size()-1;



    //	fprintf(stderr,"\nCreating Mipmapping GPU Octree");
    //	Fusion_AoS *fusion = new FusionMipMap(0,0,0,DEFAULT_SCALE,DISTANCETHRESHOLD,0,volumeColor);

        fprintf(stderr,"\nCreating Mipmapping CPU Octree");
        FusionMipMapCPU *fusion = new FusionMipMapCPU(0,0,0,scale,threshold,0,volumeColor);

        fusion->setThreadMeshing(threadMeshing);
        fusion->setDepthChecks(depthConstistencyChecks);
        fusion->setIncrementalMeshing(performIncrementalMeshing);

        fprintf(stderr,"\nCreating Viewer\n");

    //	OnlineFusionViewerManipulated viewer(false);
    //--------------------------
        pdata->_poses = p_poses;
        pdata->_depthNames = p_depthNames;
        pdata->_rgbNames = p_rgbNames;

        pdata->_fusion = fusion;
        pdata->_imageDepthScale = imageDepthScale;
        pdata->_maxCamDistance = maxCamDistance;
        pdata->_currentFrame = (long int)startimage-1;

        pdata->_nextStopFrame = endimage;
        pdata->_newMesh = false;
        pdata->_fusionActive = true;
        pdata->_fusionAlive = true;
        pdata->_threadImageReading = false;

        if(pdata->_poses->size()==1) {
            pdata->_currentTrajectory = pdata->_poses->size()-1;
        }
    }

    pdata->pFusionWrapperIfData = OnlineFusionViewerInterface::fusionWrapperIfInit(
        pdata->_depthNames,
        pdata->_rgbNames,
        pdata->_poses,
        FusionParameter(pdata->_fusion, pdata->_imageDepthScale, pdata->_maxCamDistance, pdata->_threadImageReading, pdata->_nextStopFrame),
        &pdata->_currentFrame,
        &pdata->_currentTrajectory,
        &pdata->_newMesh,
        &pdata->_fusionActive,
        &pdata->_fusionAlive);

    pdata->depthLast = &pdata->pFusionWrapperIfData->depthNames->back();
    pdata->rgbLast = &pdata->pFusionWrapperIfData->rgbNames->back();
    pdata->pLast = &pdata->pFusionWrapperIfData->poses->back();
    printf("@@@ %s(): depthLast.size(): %ld\n", __FUNCTION__, pdata->depthLast->size());
    pdata->_currentFrame = pdata->pFusionWrapperIfData->startFrame;
    printf("@@@ %s(): done\n", __FUNCTION__);
    g_pdata = pdata;
}

extern "C" void lib_fusion_frame() {
    lib_data *pdata = g_pdata;

    OnlineFusionViewerInterface::fusionWrapperIfOneStep(
        pdata->_currentFrame,
        pdata->depthLast,
        pdata->rgbLast,
        pdata->pFusionWrapperIfData->fusion,
        pdata->pLast,
        pdata->pFusionWrapperIfData->imageDepthScale,
        pdata->pFusionWrapperIfData->maxCamDistance,
        pdata->pFusionWrapperIfData->newMesh);
}

extern "C" void lib_increment_frame() {
    lib_data *pdata = g_pdata;
    pdata->_currentFrame++;
}

extern "C" void lib_main() {
    fprintf(stderr, "@@@ %s(): hello\n", __FUNCTION__);
#if 0
    for (int i = 0; i < 2; i++) {
        lib_fusion_frame();
        lib_increment_frame();
    }
#endif
#if 0
    lib_data *pdata = g_pdata;
    fprintf(stderr, "@@@ %s(): calling the fusion loop...\n", __FUNCTION__);
    OnlineFusionViewerInterface::fusionWrapperIf(
        pdata->pFusionWrapperIfData,
//        pdata->_depthNames, pdata->_rgbNames, pdata->_poses,
//        FusionParameter(pdata->_fusion, pdata->_imageDepthScale, pdata->_maxCamDistance, pdata->_threadImageReading, pdata->_nextStopFrame),
        &pdata->_currentFrame,
        &pdata->_currentTrajectory,
//        &pdata->_newMesh,
        &pdata->_fusionActive,
        &pdata->_fusionAlive);
#endif
}

extern "C" void lib_clean() {
    lib_data *pdata = g_pdata;
    assert(g_pdata != NULL);

    printf("%s(): freeing g_pdata: %p\n", __FUNCTION__, pdata);
    //free dynamic data here
    if (pdata->pSensorData != NULL) {
        FusionMipMapCPU::freeSensorData(&pdata->pSensorData);
    }
    if (pdata->pSensorDataModel != NULL) {
        FusionMipMapCPU::freeSensorData(&pdata->pSensorDataModel);
    }

    free(pdata->pFusionWrapperIfData);

    delete pdata->_poses;  //delete pLast as well
    delete pdata->_depthNames;  //delete depthLast as well
    delete pdata->_rgbNames;  //delete rgbLast as well
    delete pdata->_fusion;

    g_pdata = NULL;
}

extern "C" void main_lib() {
    printf("@@@ %s(): hardcoding argv ...\n", __FUNCTION__);
    char *argv[] = { (char *)"onlinefusion",
                     (char *)"../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/associate.txt",
                     (char *)"--thread-fusion",
                     NULL };
    int argc = sizeof(argv)/sizeof(argv[0])-1;
    printf("@@@ %s(): argc: %d\n", __FUNCTION__, argc);

    for (int i = 0; i < argc; i++) {
        printf("@@@ %s(): argv[%d]: %s\n", __FUNCTION__, i, argv[i]);
    }
    main_orig(argc, argv);
}

CameraInfo * util_get_caminfo_current(lib_data *pdata) {
    long int currentFrame = pdata->_currentFrame;
    //printf("@@@ %s(): currentFrame: %ld\n", __FUNCTION__, currentFrame);
    //std::vector<CameraInfo> *pLast = pdata->pLast;
    return &(*pdata->pLast)[currentFrame];
}

const char * util_get_timestamp_string(lib_data *pdata, long int iFrame) {
    std::vector<std::vector<std::string> > &rRgbNames = *pdata->_rgbNames;
    std::vector<std::string> &rRgbLast = rRgbNames.back();

    if ((iFrame < 0) || (iFrame >= rRgbLast.size())) {
        fprintf(stderr, "%s(): warning: invalid frame number: %d\n",
                __FUNCTION__, iFrame);
        return NULL;
    }

    std::string rgbName = rRgbLast[iFrame];
    //rgbName: ../data-fastfusion-tum/rgbd_dataset-d2/rgb/2.610428.png
    std::size_t pos = rgbName.find("/rgb/"); // at /
    pos += 5; // at 2
    std::size_t poss = rgbName.find(".png"); // at .
    poss--; // at 8
    std::string stamp = rgbName.substr(pos, poss-pos+1);
    //fprintf(stdout, "stamp: %s\n", stamp.c_str()); // 2.610428
    return stamp.c_str();
}

//deprecated:
extern "C" void lib_get_f_timestamp_current(double *out) {
    lib_data *pdata = g_pdata;
    const char *stamp = util_get_timestamp_string(pdata, pdata->_currentFrame);
    *out = (stamp == NULL) ? -1.0 : ::atof(stamp);
}
extern "C" void lib_get_str_timestamp_current(char *p) {
    lib_data *pdata = g_pdata;
    const char *stamp = util_get_timestamp_string(pdata, pdata->_currentFrame);
    sprintf(p, "%s", stamp ? stamp : "-1.0");
}

extern "C" int lib_count_frames() {
    lib_data *pdata = g_pdata;
    return pdata->pLast->size();
}
extern "C" void lib_get_rot_current(float *af) {
    lib_data *pdata = g_pdata;

    CameraInfo *caminfo = util_get_caminfo_current(pdata);
    //Parameter helpers in src/fusion/geometryfusion_mipmap_cpu.cpp
    //FusionMipMapCPU::addMap(...)
    cv::Mat rot = caminfo->getRotation();
//    printf("@@@ %s() rot:\n(%f %f %f)\n(%f %f %f)\n(%f %f %f)\n",
//           __FUNCTION__,
//           rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
//           rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
//           rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
    af[0] = rot.at<double>(0,0);
    af[1] = rot.at<double>(0,1);
    af[2] = rot.at<double>(0,2);
    af[3] = rot.at<double>(1,0);
    af[4] = rot.at<double>(1,1);
    af[5] = rot.at<double>(1,2);
    af[6] = rot.at<double>(2,0);
    af[7] = rot.at<double>(2,1);
    af[8] = rot.at<double>(2,2);
}
extern "C" void lib_get_trans_current(float *af) {
    lib_data *pdata = g_pdata;

    CameraInfo *caminfo = util_get_caminfo_current(pdata);
    cv::Mat trans = caminfo->getTranslation();
//    printf("@@@ %s() trans: (%f %f %f)\n", __FUNCTION__,
//       trans.at<double>(0,0),
//       trans.at<double>(1,0),
//       trans.at<double>(2,0));
    af[0] = trans.at<double>(0,0);
    af[1] = trans.at<double>(1,0);
    af[2] = trans.at<double>(2,0);
}
extern "C" void lib_get_intrinsic_current(float *af) {
    lib_data *pdata = g_pdata;

    CameraInfo *caminfo = util_get_caminfo_current(pdata);
    cv::Mat intr = caminfo->getIntrinsic();
//    printf("@@@ %s() intrinsic:\n(%f %f)\n(%f %f)\n", __FUNCTION__,
//       intr.at<double>(0,0), intr.at<double>(1,1),
//       intr.at<double>(0,2), intr.at<double>(1,2));
    // c.f. see camPamsFloat{} in src/fusion/definitions.h
    af[0] = intr.at<double>(0,0); //fx
    af[1] = intr.at<double>(1,1); //fy
    af[2] = intr.at<double>(0,2); //cx
    af[3] = intr.at<double>(1,2); //cy
}
extern "C" void lib_get_path_depth_current(char *p) {
    lib_data *pdata = g_pdata;

    long int currentFrame = pdata->_currentFrame;
    std::vector<std::string> *depthLast = pdata->depthLast;
    sprintf(p, "%s", (*depthLast)[currentFrame].c_str());
}
extern "C" void lib_get_path_rgb_current(char *p) {
    lib_data *pdata = g_pdata;

    long int currentFrame = pdata->_currentFrame;
    std::vector<std::string> *rgbLast = pdata->rgbLast;
    sprintf(p, "%s", (*rgbLast)[currentFrame].c_str());
}

extern "C" void lib_write_ply(int i_stamp) {
    lib_data *pdata = g_pdata;
    FusionMipMapCPU *fusion = pdata->pFusionWrapperIfData->fusion;
    char filename[64];
    sprintf(filename, "frame_%d", i_stamp);  // .ply auto appended
    return fusion->writePLY(filename);
}
extern "C" size_t lib_get_vertex_count() {
    lib_data *pdata = g_pdata;
    FusionMipMapCPU *fusion = pdata->pFusionWrapperIfData->fusion;
    return fusion->getVertexCount();
}
extern "C" size_t lib_get_face_vertices_count() {
    lib_data *pdata = g_pdata;
    FusionMipMapCPU *fusion = pdata->pFusionWrapperIfData->fusion;
    return fusion->getFaceVerticesCount();
}
extern "C" void lib_get_vbo_index(size_t c_elems, int *ai) {
    lib_data *pdata = g_pdata;
    FusionMipMapCPU *fusion = pdata->pFusionWrapperIfData->fusion;
    fusion->copyVboIndex(c_elems, ai);
}
extern "C" void lib_get_vbo_vertex(size_t c_elems,
                    float *af_scaled, float f_scale) {
    lib_data *pdata = g_pdata;
    FusionMipMapCPU *fusion = pdata->pFusionWrapperIfData->fusion;
    fusion->copyVboVertex(c_elems, af_scaled, f_scale);
}

void util_copy_sensor_data(SensorData *pSensorData, size_t c_elems,
                    float *af_scaled, float f_scale) {
    assert(pSensorData != NULL);
    size_t cPoints = pSensorData->cPoints;
    assert(c_elems == cPoints * 8);

    std::vector<int> &rSensorX = *pSensorData->pX;
    std::vector<int> &rSensorY = *pSensorData->pY;
    std::vector<Vertex3f> &rSensorVer = *pSensorData->pVertex;
    std::vector<Color3b> &rSensorRgb = *pSensorData->pRgb;

    for (unsigned int i = 0; i < cPoints; i++) {
        af_scaled[i*8] = rSensorVer[i].x * f_scale;
        af_scaled[i*8+1] = rSensorVer[i].y * f_scale;
        af_scaled[i*8+2] = rSensorVer[i].z * f_scale;
        af_scaled[i*8+3] = rSensorRgb[i].r/255.;
        af_scaled[i*8+4] = rSensorRgb[i].g/255.;
        af_scaled[i*8+5] = rSensorRgb[i].b/255.;
        af_scaled[i*8+6] = rSensorX[i];
        af_scaled[i*8+7] = rSensorY[i];
    }
}
extern "C" void lib_append_posinfo_disk() {
    lib_data *pdata = g_pdata;

    std::vector<std::vector<CameraInfo> > &rPoses = *pdata->_poses;
    std::vector<CameraInfo> &rPoseLast = rPoses.back();
    CameraInfo pose = rPoseLast[pdata->_currentFrame];
    cv::Mat rot = pose.getRotation();
    cv::Mat trans = pose.getTranslation();
    cv::Mat intr = pose.getIntrinsic();

    const char *stamp = util_get_timestamp_string(pdata, pdata->_currentFrame);
    if (stamp == NULL) {
        stamp = "unknown";
    }

    FILE *pFile = fopen("posinfo.csv", "a");
        fprintf(pFile, "%s;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n",
            stamp,
            rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
            rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
            rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2),
            trans.at<double>(0,0), trans.at<double>(1,0), trans.at<double>(2,0),
            intr.at<double>(0,0), intr.at<double>(1,1), //fx, fy
            intr.at<double>(0,2), intr.at<double>(1,2) //cx, cy
            );
    fclose(pFile);
}

size_t util_compute_sensor_data(SensorData *pSensorData,
    const cv::Mat &rot, const cv::Mat &trans, const cv::Mat &intr,
    const cv::Mat &depth, const cv::Mat &rgb,
    float imageDepthScale, float maxCamDistance,
    char *stamp,
    boost::optional<cv::Mat> hdataOpt,
    int dump_file)
{
    FusionMipMapCPU::getSensorData(pSensorData,
        rot, trans, intr, depth, rgb,
        1.0f/imageDepthScale, maxCamDistance,
        hdataOpt);

    if (dump_file > 0) {
        std::vector<int> &rSensorX = *pSensorData->pX;
        std::vector<int> &rSensorY = *pSensorData->pY;
        std::vector<Vertex3f> &rSensorVer = *pSensorData->pVertex;
        std::vector<Color3b> &rSensorRgb = *pSensorData->pRgb;
        size_t cPoints = pSensorData->cPoints;

        //fprintf(stderr, "_currentFrame: %d\n", pdata->_currentFrame);
        char filename[256];
        sprintf(filename, "%s_%s.csv", stamp,
//            hdataOpt == boost::none ? "raw" : "model");  //fixme: why error on linux?
            hdataOpt ? "model" : "raw");
        FILE *pFile = fopen(filename, "w");
        for (int i = 0; i < cPoints; i++) {
            fprintf(pFile, "%d;%d;%f;%f;%f;%d;%d;%d\n",
                rSensorX[i], rSensorY[i],
                rSensorVer[i].x, rSensorVer[i].y, rSensorVer[i].z,
                rSensorRgb[i].r, rSensorRgb[i].g, rSensorRgb[i].b
                );
        }
        fclose(pFile);
    }

    return pSensorData->cPoints;
}
extern "C" size_t lib_compute_sensor_data(
    float *af_rot, float *af_trans, float *af_intr,
    float *ai_depth, uchar *ai_b, uchar *ai_g, uchar *ai_r,
    float *af_hdata_opt,
    float f_imageDepthScale, float f_maxCamDistance,
    char *stamp,
    float *af_scaled, float f_scale,
    int dump_file)
{
//    lib_data *pdata = g_pdata;

    SensorData *pSensorData = NULL;
    FusionMipMapCPU::allocateSensorData(&pSensorData);

//    long int currentFrame = pdata->_currentFrame;
//    std::vector<CameraInfo> *pLast = pdata->pLast;
//    CameraInfo caminfo = (*pLast)[currentFrame];

//	cv::Mat rot = caminfo.getRotation();  //deprecated
//    printf("@@@ %f %f %f %f %f %f %f %f %f\n",
//           af_rot[0], af_rot[1], af_rot[2],
//           af_rot[3], af_rot[4], af_rot[5],
//           af_rot[6], af_rot[7], af_rot[8]);
    cv::Mat rot = cv::Mat(3, 3, CV_32F, af_rot);
    rot.convertTo(rot, CV_64F); // important!
//    printf("@@@ %s() rot:\n(%f %f %f)\n(%f %f %f)\n(%f %f %f)\n",
//           __FUNCTION__,
//           rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
//           rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
//           rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

//	cv::Mat trans = caminfo.getTranslation();
    printf("@@@ %f %f %f\n", af_trans[0], af_trans[1], af_trans[2]);
    cv::Mat trans = cv::Mat(3, 1, CV_32F, af_trans);
    trans.convertTo(trans, CV_64F); // important!
    printf("@@@ %f %f %f\n", trans.at<double>(0,0),
        trans.at<double>(1,0), trans.at<double>(2,0));

//	cv::Mat intr = caminfo.getIntrinsic();
    cv::Mat intr = cv::Mat(3, 3, CV_32F);
    intr.at<float>(0,0) = af_intr[0];
    intr.at<float>(1,1) = af_intr[1];
    intr.at<float>(0,2) = af_intr[2];
    intr.at<float>(1,2) = af_intr[3];
    intr.convertTo(intr, CV_64F); // important!


    double minVal; double maxVal;
//    std::vector<std::string> *depthLast = pdata->depthLast;
//    cv::Mat depth = cv::imread((*depthLast)[currentFrame],-1);
//    cv::minMaxLoc(depth, &minVal, &maxVal);
//    printf("@@@ min, max: %f, %f", minVal, maxVal);
    cv::Mat depth = cv::Mat(480, 640, CV_16S, ai_depth);
//    cv::minMaxLoc(depth, &minVal, &maxVal);
//    printf("@@@ min, max: %f, %f", minVal, maxVal);



//    std::vector<std::string> *rgbLast = pdata->rgbLast;
//    cv::Mat rgb = cv::imread((*rgbLast)[currentFrame]);
//    cv::imwrite("hoge.png", rgb);
    cv::Mat rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0,255));
    const uchar *rgbData = (const uchar*)rgb.data;
    const int step = rgb.step;
    const int channels = rgb.channels();
    int nx = rgb.cols;
    int ny = rgb.rows;
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            size_t index = y*nx+x;
            size_t indexColor = channels * index;
//            uchar b = rgbData[indexColor];
//            uchar g = rgbData[indexColor+1];
//            uchar r = rgbData[indexColor+2];
//            printf("db dg dr: %d %d %d ",
//                b-ai_b[index], g-ai_g[index], r-ai_r[index]);
            //http://stackoverflow.com/questions/9974946/how-to-edit-read-pixel-values-in-opencv-from-mat-variable
            rgb.data[step*y + channels*x + 0] = ai_b[index];
            rgb.data[step*y + channels*x + 1] = ai_g[index];
            rgb.data[step*y + channels*x + 2] = ai_r[index];
        }
    }
//    cv::imwrite("foo.png", rgb);

//    float f_imageDepthScale = pdata->pFusionWrapperIfData->imageDepthScale;
//    float f_maxCamDistance = pdata->pFusionWrapperIfData->maxCamDistance;
    printf("@@@ imageDepthScale: %f maxCamDistance: %f\n",
        f_imageDepthScale, f_maxCamDistance);

    printf("@@@ stamp: %s\n", stamp);

//    cv::Mat *pHdataOpt = NULL;
//    if (af_hdata != NULL) {
//        cv::Mat hdataOpt = cv::Mat(480, 640, CV_32F, af_hdata);
//        pHdataOpt = &hdataOpt;
//    }
// fixme: why this doesnt compile...
//    boost::optional<cv::Mat> hdataOpt = (af_hdata != NULL) ?
//        (cv::Mat(480, 640, CV_32F, af_hdata)) : (boost::none);
    boost::optional<cv::Mat> hdataOpt = boost::none;
    if (af_hdata_opt != NULL) {
        hdataOpt = cv::Mat(480, 640, CV_32F, af_hdata_opt);
    }

    size_t cPoints = util_compute_sensor_data(
        pSensorData,
        rot, trans, intr,
        depth, rgb,
        f_imageDepthScale, f_maxCamDistance,
        stamp,
//        pHdataOpt,
        hdataOpt,
        dump_file);
    size_t cElements = cPoints * 8;
    util_copy_sensor_data(pSensorData,
                          cElements, af_scaled, f_scale);
    FusionMipMapCPU::freeSensorData(&pSensorData);
    return cElements;
}


extern "C" size_t lib_compute_sensor_data_model(float *af_hdata, int dump_file) {
    lib_data *pdata = g_pdata;

    if (pdata->pSensorDataModel == NULL) {
        fprintf(stderr, "@@@ %s(): allocating pSensorDataModel\n", __FUNCTION__);
        FusionMipMapCPU::allocateSensorData(&pdata->pSensorDataModel);
    }

    //assume size of af_hdata = 640x480
    fprintf(stdout, "@@@ %s(): af_hdata[0]: %f\n", __FUNCTION__, af_hdata[0]);

//hdata plugin test
//    float hdata[480*640] = {0.2, 0.2};  // others become 0
//    float hdata[480*640] = {0, 0};  // others become 0
//    hdata[75] = 1.979800;

//    cv::Mat hdataOpt = cv::Mat(480, 640, CV_32F, hdata);  //hdata plugin test
    cv::Mat hdataOpt = cv::Mat(480, 640, CV_32F, af_hdata);
    cv::Mat *pHdataOpt = &hdataOpt;

//todo: !!!!!!!!!!!!!!!!!!!!
//    return util_compute_sensor_data(pdata->pSensorDataModel, pdata,
//                                    pHdataOpt, dump_file);
}

extern "C" void lib_get_vertex_list(size_t c_vertex,
    float *af_0, float *af_1, float *af_2,
    int *ai_0, int *ai_1, int *ai_2) {
    lib_data *pdata = g_pdata;
    FusionMipMapCPU *fusion = pdata->pFusionWrapperIfData->fusion;
    fusion->copyPlyVertex(c_vertex, af_0, af_1, af_2, ai_0, ai_1, ai_2);
}
extern "C" void lib_get_face_list(size_t c_faces,
    int *ai_0, int *ai_1, int *ai_2, int *ai_3) {
    lib_data *pdata = g_pdata;
    FusionMipMapCPU *fusion = pdata->pFusionWrapperIfData->fusion;
    fusion->copyPlyFace(c_faces, ai_0, ai_1, ai_2, ai_3);
}

int main(int argc, char *argv[]) {
#if 1
#warning will run in simulate-ctypes mode
    int dump_file = 0;
    lib_init((char *)"../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/associate.txt");
    //lib_init((char *)"../data-fastfusion-tum/rgbd_dataset-d2/associate.txt");
    //lib_main();

    lib_fusion_frame();
//todo: deprecated
//    lib_compute_sensor_data_raw(dump_file);
    lib_increment_frame();

    lib_fusion_frame();
//todo: deprecated
//    lib_compute_sensor_data_raw(dump_file);
    lib_increment_frame();
//    for (int i = 0; i < 400; i++) {
//        lib_fusion_frame();
//        lib_compute_sensor_data_raw(dump_file);
//        lib_increment_frame();
//    }


    lib_clean();
#else
    main_orig(argc, argv);
#endif
}
