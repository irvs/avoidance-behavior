// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <fstream>
#include <string>
#include <iostream>

#include <math.h>

#include <algorithm>
#include <time.h>
#ifndef WIN32
#include <sys/resource.h>
#endif

#include <boost/filesystem.hpp>

#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ExecutablePath>

//#define DEBUG_MODE
//#define SET_TOLERANCE_MODE

#include "GraspController.h"
#include "PlaceController.h"
#include "readtext.h"
#include "VectorMath.h"
#include "ForceClosureTest.h"

#include "GraspPluginManager.h"

//#define BOOST_PYTHON_STATIC_LIB
//#ifndef NDEBUG
//#define BOOST_DEBUG_PYTHON
//#endif
#include <boost/python.hpp>
#include <boost/make_shared.hpp>

#define deg2Rad(x)   ((x)*(3.141596)/(180.0))
#define m_pi    (3.141596)



namespace grasp{


void _initrand(){
#ifdef WIN32
	  srand((unsigned)time(NULL));
#else
	  srand48(time(0));
#endif
	}

#ifdef WIN32
double getrusage_sec() {
	return clock();
}
#else
double getrusage_sec() {
	struct rusage t;
	struct timeval tv;
	getrusage(RUSAGE_SELF, &t);
	tv = t.ru_utime;
	return tv.tv_sec + (double)tv.tv_usec*1e-6;
}
#endif

class PythonConverter{
	public:
		static PythonConverter* instance(){
			static PythonConverter* instance = new PythonConverter;
			return instance;
		}
		static cnoid::Vector3 setVector3(double a, double b, double c){
			return cnoid::Vector3(a,b,c);
		}
};

}

BOOST_PYTHON_MODULE( grasp )
{
using namespace boost::python;
 //   class_<grasp::GraspController, boost::noncopyable>("GraspController", no_init)
 //       .def("instance", &grasp::GraspController::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
 //   ;
    class_<grasp::PlanBase, boost::noncopyable>("PlanBase", no_init)
        .def("instance", &grasp::PlanBase::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("initial", &grasp::PlanBase::initial)
//        .def("doGraspPlanning", &grasp::PlanBase::doGraspPlanning)
 //       .def("doPlacePlanning", &grasp::PlanBase::doPlacePlanning)
  //      .def("doPickAndPlacePlanning", &grasp::PlanBase::doPickAndPlacePlanning)
		.add_property("stopFlag", &grasp::PlanBase::stopFlag) //test
    ;
	class_<cnoid::BodyItemPtr>("BodyItemPtr")
	;
	class_<cnoid::Vector3>("Vector3")
	;
    class_<grasp::PythonConverter>("PythonConverter")
		.def("Vector3", &grasp::PythonConverter::setVector3).staticmethod("Vector3")
	;
}

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace boost;

MotionState PlanBase::getMotionState(double time){
	MotionState ret;

	ret.jointSeq = VectorXd(body()->numJoints());
	for(int i=0;i<body()->numJoints();i++)
        ret.jointSeq[i] = body()->joint(i)->q();

    ret.pos = body()->link(0)->p();
    ret.rpy = rpyFromRot(body()->link(0)->R());

	ret.graspingState = getGraspingState();
	ret.graspingState2 = getGraspingState2();
	if( getGraspingState()==GRASPING ){
		ret.objectPalmPos = targetArmFinger->objectPalmPos;
		ret.objectPalmRot = targetArmFinger->objectPalmRot;
	}
	ret.objectContactState = getObjectContactState();
	ret.pathPlanDOF = pathPlanDOF;
	ret.tolerance = tolerance;
	ret.time = time;
	ret.id = motionId;
	return ret;
}

void PlanBase::setMotionState(MotionState gm){
	for(int i=0;i<body()->numJoints();i++){
        body()->joint(i)->q() = gm.jointSeq[i];
	}
    body()->link(0)->p() = gm.pos;
    body()->link(0)->R() = rotFromRpy(gm.rpy);

	setGraspingState(gm.graspingState);
	setGraspingState2(gm.graspingState2);
	if( getGraspingState()==GRASPING ){
		targetArmFinger->objectPalmPos = gm.objectPalmPos;
		targetArmFinger->objectPalmRot = gm.objectPalmRot;
	}
	setObjectContactState(gm.objectContactState);
	pathPlanDOF = gm.pathPlanDOF;
	setTolerance(gm.tolerance);
	calcForwardKinematics();
	motionId = gm.id;
}

PlanBase::PlanBase()  : 	os (MessageView::mainInstance()->cout() )
{
//	bodyItemRobot = NULL;
//	bodyItemGRC = NULL;
	targetObject = NULL;
	targetArmFinger=NULL;
	stopFlag = false;
//	refSize = refCsSize = 0;
//	arm = NULL;
//	fingers = NULL;
	graspingState = NOT_GRASPING;
	graspingState2 = NOT_GRASPING;
	tolerance = 0.0;
	ulimitMap=Vector3(1,1,1);
	llimitMap = Vector3(-1,-1,-1);
	doInitialCollision=true;
	useObjectSafeBoundingBox = false;
	useRobotSafeBoundingBox = false;
	boundingBoxSafetySize = Vector3(0.005,0.005,0.005);
	motionId = -1;

    if ( PyImport_AppendInittab( (char *)"grasp", initgrasp ) == -1 ) {
        MessageView::mainInstance()->put("faild init Grasp Module");
//        return;
    }
}

PlanBase::~PlanBase() {
	RemoveAllPointCloudEnvironment();
}

PlanBase* PlanBase::instance(PlanBase *gc) {
	static PlanBase* instance = (gc) ? gc : new PlanBase();
	if(gc) instance = gc;
	return instance;
}

TargetObject::TargetObject(cnoid::BodyItemPtr bodyItem){
	bodyItemObject = bodyItem;
	object = bodyItemObject->body()->link(0);
    objVisPos = object->p();
    objVisRot = object->R();
    objMass = object->m();
	offsetApplied = false;
}


void PlanBase::SetGraspedObject(cnoid::BodyItemPtr bodyItem){

	targetObject = new TargetObject(bodyItem); //shoud be chaged;

	Box& OCP = targetObject->OCP;
	calcBoundingBox(object()->coldetModel(), OCP.edge, OCP.p, targetObject->objCoM_, OCP.R);
	if(targetArmFinger){
		for(int i=0;i<nFing();i++) fingers(i)->coldetLinkPair(targetObject->bodyItemObject);
#ifdef  CNOID_10_11_12_13
		arm()->palmObjPair = new ColdetLinkPair(palm(),object() );
#else
		arm()->palmObjPair = make_shared<ColdetLinkPair>(body(), palm(), bodyItem->body(), object() );
#endif
	}
	string tagId = bodyItem->name();
	if(objTag2Item.find(tagId) == objTag2Item.end()){
		objTag2Item.insert( pair <string,BodyItemPtr>(tagId, bodyItem) );
	}
	setObjectContactState(ON_ENVIRONMENT) ;

	targetObject->safeBoundingBox = ColdetPairData::getSafeBoundingBox(object()->coldetModel(), boundingBoxSafetySize);

	initialCollision();
}

ArmFingers::ArmFingers(cnoid::BodyItemPtr bodyItem, const YamlMapping& gSettings) :  os (MessageView::mainInstance()->cout() )
{
	bodyItemRobot = bodyItem;

#ifdef CNOID_10_11
	boost::filesystem::path robotfullpath( bodyItemRobot->modelFilePath() );
#else
	boost::filesystem::path robotfullpath( bodyItemRobot->lastAccessedFilePath() );
#endif

	cout << robotfullpath.string() << endl;
	bodyItemRobotPath = boost::filesystem::path (robotfullpath.branch_path()).string();
	dataFilePath = bodyItemRobotPath + "/data/";

	//READ YAML setting

	palm = bodyItemRobot->body()->link(gSettings["palm"].toString());
	base = bodyItemRobot->body()->link(gSettings["base"].toString());

	const YamlSequence& tips = *gSettings["fingerEnds"].toSequence();

	nFing = tips.size();
	fingers = new FingerPtr[nFing];

	arm=NULL;
	for (int i = 0;i < tips.size();i++) {
		fingers[i]=NULL;
	}

	if( gSettings.find("GrasplotPluginDir")->type() == YAML_SCALAR ){
#ifdef WIN32
		string pluginPath =  cnoid::executableTopDirectory() + string("\\bin\\") + bodyItemRobot->name() + string("\\");
#else
		string pluginPath = bodyItemRobotPath + "/" + gSettings["GrasplotPluginDir"].toString();
#endif
		os << "Grasplot Plugin Path " << pluginPath << endl;
		gPluginManager.scanPluginFiles(pluginPath);

		arm = (Arm *)gPluginManager.loadGrasplotPlugin(bodyItemRobot->body(),base,palm, "Arm");

		for (int i = 0;i < tips.size();i++) {
			if(!fingers[i]) fingers[i] = (Finger *)gPluginManager.loadGrasplotPlugin
					(bodyItemRobot->body(), palm, bodyItemRobot->body()->link(tips[i].toString()), "Finger");
		}
	}
	if(!arm){
		arm = new Arm(bodyItemRobot->body(),base, palm);
	}
	for (int i = 0;i < tips.size();i++) {
		if(!fingers[i]) fingers[i] = new Finger(bodyItemRobot->body(), palm, bodyItemRobot->body()->link(tips[i].toString()) );
		fingers[i]->number = i;
	}

	if( gSettings.find("dataFilePath")->type() == YAML_SCALAR ){
		dataFilePath = bodyItemRobotPath + "/" + gSettings["dataFilePath"].toString() +"/";
	}

	if( gSettings.find("armStandardPose")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["armStandardPose"].toSequence();
		for(int i=0;i<list.size();i++){
			arm->armStandardPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("armFinalPose")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["armFinalPose"].toSequence();
		for(int i=0;i<list.size();i++){
			arm->armFinalPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOpenPose")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["fingerOpenPose"].toSequence();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOpenPoseOffset")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["fingerOpenPoseOffset"].toSequence();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPoseOffset.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerCloseOffset")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["fingerCloseOffset"].toSequence();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerCloseOffset.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOffset")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["fingerOffset"].toSequence();
		for(int j=0; j<nFing; j++)
			fingers[j]->offset = list[0].toDouble();
	}

	if( gSettings.find("pythonInterface")->type() == YAML_SCALAR ){
		pythonInterface = bodyItemRobotPath + "/" + gSettings["pythonInterface"].toString();
	}else{
		pythonInterface = "/NULL";
	}

	vector <InterLink> & interLinkList = PlanBase::instance()->interLinkList;
	if( gSettings.find("interlink")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["interlink"].toSequence();
		for(int i=0;i<list.size();i++){
			const YamlSequence& ilist = *list[i].toSequence();
			Link* master = bodyItemRobot->body()->link(ilist[0].toString());
			double baseratio = ilist[1].toDouble();
			for(int j=1;j<ilist.size()/2;j++){
				InterLink temp;
				temp.master = master;
				temp.slave = bodyItemRobot->body()->link(ilist[2*j].toString());
				temp.ratio = ilist[2*j+1].toDouble()/baseratio;
				interLinkList.push_back(temp);
			}

		}
	}
	if( gSettings.find("approachOffset")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["approachOffset"].toSequence();
		for(int i=0;i<list.size();i++){
			arm->approachOffset[i] = list[i].toDouble();
		}
	}

	if( gSettings.find("selfContactPair")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["selfContactPair"].toSequence();
		if(list[0].type() == YAML_SCALAR){
			for(int i=0;i<list.size()/2;i++){
				contactLinks.insert ( make_pair ( list[i*2].toString(), list[i*2+1].toString() ) );
			}
		}
		if(list[0].type() == YAML_SEQUENCE){
			for(int i=0;i<list.size();i++){
				const YamlSequence& plist = *list[i].toSequence();
				for(int j=0;j<plist.size();j++){
					for(int k=j+1;k<plist.size();k++){
						contactLinks.insert ( make_pair (plist[j].toString(), plist[k].toString() )  );
					}
				}
			}


		}
	}
	if( gSettings.find("movableArea")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["movableArea"].toSequence();
		const YamlSequence& ilist = *list[0].toSequence();
		PlanBase::instance()->llimitMap = Vector3 ( ilist[0].toDouble(),ilist[1].toDouble(),ilist[2].toDouble() );
		const YamlSequence& ilist1 = *list[1].toSequence();
		PlanBase::instance()->ulimitMap = Vector3 ( ilist1[0].toDouble(),ilist1[1].toDouble(),ilist1[2].toDouble() );
	}

	cout << "llimit" << PlanBase::instance()->llimitMap.transpose() << " ";
	cout << "ulimit" << PlanBase::instance()->ulimitMap.transpose() << endl;

	if( gSettings.find("InterObject")->type() == YAML_SEQUENCE ){
		const YamlSequence& list = *gSettings["InterObject"].toSequence();
		for(int i=0;i<list.size();i++){
			InterObject tempo;
			const YamlSequence& ilist = *list[i].toSequence();
			tempo.master  = bodyItemRobot->body()->link(ilist[0].toString());

			BodyItemPtr temp = new BodyItem();
			if( !temp->loadModelFile(dataFilePath +ilist[1].toString()) ){
				os << "modelLoadError: " << dataFilePath +ilist[1].toString() << endl;
				break;
			}
			temp->setName(ilist[1].toString());
			bodyItemRobot->addChildItem (temp);
			tempo.slaveItem = temp;
			const YamlSequence& vlist = *ilist[2].toSequence();
			tempo.relativePos = Vector3 ( vlist[0].toDouble(),vlist[1].toDouble(),vlist[2].toDouble() );
			const YamlSequence& vlist2 = *ilist[3].toSequence();
			tempo.relativeRot << vlist2[0].toDouble(), vlist2[1].toDouble(), vlist2[2].toDouble(), vlist2[3].toDouble(), vlist2[4].toDouble(), vlist2[5].toDouble(), vlist2[6].toDouble(), vlist2[7].toDouble(), vlist2[8].toDouble();
			tempo.setInterObject();
			tempo.type = InterObject::ROBOT;
			PlanBase::instance()->interObjectList.push_back(tempo);
		}
	}

	if( gSettings.find("mu")->type() == YAML_SCALAR){
		mu = gSettings["mu"].toDouble();
	}else{
		mu = 0.5;
	}

	if( gSettings.find("fmax")->type() == YAML_SCALAR){
		fmax = gSettings["fmax"].toDouble();
	}else{
		fmax = 10;
	}

	if( gSettings.find("hmax")->type() == YAML_SCALAR){
		hmax = gSettings["hmax"].toDouble();
	}else{
		hmax = 0.005;
	}

	handJoint = new LinkTraverse(palm);
	nHandLink = handJoint->numLinks();

	if (gSettings.find("name")->type() == YAML_SCALAR ){
		name = gSettings["name"].toString();
	}else{
		static int i=0;
		stringstream namenum;
		namenum << arm << i;
		name = namenum.str();
	}

}

bool PlanBase::SetGraspingRobot(cnoid::BodyItemPtr bodyItem_){
	//setting robot

	armsList.clear(); //Temoporary;  will delete menbers
	interLinkList.clear();
	targetArmFinger = NULL;

	//READ YAML setting
	if( bodyItem_->body()->info()->find("graspPluginSetting")->type() == YAML_SEQUENCE){ // multi arm
		const YamlSequence& glist = *(*bodyItem_->body()->info())["graspPluginSetting"].toSequence();
		for(int i=0;i<glist.size();i++){
			const YamlMapping& gSettings = *glist[i].toMapping();
			if ( gSettings.isValid() && !gSettings.empty()) {
				targetArmFinger = new ArmFingers(bodyItem_, gSettings);
				armsList.push_back(targetArmFinger);
			}
		}
	}
	else{ // single arm
		const YamlMapping& gSettings = *bodyItem_->body()->info()->findMapping("graspPluginSetting");
		if ( gSettings.isValid() && !gSettings.empty()) {
			targetArmFinger = new ArmFingers(bodyItem_, gSettings);
			armsList.push_back(targetArmFinger);
		}
	}

	targetArmFinger = armsList[0];

	if(targetArmFinger == NULL){
		os << "ERROR graspPluginSetting is not found in yaml" << endl;
		return false;
	}

	robTag2Arm.clear();
	for(int i=0;i<armsList.size();i++){
		armsList[i]->id = i;
		string tagId = armsList[i]->name;
		if(robTag2Arm.find(tagId) != robTag2Arm.end()){
			os << "Error: the tagId is already recorded " << tagId << endl;
			continue;
		}else{
			robTag2Arm.insert( pair <string,ArmFingers*>(tagId, armsList[i]));
		}
	}

	os  << bodyItem_->name() << " has " <<armsList.size() << " arm(s) "<< endl;

	if(targetObject){
		for(int i=0;i<nFing();i++) fingers(i)->coldetLinkPair(targetObject->bodyItemObject);
#ifdef  CNOID_10_11_12_13
		arm()->palmObjPair = new ColdetLinkPair(palm(),object() );
#else
		arm()->palmObjPair = make_shared<ColdetLinkPair>(body(), palm(), targetObject->bodyItemObject->body(), object() );
#endif
	}

	bodyItemRobot()->body()->calcForwardKinematics();

	graspMotionSeq.clear();
	setGraspingState(NOT_GRASPING);
	setGraspingState2(NOT_GRASPING);

	robotSelfPairs.clear();
	for(unsigned int i=0;i<bodyItemRobot()->body()->numLinks();i++){ // If initial position is not collided, it is stored as
		for(unsigned int j=i+1;j<bodyItemRobot()->body()->numLinks();j++){
			bool pass = false;
			pair<multimap<string, string>::iterator, multimap<string, string>::iterator> ppp;
			ppp = targetArmFinger->contactLinks.equal_range(bodyItemRobot()->body()->link(i)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == bodyItemRobot()->body()->link(j)->name()) pass = true;
			}
			ppp = targetArmFinger->contactLinks.equal_range(bodyItemRobot()->body()->link(j)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == bodyItemRobot()->body()->link(i)->name()) pass = true;
			}
			if(pass) continue;
#ifdef  CNOID_10_11_12_13
			ColdetLinkPairPtr temp= new ColdetLinkPair(bodyItemRobot()->body()->link(i),bodyItemRobot()->body()->link(j));
#else
			ColdetLinkPairPtr temp = make_shared<ColdetLinkPair>(bodyItemRobot()->body(), bodyItemRobot()->body()->link(i), bodyItemRobot()->body(), bodyItemRobot()->body()->link(j) );
#endif
			temp->updatePositions();
			int t1,t2;
			double p1[3],p2[3];
			double distance = temp->computeDistance(t1,p1,t2,p2);
			if(distance>1.0e-04)	robotSelfPairs.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"collide on initial condition at robotSelfPair"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl;
#endif
		}
	}
/*	for(unsigned int i=0; i<arm()->arm_path->numJoints(); i++){
		for(unsigned int j=i+2; j<arm()->arm_path->numJoints(); j++){
			robotSelfPairs.push_back(new ColdetLinkPair(arm()->arm_path->joint(i), arm()->arm_path->joint(j)) );
		}
	}

*/
	initialCollision();
	return true;
}


bool PlanBase::flush(){
	static int cnt=0;
	cnt++;

	if(stopFlag){
		stopFlag=false;
		throw(cnt);
	}
/* it will be GraspController
	if(bodyItemGRC){
		bodyItemGRC->body()->link(0)->R = palm()->R*(GRCmax.R);
		bodyItemGRC->body()->link(0)->p = palm()->p+palm()->R*GRCmax.p;
		bodyItemGRC->notifyKinematicStateChange();
	}
*/
//	bodyItemRobot()->body()->calcForwardKinematics();
	if(targetArmFinger) bodyItemRobot()->notifyKinematicStateChange();
	if(targetObject) targetObject->bodyItemObject->notifyKinematicStateChange();
	MessageView::mainInstance()->flush();

#ifdef  DEBUG_MODE
	usleep(100000);
#endif
	return true;

}

void PlanBase::calcForwardKinematics(){

	setInterLink();

	bodyItemRobot()->body()->calcForwardKinematics();

	if(graspingState==GRASPING) {
		if(nFing()>0){
            object()->R() = fingers(0)->tip->R()*(targetArmFinger->objectPalmRot);
            object()->p() = fingers(0)->tip->p()+fingers(0)->tip->R()*targetArmFinger->objectPalmPos;
		}else{
			object()->R() = palm()->R()*(targetArmFinger->objectPalmRot);
			object()->p() = palm()->p()+palm()->R()*targetArmFinger->objectPalmPos;
		}
	}
	else if(graspingState2==GRASPING) {
		if(nFing(1)>0) {
            object()->R() = fingers(1,0)->tip->R()*(armsList[1]->objectPalmRot);
			object()->p() = fingers(1,0)->tip->p()+fingers(1,0)->tip->R()*armsList[1]->objectPalmPos;
		} else {
			object()->R() = palm(1)->R()*(armsList[1]->objectPalmRot);
			object()->p() = palm(1)->p()+palm(1)->R()*armsList[1]->objectPalmPos;
		}
	}
	for(int i=0;i<interObjectList.size();i++){
//		if(interObjectList[i].type == InterObject::GRASPED_OBJECT){
			interObjectList[i].setInterObject();
//		}
	}
	//cout << "pos y "<< bodyItemRobot()->body()->link(0)->p.transpose() <<  rpyFromRot(bodyItemRobot()->body()->link(0)->R)[2] << endl;
}

bool PlanBase::isColliding(){
//	cnoid::ColdetLinkPairPtr* robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs;
	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return true;
		}
	}
	int sizeRobotEnv = robotEnvPairs.size();
	if(useRobotSafeBoundingBox) sizeRobotEnv = safeRobotEnvPairs.size();
	for(int i=0;i<sizeRobotEnv;i++){
		ColdetLinkPairPtr testPair;
		if(useRobotSafeBoundingBox) testPair= safeRobotEnvPairs[i];
		else testPair= robotEnvPairs[i];

		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"robot env collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return true;
		}
	}
	for(int i=0;i<robotPointCloudPairs.size();i++){
		bool coll = isCollidingPointCloud(robotPointCloudPairs[i].second,robotPointCloudPairs[i].first);
		if(coll){
#ifdef DEBUG_MODE
			cout << "robot pointcloud collide" << endl;
#endif
			return true;
		}
	}
	if(checkAllGraspingState()==NOT_GRASPING){
		for(int i=0;i<robotObjPairs.size();i++){
			ColdetLinkPairPtr testPair = robotObjPairs[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
				colPairName[0] = testPair->model(0)->name();
				colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
				cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}
	}
	if(getObjectContactState()==OFF_ENVIRONMENT){
		for(int i=0;i<objEnvPairs.size();i++){
			ColdetLinkPairPtr testPair = objEnvPairs[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
				colPairName[0] = testPair->model(0)->name();
				colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
				cout <<"obj env collide " << i  << " "<<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}
		for(int i=0;i<objPointCloudPairs.size();i++){
			bool coll = isCollidingPointCloud(objPointCloudPairs[i].second ,objPointCloudPairs[i].first);
			if(coll){
#ifdef DEBUG_MODE
				cout << "obj pointcloud collide" << endl;
#endif
				return true;
			}
		}
	}

	for(int i=0;i<interObjectList.size();i++){
		if( interObjectList[i].isColliding() ) return true;
	}

	return false;
}

bool PlanBase::isCollidingPointCloud(PointCloudEnv* pc_env, BodyItemPtr item, double tol){
	for(int i=0; i<item->body()->numLinks(); i++) {
		ColdetModelPtr c = item->body()->link(i)->coldetModel();
		vector<Vector3> bbdata;
		c->getBoundingBoxData(0, bbdata);
		vector<Vector3> v(8);
		v[0] = bbdata[0] + Vector3( bbdata[1](0),  bbdata[1](1),  bbdata[1](2));
		v[1] = bbdata[0] + Vector3( bbdata[1](0),  bbdata[1](1), -bbdata[1](2));
		v[2] = bbdata[0] + Vector3( bbdata[1](0), -bbdata[1](1),  bbdata[1](2));
		v[3] = bbdata[0] + Vector3( bbdata[1](0), -bbdata[1](1), -bbdata[1](2));
		v[4] = bbdata[0] + Vector3(-bbdata[1](0),  bbdata[1](1),  bbdata[1](2));
		v[5] = bbdata[0] + Vector3(-bbdata[1](0),  bbdata[1](1), -bbdata[1](2));
		v[6] = bbdata[0] + Vector3(-bbdata[1](0), -bbdata[1](1),  bbdata[1](2));
		v[7] = bbdata[0] + Vector3(-bbdata[1](0), -bbdata[1](1), -bbdata[1](2));

		double max_x, max_y, max_z, min_x, min_y, min_z;
		max_x = max_y = max_z = -DBL_MAX;
		min_x = min_y = min_z = DBL_MAX;
		for (int j = 0; j < 8; j++) {
			Vector3 rv;
            rv = item->body()->link(i)->R() * v[j] + item->body()->link(i)->p();
			max_x = std::max(max_x, rv(0));
			min_x = std::min(min_x, rv(0));
			max_y = std::max(max_y, rv(1));
			min_y = std::min(min_y, rv(1));
			max_z = std::max(max_z, rv(2));
			min_z = std::min(min_z, rv(2));
		}

		max_x += tol; max_y += tol; max_z += tol;
		min_x -= tol; min_y -= tol; min_z -= tol;

		if(max_x < pc_env->min_x || min_x > pc_env->max_x ||
			max_y < pc_env->min_y || min_y > pc_env->max_y ||
			max_z < pc_env->min_z || min_z > pc_env->max_z) {
				continue;
		}

		vector<Vector3> target_p;
		for(int j=0;j<pc_env->p().size();j++){
			if(pc_env->p()[j](0) < max_x && pc_env->p()[j](0) > min_x &&
				pc_env->p()[j](1) < max_y && pc_env->p()[j](1) > min_y &&
				pc_env->p()[j](2) < max_z && pc_env->p()[j](2) > min_z){
					target_p.push_back(pc_env->p()[j]);
			}
		}

		if(target_p.empty()) continue;

		if(item->body()->link(i)->coldetModel()->checkCollisionWithPointCloud(target_p, tol)){
			return true;
		}
	}
	return false;
}
/* //オリジナル
bool PlanBase::isCollidingPointCloud(const vector<Vector3>& p, BodyItemPtr item, double tol){
	for(int i=0; i<item->body()->numLinks(); i++){
			if(item->body()->link(i)->coldetModel()->checkCollisionWithPointCloud(p, tol)){
					return true;
			}
	}
	return false;
}
*/
//①一点Ver
int PlanBase::isCollidingPointCloud(const vector<Vector3>& p, BodyItemPtr item, double tol){
  //sleep(2); //test
  for(int i=0; i<item->body()->numLinks(); i++){
    int judge = item->body()->link(i)->coldetModel()->checkCollisionWithPointCloud(p, tol);
    if(judge) {
        return judge;
    }
  }
  return 0;
}
//入力ベクタpの干渉点をcol_resultに詰める
void PlanBase::isCollidingPointCloud2(const vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, vector<cnoid::Vector3>& col_result, double tol){
  for(int i=0; i<item->body()->numLinks(); i++){
    item->body()->link(i)->coldetModel()->checkCollisionWithPointCloud2(p, tol, col_result);
  }
}
//2の重複を消したもの
void PlanBase::isCollidingPointCloud3(const vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, vector<cnoid::Vector3>& col_result, double tol){
  vector<int> judge;
  for(int i=0; i<p.size(); i++){
    judge.push_back(0);
  }
  //for(int i=0; i<item->body()->numLinks(); i++){
  for(int i=3; i<11; i++){ //for pal Right Arm
    item->body()->link(i)->coldetModel()->checkCollisionWithPointCloud3(p, tol, judge);
  }
  for(int i=0; i<p.size(); i++){
    if(judge[i]){
      col_result.push_back(p[i]);
    }
  }
}

//入力ベクタpから干渉点を削除する
void PlanBase::removeCollidingPointCloud(vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, double tol){
  for(int i=0; i<item->body()->numLinks(); i++){
    item->body()->link(i)->coldetModel()->removeCollisionWithPointCloud(p, tol);
  }
}
void PlanBase::removeCollidingPointCloud2(const vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, vector<cnoid::Vector3>& col_result, double tol){
  vector<int> judge;
  for(int i=0; i<p.size(); i++){
    judge.push_back(0);
  }
  for(int i=0; i<item->body()->numLinks(); i++){
    item->body()->link(i)->coldetModel()->removeCollisionWithPointCloud2(p, tol, judge);
  }
  for(int i=0; i<p.size(); i++){
    if(!judge[i]){
      col_result.push_back(p[i]);
    }
  }
}

bool PlanBase::isCollidingPointCloudFinger(const vector<Vector3>& p, double tol){

	for(int i=0; i<nFing(); i++)
		for(int j=0; j<fingers(i)->fing_path->numJoints(); j++){
			if(fingers(i)->fing_path->joint(j)->coldetModel()->checkCollisionWithPointCloud(p, tol))
					return true;
		}
	return false;
}

double PlanBase::clearance(){

//	double start = getrusage_sec();

	double min_sep=1.e10;

	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
	}
	if(checkAllGraspingState()==NOT_GRASPING){
		for(int i=0;i<robotObjPairs.size();i++){
			ColdetLinkPairPtr testPair = robotObjPairs[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
				colPairName[0] = testPair->model(0)->name();
				colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
				cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return 0;
			}
		}
	}

	for(int i=0;i<robotPointCloudPairs.size();i++){
		bool coll = isCollidingPointCloud(robotPointCloudPairs[i].second,robotPointCloudPairs[i].first);
		if(coll){
#ifdef DEBUG_MODE
			cout << "robot pointcloud collide" << endl;
#endif
			return 0;
		}
	}

	for(int i=0;i<robotEnvPairs.size();i++){
		ColdetLinkPairPtr testPair = robotEnvPairs[i];
		testPair->updatePositions();

		testPair->setTolerance(tolerance);
		if( testPair->detectIntersection() ){
#ifdef DEBUG_MODE
			os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
		continue;

		int t1,t2;
		double p1[3],p2[3];
		double distance = testPair->computeDistance(t1,p1,t2,p2);
		if(distance <=tolerance){
#ifdef DEBUG_MODE
			os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return distance;
		}
		else {
			os << distance << endl;
		}
		if(distance < min_sep){
			min_sep = distance;
		}
	}


	if(getObjectContactState()==OFF_ENVIRONMENT){
		for(int i=0;i<objEnvPairs.size();i++){
			ColdetLinkPairPtr testPair = objEnvPairs[i];
			testPair->updatePositions();

			testPair->setTolerance(tolerance);
			if( testPair->detectIntersection() ){
#ifdef DEBUG_MODE
				os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return 0;
			}
			continue;

			int t1,t2;
			double p1[3],p2[3];
			double distance = testPair->computeDistance(t1,p1,t2,p2);
			if(distance <=tolerance){
#ifdef DEBUG_MODE
				os <<"obj-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return distance;
			}
			else {
				os << distance << endl;
			}
			if(distance < min_sep){
				min_sep = distance;
			}
		}

		for(int i=0;i<objPointCloudPairs.size();i++){
			bool coll = isCollidingPointCloud(objPointCloudPairs[i].second, objPointCloudPairs[i].first);
			if(coll){
	#ifdef DEBUG_MODE
				cout << "obj pointcloud collide" << endl;
	#endif
				return 0;
			}
		}
	}
//	double end = getrusage_sec();
//	cout << "time clearance" << objEnvPairs.size() << " "<< end - start << endl;


	return min_sep;
}


void PlanBase::setGraspingState(int state){
	if(state==GRASPING){
		if(nFing() > 0){
			targetArmFinger->objectPalmPos = trans(Matrix3(fingers(0)->tip->R()))*(object()->p() - fingers(0)->tip->p());
			targetArmFinger->objectPalmRot = trans(Matrix3(fingers(0)->tip->R()))*object()->R();
		}else{
			targetArmFinger->objectPalmPos = trans(Matrix3(palm()->R()))*(object()->p() - palm()->p());
			targetArmFinger->objectPalmRot = trans(Matrix3(palm()->R()))*object()->R() ;
		}
	}
	graspingState = state;
}

void PlanBase::setGraspingState2(int state){
	if(armsList.size() >1 && state==GRASPING){
		if(nFing(1)>0) {
			armsList[1]->objectPalmPos = trans(Matrix3(fingers(1,0)->tip->R()))*(object()->p() - fingers(1,0)->tip->p());
			armsList[1]->objectPalmRot = trans(Matrix3(fingers(1,0)->tip->R()))*object()->R();
		}else {
			armsList[1]->objectPalmPos = trans(Matrix3(palm(1)->R()))*(object()->p() - palm(1)->p());
			armsList[1]->objectPalmRot = trans(Matrix3(palm(1)->R()))*object()->R() ;
		}
	}
	graspingState2 = state;
}

void PlanBase::setTrajectoryPlanDOF(){

	pathPlanDOF.clear();

	for(int i=0; i<arm()->nJoints; i++)
        pathPlanDOF.push_back(arm()->arm_path->joint(i)->jointId());
	for(int i=0; i<nFing(); i++)
		for(int j=0; j<fingers(i)->nJoints; j++)
            pathPlanDOF.push_back(fingers(i)->fing_path->joint(j)->jointId());

#ifdef DEBUG_MODE
	cout << "Plan DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}

void PlanBase::setTrajectoryPlanDOF(int k){

	pathPlanDOF.clear();

	for(int i=0; i<arm(k)->nJoints; i++)
        pathPlanDOF.push_back(arm(k)->arm_path->joint(i)->jointId());
	for(int i=0; i<nFing(k); i++)
		for(int j=0; j<fingers(k,i)->nJoints; j++)
            pathPlanDOF.push_back(fingers(k,i)->fing_path->joint(j)->jointId());

#ifdef DEBUG_MODE
	cout << "Plan DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}

void PlanBase::setTrajectoryPlanMapDOF(){

	pathPlanDOF.clear();

	int top = body()->numJoints();
	pathPlanDOF.push_back(top); //position x
	pathPlanDOF.push_back(top+1); //position y;
	pathPlanDOF.push_back(top+5); //yaw;

	//ulimitMap = Vector3(3.0,3.0,3.0);
	//llimitMap = Vector3(-3.0,-3.0,-3.0);

#ifdef DEBUG_MODE
	cout << "Plan Map DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}


double PlanBase::calcContactPoint(ColdetLinkPairPtr cPair, Vector3 &Po, Vector3 &Pf, Vector3 &objN, Vector3 &fingerN) {

//	int Ik = t;

	double p1[3] = {0}, p2[3] = {0};
	int tid1, tid2;

//	ColdetLinkPairPtr cPair = linkObjPair[Ik];
//	ColdetModelPtr model = cPair->model(0);

    cPair->model(0)->setPosition(cPair->link(0)->R(), cPair->link(0)->p());
    cPair->model(1)->setPosition(cPair->link(1)->R(), cPair->link(1)->p());
//	cout << cPair->link(0)->p << cPair->link(1)->p << endl;

	double dsn = cPair->computeDistance(tid1, &p1[0], tid2, &p2[0]);


	Link* links[2];
	links[0] = cPair->link(0);
	links[1] = cPair->link(1);

	int v[2][3];
	links[0]->coldetModel()->getTriangle(tid1, v[0][0], v[0][1], v[0][2]);
	links[1]->coldetModel()->getTriangle(tid2, v[1][0], v[1][1], v[1][2]);

	float p[3];
	Vector3 n[3];

	for (int i = 1; i < 2;i++) {
		for (int j = 0; j < 3;j++) {
			links[i]->coldetModel()->getVertex(v[i][j], p[0], p[1], p[2]);
			n[j] = Vector3 (p[0], p[1], p[2]);
		}
	}

	Pf = Vector3(p1[0], p1[1], p1[2]);
	Po = Vector3(p2[0], p2[1], p2[2]);
//	cout << Po << Pf << endl;

	//Po = trans(cPair->link(1)->R) * Po - cPair->link(1)->p; //bug? ochi
    Po = trans(Matrix3(cPair->link(1)->R())) * (Po - cPair->link(1)->p());
//	alias(Pf) = trans(cPair->link(0)->R) * Pf - cPair->link(0)->p;


	Vector3 objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	objN = objN2 / norm2(objN2);

	for (int j = 0; j < 3;j++) {
		links[0]->coldetModel()->getVertex(v[0][j], p[0], p[1], p[2]);
		n[j] = Vector3 (p[0], p[1], p[2]);
	}

	objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	//normal vector of finger in the object local corrdinate
    fingerN = trans(Matrix3(cPair->link(1)->R())) * (cPair->link(0)->R()) * (objN2 / norm2(objN2));

	return dsn;

}

Matrix3 makeOrthogonal(const MatrixXd A, const VectorXd b)
{
	vector<double> c;
	for(size_t i=0; i<b.size(); i++)
			c.push_back(fabs(b(i)));

	Matrix3 A_ = d2v(A);
	int j=argmax(c);
	Vector3 v0 = A_.col(j);
	Vector3 v1 = A_.col((j+1)%3);

	double r=dot(v0,v1);
	v1 = (-r*v0 + v1)/sqrt(1-r*r);
	Vector3 v2 = cross(v0,v1);

	for(int i=0; i<3; i++){
		A_(i, (j+1)%3) = v1(i);
		A_(i, (j+2)%3) = v2(i);
	}

	return A_;
}

void PlanBase::calcBoundingBox(ColdetModelPtr model, Vector3 &edge, Vector3& center, Vector3& com, Matrix3& Rot) {
	//objVis and objPos shoulde be defined in advance.

	class Triangle{
	public:
		cnoid::Vector3 ver[3];
		float area;
	};

	// convert coldetmodel to objectshape
	float out_x, out_y, out_z;
	int v0,v1,v2;

	int nVerticies = model->getNumVertices();
	int nTriangles = model->getNumTriangles();

	Vector3* verticies = new Vector3[nVerticies];
	Triangle* triangles = new Triangle[nTriangles];

	for(int i=0;i<nVerticies;i++){
        model->getVertex(i, out_x, out_y, out_z);
		verticies[i][0] = out_x;
		verticies[i][1] = out_y;
		verticies[i][2] = out_z;
	}

	for(int i=0;i<nTriangles;i++){
        model->getTriangle(i, v0,v1,v2);
		triangles[i].ver[0] = verticies[v0];
		triangles[i].ver[1] = verticies[v1];
		triangles[i].ver[2] = verticies[v2];
	}

	// calc distribution
	Vector3 pt;
	MatrixXd distribute = MatrixXd::Zero(3, 3);
	Vector3 average(0, 0, 0);

	for(int i=0;i<nTriangles;i++){
		Vector3 e1 (triangles[i].ver[1] - triangles[i].ver[0]);
		Vector3 e2 (triangles[i].ver[2] - triangles[i].ver[0]);
		triangles[i].area = norm2 ( cross(e1,e2) ) /2.0;
	}

	Matrix3 aq,aq_n_sum,aq_p_sum;
	std::vector<double>* aq_p[3][3];
	std::vector<double>* aq_n[3][3];
	Vector3 sumCenter(0,0,0);
	double sumArea=0;
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		aq(i,j)=0;
		aq_p_sum(i,j)=0;
		aq_n_sum(i,j)=0;
		aq_p[i][j] = new std::vector<double>();
		aq_n[i][j] = new std::vector<double>();
	}

	for(int l=0;l<nTriangles;l++){
		Triangle& t = triangles[l];
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					double tmp_aq;
					tmp_aq = t.area*(t.ver[i][j] * t.ver[i][k])/6.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t.area*(t.ver[i][j] * t.ver[(i+1)%3][k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t.area*(t.ver[(i+1)%3][j] * t.ver[i][k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
				}
			}
		}
		sumArea +=t.area;
		sumCenter  =  sumCenter + t.area/3.0* Vector3 ( t.ver[0] + t.ver[1] + t.ver[2]);
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		std::sort(aq_p[i][j]->begin(),aq_p[i][j]->end(),LessAbs());
		for(int n=0;n<aq_p[i][j]->size();n++){
			aq_p_sum(i,j) += aq_p[i][j]->at(n);
		}
		std::sort(aq_n[i][j]->begin(),aq_n[i][j]->end(),LessAbs());
		for(int n=0;n<aq_n[i][j]->size();n++){
			aq_n_sum(i,j) += aq_n[i][j]->at(n);
		}
		delete aq_p[i][j];
		delete aq_n[i][j];
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {aq(i,j) = aq_p_sum(i,j) + aq_n_sum(i,j);}
	average = com = sumCenter/sumArea;
	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			distribute(j,k) = aq(j,k) - com[j] * com[k] * sumArea;
		}
	}
	MatrixXd evec(3, 3);
	VectorXd eval(3);
	int info;
	Eigen::EigenSolver<MatrixXd> es(distribute);
	eval = es.eigenvalues().real();
	evec = es.eigenvectors().real();

	Rot = makeOrthogonal(evec, eval);
	//Rot =  (d2v(evec));

	Vector3 e[3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			e[j][i] = Rot(i, j);

	Vector3 pt_max(0, 0, 0), pt_min(0, 0, 0);

	for(int l=0;l<nVerticies;l++){
		Vector3 pt = verticies[l];
		for (int j = 0; j < 3; j++) {
			double tmp = dot(e[j], Vector3(pt - average));
			if (tmp > pt_max[j]) pt_max[j] = tmp;
			if (tmp < pt_min[j]) pt_min[j] = tmp;
		}
	}

	//Rot =  (d2v(evec));

	edge =  (pt_max - pt_min);
	center =  average + 0.5 * Rot * (pt_max + pt_min);
//	com = average;

//	alias(Rot)  = objVisRot() * Rot;
//	alias(center) = objVisRot() * center + objVisPos();
//	alias(com)    = objVisRot() * com   + objVisPos();
#ifdef DEBUG_MODE
	cout << "bouding box size"<< edge.transpose() << endl<< Rot  <<endl;
#endif

	delete []	verticies;
	delete []  triangles;


	return;
}




//Including Enveloping grasp
//bool PlanBase::sampleFinalPos2(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs, int iterate)
bool PlanBase::initial() {
	if ( !targetObject || !targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}

	RemoveEnvironment(targetObject->bodyItemObject);

    targetObject->objVisPos = object()->p();
    targetObject->objVisRot = object()->R();

	setGraspingState(NOT_GRASPING);
	setGraspingState2(NOT_GRASPING);
	graspMotionSeq.push_back ( getMotionState() );

	initialCollision();

	_initrand();
	return true;
}

void PlanBase::initialCollision(){

	/////////////////////////////////////////////////////////// temporal change
	initialCollisionWithMemory();
	return;
	///////////////////////////////////////////////////////////

	if(!doInitialCollision) return;

	robotEnvPairs.clear();
	robotObjPairs.clear();
	objEnvPairs.clear();
	robotPointCloudPairs.clear();
	objPointCloudPairs.clear();

	if(targetArmFinger==NULL) {
		return;
	}
	for(unsigned int j=0;j<bodyItemRobot()->body()->numLinks();j++){
		for( list<BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end(); it++){
			for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
				ColdetLinkPairPtr temp= new ColdetLinkPair(bodyItemRobot()->body()->link(j), (*it)->body()->link(i));
#else
				ColdetLinkPairPtr temp = make_shared<ColdetLinkPair>(bodyItemRobot()->body(),bodyItemRobot()->body()->link(j), (*it)->body(), (*it)->body()->link(i) );
#endif
				temp->updatePositions();
				int t1,t2;
				double p1[3],p2[3];
				double distance = temp->computeDistance(t1,p1,t2,p2);
				if(distance>1.0e-04)	robotEnvPairs.push_back(temp);
#ifdef DEBUG_MODE
				else os <<"collide on initial condition robot and env"  <<distance <<" "<< temp->model(0)->name() <<" " << (*it)->body()->name() << endl;
#endif
			}
		}
	}
/*	for(unsigned int j=0;j<arm()->arm_path->numJoints();j++){
		for( list<BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end(); it++){
			ColdetLinkPairPtr temp= new ColdetLinkPair(arm()->arm_path->joint(j), (*it)->body()->link(0));
			temp->model(0)->setPosition(temp->link(0)->R, temp->link(0)->p);
			temp->model(1)->setPosition(temp->link(1)->R, temp->link(1)->p);
			int t1,t2;
			double p1[3],p2[3];
			double distance = temp->computeDistance(t1,p1,t2,p2);
			if(distance>1.0e-03)	armEnvPairs.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"tollerance collide on initial condition"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl;
#endif
			os <<"tollerance collide on initial condition"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl;
		}
	}
*/	if(targetObject){
		for(unsigned int j=0;j<bodyItemRobot()->body()->numLinks();j++){
#ifdef  CNOID_10_11_12_13
				robotObjPairs.push_back(new ColdetLinkPair(bodyItemRobot()->body()->link(j), object() ));
#else
				robotObjPairs.push_back(make_shared<ColdetLinkPair>(bodyItemRobot()->body(),bodyItemRobot()->body()->link(j),targetObject->bodyItemObject->body(),object()));
#endif


			}
		ColdetModelPtr backup = object()->coldetModel();
		if(useObjectSafeBoundingBox){
			object()->setColdetModel(targetObject->safeBoundingBox);
		}
		for(list<cnoid::BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end();it++){
			for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
				objEnvPairs.push_back(new ColdetLinkPair(object(), (*it)->body()->link(i)));
#else
				objEnvPairs.push_back( make_shared<ColdetLinkPair>(targetObject->bodyItemObject->body(), object(), (*it)->body(), (*it)->body()->link(i)));
#endif
			}
        }
        object()->setColdetModel(backup);
	}


	for(list<PointCloudEnv*>::iterator it = pointCloudEnv.begin(); it != pointCloudEnv.end(); ++it){
		pair<BodyItemPtr, PointCloudEnv*> robo_pointcloud(bodyItemRobot(), *it);
		robotPointCloudPairs.push_back(robo_pointcloud);
		if(targetObject){
			pair<BodyItemPtr, PointCloudEnv*> obj_pointcloud(targetObject->bodyItemObject, *it);
			objPointCloudPairs.push_back(obj_pointcloud);
		}
	}

	for(int i=0;i<interObjectList.size();i++) interObjectList[i].initialCollision();
}

void PlanBase::initialCollisionWithMemory(){

//	static vector<ColdetPairData*> coldetPairData;

	robotEnvPairs.clear();
	safeRobotEnvPairs.clear();
	robotObjPairs.clear();
	objEnvPairs.clear();
	robotPointCloudPairs.clear();
	objPointCloudPairs.clear();

	if(targetArmFinger) {
		for( list<BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end(); it++){
			BodyItemPtr bodyItem1= bodyItemRobot();
			BodyItemPtr bodyItem2= *it;
			bool find=false;
			for(int k=0;k<coldetPairData.size();k++){
				if( ( (coldetPairData[k]->bodyItem1== bodyItem1) && (coldetPairData[k]->bodyItem2== bodyItem2) ) ||
				       ((coldetPairData[k]->bodyItem2== bodyItem2) && (coldetPairData[k]->bodyItem1== bodyItem1) ) ){
					find=true;
					robotEnvPairs.insert( robotEnvPairs.end(),  coldetPairData[k]->coldetLinkPairs.begin(), coldetPairData[k]->coldetLinkPairs.end() );
					break;
				}

			}
			if(find) continue;
			ColdetPairData * temp = new ColdetPairData(bodyItem1, bodyItem2);
			coldetPairData.push_back(temp);
			robotEnvPairs.insert( robotEnvPairs.end(),  temp->coldetLinkPairs.begin(), temp->coldetLinkPairs.end() );
		}

		for( list<BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end(); it++){
			BodyItemPtr bodyItem1= bodyItemRobot();
			BodyItemPtr bodyItem2= *it;
			bool find=false;
			for(int k=0;k<safeColdetPairData.size();k++){
				if( ( (safeColdetPairData[k]->bodyItem1== bodyItem1) && (safeColdetPairData[k]->bodyItem2== bodyItem2) ) ||
				       ((safeColdetPairData[k]->bodyItem2== bodyItem2) && (safeColdetPairData[k]->bodyItem1== bodyItem1) ) ){
					find=true;
					safeRobotEnvPairs.insert( safeRobotEnvPairs.end(),  safeColdetPairData[k]->coldetLinkPairs.begin(), safeColdetPairData[k]->coldetLinkPairs.end() );
					break;
				}

			}
			if(find) continue;
			ColdetPairData * temp = new ColdetPairData(bodyItem1, bodyItem2, false, true);
			safeColdetPairData.push_back(temp);
			safeRobotEnvPairs.insert( safeRobotEnvPairs.end(),  temp->coldetLinkPairs.begin(), temp->coldetLinkPairs.end() );
		}

	}

	if(targetObject){
		if(targetArmFinger) {
			BodyItemPtr bodyItem1= bodyItemRobot();
			BodyItemPtr bodyItem2= targetObject->bodyItemObject;
			bool find=false;
			for(int k=0;k<coldetPairData.size();k++){
				if( ( (coldetPairData[k]->bodyItem1== bodyItem1) && (coldetPairData[k]->bodyItem2== bodyItem2) ) ||
				       ((coldetPairData[k]->bodyItem2== bodyItem2) && (coldetPairData[k]->bodyItem1== bodyItem1) ) ){
					find=true;
					robotObjPairs.insert( robotObjPairs.end(),  coldetPairData[k]->coldetLinkPairs.begin(), coldetPairData[k]->coldetLinkPairs.end() );
					break;
				}

			}
			if(!find){
				ColdetPairData * temp = new ColdetPairData(bodyItem1, bodyItem2);
				coldetPairData.push_back(temp);
				robotObjPairs.insert( robotObjPairs.end(),  temp->coldetLinkPairs.begin(), temp->coldetLinkPairs.end() );
			}
		}

		for(list<cnoid::BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end();it++){
			BodyItemPtr bodyItem1= targetObject->bodyItemObject;
			BodyItemPtr bodyItem2= *it;
			bool find=false;
			for(int k=0;k<coldetPairData.size();k++){
				if( ( (coldetPairData[k]->bodyItem1== bodyItem1) && (coldetPairData[k]->bodyItem2== bodyItem2) ) ||
				       ((coldetPairData[k]->bodyItem2== bodyItem2) && (coldetPairData[k]->bodyItem1== bodyItem1) ) ){
					find=true;
					objEnvPairs.insert( objEnvPairs.end(),  coldetPairData[k]->coldetLinkPairs.begin(), coldetPairData[k]->coldetLinkPairs.end() );
					break;
				}
			}
			if(find) continue;
			ColdetPairData * temp = new ColdetPairData(bodyItem1, bodyItem2);
			coldetPairData.push_back(temp);
			objEnvPairs.insert( objEnvPairs.end(),  temp->coldetLinkPairs.begin(), temp->coldetLinkPairs.end() );
		}
		if(useObjectSafeBoundingBox){
			objEnvPairs.clear();
            ColdetModelPtr backup = object()->coldetModel();
            object()->setColdetModel(targetObject->safeBoundingBox);
			for(list<cnoid::BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end();it++){
				for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
					objEnvPairs.push_back(new ColdetLinkPair(object(), (*it)->body()->link(i)));
#else
					objEnvPairs.push_back( make_shared<ColdetLinkPair>(targetObject->bodyItemObject->body(), object(), (*it)->body(), (*it)->body()->link(i)));
#endif
				}
			}
//			object()->coldetModel() = backup;
			object()->setColdetModel(backup);
		}
	}
	for(list<PointCloudEnv*>::iterator it = pointCloudEnv.begin(); it != pointCloudEnv.end(); ++it){
		if (targetArmFinger) {
			pair<BodyItemPtr, PointCloudEnv*> robo_pointcloud(bodyItemRobot(), *it);
			robotPointCloudPairs.push_back(robo_pointcloud);
		}
		if (targetObject) {
			pair<BodyItemPtr, PointCloudEnv*> obj_pointcloud(targetObject->bodyItemObject, *it);
			objPointCloudPairs.push_back(obj_pointcloud);
		}
	}
	for(int i=0;i<interObjectList.size();i++) interObjectList[i].initialCollision();

	os << "coldetPairData size" << coldetPairData.size() << endl;
}



void PlanBase::setObjPos(const cnoid::Vector3& P, const cnoid::Matrix3 R){

	targetObject->objVisPos = P;
	targetObject->objVisRot = R;
    object()->p()=P;
	object()->R()=R;

	targetObject->offsetApplied = false;

	return;

}

void PlanBase::setVisOffset(const cnoid::Vector3& P)
{
	if(!targetObject->offsetApplied){
		targetObject->objVisPos += P;
        object()->p() += P;
		targetObject->offsetApplied = true;
	}
}

void PlanBase::setVisOffsetR()
{
	Vector3 y = rpyFromRot(targetObject->objVisRot);
	if( fabs(y(0))<0.1 ) y(0)=0;
	else if(fabs(y(0)-1.5708)<0.1 ) y(0) =  1.5708;
	else if(fabs(y(0)+1.5708)<0.1 ) y(0) = -1.5708;
	else if(fabs(y(0)-3.1415)<0.1 ) y(0) =  3.1415;
	else if(fabs(y(0)+3.1415)<0.1 ) y(0) = -3.1415;

	if( fabs(y(1))<0.1 ) y(1)=0;
	else if(fabs(y(1)-1.5708)<0.1 ) y(1) =  1.5708;
	else if(fabs(y(1)+1.5708)<0.1 ) y(1) = -1.5708;
	else if(fabs(y(1)-3.1415)<0.1 ) y(1) =  3.1415;
	else if(fabs(y(1)+3.1415)<0.1 ) y(1) = -3.1415;

	targetObject->objVisRot = rotFromRpy(y);
    object()->R() = rotFromRpy(y);
}

void PlanBase::removeVisOffset(const cnoid::Vector3& P)
{
	if(targetObject->offsetApplied){
		targetObject->objVisPos -= P;
        object()->p() -= P;
		targetObject->offsetApplied = false;
	}
}

void PlanBase::setInterLink(){
	if(interLinkList.empty()) return;
	for(int i=0; i<interLinkList.size();i++){
        interLinkList[i].slave->q() = interLinkList[i].master->q() *interLinkList[i].ratio;
        if( interLinkList[i].slave->q() < interLinkList[i].slave->q_lower()) interLinkList[i].slave->q() = interLinkList[i].slave->q_lower();
        if( interLinkList[i].slave->q() > interLinkList[i].slave->q_upper()) interLinkList[i].slave->q() = interLinkList[i].slave->q_upper();
	}
}
