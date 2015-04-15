#include <velodyne_bar.h>
#include <SgPointsGet.h>

//------------------------------------------------------------------------------------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

bool VelodyneBar::isRosInit = false;

//------------------------------------------------------------------------------------------------------------------------------------------------------------
VelodyneBar* VelodyneBar::instance() {
  static VelodyneBar* instance = new VelodyneBar();
  return instance;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------
VelodyneBar::VelodyneBar() :
    ToolBar("VelodyneBar"), mes( *MessageView::mainInstance()), os(MessageView::mainInstance()->cout()), argc(), argv() {
  // ros init
  try {
    if( !isRosInit) {
      ros::init(argc, argv, "velodyne_receiver");
      isRosInit = true;
      cout << "Success: connecting roscore.." << endl;
    }
  } catch (...) {
    cout << "Error: ros init" << endl;
  }

  addSeparator();
  addLabel(("[Velodyne]"));

  //addButton(("Test"), ("Test Button"))->sigClicked().connect(bind( &VelodyneBar::onTestButtonClicked, this));
  //addButton(("PCD"), ("Receiver the point cloud data"))->sigClicked().connect(bind( &VelodyneBar::onPCDButtonClicked, this));
  addButton(("START"), ("Start to collision detection"))->sigClicked().connect(bind( &VelodyneBar::onSTARTButtonClicked, this));
  //addButton(("Collision"), ("Collision Detection"))->sigClicked().connect(bind( &VelodyneBar::onCollisionButtonClicked, this));
  addButton(("Remove"), ("Remove Pointcloud"))->sigClicked().connect(bind( &VelodyneBar::onRemoveButtonClicked, this));
  addButton(("r++"), ("+1m"))->sigClicked().connect(bind( &VelodyneBar::onPlusButtonClicked, this));
  addButton(("r--"), ("-1m"))->sigClicked().connect(bind( &VelodyneBar::onMinusButtonClicked, this));
  addButton(("x++"), ("-1m"))->sigClicked().connect(bind( &VelodyneBar::addX, this));
  addButton(("x--"), ("-1m"))->sigClicked().connect(bind( &VelodyneBar::subX, this));
  addButton(("y++"), ("-1m"))->sigClicked().connect(bind( &VelodyneBar::addY, this));
  addButton(("y--"), ("-1m"))->sigClicked().connect(bind( &VelodyneBar::subY, this));
  addButton(("z++"), ("-1m"))->sigClicked().connect(bind( &VelodyneBar::addZ, this));
  addButton(("z--"), ("-1m"))->sigClicked().connect(bind( &VelodyneBar::subZ, this));
  //addButton(("ChangeTheta"), ("Reset Positon"))->sigClicked().connect(bind(&VelodyneBar::changeTheta, this));
  //addButton(("Theta++"), ("Reset Positon"))->sigClicked().connect(bind(&VelodyneBar::plusTheta, this));
  //addButton(("Theta--"), ("Reset Positon"))->sigClicked().connect(bind(&VelodyneBar::minusTheta, this));
  //addButton(("Calc"), ("Reset Positon"))->sigClicked().connect(bind(&VelodyneBar::calcArmPosition, this));
  //addButton(("Arm"), ("Move Arm"))->sigClicked().connect(bind( &VelodyneBar::moveArm, this));
  //addButton(("ModelCollision"), ("check model collision"))->sigClicked().connect(bind( &VelodyneBar::checkModelCollision, this));
  addButton(("Reset"), ("Reset Positon"))->sigClicked().connect(bind( &VelodyneBar::onResetButtonClicked, this));
  addButton(("Pos"), ("View Position"))->sigClicked().connect(bind( &VelodyneBar::viewPosition, this));
  addButton(("SetRealPos"), ("Set Real Position"))->sigClicked().connect(bind( &VelodyneBar::setPoseButtonClicked, this));
  addButton(("GetPose"), ("Get Real Position"))->sigClicked().connect(bind( &VelodyneBar::getPose, this));
  addButton(("TraceReal"), ("Trace Simulation"))->sigClicked().connect(bind( &VelodyneBar::traceTrajectory, this));
  addButton(("TraceSim"), ("Trace Simulation"))->sigClicked().connect(bind( &VelodyneBar::traceTrajectorySimulation, this));
  //アイテム選択で呼び出し
  ItemTreeView::mainInstance()->sigSelectionChanged().connect(bind( &VelodyneBar::onItemSelectionChanged, this, _1));

  // ros nodehandle, topic, service init
  static ros::NodeHandle nh;
  subscribe_pcd = nh.subscribe("velodyne_points", 10, &VelodyneBar::receivePointCloudData, this);

  static boost::thread t(boost::bind( &VelodyneBar::rosOn, this)); //testボタン押すのめんどくさいので一時的に移植

  control_client = nh.serviceClient < tms_msg_rc::smartpal_control > ("sp5_control");

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------
VelodyneBar::~VelodyneBar() {
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onTestButtonClicked() {
  static boost::thread t(boost::bind( &VelodyneBar::rosOn, this));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
SgInvariantGroupPtr group = new SgInvariantGroup();
SgVertexArrayPtr vertices = new SgVertexArray();

void VelodyneBar::onPCDButtonClicked() {
  /*
   vertices->reserve(pointCloudData.points.size());  //たくさんの点群の場合

   for (int i = 0; i < pointCloudData.points.size(); i++) {
   SgVector3 vertex = SgVector3(pointCloudData.points[i].x, pointCloudData.points[i].y, pointCloudData.points[i].z + 1);
   vertices->push_back(vertex);
   }
   SgPointSetPtr pointSet = new SgPointSet();
   pointSet->setPointSize(100);
   pointSet->setVertices(vertices);

   if (pointSet) {
   group->addChild(pointSet);
   SceneView::instance()->addEntity(group);
   //os << pointSet->vertices()->size() << " points have been added." << endl;
   }
   */

  os << "targetBodyItems size = " << targetBodyItems.size() << endl;

  if(targetBodyItems.size() != 1) {
    os << "Please select one bodyitem" << endl;
    return;
  }

  SgPointsRenderer::SgLastRenderer(0, true);
  SgGroupPtr node = (SgGroup*) targetBodyItems[0]->body()->link(0)->shape();
  SgPointsGet visit;
  node->accept(visit);

  if(visit.shape.size() == 0) {
    os << "no shape node" << visit.shape.size() << endl;
    return;
  }

  for(int i = 0; i < pointCloudData.points.size(); i++) {
    pointCloudData.points[i].z += 1;
  }
  SgPointsRenderer* cr = SgPointsRenderer::SgLastRenderer(0, false);
  cr = new SgPointsRenderer( &pointCloudData);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  os << "pcd size = " << pointCloudData.points.size() << endl;

  ItemTreeView::mainInstance()->checkItem(targetBodyItems[0], false);
  MessageView::mainInstance()->flush();
  ItemTreeView::mainInstance()->checkItem(targetBodyItems[0], true);
  MessageView::mainInstance()->flush();
  //sleep(2); }//２秒毎繰り返しwhile
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
int col_start = 0;
int col_start2 = 1;
int neutral = 1;
int neutral2 = 2; //2 is moving mode
vector<Vector3> col_result, col_result2;
double r = 0.9;
double cur_x, cur_y;
double ave_x = 1, ave_y = 1, ave_z = 1;
double arm_velocity = 0.8;
double deg_velocity = 60;
const int number_of_wait = 10;
int wait_time = 10;
double col_r = 100;
double tmp;

//const double theta_max[7] = {178, 14, 119, 129, 119, 43, 58};
//const double theta_min[7] = { -44, -109, -119, -1, -119, -15, -88};
//const double theta_max[7] = {120, -10, 90, 120, 10, 10, 10};
//const double theta_min[7] = {-20, -90, 0, 20, -10, -10, -10};

const double theta_max[7] = {80, -10, 90, 120, 10, 10, 10}; //default
//const double theta_min[7] = {-20, -90, -10, 20, -10, -10, -10}; //default //danger!!!!!!!!!!1
const double theta_min[7] = {-30, -90, 0, 20, -10, -10, -10}; //with chipstar //very safe
//const double theta_max[7] = {60, -10, 60, 120, 10, 10, 10};//with grip
//const double theta_min[7] = {0, -30, 0, 80, -10, -10, -10};//with grip

//const double neutral_angle[4] = {30 * (PI / 180),-45 * (PI / 180),40 * (PI / 180),90 * (PI / 180)}; //origin
//const double neutral_angle[4] = {30 * (PI / 180), -45 * (PI / 180), 40 * (PI / 180), 80 * (PI / 180)};
const double neutral_angle[4] = {30 * (PI / 180), -10 * (PI / 180), 30 * (PI / 180), 80 * (PI / 180)}; //default
//const double neutral_angle[4] = {30 * (PI / 180), -10 * (PI / 180), 30 * (PI / 180), 80 * (PI / 180)}; //default of 2
//const double neutral_angle[4] = {-30 * (PI / 180), 0 * (PI / 180), 0 * (PI / 180), 120 * (PI / 180)}; //test of 2
//const double neutral_angle[4] = {0 * (PI / 180),-45 * (PI / 180),90 * (PI / 180),80 * (PI / 180)};
//const double neutral_angle[4] = {20 * (PI / 180),-15 * (PI / 180),0 * (PI / 180),90 * (PI / 180)}; //with chipstar
double input_angle[4] = {neutral_angle[0], neutral_angle[1], neutral_angle[2], neutral_angle[3]};
//double input_angle2[4] = {neutral_angle2[0], neutral_angle2[1], neutral_angle2[2], neutral_angle2[3]};
int moving_mode = 1; //1で動作モード、0で回避モード
int moving_stage = 0; //0~(max1-1)
int moving_stage_flag[17] = {0,0,0,1,1,0,0,1,0,0,0,1,0,0,1,1,0};
int moving_stage_next[17] = {0,0,0,1,2,0,0,5,0,0,0,9,0,0,13,13,0};

//vector<vector<int> > v(max1, vector<int>(max2));
 
int max1 = 17; //動作数
int max2 = 10; //入力角度数
//const vector<vector<int> > v(max1, vector<int>(max2));
/*double trj1[9][10] = { { -0.000000, 0.000000, 0.000000, -10.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -10.000000}, { -3.257106, 13.028425, 17.667728, -17.809287, -2.926289, 73.319758,
    22.776568, 3.174861, -6.390485, -57.837527}, { -2.783433, 11.133732, 23.031068, -17.545459, -3.752058, 73.649025, 17.768915, 2.039502, -4.498161, -57.868737}, { -5.717445, 22.869780, 29.476996,
    -14.649868, -5.842745, 70.287520, 15.567830, 8.207946, -2.647503, -57.868737}, { -5.717445, 22.869780, 29.476996, -14.649868, -5.842745, 70.287520, 15.567830, 8.207946, -2.647503, -23.491270}, {
    -4.890563, 19.562251, 30.411269, -14.811971, -5.828565, 71.094462, 15.671102, 4.016360, -2.472068, -23.491270}, { -3.139890, 12.559561, 16.833554, -13.379875, -5.479047, 70.451630, 32.475706,
    8.916440, -13.648250, -29.268933}, { -2.817979, 11.271917, 14.956688, -13.010939, -4.929374, 63.801394, 29.756314, 8.170620, -12.548281, -28.839457}, { -0.000000, 0.000000, 0.000000, -10.00000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -23.491270}};
*/ /*
double trj1[9][10] = { { -0.000000, 0.000000, 0.000000, -10.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -10.000000}, 
{ -3.257106, 13.028425, 17.667728, -17.809287, -2.926289, 73.319758,22.776568, 3.174861, -6.390485, -57.837527},
 { -2.783433, 11.133732, 23.031068, -17.545459, -3.752058, 73.649025, 17.768915, 2.039502, -4.498161, -57.868737}, 
 { -5.717445, 22.869780, 29.476996,-14.649868, -5.842745, 70.287520, 15.567830, 8.207946, -2.647503, -57.868737}, 
 { -5.717445, 22.869780, 29.476996, -14.649868, -5.842745, 70.287520, 15.567830, 8.207946, -2.647503, -23.491270}, 
 { -4.890563, 19.562251, 30.411269, -14.811971, -5.828565, 71.094462, 15.671102, 4.016360, -2.472068, -23.491270}, 
 { -3.139890, 12.559561, 16.833554, -13.379875, -5.479047, 85.451630, 15.671102, 4.016360, -2.472068, -29.268933}, 
 { -2.817979, 11.271917, 5.956688, -13.010939, -4.929374, 63.801394, 15.671102, 4.016360, -2.472068, -29.268933}, 
 { -0.000000, 0.000000, 0.000000, -10.00000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -23.491270}};
*/
/*
double trj1[17][10] = { //ver q6-8
 	{0,0,0,-10,0,0,0,0,0,-10},//neutral

 	{0,0,-10,-10,0,30,0,-8,0,-10},//move
 	{0,0,-30,0,0,120,0,-8,0,-10},//move
 	{0,0,-30,0,0,120,0,-8,0,-45},//open
 	{0,0,20,0,0,45,0,17,0,-45},//move
 	{0,0,20,0,0,45,0,17,0,-30},//close
 	{0,0,-30,0,0,120,0,-8,0,-30},//move
 	{0,0,-10,-10,0,30,0,-8,0,-30},//move

 	{0,0,0,-10,0,0,0,-8,0,-30},//move neutral

 	{0,0,-10,-10,0,30,0,-8,0,-30},//move
 	{0,0,-30,0,0,120,0,-8,0,-30},//move
  	{0,0,20,0,0,45,0,17,0,-30},//move
 	{0,0,20,0,0,45,0,17,0,-45},//open
	{0,0,-30,0,0,120,0,-8,0,-45},//move
	{0,0,-30,0,0,120,0,-8,0,-10},//close
	{0,0,-10,-10,0,30,0,-8,0,-10},//move

	{0,0,0,-10,0,0,0,-8,0,-10}//neutral
 };
 */
double trj1[17][10] = { //original
 	{0,0,0,-10,0,0,0,0,0,-10},//neutral

 	{0,0,-10,-10,0,30,0,0,0,-10},//move
 	{0,0,-30,0,0,120,0,0,0,-10},//move
 	{0,0,-30,0,0,120,0,0,0,-45},//open
 	{0,0,20,0,0,45,0,25,0,-45},//move
 	{0,0,20,0,0,45,0,25,0,-30},//close
 	{0,0,-30,0,0,120,0,0,0,-30},//move
 	{0,0,-10,-10,0,30,0,0,0,-30},//move

 	{0,0,0,-10,0,0,0,0,0,-30},//move neutral

 	{0,0,-10,-10,0,30,0,0,0,-30},//move
 	{0,0,-30,0,0,120,0,0,0,-30},//move
  	{0,0,20,0,0,45,0,25,0,-30},//move
 	{0,0,20,0,0,45,0,25,0,-45},//open
	{0,0,-30,0,0,120,0,0,0,-45},//move
	{0,0,-30,0,0,120,0,0,0,-10},//close
	{0,0,-10,-10,0,30,0,0,0,-10},//move

	{0,0,0,-10,0,0,0,0,0,-10}//neutral
 };
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onSTARTButtonClicked() {
  if(col_start == 0) {
    col_start = 1;
    os << "START!" << endl;
  } else {
    col_start = 0;
    os << "STOP!" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems) {
  selectedBodyItems_ = bodyItems;
  //os << "(ve)selectedBodyItems_ size = " << selectedBodyItems_.size() << endl;
  targetBodyItems = selectedBodyItems_;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onCollisionButtonClicked() {
  cout << "1" << endl;
  //os << "pcd size = " << pointCloudData.points.size() << endl;
  if(targetBodyItems.size() != 1) {
    os << "Please select one bodyitem" << endl;
    return;
  }
  cout << "1.1" << endl;
  //実機の関節角を取る
  //getPose();
  cout << "1.3" << endl;
  //Velodyneからの受信点群
  vector < Vector3 > p;
  //clock_t start = clock();

  for(int i = 0; i < pointCloudData.points.size(); i++) {
    if(pointCloudData.points[i].y > -1) {
      if( (pointCloudData.points[i].y - 0.055 < 1.3) && (pointCloudData.points[i].z + 1.43 > 0.75) ) { //grep and evade
        p.push_back(Vector3(pointCloudData.points[i].x, pointCloudData.points[i].y - 0.055, pointCloudData.points[i].z + 1.43));
      }
    }
  }
  cout << "2" << endl;
  os << "." << endl;
  os << "size0 = " << p.size() << endl;
  //onRemoveButtonClicked();
  //viewCollidingPoints (p);
  //sleep(2);

  //モデルの位置更新
  for(int j = 0; j < targetBodyItems[0]->body()->numLinks(); j++) { //パルのリンク3~10右腕
    targetBodyItems[0]->body()->link(j)->coldetModel()->setPosition(targetBodyItems[0]->body()->link(j)->T());
  }

  //os << "p(" << targetBodyItems[0]->body()->link(0)->p()[0] << ", " << targetBodyItems[0]->body()->link(0)->p()[1] << ", " << targetBodyItems[0]->body()->link(0)->p()[2] << ")" << endl;

  //vector<Vector3> col_result, col_result2;
  col_result.clear(); //reset
  col_result2.clear();
  //干渉判定、干渉点だけをcol_resultへ詰める
  PlanBase::instance()->isCollidingPointCloud3(p, targetBodyItems[0], col_result, r);
  os << "size1 = " << col_result.size() << endl;

  //干渉点からロボット自身の点を除く
  PlanBase::instance()->removeCollidingPointCloud2(col_result, targetBodyItems[0], col_result2, 0.2);
  os << "size2 = " << col_result2.size() << endl;
  //clock_t end = clock();
  cout << "3" << endl;
  evade();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::viewCollidingPoint(double x, double y, double z) {
  vertices->reserve(pointCloudData.points.size());  //たくさんの点群の場合

  SgVector3 vertex = SgVector3(x, y, z);
  vertices->push_back(vertex);
  for(int i = 0; i < 20; i++) {
    vertex = SgVector3(x + 0.001 * i, y, z);
    vertices->push_back(vertex);
    vertex = SgVector3(x - 0.001 * i, y, z);
    vertices->push_back(vertex);
    vertex = SgVector3(x, y + 0.001 * i, z);
    vertices->push_back(vertex);
    vertex = SgVector3(x, y - 0.001 * i, z);
    vertices->push_back(vertex);
    vertex = SgVector3(x, y, z + 0.001 * i);
    vertices->push_back(vertex);
    vertex = SgVector3(x, y, z - 0.001 * i);
    vertices->push_back(vertex);
  }

  SgPointSetPtr pointSet = new SgPointSet();
  pointSet->setVertices(vertices);

  if(pointSet) {
    group->addChild(pointSet);
    SceneView::instance()->addEntity(group);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::viewCollidingPoints(vector<Vector3>& p) {

  vertices->reserve(pointCloudData.points.size());  //たくさんの点群の場合

  for(int i = 0; i < p.size(); i++) {
    //SgVector3 vertex = SgVector3(p[i][0] + cur_x, p[i][1] + cur_y, p[i][2]+1.35);
    SgVector3 vertex = SgVector3(p[i][0], p[i][1], p[i][2]);
    vertices->push_back(vertex);
  }

  SgPointSetPtr pointSet = new SgPointSet();
  pointSet->setVertices(vertices);

  if(pointSet) {
    group->addChild(pointSet);
    SceneView::instance()->addEntity(group);
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onPlusButtonClicked() {
  r = r + 0.1;
  os << "r : " << setprecision(2) << r << "m" << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onMinusButtonClicked() {
  if(r > 0) {
    r = r - 0.1;
  }
  os << "r : " << setprecision(2) << r << "m" << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
double mem_arm[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
void VelodyneBar::addX() {
  /*
   PlanBase::instance()->arm()->IK_arm(Vector3(PlanBase::instance()->palm()->p()[0] + 0.01, PlanBase::instance()->palm()->p()[1], PlanBase::instance()->palm()->p()[2]),
   PlanBase::instance()->palm()->R());
   PlanBase::instance()->calcForwardKinematics();
   PlanBase::instance()->flush();
   
   if(targetBodyItems.size() != 1) {
   os << "Please select one bodyitem" << endl;
   return;
   }
   */
  /*
   for (int j = 0; j < targetBodyItems[0]->body()->numLinks(); j++) {
   targetBodyItems[0]->body()->link(j)->coldetModel()->setPosition(targetBodyItems[0]->body()->link(j)->T());
   }
   */
  //ave_x += 0.1;
  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
  /*for(int i = 0; i < 7; i++) {
   mem_arm[i] = PlanBase::instance()->arm()->arm_path->joint(i)->q();
   }*/

  arm_velocity += 0.1;
  os << "arm_vel = " << arm_velocity << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::subX() {
  /*
   PlanBase::instance()->arm()->IK_arm(Vector3(PlanBase::instance()->palm()->p()[0] - 0.01, PlanBase::instance()->palm()->p()[1], PlanBase::instance()->palm()->p()[2]),
   PlanBase::instance()->palm()->R());
   PlanBase::instance()->calcForwardKinematics();
   PlanBase::instance()->flush();
   */
  //ave_x -= 0.1;
  //targetBodyItems[0]->body()->link(0)->p()[0] -= 0.1;
  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
  /*for(int i = 0; i < 7; i++) {
   PlanBase::instance()->arm()->arm_path->joint(i)->q() = mem_arm[i];
   }*/
  if(arm_velocity > 0.1) arm_velocity -= 0.1;
  os << "arm_vel = " << arm_velocity << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
int joint_number = 6;
void VelodyneBar::addY() {
  /*
   PlanBase::instance()->arm()->IK_arm(Vector3(PlanBase::instance()->palm()->p()[0], PlanBase::instance()->palm()->p()[1] + 0.01, PlanBase::instance()->palm()->p()[2]),
   PlanBase::instance()->palm()->R());
   *//*
   PlanBase::instance()->arm()->arm_path->joint(joint_number)->q() += 0.1;
   os << "joint_number is " << joint_number << endl;
   os << PlanBase::instance()->arm()->arm_path->joint(joint_number)->q() << endl;
   PlanBase::instance()->calcForwardKinematics();
   PlanBase::instance()->flush();
   */
  /*
   os << PlanBase::instance()->bodyItemRobot()->body()->link(joint_number)->p()[0] << ", "
   << PlanBase::instance()->bodyItemRobot()->body()->link(joint_number)->p()[1] << ", "
   << PlanBase::instance()->bodyItemRobot()->body()->link(joint_number)->p()[2] << endl;

   targetBodyItems[0]->body()->link(joint_number)->p()[0] += 0.1;
   PlanBase::instance()->calcForwardKinematics();
   PlanBase::instance()->flush();

   for(int j = 0; j < targetBodyItems[0]->body()->numLinks(); j++) { //パルのリンク3~10右腕
   targetBodyItems[0]->body()->link(j)->coldetModel()->setPosition(targetBodyItems[0]->body()->link(j)->T());
   }*/
  //ave_y += 0.1;
  //targetBodyItems[0]->body()->link(0)->p()[1] += 0.1;
  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
  deg_velocity += 10;
  os << "deg_vel = " << deg_velocity << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::subY() {
  /*
   if(joint_number != 10) {
   joint_number++;
   os << "joint_number is " << joint_number << endl;
   } else {
   joint_number = 3;
   os << "joint_number is 3" << endl;
   }*/
  /* 
   PlanBase::instance()->arm()->IK_arm(Vector3(PlanBase::instance()->palm()->p()[0], PlanBase::instance()->palm()->p()[1] - 0.01, PlanBase::instance()->palm()->p()[2]),
   PlanBase::instance()->palm()->R());
   PlanBase::instance()->calcForwardKinematics();
   PlanBase::instance()->flush();
   */
  //ave_y -= 0.1;
  //targetBodyItems[0]->body()->link(0)->p()[1] -= 0.1;
  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
  if(deg_velocity > 10) deg_velocity -= 10;
  os << "deg_vel = " << deg_velocity << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::addZ() {
  /*
  PlanBase::instance()->arm()->IK_arm(Vector3(PlanBase::instance()->palm()->p()[0], PlanBase::instance()->palm()->p()[1], PlanBase::instance()->palm()->p()[2] + 0.01),
  PlanBase::instance()->palm()->R());
  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();
  */
  os << "j(0)" << PlanBase::instance()->arm()->arm_path->joint(0)->q() * (180 / PI) << endl;
  os << "j(1)" << PlanBase::instance()->arm()->arm_path->joint(1)->q() * (180 / PI) << endl;
  os << "j(2)" << PlanBase::instance()->arm()->arm_path->joint(2)->q() * (180 / PI) << endl;
  os << "j(3)" << PlanBase::instance()->arm()->arm_path->joint(3)->q() * (180 / PI) << endl;
  ave_z += 0.1;
  os << "link10 " << PlanBase::instance()->bodyItemRobot()->body()->link(10)->p() << endl;
  //targetBodyItems[0]->body()->link(0)->p()[2] += 0.1;
  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::subZ() {
  /*
   PlanBase::instance()->arm()->IK_arm(Vector3(PlanBase::instance()->palm()->p()[0], PlanBase::instance()->palm()->p()[1], PlanBase::instance()->palm()->p()[2] - 0.01),
   PlanBase::instance()->palm()->R());
   PlanBase::instance()->calcForwardKinematics();
   PlanBase::instance()->flush();
   */
  ave_z -= 0.1;
  //targetBodyItems[0]->body()->link(0)->p()[2] -= 0.1;
  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
  sp_control_srv.request.unit = 2; //Unit:2 ArmR
  sp_control_srv.request.cmd = 8;  //Cmd :8 getPose
  sp_control_srv.request.arg[0] = 0;  //arg[0]:flameID
  cout << "1.2" << endl;
  if(control_client.call(sp_control_srv)) {
    //os << "[TmsAction] Succeed to call service" << endl;
    int8_t result = sp_control_srv.response.result;
    //os << "[TmsAction] action arm result = " << (int) result << endl;
    //モデルに同期させる
    for(int i = 0; i < 7; i++) {
      PlanBase::instance()->arm()->arm_path->joint(i)->q() = sp_control_srv.response.val[i] * (PI / 180);
      os << "arg[" << i << "] = " << sp_control_srv.response.val[i] << endl;

      PlanBase::instance()->calcForwardKinematics();
      PlanBase::instance()->flush();
    }
  } else {
    os << "[TmsAction] Failed to call service sp5_control" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
double theta[5] = {0, 0, 0, 0, 0};
int itheta[5] = {0, 0, 0, 0, 0};
int theta_number = 1;
void VelodyneBar::changeTheta() {
  if(theta_number == 4) {
    theta_number = 1;
  } else {
    theta_number++;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::plusTheta() {
  itheta[theta_number]++;
  theta[theta_number] = (double) itheta[theta_number] * 90 * (PI / 180);
  os << "θ = (" << theta[1] * (180 / PI) << "°, " << theta[2] * (180 / PI) << "°, " << theta[3] * (180 / PI) << "°, " << theta[4] * (180 / PI) << "°)" << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::minusTheta() {
  itheta[theta_number]--;
  theta[theta_number] = (double) itheta[theta_number] * 90 * (PI / 180);
  os << "θ = (" << theta[1] * (180 / PI) << "°, " << theta[2] * (180 / PI) << "°, " << theta[3] * (180 / PI) << "°, " << theta[4] * (180 / PI) << "°)" << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::calcArmPosition() {
  os << sin(theta[2]) << " L1 + " << (sin(theta[2]) * cos(theta[4]) - cos(theta[2]) * sin(theta[3]) * sin(theta[4])) << " L2" << endl;
  os << -sin(theta[1]) * cos(theta[2]) << " L1 + " << -(cos(theta[1]) * cos(theta[3]) * sin(theta[4]) + sin(theta[1]) * (sin(theta[2]) * sin(theta[3]) * sin(theta[4]) + cos(theta[2]) * cos(theta[4])))
      << " L2" << endl;
  os << -cos(theta[1]) * cos(theta[2]) << " L1 + " << (sin(theta[1]) * cos(theta[3]) * sin(theta[4]) - cos(theta[1]) * (sin(theta[2]) * sin(theta[3]) * sin(theta[4]) + cos(theta[2]) * cos(theta[4])))
      << " L2" << endl;
  //os << PI << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
//現在の関節角
double theta1 = 0;
double theta2 = 0;
void VelodyneBar::moveArm(double x, double y, double z) {
  /*
   //モデルを選択してない場合終了
   if(targetBodyItems.size() != 1) {
   os << "Please select one bodyitem" << endl;
   return;
   }
   */
  //選択モデルの位置更新
  for(int j = 0; j < targetBodyItems[0]->body()->numLinks(); j++) {
    targetBodyItems[0]->body()->link(j)->coldetModel()->setPosition(targetBodyItems[0]->body()->link(j)->T());
  }

  //肩関節の座標（固定）
  const double sld_x = -0.2;
  const double sld_y = 0.025;
  const double sld_z = 1.13;
  const double sld_l = 0.28;

  //肘関節の座標
  double elb_x = sld_l * sin(theta2);
  double elb_y = -sld_l * sin(theta1) * cos(theta2);
  double elb_z = -sld_l * cos(theta1) * cos(theta2);

  //os << "collision point is (" << targetBodyItems[0]->body()->link(0)->p()[0] << ", " << targetBodyItems[0]->body()->link(0)->p()[1] << ", " << targetBodyItems[0]->body()->link(0)->p()[2] << ")"
  //    << endl;
  os << "collision point is (" << x << ", " << y << ", " << z << ")" << endl;
  //干渉点（選択モデルの座標）
  /*
   double col_x = targetBodyItems[0]->body()->link(0)->p()[0] - sld_x;
   double col_y = targetBodyItems[0]->body()->link(0)->p()[1] - sld_y;
   double col_z = targetBodyItems[0]->body()->link(0)->p()[2] - sld_z;
   */
  double col_x = x - sld_x;
  double col_y = y - sld_y;
  double col_z = z - sld_z;

  os << "shoulder-collision point is (" << col_x << ", " << col_y << ", " << col_z << ")" << endl;
  //干渉点方向への関節角、逃げる方向への関節角
  double col_theta1 = 0;
  double col_theta2 = 0;
  double des_theta1 = 0;
  double des_theta2 = 0;
  //分母が0の特異点を場合分け
  if(col_y * col_y + col_z * col_z == 0) {
    col_theta1 = 0;
  } else {
    col_theta1 = asin( -col_y / sqrt(col_y * col_y + col_z * col_z));
    //os << "1collision θ is (" << col_theta1 * (180 / PI) << "°, " << col_theta2 * (180 / PI) << "°)" << endl;
    /*
     if((col_z > 0) * (col_y > 0)) {
     col_theta1 = -PI - col_theta1;
     os << "2collision θ is (" << col_theta1 * (180 / PI) << "°, " << col_theta2 * (180 / PI) << "°)" << endl;
     } else if((col_z > 0) * (col_y < 0)) {
     col_theta1 = PI - col_theta1;
     os << "3collision θ is (" << col_theta1 * (180 / PI) << "°, " << col_theta2 * (180 / PI) << "°)" << endl;
     }
     */
  }
  if(col_x * col_x + col_y * col_y + col_z * col_z == 0) {
    col_theta2 = 0;
  } else {
    col_theta2 = asin(col_x / sqrt(col_x * col_x + col_y * col_y + col_z * col_z));
    //os << "4collision θ is (" << col_theta1 * (180 / PI) << "°, " << col_theta2 * (180 / PI) << "°)" << endl;
    if((col_x < 0) * (col_z > 0)) {
      col_theta2 = -PI - col_theta2;
      //os << "5collision θ is (" << col_theta1 * (180 / PI) << "°, " << col_theta2 * (180 / PI) << "°)" << endl;
    } else if((col_x > 0) * (col_z > 0)) {
      col_theta2 = PI - col_theta2;
      //os << "6collision θ is (" << col_theta1 * (180 / PI) << "°, " << col_theta2 * (180 / PI) << "°)" << endl;
    }
  }
  //os << "collision θ is (" << setprecision(1) << col_theta1* (180/PI) << "°, " << setprecision(1) << col_theta2* (180/PI) << "°)" << endl;
  //os << "before θ is (" <<  setprecision(1) << theta1* (180/PI) << "°, " << setprecision(1) << theta2* (180/PI) << "°)" << endl;

  os << "collision θ is (" << col_theta1 * (180 / PI) << "°, " << col_theta2 * (180 / PI) << "°)" << endl;
  os << "before θ is (" << theta1 * (180 / PI) << "°, " << theta2 * (180 / PI) << "°)" << endl;

  //目的角設定
  if(col_theta1 > 0) {
    des_theta1 = -col_theta1;
  } else {
    des_theta1 = -col_theta1;
  }
  if(col_theta2 > 0) {
    des_theta2 = col_theta2 - PI;
    os << "destination θ is (" << des_theta1 * (180 / PI) << "°, " << des_theta2 * (180 / PI) << "°)" << endl;
  } else {
    des_theta2 = col_theta2 + PI;
    os << "destination θ is (" << des_theta1 * (180 / PI) << "°, " << des_theta2 * (180 / PI) << "°)" << endl;
  }

  //ボーダーライン修正
  /*
   if(col_theta1 < -113 * (PI / 180)) {
   col_theta1 += 2 * PI;
   os << "θ1修正" << endl;
   }
   if((des_theta2 > 0) && (col_theta2 > 132.5 * (PI / 180)) {
   col_theta2 -= 2 * PI;
   os << "θ2修正" << endl;
   }*/
  if((des_theta1 < col_theta1) && (col_theta1 < theta1)) {
    des_theta1 += 2 * PI;
  } else if((des_theta1 > col_theta1) && (col_theta1 > theta1)) {
    des_theta1 -= 2 * PI;
  }
  if((des_theta2 < col_theta2) && (col_theta2 < theta2)) {
    des_theta2 += 2 * PI;
  } else if((des_theta2 > col_theta2) && (col_theta2 > theta2)) {
    des_theta2 -= 2 * PI;
  }
  //関節角更新
  //θ1:-44~178 θ2:-109~14
  for(int i = 0; i < 10; i++) {
    theta1 = max(min(theta1 + (des_theta1 - theta1) / 1000, 178 * (PI / 180)), -44 * (PI / 180));
    theta2 = max(min(theta2 + (des_theta2 - theta2) / 1000, 14 * (PI / 180)), -109 * (PI / 180));
  }

  //os << "after θ is (" << setprecision(1) << theta1* (180/PI) << "°, " << setprecision(1) << theta2* (180/PI) << "°)" << endl;
  os << "after θ is (" << theta1 * (180 / PI) << "°, " << theta2 * (180 / PI) << "°)" << endl;

  if(theta1 == 178 * (PI / 180)) os << "θ1 is max." << endl;
  if(theta1 == -44 * (PI / 180)) os << "θ1 is min." << endl;
  if(theta2 == 14 * (PI / 180)) os << "θ2 is max." << endl;
  if(theta2 == -109 * (PI / 180)) os << "θ2 is min." << endl;

  //PlanBase::instance()->arm()->arm_path->joint(0)->q() = asin(-elb_y/sqrt( 0.28*0.28 - elb_x*elb_x ));
  //PlanBase::instance()->arm()->arm_path->joint(1)->q() = asin(elb_x/0.28);

  PlanBase::instance()->arm()->arm_path->joint(0)->q() = theta1;
  PlanBase::instance()->arm()->arm_path->joint(1)->q() = theta2;

  //チップスター
  //if(targetObject) targetObject->bodyItemObject->notifyKinematicStateChange();

  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::moveArm2(double x, double y, double z) {

//  x = 0;
//  y = 2;
//  z = 1;
  Vector3 op(x, y, z);

  MatrixXd J = MatrixXd::Zero(6, 7);     //ヤコビアン
  //MatrixXd K = MatrixXd::Zero(3, 7);     //ヤコビアンの左半分
  MatrixXd K = MatrixXd::Zero(3, 4);     //ヤコビアンの左半分
  //MatrixXd invJ = MatrixXd::Zero(7, 3);  //逆ヤコビアン
  MatrixXd invJ = MatrixXd::Zero(4, 3);  //逆ヤコビアン
  VectorXd PA = VectorXd::Zero(3);     //手先相対位置
  VectorXd VE = VectorXd::Zero(3);     //手先速度ベクトル
  //VectorXd TH = VectorXd::Zero(7);     //関節速度ベクトル
  VectorXd TH = VectorXd::Zero(4);     //関節速度ベクトル

  PlanBase::instance()->arm()->arm_path->calcJacobian(J);

  os << "J =" << J << endl;

  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 4; j++) {
      K(i, j) = J(i, j);
    }
  }
  os << "K =" << K << endl;

  calcPseudoInverse(K, invJ);
  os << "invJ =" << invJ << endl;

//  PA(0, 0) = PlanBase::instance()->palm()->p()[0];
//  PA(1, 0) = PlanBase::instance()->palm()->p()[1];
//  PA(2, 0) = PlanBase::instance()->palm()->p()[2];
//  PA 

  //os << "PA =" << PA << endl;

  PA = PlanBase::instance()->palm()->p() - op;
  os << "PA = (" << PA(0) << ", " << PA(1) << ", " << PA(2) << ")" << endl;

  VE = PA / PA.norm() * 0.01;

  os << "VE = (" << VE(0) << ", " << VE(1) << ", " << VE(2) << ")" << endl;
  //os << "VE =" << VE << endl;

  TH = invJ * VE; //関節速度算出

  //os << "TH = (" << TH(0, 0) << ", " << TH(1, 0) << ", " << TH(2, 0) << ", " << TH(3, 0) << ", " << TH(4, 0) << ", " << TH(5, 0) << ", " << TH(6, 0) << ", " << TH(7, 0) << ")" << endl;//何故か落ちる
  os << "TH =" << TH << endl;

  //関節角更新
  //θ1:-44~178 θ2:-109~14
  //距離比較ここから
  double old_distance = PA(0) * PA(0) + PA(1) * PA(1) + PA(2) * PA(2); //相対距離記憶○
  os << "distance " << old_distance << endl; //○
  VectorXd MEM = VectorXd::Zero(7);
  for(int i = 0; i < 7; i++) {
    MEM(i) = PlanBase::instance()->arm()->arm_path->joint(i)->q();
  }
  //ここまで*/
  for(int i = 0; i < 4; i++) {
    PlanBase::instance()->arm()->arm_path->joint(i)->q() += TH(i);

    if(PlanBase::instance()->arm()->arm_path->joint(i)->q() > PlanBase::instance()->arm()->arm_path->joint(i)->ulimit() - 0.2) {
      PlanBase::instance()->arm()->arm_path->joint(i)->q() = PlanBase::instance()->arm()->arm_path->joint(i)->ulimit() - 0.2;
    } else if(PlanBase::instance()->arm()->arm_path->joint(i)->q() < PlanBase::instance()->arm()->arm_path->joint(i)->llimit() + 0.2) {
      PlanBase::instance()->arm()->arm_path->joint(i)->q() = PlanBase::instance()->arm()->arm_path->joint(i)->llimit() + 0.2;
    }
  }

  //チップスター
  //if(targetObject) targetObject->bodyItemObject->notifyKinematicStateChange();

  //距離比較ここから
  PlanBase::instance()->calcForwardKinematics();
  PA = PlanBase::instance()->palm()->p() - op; //○
  double new_distance = PA(0) * PA(0) + PA(1) * PA(1) + PA(2) * PA(2); //○
  os << "distance " << new_distance << endl; //○　

  //動作後に干渉点に近くなっていたら戻す
  if(new_distance + 0.05 < old_distance) {
    for(int i = 0; i < 7; i++) {
      PlanBase::instance()->arm()->arm_path->joint(i)->q() = MEM(i);
    }
  }
  //ここまで

  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::moveElbow(double x, double y, double z) {
  Vector3 op(x, y, z);

  Link* palm = PlanBase::instance()->palm();

  MatrixXd eJ = MatrixXd::Zero(6, 3);
  //MatrixXd inveJ = MatrixXd::Zero(3, 3);
  MatrixXd inveJ = MatrixXd::Zero(2, 3);
  cnoid::JointPathPtr elbow_shoulder_path = make_shared < JointPath > (PlanBase::instance()->arm()->base, PlanBase::instance()->arm()->arm_path->joint(3));
  elbow_shoulder_path->calcJacobian(eJ);
  //os << "eJ =" << eJ << endl;
  eJ(0, 2) = 0;
  eJ(1, 2) = 0;
  eJ(2, 2) = 0;
  calcPseudoInverse(eJ.block(0, 0, 3, 2), inveJ); //eJの上半分(3*3)の逆行列がinveJ(3*3)
  //cout << "inveJ =" << inveJ << endl;
  //cout << "elbow pos" << PlanBase::instance()->arm()->arm_path->joint(3)->p().transpose() << endl;

  VectorXd eo = PlanBase::instance()->arm()->arm_path->joint(3)->p() - op; //eo=干渉点から見た肘関節の相対座標

  // os << "j0 = " << PlanBase::instance()->arm()->arm_path->joint(0)->q() << endl;
  //os << "j1 = " << PlanBase::instance()->arm()->arm_path->joint(1)->q() << endl;
  //os << "j2 = " << PlanBase::instance()->arm()->arm_path->joint(2)->q() << endl;
  //os << "j3 = " << PlanBase::instance()->arm()->arm_path->joint(3)->q() << endl;

  // os << "eo = " << eo << endl;

  eo.normalize(); //単位行列化

  eo = 0.03 * eo;
  //cout << eo.transpose() << endl;
  //os << "eo = " << eo << endl;

  VectorXd vq = inveJ * eo;

  //os << "vq = " << vq << endl;

  for(int i = 0; i < 2; i++) {
    input_angle[i] += vq(i);

    if(input_angle[i] > theta_max[i] * (PI / 180)) {
      input_angle[i] = theta_max[i] * (PI / 180);
    } else if(input_angle[i] < theta_min[i] * (PI / 180)) {
      input_angle[i] = theta_min[i] * (PI / 180);
    }

  }
  //PlanBase::instance()->calcForwardKinematics();

  MatrixXd J = MatrixXd::Zero(6, 7);     //ヤコビアン
  MatrixXd K = MatrixXd::Zero(3, 7);     //ヤコビアンの左半分
  MatrixXd invJ = MatrixXd::Zero(7, 3);  //逆ヤコビアン
  /*static*/VectorXd PA = PlanBase::instance()->palm()->p();     //手先初期座標
  /*static*/MatrixXd RP = PlanBase::instance()->palm()->R();     //手先初期角度（ロールピッチヨウ）
  VectorXd VE = VectorXd::Zero(3);     //手先速度ベクトル
  VectorXd TH = VectorXd::Zero(7);     //関節速度ベクトル
  VectorXd dq(7);

  PlanBase::instance()->arm()->arm_path->calcJacobian(J);
  //os << "J =" << J << endl;
  calcPseudoInverse(J, invJ);
  //os << "invJ =" << invJ << endl;

  VectorXd v(6);
  Vector3 dp(PA - palm->p());
  Vector3 omega(palm->R() * omegaFromRot((palm->R()).transpose() * RP));
  setVector3(dp, v, 0);
  setVector3(omega, v, 3);

  dq = invJ * v;
  //dq = 0.95 * dq;
  /*
   for(int i = 0; i < 7; i++) {
   PlanBase::instance()->arm()->arm_path->joint(i)->q() += dq(i);

   if(PlanBase::instance()->arm()->arm_path->joint(i)->q() > PlanBase::instance()->arm()->arm_path->joint(i)->ulimit() - 0.1) {
   PlanBase::instance()->arm()->arm_path->joint(i)->q() = PlanBase::instance()->arm()->arm_path->joint(i)->ulimit() - 0.1;
   } else if(PlanBase::instance()->arm()->arm_path->joint(i)->q() < PlanBase::instance()->arm()->arm_path->joint(i)->llimit() + 0.1) {
   PlanBase::instance()->arm()->arm_path->joint(i)->q() = PlanBase::instance()->arm()->arm_path->joint(i)->llimit() + 0.1;
   }

   }*/

  for(int i = 0; i < 4; i++) {
    input_angle[i] += dq(i);
    if(input_angle[i] > theta_max[i] * (PI / 180)) {
      input_angle[i] = theta_max[i] * (PI / 180);
      os << "upper limit " << i + 1 << endl;
      os << "q(" << i + 1 << ") = " << input_angle[i] * (180 / PI) << endl;
    } else if(input_angle[i] < theta_min[i] * (PI / 180)) {
      input_angle[i] = theta_min[i] * (PI / 180);
      os << "lower limit " << i + 1 << endl;
      os << "q(" << i + 1 << ") = " << input_angle[i] * (180 / PI) << endl;
    }
  }
  //チップスター
  //if(targetObject) targetObject->bodyItemObject->notifyKinematicStateChange();

  //PlanBase::instance()->arm()->arm_path->joint(4)->q() = 0;
  //PlanBase::instance()->arm()->arm_path->joint(5)->q() = 0;
  //PlanBase::instance()->arm()->arm_path->joint(6)->q() = 0;

  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------

void VelodyneBar::moveArm3(double x, double y, double z) {

  MatrixXd J = MatrixXd::Zero(6, 7);     //ヤコビアン
  MatrixXd K = MatrixXd::Zero(3, 4);     //ヤコビアンの左半分
  MatrixXd invJ = MatrixXd::Zero(4, 3);  //逆ヤコビアン
  //VectorXd PA = VectorXd::Zero(3);     //手先相対位置
  //VectorXd VE = VectorXd::Zero(3);     //手先速度ベクトル
  VectorXd TH = VectorXd::Zero(4);     //関節速度ベクトル
  //static
  //VectorXd P = PlanBase::instance()->palm()->p(); //手先現在座標
  //static VectorXd NP = PlanBase::instance()->palm()->p(); //手先初期座標,ニュートラル
  VectorXd P = PlanBase::instance()->bodyItemRobot()->body()->link(10)->p(); //手先現在座標(10:thumb)
  static VectorXd NP = PlanBase::instance()->bodyItemRobot()->body()->link(10)->p(); //手先初期座標,ニュートラル
  //VectorXd Co = VectorXd::Zero(3); //干渉点もベクトルに入れてみた
  //Co << x, y, z;
  Vector3 Co(x, y, z);
  VectorXd VE = VectorXd::Zero(3), F1 = VectorXd::Zero(3), F2 = VectorXd::Zero(3); //力
  double w1 = 1, w2 = 2; //重み 1 2
  cnoid::JointPathPtr thumb_shoulder_path = make_shared < JointPath > (PlanBase::instance()->arm()->base, PlanBase::instance()->arm()->arm_path->joint(6));
  cout << "4.41" << endl;
  //PlanBase::instance()->arm()->arm_path->calcJacobian(J);
  thumb_shoulder_path->calcJacobian(J);
  //os << "J =" << J << endl;

  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 4; j++) {
      K(i, j) = J(i, j);
    }
  }
  //os << "K =" << K << endl; //Jを切り取ったもの

  calcPseudoInverse(K, invJ);
  //os << "invJ =" << invJ << endl; //Kの逆行列
  cout << "4.42" << endl;
  double distance1;
  if(Co.transpose() * Co == 0) {
    //os << "No Collision!1" << endl;
    distance1 = 2; //if no colliding point
  } else {
    //distance1 = sqrt((P[0] - Co[0]) * (P[0] - Co[0]) + (P[1] - Co[1]) * (P[1] - Co[1]) + (P[2] - Co[2]) * (P[2] - Co[2])); //距離
    distance1 = sqrt((P - Co).transpose() * (P - Co));
  }
  if(distance1 > 1.7) {
    F1 << 0, 0, 0;
    //os << "No Collision!2" << endl;
  } else {
    //F1 = w1 * (1.5 - distance1) * ((P - Co) / (P - Co).norm()); //近いほど強く逃げる力
    F1 = w1 * (distance1 - 1.7) * (distance1 - 1.7) * ((P - Co) / (P - Co).norm()); //近いほど強く逃げる力
  }
  //行列使ったら綺麗に書けるのでは
  double distance2 = sqrt((NP[0] - P[0]) * (NP[0] - P[0]) + (NP[1] - P[1]) * (NP[1] - P[1]) + (NP[2] - P[2]) * (NP[2] - P[2])); //距離
  F2 = w2 * (NP - P); //初期座標へ戻ろうとする力
  cout << "4.43" << endl;
  os << "distanceF1 = " << distance1 << endl;
  os << "distanceF2 = " << distance2 << endl;
  //os << "F1 = " << F1 << endl;
  //os << "F2 = " << F2 << endl;

  VE = arm_velocity * (F1 + F2);
  //os << "VE =" << VE << endl;
  os << "VE = (" << VE[0] << ", " << VE[1] << ", " << VE[2] << ")" << endl;

  TH = invJ * VE; //関節速度算出

  //os << "TH = (" << TH(0, 0) << ", " << TH(1, 0) << ", " << TH(2, 0) << ", " << TH(3, 0) << ", " << TH(4, 0) << ", " << TH(5, 0) << ", " << TH(6, 0) << ", " << TH(7, 0) << ")" << endl;//何故か落ちる
  //os << "TH =" << TH << endl;
  os << "TH = (" << TH[0] << ", " << TH[1] << ", " << TH[2] << ", " << TH[3] << ")" << endl;
  cout << "4.44" << endl;
  /*for(int i=0; i<4; i++){
   if(TH(i)>0.2) TH(i)=0.2;
   }*/

  //関節角更新
  //θ1:-44~178 θ2:-109~14
  //距離比較ここから
  double old_distance = distance1; //距離記憶○
  os << "old distance = " << old_distance << endl; //○
  VectorXd MEM = VectorXd::Zero(7);
  for(int i = 0; i < 4; i++) {
    MEM(i) = input_angle[i];
  }
  //ここまで

  /*for(int i = 0; i < 4; i++) {
   PlanBase::instance()->arm()->arm_path->joint(i)->q() += TH(i);
   if(PlanBase::instance()->arm()->arm_path->joint(i)->q() > PlanBase::instance()->arm()->arm_path->joint(i)->ulimit() - 0.2) {
   PlanBase::instance()->arm()->arm_path->joint(i)->q() = PlanBase::instance()->arm()->arm_path->joint(i)->ulimit() - 0.2;
   os << "upper limit " << i+1 << endl;
   } else if(PlanBase::instance()->arm()->arm_path->joint(i)->q() < PlanBase::instance()->arm()->arm_path->joint(i)->llimit() + 0.2) {
   PlanBase::instance()->arm()->arm_path->joint(i)->q() = PlanBase::instance()->arm()->arm_path->joint(i)->llimit() + 0.2;
   os << "lower limit " << i+1 << endl;
   }*/
cout << "4.45" << endl;
  for(int i = 0; i < 4; i++) {
    input_angle[i] += TH(i);
    if(input_angle[i] > theta_max[i] * (PI / 180)) {
      input_angle[i] = theta_max[i] * (PI / 180);
      //os << "upper limit " << i+1 << endl;
      //os << "q(" << i+1 << ") = " << input_angle[i]*(180/PI) << endl;
    } else if(input_angle[i] < theta_min[i] * (PI / 180)) {
      input_angle[i] = theta_min[i] * (PI / 180);
      //os << "lower limit " << i+1 << endl;
      //os << "q(" << i+1 << ") = " << input_angle[i]*(180/PI) << endl;
    }
  }
  /*if(PlanBase::instance()->arm()->arm_path->joint(1)->q() > -10*(PI/180)){
   os << "EROOR" << endl;
   sleep(5);
   }*/

  //距離比較ここから
  for(int i = 0; i < 4; i++) { //input angle for simulater once
    PlanBase::instance()->arm()->arm_path->joint(i)->q() = input_angle[i];
  }
  PlanBase::instance()->calcForwardKinematics();
  P = PlanBase::instance()->bodyItemRobot()->body()->link(10)->p();
  distance1 = sqrt((P - Co).transpose() * (P - Co));
  //PA = PlanBase::instance()->palm()->p() - op; //○
  //double new_distance = PA(0) * PA(0) + PA(1) * PA(1) + PA(2) * PA(2); //○
  os << "new distance = " << distance1 << endl; //○　

  if(Co.transpose() * Co) {
    //動作後に干渉点に近くなっていたら戻す
    if(distance1 + 0.0 < old_distance) {
      os << "reject" << endl;
      for(int i = 0; i < 4; i++) {
        //PlanBase::instance()->arm()->arm_path->joint(i)->q() = MEM(i);
        input_angle[i] = MEM(i);
      }
      for(int i = 0; i < 4; i++) { //back to old angle
        PlanBase::instance()->arm()->arm_path->joint(i)->q() = input_angle[i];
      }
    }
  }
  //ここまで
  cout << "4.46" << endl;
  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------

void VelodyneBar::moveArm4(double x, double y, double z) {

  MatrixXd J = MatrixXd::Zero(6, 7);     //ヤコビアン
  MatrixXd K = MatrixXd::Zero(3, 4);     //ヤコビアンの左半分
  MatrixXd invJ = MatrixXd::Zero(4, 3);  //逆ヤコビアン
  VectorXd TH = VectorXd::Zero(4);     //関節速度ベクトル
  VectorXd P = PlanBase::instance()->bodyItemRobot()->body()->link(10)->p(); //手先現在座標(10:thumb)
  //static VectorXd NP = PlanBase::instance()->bodyItemRobot()->body()->link(10)->p(); //手先初期座標,ニュートラル
  VectorXd NP(3);
  NP << 0.42256, 0.411145, 1.08591;//手先初期座標,ニュートラル
  os << NP << endl;

  Vector3 Co(x, y, z);
  VectorXd VE = VectorXd::Zero(3), F1 = VectorXd::Zero(3), F2 = VectorXd::Zero(3); //力
  double w1 = 1, w2 = 2; //重み 1 2
  cnoid::JointPathPtr thumb_shoulder_path = make_shared < JointPath > (PlanBase::instance()->arm()->base, PlanBase::instance()->arm()->arm_path->joint(6));
  cout << "4.41" << endl;
  //PlanBase::instance()->arm()->arm_path->calcJacobian(J);
  thumb_shoulder_path->calcJacobian(J);
  //os << "J =" << J << endl;

  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 4; j++) {
      K(i, j) = J(i, j);
    }
  }
  //calcPseudoInverse(K, invJ);

  invJ = K.transpose();

  cout << "4.42" << endl;
  double distance1;
  if(Co.transpose() * Co == 0) {
    //os << "No Collision!1" << endl;
    distance1 = 2; //if no colliding point
  } else {
    //distance1 = sqrt((P[0] - Co[0]) * (P[0] - Co[0]) + (P[1] - Co[1]) * (P[1] - Co[1]) + (P[2] - Co[2]) * (P[2] - Co[2])); //距離
    distance1 = sqrt((P - Co).transpose() * (P - Co));
    os << "distance1 = " << distance1 << endl;
  }
  if(distance1 > 0.9) {
    F1 << 0, 0, 0;
    //os << "No Collision!2" << endl;
  } else {
    //F1 = w1 * (1.5 - distance1) * ((P - Co) / (P - Co).norm()); //近いほど強く逃げる力
    F1 = w1 * (distance1 - 1.7) * (distance1 - 1.7) * ((P - Co) / (P - Co).norm()); //近いほど強く逃げる力
  }
  //行列使ったら綺麗に書けるのでは
  //double distance2 = sqrt((NP[0] - P[0]) * (NP[0] - P[0]) + (NP[1] - P[1]) * (NP[1] - P[1]) + (NP[2] - P[2]) * (NP[2] - P[2])); //距離
  double distance2 = sqrt((NP - P).transpose() * (NP - P));
  //F2 = w2 * (NP - P); //初期座標へ戻ろうとする力
  F2 = w2 * distance2 * distance2 * ((NP - P) / (NP - P).norm()); //初期座標へ戻ろうとする力
  cout << "4.43" << endl;
  os << "distanceF1 = " << distance1 << endl;
  os << "distanceF2 = " << distance2 << endl;

  VE = arm_velocity * (F1 + F2);
  //os << "VE =" << VE << endl;
  os << "VE = (" << VE[0] << ", " << VE[1] << ", " << VE[2] << ")" << endl;

  TH = invJ * VE; //関節速度算出

  //os << "TH = (" << TH(0, 0) << ", " << TH(1, 0) << ", " << TH(2, 0) << ", " << TH(3, 0) << ", " << TH(4, 0) << ", " << TH(5, 0) << ", " << TH(6, 0) << ", " << TH(7, 0) << ")" << endl;//何故か落ちる
  //os << "TH =" << TH << endl;
  os << "TH = (" << TH[0] << ", " << TH[1] << ", " << TH[2] << ", " << TH[3] << ")" << endl;
  cout << "4.44" << endl;
  /*for(int i=0; i<4; i++){
   if(TH(i)>0.2) TH(i)=0.2;
   }*/

  //関節角更新
  //θ1:-44~178 θ2:-109~14
  //距離比較ここから
  double old_distance = distance1; //距離記憶○
  os << "old distance = " << old_distance << endl; //○
  VectorXd MEM = VectorXd::Zero(7);
  for(int i = 0; i < 4; i++) {
    MEM(i) = input_angle[i];
  }
  //ここまで

cout << "4.45" << endl;
  for(int i = 0; i < 4; i++) {
    input_angle[i] += TH(i);
    if(input_angle[i] > theta_max[i] * (PI / 180)) {
      input_angle[i] = theta_max[i] * (PI / 180);
      //os << "upper limit " << i+1 << endl;
      //os << "q(" << i+1 << ") = " << input_angle[i]*(180/PI) << endl;
    } else if(input_angle[i] < theta_min[i] * (PI / 180)) {
      input_angle[i] = theta_min[i] * (PI / 180);
      //os << "lower limit " << i+1 << endl;
      //os << "q(" << i+1 << ") = " << input_angle[i]*(180/PI) << endl;
    }
  }

  //距離比較ここから
  for(int i = 0; i < 4; i++) { //input angle for simulater once
    PlanBase::instance()->arm()->arm_path->joint(i)->q() = input_angle[i];
  }
  PlanBase::instance()->calcForwardKinematics();
  P = PlanBase::instance()->bodyItemRobot()->body()->link(10)->p();
  distance1 = sqrt((P - Co).transpose() * (P - Co));
  //PA = PlanBase::instance()->palm()->p() - op; //○
  //double new_distance = PA(0) * PA(0) + PA(1) * PA(1) + PA(2) * PA(2); //○
  os << "new distance = " << distance1 << endl; //○　

  if(Co.transpose() * Co) {
    //動作後に干渉点に近くなっていたら戻す
    if(distance1 + 0.0 < old_distance) {
      os << "reject" << endl;
      for(int i = 0; i < 4; i++) {
        //PlanBase::instance()->arm()->arm_path->joint(i)->q() = MEM(i);
        input_angle[i] = MEM(i);
      }
      for(int i = 0; i < 4; i++) { //back to old angle
        PlanBase::instance()->arm()->arm_path->joint(i)->q() = input_angle[i];
      }
    }
  }
  //ここまで
  cout << "4.46" << endl;
  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
//void VelodyneBar::returnNeutral() {

//}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::checkModelCollision() {
  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();
  PlanBase::instance()->initialCollision();
  os << "Model Collision = " << PlanBase::instance()->isColliding() << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onResetButtonClicked() {
  //PlanBase::instance()->arm()->IK_arm(Vector3(0.334928, 1.46938, 1.46938), PlanBase::instance()->palm()->R());
  /*
  for(int i = 0; i < 7; i++) {
    PlanBase::instance()->arm()->arm_path->joint(i)->q() = 0 * (PI / 180);
  }
  theta1 = 0;
  theta2 = -45 * (PI / 180);
  */
  /*
   PlanBase::instance()->arm()->arm_path->joint(0)->q() = 0 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(1)->q() = -45 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(2)->q() = 90 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(3)->q() = 90 * (PI / 180);
   *//*
   PlanBase::instance()->arm()->arm_path->joint(0)->q() = 0 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(1)->q() = -10 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(2)->q() = 0 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(3)->q() = 90 * (PI / 180);
   *//*
   PlanBase::instance()->arm()->arm_path->joint(0)->q() = 30 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(1)->q() = -45 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(2)->q() = 40 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(3)->q() = 90 * (PI / 180);
   input_angle[0] = 30 * (PI / 180);
   input_angle[1] = -45 * (PI / 180);
   input_angle[2] = 40 * (PI / 180);
   input_angle[3] = 90 * (PI / 180);
   *//*
   PlanBase::instance()->arm()->arm_path->joint(0)->q() = 10 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(1)->q() = -45 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(2)->q() = 90 * (PI / 180);
   PlanBase::instance()->arm()->arm_path->joint(3)->q() = 50 * (PI / 180);
   */
  PlanBase::instance()->arm()->arm_path->joint(0)->q() = neutral_angle[0];
  PlanBase::instance()->arm()->arm_path->joint(1)->q() = neutral_angle[1];
  PlanBase::instance()->arm()->arm_path->joint(2)->q() = neutral_angle[2];
  PlanBase::instance()->arm()->arm_path->joint(3)->q() = neutral_angle[3];
  input_angle[0] = neutral_angle[0];
  input_angle[1] = neutral_angle[1];
  input_angle[2] = neutral_angle[2];
  input_angle[3] = neutral_angle[3];

  PlanBase::instance()->bodyItemRobot()->body()->link(0)->p() = Vector3(0, 0, 0);

  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();

  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onResetButtonClicked2() {
	/*
	os << "resetButton2" << endl;
  PlanBase::instance()->arm()->arm_path->joint(0)->q() = neutral_angle2[0];
  PlanBase::instance()->arm()->arm_path->joint(1)->q() = neutral_angle2[1];
  PlanBase::instance()->arm()->arm_path->joint(2)->q() = neutral_angle2[2];
  PlanBase::instance()->arm()->arm_path->joint(3)->q() = neutral_angle2[3];
  input_angle2[0] = neutral_angle2[0];
  input_angle2[1] = neutral_angle2[1];
  input_angle2[2] = neutral_angle2[2];
  input_angle2[3] = neutral_angle2[3];

  PlanBase::instance()->bodyItemRobot()->body()->link(0)->p() = Vector3(0, 0, 0);

  PlanBase::instance()->calcForwardKinematics();
  PlanBase::instance()->flush();

  //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
  */
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::onRemoveButtonClicked() {
  vertices->clear();
  SceneView::instance()->removeEntity(group);
  //os << "Remove!" << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::viewPosition() {
  os << "body(" << PlanBase::instance()->bodyItemRobot()->body()->link(0)->p()[0] << ", " << PlanBase::instance()->bodyItemRobot()->body()->link(0)->p()[1] << ", "
      << PlanBase::instance()->bodyItemRobot()->body()->link(0)->p()[2] << ")" << endl;

  os << "R:(" << PlanBase::instance()->palm()->R() << endl;

  os << "armP1:(" << PlanBase::instance()->palm()->p()[0] << ", " << PlanBase::instance()->palm()->p()[1] << ", " << PlanBase::instance()->palm()->p()[2] << ")" << endl;

  os << "armP1:(" << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->p()[0] << ", " << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->p()[1] << ", " << fixed
      << setprecision(2) << setw(6) << PlanBase::instance()->palm()->p()[2] << ")" << endl;
  /*
   os << "armR1:(" << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(0, 0) << ", " << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(0, 1) << ", "
   << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(0, 2) << ")" << endl;
   os << "armR2:(" << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(1, 0) << ", " << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(1, 1) << ", "
   << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(1, 2) << ")" << endl;
   os << "armR3:(" << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(2, 0) << ", " << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(2, 1) << ", "
   << fixed << setprecision(2) << setw(6) << PlanBase::instance()->palm()->R()(2, 2) << ")" << endl;
   */
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::setPose() {
  os << "setPose" << endl;
  /*for(int i = 0; i < 7; i++) {
  	//os << "q" << i+2 << "= " << PlanBase::instance()->arm()->arm_path->joint(i+2)->q() * (180 / PI) << endl;
  	os << "input angle[" << i << "]= " << input_angle[i] * (180 / PI) << endl;
  }*/
  os << "setPose" << endl;

  sp_control_srv.request.unit = 2; //Unit:2 ArmR
  sp_control_srv.request.cmd = 15; //Cmd:15 moveJointAbs

  sp_control_srv.request.arg.resize(10);
  
   sp_control_srv.request.arg[0] = input_angle[0] * (180 / PI);
   if(input_angle[1] * (180 / PI) < -10) {
   sp_control_srv.request.arg[1] = input_angle[1] * (180 / PI);
   }else{
   sp_control_srv.request.arg[1] = -10;
   }
   //sp_control_srv.request.arg[1] = -30;
   sp_control_srv.request.arg[2] = input_angle[2] * (180 / PI);
   sp_control_srv.request.arg[3] = input_angle[3] * (180 / PI);
   sp_control_srv.request.arg[7] = deg_velocity;
   /*
  sp_control_srv.request.arg[0] = PlanBase::instance()->arm()->arm_path->joint(0)->q() * (180 / PI);
  if(PlanBase::instance()->arm()->arm_path->joint(1)->q() * (180 / PI) < -10) {
    sp_control_srv.request.arg[1] = PlanBase::instance()->arm()->arm_path->joint(1)->q() * (180 / PI);
  } else {
    sp_control_srv.request.arg[1] = -10;
  }
  //sp_control_srv.request.arg[1] = -30;
  sp_control_srv.request.arg[2] = PlanBase::instance()->arm()->arm_path->joint(2)->q() * (180 / PI);
  sp_control_srv.request.arg[3] = PlanBase::instance()->arm()->arm_path->joint(3)->q() * (180 / PI);

  sp_control_srv.request.arg[4] = PlanBase::instance()->arm()->arm_path->joint(4)->q() * (180 / PI);
  sp_control_srv.request.arg[5] = PlanBase::instance()->arm()->arm_path->joint(5)->q() * (180 / PI);
  sp_control_srv.request.arg[6] = PlanBase::instance()->arm()->arm_path->joint(6)->q() * (180 / PI);
  sp_control_srv.request.arg[7] = deg_velocity;
  sp_control_srv.request.arg[8] = 0;
  sp_control_srv.request.arg[9] = 0;
*/
  for(int i = 0; i < 7; i++) {
  	//os << "q" << i+2 << "= " << PlanBase::instance()->arm()->arm_path->joint(i+2)->q() * (180 / PI) << endl;
  	os << "input angle[" << i << "]= " << input_angle[i] * (180 / PI) << endl;
  }
  /*
   for(int i = 0; i < 4; i++) {
   sp_control_srv.request.arg[i] = PlanBase::instance()->arm()->arm_path->joint(i)->q() * (180 / PI);
   if(PlanBase::instance()->arm()->arm_path->joint(i)->q() * (180 / PI) > theta_max[i]) {
   sp_control_srv.request.arg[i] = theta_max[i];
   os << "upper limit " << i+1 << endl;
   os << "q(" << i+1 << ") = " << theta_max[i] << endl;
   } else if(PlanBase::instance()->arm()->arm_path->joint(i)->q() * (180 / PI) < theta_min[i]) {
   sp_control_srv.request.arg[i] = theta_min[i];
   os << "lower limit " << i+1 << endl;
   os << "q(" << i+1 << ") = " << theta_min[i] << endl;
   }
   }
   */
  if(control_client.call(sp_control_srv)) {
    //os << "[TmsAction] Succeed to call service" << endl;
    int8_t result = sp_control_srv.response.result;
    os << "[TmsAction] action arm result = " << (int) result << endl;
  } else {
    os << "[TmsAction] Failed to call service sp5_control" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::setPoseSimulation() {
  os << "setPose" << endl;
    for(int i = 0; i < 7; i++) {
  	//os << "q" << i+2 << "= " << PlanBase::instance()->arm()->arm_path->joint(i+2)->q() * (180 / PI) << endl;
  	os << "input angle[" << i << "]= " << input_angle[i] * (180 / PI) << endl;
  }
  
   PlanBase::instance()->arm()->arm_path->joint(0)->q() = input_angle[0];
   if(input_angle[1] * (180 / PI) < -10) {
   PlanBase::instance()->arm()->arm_path->joint(1)->q() = input_angle[1];
   }else{
   PlanBase::instance()->arm()->arm_path->joint(1)->q() = -10 * (PI / 180);
   }
   //sp_control_srv.request.arg[1] = -30;
   PlanBase::instance()->arm()->arm_path->joint(2)->q() = input_angle[2];
   PlanBase::instance()->arm()->arm_path->joint(3)->q() = input_angle[3];

  for(int i = 0; i < 7; i++) {
  	//os << "q" << i+2 << "= " << PlanBase::instance()->arm()->arm_path->joint(i+2)->q() * (180 / PI) << endl;
  	os << "input angle[" << i << "]= " << input_angle[i] * (180 / PI) << endl;
  }
  if(control_client.call(sp_control_srv)) {
    //os << "[TmsAction] Succeed to call service" << endl;
    int8_t result = sp_control_srv.response.result;
    os << "[TmsAction] action arm result = " << (int) result << endl;
  } else {
    os << "[TmsAction] Failed to call service sp5_control" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::setPoseButtonClicked() {
  os << "setPoseButton" << endl;
  sp_control_srv.request.unit = 2; //Unit:2 ArmR
  sp_control_srv.request.cmd = 15; //Cmd:15 moveJointAbs

  sp_control_srv.request.arg.resize(10);

  sp_control_srv.request.arg[0] = PlanBase::instance()->arm()->arm_path->joint(0)->q() * (180 / PI);
  if(PlanBase::instance()->arm()->arm_path->joint(1)->q() * (180 / PI) < -10) {
    sp_control_srv.request.arg[1] = PlanBase::instance()->arm()->arm_path->joint(1)->q() * (180 / PI);
  } else {
    sp_control_srv.request.arg[1] = -10;
  }
  //sp_control_srv.request.arg[1] = -30;
  sp_control_srv.request.arg[2] = PlanBase::instance()->arm()->arm_path->joint(2)->q() * (180 / PI);
  sp_control_srv.request.arg[3] = PlanBase::instance()->arm()->arm_path->joint(3)->q() * (180 / PI);

  sp_control_srv.request.arg[4] = PlanBase::instance()->arm()->arm_path->joint(4)->q() * (180 / PI);
  sp_control_srv.request.arg[5] = PlanBase::instance()->arm()->arm_path->joint(5)->q() * (180 / PI);
  sp_control_srv.request.arg[6] = PlanBase::instance()->arm()->arm_path->joint(6)->q() * (180 / PI);
  sp_control_srv.request.arg[7] = deg_velocity;
  sp_control_srv.request.arg[8] = 0;
  sp_control_srv.request.arg[9] = 0;

  for(int i = 0; i < 7; i++) {
  	os << "q" << i << "= " << PlanBase::instance()->arm()->arm_path->joint(i)->q() * (180 / PI) << endl;
  }
  if(control_client.call(sp_control_srv)) {
    //os << "[TmsAction] Succeed to call service" << endl;
    int8_t result = sp_control_srv.response.result;
    //os << "[TmsAction] action arm result = " << (int) result << endl;
  } else {
    os << "[TmsAction] Failed to call service sp5_control" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::abort() {
  os << "abort" << endl;
  sp_control_srv.request.unit = 2; //Unit:2 ArmR
  sp_control_srv.request.cmd = 5; //Cmd:5 abort

  if(control_client.call(sp_control_srv)) {
    //os << "[TmsAction] Succeed to call abprt service" << endl;
    int8_t result = sp_control_srv.response.result;
    os << "[TmsAction] abort result = " << (int) result << endl;
  } else {
    os << "[TmsAction] Failed to call abort" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::getPose() {
  sp_control_srv.request.unit = 2; //Unit:2 ArmR
  sp_control_srv.request.cmd = 8;  //Cmd :8 getPose
  sp_control_srv.request.arg[0] = 0;  //arg[0]:flameID
  cout << "1.2" << endl;
  if(control_client.call(sp_control_srv)) {
    //os << "[TmsAction] Succeed to call service" << endl;
    int8_t result = sp_control_srv.response.result;
    //os << "[TmsAction] action arm result = " << (int) result << endl;
    //モデルに同期させる
    for(int i = 0; i < 7; i++) {
      //PlanBase::instance()->arm()->arm_path->joint(i+2)->q() = sp_control_srv.response.val[i] * (PI / 180);
      PlanBase::instance()->arm()->arm_path->joint(i)->q() = sp_control_srv.response.val[i] * (PI / 180);
      //os << "arg[" << i << "] = " << sp_control_srv.response.val[i] << endl;
    }
    PlanBase::instance()->calcForwardKinematics();
    PlanBase::instance()->flush();

  } else {
    os << "[TmsAction] Failed to call service sp5_control" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::traceTrajectory() {

  for(int i = 0; i < max1; i++) {
  	  	////////////////GripperR///////////////////////////////////////////////////
    sp_control_srv.request.unit = 4; //Unit:4 GripperR
    sp_control_srv.request.cmd = 15; //Cmd:15 moveAbs
    sp_control_srv.request.arg.resize(3);
    sp_control_srv.request.arg[0] = trj1[i][9];
    sp_control_srv.request.arg[1] = 10;
    sp_control_srv.request.arg[2] = 10;

    if(control_client.call(sp_control_srv)) {
      //os << "[TmsAction] Succeed to call service" << endl;
      //int8_t result = sp_control_srv.response.result;
      //os << "[TmsAction] action gripper result = " << (int) result << endl;
      os << "[stage" << i << ":Gripper] (" << trj1[i][9] << ")" << endl;
    } else {
      os << "[TmsAction] Failed to call service sp5_control Lumba" << endl;
    }
    if(i > 0){ if(trj1[i][9] != trj1[i-1][9]){ sleep(2); } }
    ////////////////ArmR///////////////////////////////////////////////////
    sp_control_srv.request.unit = 2; //Unit:2 ArmR
    sp_control_srv.request.cmd = 15; //Cmd:15 moveJointAbs
    sp_control_srv.request.arg.resize(10);
    for(int j = 2; j < 9; j++) {
      sp_control_srv.request.arg[j-2] = trj1[i][j];
    }
    sp_control_srv.request.arg[7] = deg_velocity;
    if(control_client.call(sp_control_srv)) {
      //os << "[TmsAction] Succeed to call service" << endl;
      //int8_t result = sp_control_srv.response.result;
      //os << "[TmsAction] action arm result = " << (int) result << endl;
      os << "[stage" << i << ":ArmR]" << endl;
    } else {
      os << "[TmsAction] Failed to call service sp5_control ArmR" << endl;
    }
    ////////////////State check///////////////////////////////////////////////////
    while(1){
      sp_control_srv.request.unit = 2; //Unit:2 ArmR
      sp_control_srv.request.cmd = 7;  //Cmd :8 getPose
      sp_control_srv.request.arg[0] = 0;  //arg[0]:flameID
      if(control_client.call(sp_control_srv)) {
        //os << "[TmsAction] Succeed to call service" << endl;
        int8_t result = sp_control_srv.response.result;
        //os << "[TmsAction] action arm result = " << (int) result << endl;
        if(result == 18){
          break;
        }
      }
    } //loop end
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::traceTrajectorySimulation() {
  for(int i = 0; i < max1; i++) {
    ////////////////ArmR///////////////////////////////////////////////////
    for(int j = 0; j < 7; j++) {
      PlanBase::instance()->arm()->arm_path->joint(j)->q() = trj1[i][j+2] * (PI / 180);
      os << "joint(" << j << ") = " << trj1[i][j+2] << endl;
    }
    PlanBase::instance()->calcForwardKinematics();
    PlanBase::instance()->flush();
    sleep(1);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::evade() {
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(col_result2.size() > 30) {
    cout << "4" << endl;
    os << "count : " << wait_time << endl;
    if(neutral || (wait_time == 0)) { //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
      cout << "4.1" << endl;
      onRemoveButtonClicked();

      col_r = 100;
      tmp = 0;
      cout << "4.2" << endl;
      for(int i = 0; i < col_result2.size(); i++) {
        cout << "4.21" << endl;
        tmp = (col_result2[i][0] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[0]) * (col_result2[i][0] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[0])
            + (col_result2[i][1] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[1]) * (col_result2[i][1] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[1])
            + (col_result2[i][2] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[2]) * (col_result2[i][2] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[2]);
        if(col_r > tmp) {
          cout << "4.22" << endl;
          col_r = tmp;
          ave_x = col_result2[i][0];
          ave_y = col_result2[i][1];
          ave_z = col_result2[i][2];
        }
        cout << "4.23" << endl;
      }
      tmp = sqrt(tmp);

      //干渉点座標表示
      os << setprecision(2) << "r : " << r << "m, " <</* "time : " << setprecision(3) << (double) (end - start) / CLOCKS_PER_SEC << " sec." << */" Colliding Point : " << "(" << fixed << setprecision(2)
          << setw(5) << ave_x << ", " << fixed << setprecision(2) << setw(5) << ave_y << ", " << fixed << setprecision(2) << setw(5) << ave_z << ", " << fixed << setprecision(2) << setw(5) << col_r
          << ")" << endl;
	    cout << "4.3" << endl;


      viewCollidingPoint(ave_x, ave_y, ave_z);

      cout << "4.4" << endl;
      moveArm4(ave_x, ave_y, ave_z);

      cout << "4.5" << endl;

      if(!neutral){ abort(); } //if moving, abort
      //setPoseSimulation();
      setPose();
      cout << "4.6" << endl;
      wait_time = number_of_wait;
      neutral = 0;
      //cout << "wait_time is " << wait_time << endl;
    } else { //count tochu //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
      wait_time--;
      cout << "wait_time is " << wait_time << endl;
    } //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  } else {
    cout << "5" << endl;
    os << setprecision(2) << "r : " << r << "m, " << /*",  time : " << setprecision(3) << (double) (end - start) / CLOCKS_PER_SEC << " sec." <<*/ " Colliding Point : Nothing" << endl;
    //moveArm3(0, 0, 0);
    onRemoveButtonClicked();
    if( !neutral) {
      cout << "5.1" << endl;
      wait_time = number_of_wait; //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
      //onResetButtonClicked();
      for(int i=0; i<4; i++){
        input_angle[i] = neutral_angle[i]; //ResetButton
      }
      cout << "5.2" << endl;
      abort(); //moving check surubeki?
      //setPoseSimulation();
      setPose();
      neutral = 1;
      cout << "5.3" << endl;
      //cout << "N" << endl;
    } else {
      cout << "neutral" << endl;
    }
    //if(!neutral) returnNeutral();
    //neutral = 1;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::evade2() { //main
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(!moving_stage_flag[moving_stage]){
   	os << "Moving mode:stage " << moving_stage << endl;
  	grip();
  }else if(col_result2.size() > 30) {
  	if(moving_mode){ //moving mode and receive
  	  os << "Moving mode and receive" << endl;
  	  os << "moving_mode = " << moving_mode << endl;
      onResetButtonClicked();
      setPose();
      if((moving_stage == 4) || (moving_stage == 14)){
      	os << "moving_stage is " << moving_stage << ", and close gripper" << endl;
        sp_control_srv.request.unit = 4; //Unit:4 GripperR
        sp_control_srv.request.cmd = 15; //Cmd:15 moveAbs
        sp_control_srv.request.arg.resize(3);
        sp_control_srv.request.arg[0] = -10;
        sp_control_srv.request.arg[1] = 10;
        sp_control_srv.request.arg[2] = 10;
        if(control_client.call(sp_control_srv)) {
          os << "[TmsAction] Succeed to call service" << endl;
          int8_t result = sp_control_srv.response.result;
          os << "[TmsAction] action arm result = " << (int) result << endl;
        } else {
          os << "[TmsAction] Failed to call service sp5_control Lumba" << endl;
        }
        sleep(1);
      }
  	  moving_mode = 0;
  	  os << "change to evade mode" << endl;
  	  os << "moving_mode = " << moving_mode << endl;
  	}else{ //evade mode and reveive
  	  os << "evade mode and receive" << endl;
      cout << "4" << endl;
      if(neutral2 || (wait_time == 0)) {
        onRemoveButtonClicked();
        neutral2 = 0;
        col_r = 100;
        tmp = 0;
        for(int i = 0; i < col_result2.size(); i++) {
          tmp = (col_result2[i][0] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[0]) * (col_result2[i][0] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[0])
              + (col_result2[i][1] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[1]) * (col_result2[i][1] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[1])
              + (col_result2[i][2] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[2]) * (col_result2[i][2] - PlanBase::instance()->bodyItemRobot()->body()->link(10)->p()[2]);
          if(col_r > tmp) {
            col_r = tmp;
            ave_x = col_result2[i][0];
            ave_y = col_result2[i][1];
            ave_z = col_result2[i][2];
          }
        }
        tmp = sqrt(tmp);
        //干渉点座標表示
        os << setprecision(2) << "r : " << r << "m, " <</* "time : " << setprecision(3) << (double) (end - start) / CLOCKS_PER_SEC << " sec." << */" Colliding Point : " << "(" << fixed << setprecision(2)
            << setw(5) << ave_x << ", " << fixed << setprecision(2) << setw(5) << ave_y << ", " << fixed << setprecision(2) << setw(5) << ave_z << ", " << fixed << setprecision(2) << setw(5) << col_r
            << ")" << endl;
        //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
        viewCollidingPoint(ave_x, ave_y, ave_z);
        moveArm3(ave_x, ave_y, ave_z);
        moveElbow(ave_x, ave_y, ave_z);
        setPose();
        wait_time = number_of_wait;
      } else { //count tochu 
        wait_time--;
        cout << "wait_time is " << wait_time << endl;
      }
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    } 
  } else if(!moving_mode){ //evade mode and no reveive
  	os << "evade mode and no receive" << endl;
    cout << "5" << endl;
    os << setprecision(2) << "r : " << r << "m, " 
       << /*",  time : " << setprecision(3) << (double) (end - start) / CLOCKS_PER_SEC << " sec." <<*/ " Colliding Point : Nothing" << endl;
    onRemoveButtonClicked();
    
    if(!neutral2) {
      wait_time = number_of_wait;
      onResetButtonClicked();
      //abort(); //moving check surubeki?
      setPose();
      neutral2 = 1;
      //cout << "N" << endl;
    } else { //evade mode, neutral and no receive
      cout << "Back to moving mode" << endl;
      moving_mode = 1;
      moving_stage = moving_stage_next[moving_stage];
    }
    //if(!neutral2) returnNeutral2();
    //neutral2 = 1;
  } else { //moving mode and no receive
    os << "Moving mode and no receive" << endl;
    os << "Moving mode:stage " << moving_stage << endl;
    grip();  
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::grip() {
  ////////////////GripperR///////////////////////////////////////////////////
  sp_control_srv.request.unit = 4; //Unit:4 GripperR
  sp_control_srv.request.cmd = 15; //Cmd:15 moveAbs
  sp_control_srv.request.arg.resize(3);
  sp_control_srv.request.arg[0] = trj1[moving_stage][9];
  sp_control_srv.request.arg[1] = 10;
  sp_control_srv.request.arg[2] = 10;
  if(control_client.call(sp_control_srv)) {
    os << "[TmsAction] Succeed to call service" << endl;
    int8_t result = sp_control_srv.response.result;
    os << "[TmsAction] action arm result = " << (int) result << endl;
  } else {
    os << "[TmsAction] Failed to call service sp5_control GripperR" << endl;
  }
  if(moving_stage > 0){ if(trj1[moving_stage][9] != trj1[moving_stage-1][9]){ sleep(1); } }
  ////////////////ArmR///////////////////////////////////////////////////
  sp_control_srv.request.unit = 2; //Unit:2 ArmR
  sp_control_srv.request.cmd = 15; //Cmd:15 moveJointAbs
  sp_control_srv.request.arg.resize(10);
  for(int j = 2; j < 9; j++) {
    sp_control_srv.request.arg[j-2] = trj1[moving_stage][j];
  }
  sp_control_srv.request.arg[7] = deg_velocity;
  if(control_client.call(sp_control_srv)) {
    os << "[TmsAction] Succeed to call service" << endl;
    int8_t result = sp_control_srv.response.result;
    os << "[TmsAction] action arm result = " << (int) result << endl;
  } else {
    os << "[TmsAction] Failed to call service sp5_control ArmR" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::forEvadeAndGrip() { //main
  cout << "1" << endl;
  if(targetBodyItems.size() != 1) {
    os << "Please select one bodyitem" << endl;
    return;
  }
  cout << "1.1" << endl;
  //実機の関節角を取る
  //getPose();
  cout << "1.3" << endl;
  //Velodyneからの受信点群
  vector < Vector3 > p;

  for(int i = 0; i < pointCloudData.points.size(); i++) {
    //if(pointCloudData.points[i].x < 1.3) {
    if( (pointCloudData.points[i].y - 0.055 < 1.2) && (pointCloudData.points[i].y - 0.055 > 0) && (pointCloudData.points[i].z + 1.43 > 0.9) ) { //grep and evade
      p.push_back(Vector3(pointCloudData.points[i].x, pointCloudData.points[i].y - 0.055, pointCloudData.points[i].z + 1.43));
    }
  }
  cout << "2" << endl;
  os << "." << endl;
  os << "size0 = " << p.size() << endl;
  //onRemoveButtonClicked();
  //viewCollidingPoints (p);
  //sleep(2);

  //モデルの位置更新
  for(int j = 0; j < targetBodyItems[0]->body()->numLinks(); j++) { //パルのリンク3~10右腕
    targetBodyItems[0]->body()->link(j)->coldetModel()->setPosition(targetBodyItems[0]->body()->link(j)->T());
  }
  col_result.clear();
  col_result2.clear();
  //干渉判定、干渉点だけをcol_resultへ詰める
  PlanBase::instance()->isCollidingPointCloud3(p, targetBodyItems[0], col_result, r);
  os << "size1 = " << col_result.size() << endl;

  //干渉点からロボット自身の点を除く
  PlanBase::instance()->removeCollidingPointCloud2(col_result, targetBodyItems[0], col_result2, 0.2);
  os << "size2 = " << col_result2.size() << endl;
  cout << "3" << endl;
  
  abort();
  evade2();

  if(moving_mode){
    moving_stage++;
    if(moving_stage == max1) {
      onSTARTButtonClicked();
      moving_stage = 0;
    }
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void VelodyneBar::rosOn() {
  os << "test button" << endl;
  static ros::Rate loop_rate(10); // 0.1sec

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
//int flag = 0;
void VelodyneBar::receivePointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if(col_start) {
  	cout << "1" << endl;
    getPose();//if real move, it is needed.
    cout << "2" << endl;
    pcl::PointCloud < pcl::PointXYZ > cloud;
    pcl::fromROSMsg( *msg, cloud);

    pointCloudData = cloud;
    //callSynchronously(bind( &VelodyneBar::onRemoveButtonClicked, VelodyneBar::instance()));
    //callSynchronously(bind( &VelodyneBar::onCollisionButtonClicked, VelodyneBar::instance()));
    //onRemoveButtonClicked();
    onCollisionButtonClicked();
    //forEvadeAndGrip();
    cout << "3" << endl;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
