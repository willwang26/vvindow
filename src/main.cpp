
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osg/StateSet>
#include <osg/TextureCubeMap>
#include <osg/TexGen>
#include <osg/Point>
#include <osg/TexEnvCombine>
#include <osgUtil/ReflectionMapGenerator>
#include <osgUtil/HighlightMapGenerator>
#include <osgUtil/HalfWayMapGenerator>

#include <iostream>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

unsigned int g_targetNumVerticesPerGeometry = 10000;
bool g_mirror = true;

osg::Node* pointCloud2Geode(const pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	osg::Geode* geode = new osg::Geode;

	osg::Geometry* geometry = new osg::Geometry;

	osg::Vec3Array* vertices = new osg::Vec3Array;
	osg::Vec3Array* normals = new osg::Vec3Array;
	osg::Vec4ubArray* colours = new osg::Vec4ubArray;

	osg::Vec3 pos;
	osg::Vec3 normal(0.0,0.0,1.0);
	int r=255,g=255,b=255,a=255;

	for(unsigned int i=0; i<cloud.points.size(); ++i)
	{
		pos.set(g_mirror?-cloud.points.at(i).x:cloud.points.at(i).x, -cloud.points.at(i).y, -cloud.points.at(i).z);
		// unpack rgb into r/g/b
		uint32_t rgb = *reinterpret_cast<const int*>(&cloud.points.at(i).rgb);
		r = (rgb >> 16) & 0x0000ff;
		g = (rgb >> 8)  & 0x0000ff;
		b = (rgb)       & 0x0000ff;

		if (vertices->size()>=g_targetNumVerticesPerGeometry)
		{
			// finishing setting up the current geometry and add it to the geode.
			geometry->setUseDisplayList(false);
			geometry->setUseVertexBufferObjects(false);
			geometry->setVertexArray(vertices);
			geometry->setNormalArray(normals);
			geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
			geometry->setColorArray(colours);
			geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
			geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS,0,vertices->size()));

			geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f), osg::StateAttribute::ON);

			geode->addDrawable(geometry);

			// allocate a new geometry
			geometry = new osg::Geometry;

			vertices = new osg::Vec3Array;
			normals = new osg::Vec3Array;
			colours = new osg::Vec4ubArray;

			vertices->reserve(g_targetNumVerticesPerGeometry);
			normals->reserve(g_targetNumVerticesPerGeometry);
			colours->reserve(g_targetNumVerticesPerGeometry);

		}

		vertices->push_back(pos);
		normals->push_back(normal);
		colours->push_back(osg::Vec4ub(r,g,b,a));
	}


	geometry->setUseDisplayList(false);
	geometry->setUseVertexBufferObjects(false);
	geometry->setVertexArray(vertices);
	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->setColorArray(colours);
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS,0,vertices->size()));

	geode->addDrawable(geometry);

	return geode;

}

osg::Node * createCoordinate(double size = 0.05) // 5 cm
{
	osg::Geode * geode = new osg::Geode();
	osg::Geometry* linesGeom = new osg::Geometry();

	//coordinate red x, green y, blue z, 5 cm
	osg::Vec3Array* vertices = new osg::Vec3Array(6);
	(*vertices)[0].set(0, 0, 0);
	(*vertices)[1].set(0.05, 0, 0);
	(*vertices)[2].set(0, 0, 0);
	(*vertices)[3].set(0, 0.05, 0);
	(*vertices)[4].set(0, 0, 0);
	(*vertices)[5].set(0, 0, 0.05);

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
	colors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
	colors->push_back(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));

	linesGeom->setVertexArray(vertices);
	linesGeom->setColorArray(colors);
	linesGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
	linesGeom->setNormalArray(normals);
	linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,14));

	geode->addDrawable(linesGeom);
	return geode;
}

osg::Node * createGrid(double screenWidth, double screenHeight, osg::Vec4 color = osg::Vec4(1.0f,1.0f,1.0f,1.0f))
{
	int wStepRatio = 6;
	int hStepRatio = 4;
	double w = screenWidth/double(wStepRatio);
	double h = screenHeight/double(hStepRatio);

	osg::Geode * geode = new osg::Geode();

	// create LINES
	osg::Geometry* linesGeom = new osg::Geometry();
	osg::Vec3Array* vertices = new osg::Vec3Array();

	for(double i=-screenWidth/2.0; i<=screenWidth/2.0; i+=w)
	{
		vertices->push_back(osg::Vec3(i, -screenHeight/2, 0));
		vertices->push_back(osg::Vec3(i, screenHeight/2, 0));
	}

	for(double i=-screenHeight/2.0; i<=screenHeight/2.0; i+=h)
	{
		vertices->push_back(osg::Vec3(-screenWidth/2, i, 0));
		vertices->push_back(osg::Vec3(screenWidth/2, i, 0));
	}

	linesGeom->setVertexArray(vertices);

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(color);
	linesGeom->setColorArray(colors);
	linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
	linesGeom->setNormalArray(normals);
	linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);


	linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,vertices->size()));


	// add the points geometry to the geode.
	geode->addDrawable(linesGeom);

	return geode;
}

osg::Node * createCubes(double screenWidth, double screenHeight)
{
	double w = screenWidth/6.0;
	double h = screenHeight/4.0;
	double d = screenHeight/4.0;

	osg::Geode * geode = new osg::Geode();
	osg::ShapeDrawable * b0 = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(-w*3 + w/2.0, h+h/2.0, -d/2.0), w, h, d));
	osg::ShapeDrawable * b1 = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(-w*2 + w/2.0, h+h/2.0, -d-d/2.0), w, h, d));
	osg::ShapeDrawable * b2 = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(-w*1 + w/2.0, h+h/2.0, -d/2.0), w, h, d));
	osg::ShapeDrawable * b3 = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(w*0 + w/2.0, h+h/2.0, -d/2.0), w, h, d));
	osg::ShapeDrawable * b4 = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(w*1 + w/2.0, h+h/2.0, -d-d/2.0), w, h, d));
	osg::ShapeDrawable * b5 = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(w*2 + w/2.0, h+h/2.0, -d/2.0), w, h, d));
	osg::ShapeDrawable * b6 = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(-w/2, -h/2.0, d/2.0), w, h, d));
	b0->setColor(osg::Vec4d(1.0, 0.0, 0.0, 1.0));
	b1->setColor(osg::Vec4d(1.0, 1.0, 0.0, 1.0));
	b2->setColor(osg::Vec4d(1.0, 0.0, 0.0, 1.0));
	b3->setColor(osg::Vec4d(1.0, 1.0, 0.0, 1.0));
	b4->setColor(osg::Vec4d(1.0, 0.0, 0.0, 1.0));
	b5->setColor(osg::Vec4d(1.0, 1.0, 0.0, 1.0));
	b6->setColor(osg::Vec4d(1.0, 1.0, 1.0, 1.0));
	geode->addDrawable(b0);
	geode->addDrawable(b1);
	geode->addDrawable(b2);
	geode->addDrawable(b3);
	geode->addDrawable(b4);
	geode->addDrawable(b5);
	geode->addDrawable(b6);

	return geode;
}


osg::ref_ptr<osg::Node> cloudGeode;
osg::ref_ptr<osg::PositionAttitudeTransform> patCloud;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*msg, *cloud);

	if(cloudGeode.get() != 0)
	{
		patCloud->removeChild(cloudGeode);
	}
	//initialize the cloud
	cloudGeode = pointCloud2Geode(*cloud);
	patCloud->addChild(cloudGeode);
}

osg::Vec3d origin;
double scale = 1.0f;

class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:

        KeyboardEventHandler(osg::PositionAttitudeTransform * pat):
            _pat(pat) {}

        virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
        {
        	bool handled = false;
            switch(ea.getKey())
            {
                case(osgGA::GUIEventAdapter::KEY_Down):
                {
                	if(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
                	{
                		//rotate down
                		osg::Quat at = _pat->getAttitude();
                		_pat->setAttitude(at*osg::Quat(osg::PI/32, osg::Vec3d(1,0,0)));
                	}
                	else if(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)
                	{
                		//move down
                		osg::Vec3d pos = _pat->getPosition();
						pos.y()-=0.01;
						_pat->setPosition(pos);
                	}
                	else
                	{
                		//move toward the user
                		osg::Vec3d pos = _pat->getPosition();
						pos.z()+=0.01;
						_pat->setPosition(pos);
                	}

                	handled = true;
                	break;
                }
                case(osgGA::GUIEventAdapter::KEY_Up):
                {
                	if(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
					{
						//rotate up
						osg::Quat at = _pat->getAttitude();
						_pat->setAttitude(at*osg::Quat(-osg::PI/32, osg::Vec3d(1,0,0)));
					}
					else if(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)
					{
						//move up
						osg::Vec3d pos = _pat->getPosition();
						pos.y()+=0.01;
						_pat->setPosition(pos);
					}
					else
					{
						//move away from user
						osg::Vec3d pos = _pat->getPosition();
						pos.z()-=0.01;
						_pat->setPosition(pos);
					}
					handled = true;
					break;
                }
                case(osgGA::GUIEventAdapter::KEY_Right):
				{
                	if(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
					{
						//rotate right
						osg::Quat at = _pat->getAttitude();
						_pat->setAttitude(at*osg::Quat(osg::PI/32, osg::Vec3d(0,1,0)));
					}
					else
					{
						//move right
						osg::Vec3d pos = _pat->getPosition();
						pos.x()+=0.01;
						_pat->setPosition(pos);
					}
					handled = true;
					break;
				}
				case(osgGA::GUIEventAdapter::KEY_Left):
				{
					if(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
					{
						//rotate left
						osg::Quat at = _pat->getAttitude();
						_pat->setAttitude(at*osg::Quat(-osg::PI/32, osg::Vec3d(0,1,0)));
					}
					else
					{
						//move left
						osg::Vec3d pos = _pat->getPosition();
						pos.x()-=0.01;
						_pat->setPosition(pos);
					}
					handled = true;
					break;
				}
                case(osgGA::GUIEventAdapter::KEY_Plus):
			   {
					osg::Vec3d scale = _pat->getScale();
					scale.x() *= 1.10;
					scale.y() *= 1.10;
					scale.z() *= 1.10;
					_pat->setScale(scale);
					handled = true;
					break;
			   }
                case(osgGA::GUIEventAdapter::KEY_Minus):
			   {
                	osg::Vec3d scale = _pat->getScale();
					scale.x() *= 0.9;
					scale.y() *= 0.9;
					scale.z() *= 0.9;
					_pat->setScale(scale);
					handled = true;
					break;
			   }

                case(osgGA::GUIEventAdapter::KEY_H):
			   {
                	_pat->setPosition(origin);
					_pat->setScale(osg::Vec3d(scale,scale,scale));
				   return true;
			   }

                default:
                    break;
            }
            if(handled)
            {
				ROS_INFO("Position=%f,%f,%f, Scale=%f",
					_pat->getPosition().x(),
					_pat->getPosition().y(),
					_pat->getPosition().z(),
					_pat->getScale().x());
            }
            return handled;
        }

        osg::ref_ptr<osg::PositionAttitudeTransform> _pat;
};

int main( int argc, char **argv )
{
	ros::init(argc, argv, "vvindow");
	ros::NodeHandle np("~");

	bool cloud = false;
	bool stereo = true;
	bool blackBg = true;
	std::string modelPath = "";
	double screenWidth = 0.52; // 24" DELL U2410
	double screenHeigth = 0.325; // 24" DELL U2410
	double x=0.0, y=0.0 ,z=0.0;

	np.param("cloud", cloud, cloud);
	np.param("stereo", stereo, stereo);
	np.param("blackBg", blackBg, blackBg);
	np.param("modelPath", modelPath, modelPath);
	np.param("screenWidth", screenWidth, screenWidth);
	np.param("screenHeight", screenHeigth, screenHeigth);
	np.param("originX", x, x);
	np.param("originY", y, y);
	np.param("originZ", z, z);
	np.param("scale", scale, scale);
	origin.set(x,y,z);

	ROS_INFO("cloud=%s", cloud?"true":"false");
	ROS_INFO("stereo=%s", stereo?"true":"false");
	ROS_INFO("blackBg=%s", blackBg?"true":"false");
	ROS_INFO("modelPath=%s", modelPath.c_str());
	ROS_INFO("screenWidth=%f", screenWidth);
	ROS_INFO("screenHeight=%f", screenHeigth);
	ROS_INFO("origin=%f,%f,%f", origin.x(), origin.y(), origin.z());
	ROS_INFO("scale=%f", scale);

	ros::NodeHandle n;
	ros::Subscriber sub;

	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
    //viewer.setCameraManipulator( new osgGA::TrackballManipulator() );

	// load the scene.
    osg::ref_ptr<osg::PositionAttitudeTransform> root = new osg::PositionAttitudeTransform();
    root->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    root->setPosition(origin);
    root->setScale(osg::Vec3d(scale,scale,scale));
    //create_specular_highlights(root->getOrCreateStateSet());

    viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::ThreadingHandler);
	viewer.addEventHandler(new KeyboardEventHandler(root.get()));

    //model
	osg::ref_ptr<osg::Node> loadedModel = osgDB::readRefNodeFile(modelPath);
	if (loadedModel)
	{
		root->addChild(loadedModel.get());
		osg::Vec3 posInWorld = loadedModel->getBound().center() * osg::computeLocalToWorld(loadedModel->getParentalNodePaths()[0]);
		ROS_INFO("Object position=%f,%f,%f radius=%f",posInWorld.x(), posInWorld.y(),posInWorld.z(), loadedModel->getBound().radius());
		root->setAttitude(osg::Quat(-osg::PI_2, osg::Vec3d(1,0,0)));
	}
	else if(cloud)
	{
		//cloud
		patCloud = new osg::PositionAttitudeTransform();
		root->addChild(patCloud);
		sub = n.subscribe("cloud", 1, cloudCallback);
	}
	else
	{
		//grid
		osg::Vec4 color(6.0f,6.0f,6.0f,1.0f);
		osg::ref_ptr<osg::Node> cubes = createCubes(screenWidth, screenHeigth);
		osg::ref_ptr<osg::Node> gridBottom = createGrid(screenWidth, screenHeigth, color);
		osg::ref_ptr<osg::Node> gridTop = createGrid(screenWidth, screenHeigth, color);
		osg::ref_ptr<osg::Node> gridLeft = createGrid(screenWidth, screenHeigth, color);
		osg::ref_ptr<osg::Node> gridRight = createGrid(screenWidth, screenHeigth, color);
		osg::ref_ptr<osg::Node> gridBack = createGrid(screenWidth, screenHeigth, color);
		osg::ref_ptr<osg::Node> coordinate = createCoordinate();
		osg::ref_ptr<osg::PositionAttitudeTransform> patGridBottom = new osg::PositionAttitudeTransform();
		osg::ref_ptr<osg::PositionAttitudeTransform> patGridTop = new osg::PositionAttitudeTransform();
		osg::ref_ptr<osg::PositionAttitudeTransform> patGridLeft = new osg::PositionAttitudeTransform();
		osg::ref_ptr<osg::PositionAttitudeTransform> patGridRight = new osg::PositionAttitudeTransform();
		osg::ref_ptr<osg::PositionAttitudeTransform> patGridBack = new osg::PositionAttitudeTransform();
		patGridBottom->setPosition(osg::Vec3d(0.0, -screenHeigth/2, -screenHeigth/2));
		patGridTop->setPosition(osg::Vec3d(0.0, screenHeigth/2, -screenHeigth/2));
		patGridLeft->setPosition(osg::Vec3d(-screenWidth/2, 0.0, -screenWidth/2));
		patGridRight->setPosition(osg::Vec3d(screenWidth/2, 0.0, -screenWidth/2));
		patGridBack->setPosition(osg::Vec3d(0.0, 0.0, -screenWidth));
		patGridBottom->setAttitude(osg::Quat(-osg::PI_2, osg::Vec3d(1.0, 0.0, 0.0)));
		patGridTop->setAttitude(osg::Quat(osg::PI_2, osg::Vec3d(1.0, 0.0, 0.0)));
		patGridLeft->setAttitude(osg::Quat(osg::PI_2, osg::Vec3d(0.0, 1.0, 0.0)));
		patGridRight->setAttitude(osg::Quat(-osg::PI_2, osg::Vec3d(0.0, 1.0, 0.0)));
		patGridBottom->addChild(gridBottom.get());
		patGridTop->addChild(gridTop.get());
		patGridLeft->addChild(gridLeft.get());
		patGridRight->addChild(gridRight.get());
		patGridBack->addChild(gridBack.get());

		std::cout << argv[0] <<": No data loaded... using box instead" << std::endl;
		root->addChild(cubes.get());
		root->addChild(coordinate.get());

		root->addChild(patGridBottom.get());
		root->addChild(patGridTop.get());
		root->addChild(patGridLeft.get());
		root->addChild(patGridRight.get());
		root->addChild(patGridBack.get());
	}

	viewer.setSceneData(root.get());

	// Get screen resolution
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
		return -1;
	}
	unsigned int width, height;
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

	// setup stereo cameras
	osg::ref_ptr<osg::Camera> cameraLeft = 0;
	osg::ref_ptr<osg::Camera> cameraRight = 0;
	if(stereo)
	{
		// left window + left slave camera
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = 0;
		traits->y = 0;
		traits->width = width;
		traits->height = height;
		traits->windowDecoration = false;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		osg::ref_ptr<osg::Viewport> vp=new osg::Viewport(0,0, traits->width, traits->height);
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

		//camera left
		cameraLeft = new osg::Camera;
		cameraLeft->setGraphicsContext(gc.get());
		cameraLeft->setViewport(vp.get());
		cameraLeft->setDrawBuffer(GL_BACK);
		cameraLeft->setReadBuffer(GL_BACK);
		cameraLeft->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		cameraLeft->setColorMask(true, false, false, true);
		cameraLeft->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		//cameraLeft->setClearColor(osg::Vec4d());
		viewer.addSlave(cameraLeft.get(), osg::Matrixd(), osg::Matrixd());

		//camera right
		cameraRight = new osg::Camera;
		cameraRight->setGraphicsContext(gc.get());
		cameraRight->setViewport(vp.get());
		cameraRight->setDrawBuffer(GL_BACK);
		cameraRight->setReadBuffer(GL_BACK);
		cameraRight->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		cameraRight->setColorMask(false, true, true, true);
		cameraRight->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		//cameraRight->setClearColor(osg::Vec4d());
		viewer.addSlave(cameraRight.get(), osg::Matrixd(), osg::Matrixd());
	}
    
    viewer.realize();

    osg::Camera * camera = 0;
	osg::Vec3d vu(0.000000,1.000000,0.00000); // view up direction
	if(!stereo)
	{
		camera = viewer.getCamera();
		osg::Vec3d vp(0,0,1.0); // view position
		osg::Vec3d vd = vp + osg::Vec3d(0.0,0.0,-1.0); // view direction

		double fovy, aspectRatio, zNear, zFar;
		camera->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
		ROS_INFO("perspective: fovy=%f, aspectRatio=%f, zNear=%f, zFar=%f", fovy, aspectRatio, zNear, zFar);

		double left, right, top, bottom, near, far;
		camera->getProjectionMatrixAsFrustum(left, right, top, bottom, near, far);
		ROS_INFO("frustrum: left=%f, right=%f, top=%f, bottom=%f, near=%f, far=%f", left, right, top, bottom, near, far);

		double dist = vp.z();
		double fovx1 = atan((-screenWidth/2.0f - vp.x()) / dist);
		double fovx2 = atan((screenWidth/2.0f - vp.x()) / dist);
		left = near*tan(fovx1);
		right = near*tan(fovx2);
		double fovy1 = atan((screenHeigth/2.0f - vp.y()) / dist);
		double fovy2 = atan((-screenHeigth/2.0f - vp.y()) / dist);
		top = near*tan(fovy1);
		bottom = near*tan(fovy2);
		camera->setProjectionMatrixAsFrustum(left, right, bottom, top, near, far);

		camera->setViewMatrixAsLookAt(vp, vd, vu);
		ROS_INFO("view: vp=(%f,%f,%f), vd=(%f,%f,%f), vu=(%f,%f,%f)",
				vp.x(), vp.y(), vp.z(), vd.x(), vd.y(), vd.z(), vu.x(), vu.y(), vu.z());

		if(blackBg)
		{
			camera->setClearColor(osg::Vec4d());
		}
	}

	double left, right, top, bottom, near=0.01, far=10000.0;
	tf::TransformListener tfListener;
	while(!viewer.done() && !ros::isShuttingDown())
    {
		ros::spinOnce();

		//update head position
		try {
			tf::StampedTransform transform;
			bool tfSet = false;
			if(tfListener.frameExists("/head_1"))
			{
				tfListener.lookupTransform("/base_link", "/head_1", ros::Time(0.0), transform);
				tfSet = true;
			}
			else if(tfListener.frameExists("/head_2"))
			{
				tfListener.lookupTransform("/base_link", "/head_2", ros::Time(0.0), transform);
				tfSet = true;
			}
			if(tfSet)
			{
				tf::Vector3 pos = transform.getOrigin();

				// up : 0.55 43 43 0.55
				// pencher a droite : 40 57 27 66
				// 70 33 60 23
				double yaw,pitch,roll;
				tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw,pitch,roll);
				double headAngle = pitch;
				//ROS_INFO("y,p,r=%f,%f,%f", yaw, pitch, roll);

				//misc
				//double DTOR = 0.0174532925;
				double dist = pos.x();

				if(!stereo)
				{
					//single camera (using only fovy, assuming user stays in front)
					double fovx1 = atan((-screenWidth/2.0f - pos.y()) / dist);
					double fovx2 = atan((screenWidth/2.0f - pos.y()) / dist);
					left = near*tan(fovx1);
					right = near*tan(fovx2);

					double fovy1 = atan((screenHeigth/2.0f - pos.z()) / dist);
					double fovy2 = atan((-screenHeigth/2.0f - pos.z()) / dist);
					top = near*tan(fovy1);
					bottom = near*tan(fovy2);

					//ROS_INFO("Head detected = %f,%f,%f fovx=%f,%f fovy=%f,%f l/r=%f,%f t/b=%f,%f", pos.x(), pos.y(), pos.z(), fovx1/DTOR, fovx2/DTOR, fovy1/DTOR, fovy2/DTOR, left, right, top, bottom);

					//printf("frustrum: left=%f, right=%f, bottom=%f, top=%f, near=%f, far=%f\n", left, right, bottom, top, near, far);
					camera->setProjectionMatrixAsFrustum(left, right, bottom, top, near, far);

					double s = 1.0;
					osg::Vec3d newVp = osg::Vec3d(pos.y()*s, pos.z()*s, pos.x()*s); // view position

					camera->setViewMatrixAsLookAt(newVp, newVp+osg::Vec3d(0,0,-1), vu);
					//ROS_INFO("view: vp=(%f,%f,%f), vd=(%f,%f,%f), vu=(%f,%f,%f)",
					//		newVp.x(), newVp.y(), newVp.z(), vd.x(), vd.y(), vd.z(), vu.x(), vu.y(), vu.z());
				}
				else
				{
					double eyesDistance = 0.05;// cm
					double eyeDx = eyesDistance/2.0 * cos(headAngle); //right eye
					double eyeDy = -eyesDistance/2.0 * sin(headAngle); //right eye

					//camera left
					{
						double fovx1 = atan((-screenWidth/2.0f - (pos.y()-eyeDx)) / dist);
						double fovx2 = atan((screenWidth/2.0f - (pos.y()-eyeDx)) / dist);
						left = near*tan(fovx1);
						right = near*tan(fovx2);

						double fovy1 = atan((screenHeigth/2.0f - (pos.z()-eyeDy)) / dist);
						double fovy2 = atan((-screenHeigth/2.0f - (pos.z()-eyeDy)) / dist);
						top = near*tan(fovy1);
						bottom = near*tan(fovy2);

						//ROS_INFO("Head detected = %f,%f,%f fovx=%f,%f fovy=%f,%f l/r=%f,%f t/b=%f,%f", pos.x(), pos.y(), pos.z(), fovx1/DTOR, fovx2/DTOR, fovy1/DTOR, fovy2/DTOR, left, right, top, bottom);

						//ROS_INFO("frustrumL: left=%f, right=%f, bottom=%f, top=%f, near=%f, far=%f\n", left, right, bottom, top, near, far);
						cameraLeft->setProjectionMatrixAsFrustum(left, right, bottom, top, near, far);

						osg::Vec3d newVp = osg::Vec3d(pos.y()-eyeDx, pos.z()-eyeDy, pos.x()); // view position

						cameraLeft->setViewMatrixAsLookAt(newVp, newVp+osg::Vec3d(0,0,-1), vu);
						//ROS_INFO("viewL: vp=(%f,%f,%f), vd=(%f,%f,%f), vu=(%f,%f,%f)",
						//		newVp.x(), newVp.y(), newVp.z(), vd.x(), vd.y(), vd.z(), vu.x(), vu.y(), vu.z());
					}

					//camera right
					{
						double fovx1 = atan((-screenWidth/2.0f - (pos.y()+eyeDx)) / dist);
						double fovx2 = atan((screenWidth/2.0f - (pos.y()+eyeDx)) / dist);
						left = near*tan(fovx1);
						right = near*tan(fovx2);

						double fovy1 = atan((screenHeigth/2.0f - (pos.z()+eyeDy)) / dist);
						double fovy2 = atan((-screenHeigth/2.0f - (pos.z()+eyeDy)) / dist);
						top = near*tan(fovy1);
						bottom = near*tan(fovy2);

						//ROS_INFO("Head detected = %f,%f,%f fovx=%f,%f fovy=%f,%f l/r=%f,%f t/b=%f,%f", pos.x(), pos.y(), pos.z(), fovx1/DTOR, fovx2/DTOR, fovy1/DTOR, fovy2/DTOR, left, right, top, bottom);

						//ROS_INFO("frustrumR: left=%f, right=%f, bottom=%f, top=%f, near=%f, far=%f\n", left, right, bottom, top, near, far);
						cameraRight->setProjectionMatrixAsFrustum(left, right, bottom, top, near, far);

						osg::Vec3d newVp = osg::Vec3d(pos.y()+eyeDx, pos.z()+eyeDy, pos.x()); // view position

						cameraRight->setViewMatrixAsLookAt(newVp, newVp+osg::Vec3d(0,0,-1), vu);
						//ROS_INFO("viewR: vp=(%f,%f,%f), vd=(%f,%f,%f), vu=(%f,%f,%f)",
						//		newVp.x(), newVp.y(), newVp.z(), vd.x(), vd.y(), vd.z(), vu.x(), vu.y(), vu.z());
					}
				}

			}
		}
		catch (tf::ExtrapolationException & e) {ROS_ERROR("cannot lookup transform...");}
		catch(tf::LookupException & e) {ROS_ERROR("cannot lookup transform...");}
		catch(tf::ConnectivityException & e) {ROS_ERROR("cannot lookup transform...");}

        viewer.frame();
    }

    return 0;
}
