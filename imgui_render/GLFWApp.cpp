#include "GLFWApp.h"
#include <glad/glad.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <implot.h>
#include <examples/imgui_impl_glfw.h>
#include <examples/imgui_impl_opengl3.h>
#include <iostream>
#include <experimental/filesystem>
#include "Environment.h"
#include "BVH.h"
#include "Muscle.h"
#include "GLfunctions.h"
using namespace MASS;
static const double PI = acos(-1);
namespace fs = std::experimental::filesystem;
GLFWApp::GLFWApp(int argc, char** argv)
        : mFocus(true), mSimulating(false), mDrawOBJ(true), mDrawShadow(false),
          mPhysics(true),
          mTrans(0.0, 0.0, 0.0),
          mEye(0.0, 0.0, 1.0),
          mUp(0.0, 1.0, 0.0),
          mZoom(1.0),
          mPersp(45.0),
          mRotate(false),
          mTranslate(false),
          mZooming(false),
          mFramerate(60), 
          mResolution(100)
          {

	width = 1920; height = 1080;
	viewportWidth = 1920;
	imguiWidth = 400;
    mTrackball.setTrackball(Eigen::Vector2d(viewportWidth * 0.5, height * 0.5), viewportWidth * 0.5);
    mTrackball.setQuaternion(Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())));
	mZoom = 0.25;
	mFocus = false;

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    window = glfwCreateWindow(width, height, "render", nullptr, nullptr);
	if (window == NULL) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
	    glfwTerminate();
	    exit(EXIT_FAILURE);
	}
	
    glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
	    std::cerr << "Failed to initialize GLAD" << std::endl;
	    exit(EXIT_FAILURE);
	}
	glViewport(0, 0, width, height);
	
    glfwSetWindowUserPointer(window, this);
	auto framebufferSizeCallback = [](GLFWwindow* window, int width, int height) {
	    GLFWApp* app = static_cast<GLFWApp*>(glfwGetWindowUserPointer(window));
	    app->width = width;
	    app->height = height;
	    glViewport(0, 0, width, height);
	};
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	auto keyCallback = [](GLFWwindow* window, int key, int scancode, int action, int mods) {
	    auto& io = ImGui::GetIO();
	    if (!io.WantCaptureKeyboard) {
            GLFWApp* app = static_cast<GLFWApp*>(glfwGetWindowUserPointer(window));
            app->keyboardPress(key, scancode, action, mods);
        }
	};
	
    glfwSetKeyCallback(window, keyCallback);
	auto cursorPosCallback = [](GLFWwindow* window, double xpos, double ypos) {
        auto& io = ImGui::GetIO();
        if (!io.WantCaptureMouse) {
            GLFWApp* app = static_cast<GLFWApp*>(glfwGetWindowUserPointer(window));
            app->mouseMove(xpos, ypos);
        }
	};
	
    glfwSetCursorPosCallback(window, cursorPosCallback);
	
    auto mouseButtonCallback = [](GLFWwindow* window, int button, int action, int mods) {
        auto& io = ImGui::GetIO();
        if (!io.WantCaptureMouse) {
            GLFWApp* app = static_cast<GLFWApp*>(glfwGetWindowUserPointer(window));
            app->mousePress(button, action, mods);
        }
	};
	glfwSetMouseButtonCallback(window, mouseButtonCallback);
	
    auto scrollCallback = [](GLFWwindow* window, double xoffset, double yoffset) {
	    auto& io = ImGui::GetIO();
	    if (!io.WantCaptureMouse) {
            GLFWApp* app = static_cast<GLFWApp*>(glfwGetWindowUserPointer(window));
            app->mouseScroll(xoffset, yoffset);
	    }
	};
	glfwSetScrollCallback(window, scrollCallback);

	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 150");
	ImPlot::CreateContext();
    SetFocusing();

    mCameraMoving = 0;
    mTrackball.setQuaternion(Eigen::Quaterniond::Identity());
}

GLFWApp::~GLFWApp() {
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

void GLFWApp::setEnv(Environment* env, int argc, char** argv) {
    mEnv = env;
    mEnv->Initialize_from_path(argv[1]);
    mEnv->SetParamState(mEnv->GetNormalV());

    // multiple envs
    auto min_param = mEnv->GetMinV();
    auto max_param = mEnv->GetMaxV();

    for (int id=0; id<10; id++) {
        mEnvs.emplace_back(new MASS::Environment(true));
        mEnvs[id]->Initialize_from_path(argv[1]);
        Eigen::VectorXd param(mEnv->GetNumParamState());
        for(int i=0; i<param.size(); i++)
            param[i] = (double)rand()/RAND_MAX*(max_param[i]-min_param[i])+min_param[i];
        mEnvs[id]->SetParamState(param);

        mEnvs[id]->SetSeed(Eigen::Vector3d((((double)rand()/RAND_MAX)-0.5)/0.5,0,(((double)rand()/RAND_MAX)-0.5)/0.5));
    }

    Reset();

    mFocus = true;
}

void GLFWApp::startLoop() {
    double display_time = 0.0;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, true);
        }
        if(mSimulating)
            update();
        
        while (glfwGetTime() < (display_time + 1.0 / mFramerate))
        { 
        }
        display_time = glfwGetTime();

        draw();
        
        glfwSwapBuffers(window);
    }
}
void GLFWApp::keyboardPress(int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_SPACE: mSimulating = !mSimulating; break;
            case GLFW_KEY_R: Reset(); break;
            case GLFW_KEY_F: mFocus = !mFocus; break;
            case GLFW_KEY_S: update(); break;
            case GLFW_KEY_7: 
            case GLFW_KEY_KP_7: 
            mCameraMoving = 1; break;
            case GLFW_KEY_5: 
            case GLFW_KEY_KP_5: 
            mCameraMoving = 0; mTrackball.setQuaternion(Eigen::Quaterniond::Identity()); break;
            case GLFW_KEY_9: 
            case GLFW_KEY_KP_9:
            mCameraMoving = -1; break;
            case GLFW_KEY_4:
            case GLFW_KEY_KP_4:
            {
                mCameraMoving = 0;
                Eigen::Quaterniond r = Eigen::Quaterniond(Eigen::AngleAxisd(0.01 * M_PI, Eigen::Vector3d::UnitY()))  * mTrackball.getCurrQuat();
                mTrackball.setQuaternion(r);
                break;
            }
            case GLFW_KEY_6:
            case GLFW_KEY_KP_6:
            {
                mCameraMoving = 0;
                Eigen::Quaterniond r = Eigen::Quaterniond(Eigen::AngleAxisd(-0.01 * M_PI, Eigen::Vector3d::UnitY()))  * mTrackball.getCurrQuat();
                mTrackball.setQuaternion(r);
                break;
            }
            case GLFW_KEY_8:
            case GLFW_KEY_KP_8:
            {
                mCameraMoving = 0;
                Eigen::Quaterniond r = Eigen::Quaterniond(Eigen::AngleAxisd(0.01 * M_PI, Eigen::Vector3d::UnitX()))  * mTrackball.getCurrQuat();
                mTrackball.setQuaternion(r);
                break;
            }
            case GLFW_KEY_2:
            case GLFW_KEY_KP_2:
            {
                mCameraMoving = 0;
                Eigen::Quaterniond r = Eigen::Quaterniond(Eigen::AngleAxisd(-0.01 * M_PI, Eigen::Vector3d::UnitX()))  * mTrackball.getCurrQuat();
                mTrackball.setQuaternion(r);
                break;
            }
            default: break;
        }
    }
}

void GLFWApp::mouseMove(double xpos, double ypos) {
    double deltaX = xpos - mMouseX;
    double deltaY = ypos - mMouseY;
    mMouseX = xpos;
    mMouseY = ypos;
    if (mRotate)
    {
        if (deltaX != 0 || deltaY != 0) {
            mTrackball.updateBall(xpos, height - ypos);
        }
    }
    if (mTranslate)
    {
        Eigen::Matrix3d rot;
        rot = mTrackball.getRotationMatrix();
        mTrans += (1 / mZoom) * rot.transpose()
                  * Eigen::Vector3d(deltaX, -deltaY, 0.0);
    }
    if (mZooming)
    {
        mZoom = std::max(0.01, mZoom + deltaY * 0.01);
    }
}

void GLFWApp::mousePress(int button, int action, int mods) {
    mMouseDown = true;
    if (action == GLFW_PRESS) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            mRotate = true;
            mTrackball.startBall(mMouseX, height - mMouseY);
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            mTranslate = true;
        }
    }
    else if (action == GLFW_RELEASE) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            mRotate = false;
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            mTranslate = false;
        }
    }

}

void GLFWApp::mouseScroll(double xoffset, double yoffset) {
    mZoom += yoffset * 0.01;
}

void GLFWApp::draw()
{
    initGL();
    initLights();
    initFog();
    SetFocusing();

    /* Preprocessing */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, width, height);
    gluPerspective(mPersp, width / height, 0.1, 10.0);
    gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    mTrackball.setCenter(Eigen::Vector2d(width*0.5, height*0.5));
	mTrackball.setRadius(std::min(width, height)/2.5);
    mTrackball.applyGLRotation();

    {
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glLineWidth(2.0);
        if (mRotate || mTranslate || mZooming)
        {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            glVertex3f(-0.1f, 0.0f, -0.0f);
            glVertex3f(0.15f, 0.0f, -0.0f);
            glEnd();

            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINES);
            glVertex3f(0.0f, -0.1f, 0.0f);
            glVertex3f(0.0f, 0.15f, 0.0f);
            glEnd();

            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            glVertex3f(0.0f, 0.0f, -0.1f);
            glVertex3f(0.0f, 0.0f, 0.15f);
            glEnd();
        }
    }

    glScalef(mZoom, mZoom, mZoom);
    glTranslatef(mTrans[0] * 0.001, mTrans[1] * 0.001, mTrans[2] * 0.001);


    GLfloat matrix[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    A << matrix[0], matrix[4], matrix[8],
            matrix[1], matrix[5], matrix[9],
            matrix[2], matrix[6], matrix[10];
    b << matrix[12], matrix[13], matrix[14];
    mViewMatrix.linear() = A;
    mViewMatrix.translation() = b;

    auto ground = mEnv->GetGround();
    float y = ground->getBodyNode(0)->getTransform().translation()[1] +
              dynamic_cast<const BoxShape *>(ground->getBodyNode(
                      0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1] * 0.5;
    DrawGround(y);

    for(int i=0; i<mEnvs.size(); i+=2) {
        Eigen::Vector3d p(i*0.2,0,0);
        p+=mEnvs[i]->GetSeed();

        glPushMatrix();
        glTranslatef(p[0],p[1],p[2]);

        if (mPhysics) {
            DrawSkeleton(mEnvs[i]->GetCharacter()->GetSkeleton());
            if(mEnvs[i]->GetUseMuscle()) {
                DrawMuscles(mEnvs[i]->GetCharacter()->GetMuscles());
            }
        }
        glPopMatrix();
    }

    for(int i=1; i<mEnvs.size(); i+=2) {
        Eigen::Vector3d p(0.1+i*0.2,0,5);
        p+=mEnvs[i]->GetSeed();

        glPushMatrix();
        glTranslatef(p[0],p[1],p[2]);
        glRotatef(180.0,0,1,0);

        if (mPhysics) {
            DrawSkeleton(mEnvs[i]->GetCharacter()->GetSkeleton());
            if(mEnvs[i]->GetUseMuscle()) {
                DrawMuscles(mEnvs[i]->GetCharacter()->GetMuscles());
            }
        }
        glPopMatrix();
    }
}

void GLFWApp::initGL() {
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glShadeModel(GL_SMOOTH);
    glPolygonMode(GL_FRONT, GL_FILL);
}

void GLFWApp::initFog() {
	glEnable(GL_FOG);
	GLfloat fogColor[] = {1,1,1,1};
  	glFogfv(GL_FOG_COLOR,fogColor);
  	glFogi(GL_FOG_MODE,GL_LINEAR);
  	glFogf(GL_FOG_START, 0.0);
  	glFogf(GL_FOG_END,8.0);
}

void GLFWApp::initLights() {
    static float ambient[] = {0.2, 0.2, 0.2, 1.0};
    static float diffuse[] = {0.6, 0.6, 0.6, 1.0};
    static float front_mat_shininess[] = {60.0};
    static float front_mat_specular[] = {0.2, 0.2, 0.2, 1.0};
    static float front_mat_diffuse[] = {0.5, 0.28, 0.38, 1.0};
    static float lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
    static float lmodel_twoside[] = {GL_FALSE};
    GLfloat position[] = {1.0, 0.0, 0.0, 0.0};
    GLfloat position1[] = {-1.0, 0.0, 0.0, 0.0};
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, position1);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, front_mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, front_mat_diffuse);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);
}

void GLFWApp::update()
{
    int num = mEnv->GetSimulationHz() / mEnv->GetControlHz();
    num /= (mFramerate / mEnv->GetControlHz());
    
    if(mSimCount % (mFramerate / mEnv->GetControlHz()) == 0) // Set Action Part 
    {
        Eigen::VectorXd action = Eigen::VectorXd::Zero(mEnv->GetNumAction());
        mEnv->GetReward();
        mEnv->SetAction(action);
    }
    
    for(int i = 0; i < num; i++)
        for(auto env: mEnvs)
            env->Step(); 

    mSimCount++;
}

void GLFWApp::Reset()
{
    for(auto env: mEnvs)
	   env->Reset();
    mSimCount = 0;
}

void GLFWApp::SetFocusing()
{
    if(mFocus)
    {
        mTrans = -mEnv->GetWorld()->getSkeleton("Human")->getCOM();
		mTrans[1] = -1.0;
		mTrans *= 1000;
    }

	Eigen::Quaterniond origin_r = mTrackball.getCurrQuat();
	if (mCameraMoving == 1 && Eigen::AngleAxisd(origin_r).angle() < 0.5 * M_PI)
	{
		Eigen::Quaterniond r = Eigen::Quaterniond(Eigen::AngleAxisd(mCameraMoving * 0.01 * M_PI, Eigen::Vector3d::UnitY())) * origin_r;
		mTrackball.setQuaternion(r);
	}
	else if (mCameraMoving == -1 && Eigen::AngleAxisd(origin_r).axis()[1] > 0)
	{
		Eigen::Quaterniond r = Eigen::Quaterniond(Eigen::AngleAxisd(mCameraMoving * 0.01 * M_PI, Eigen::Vector3d::UnitY())) * origin_r;
		mTrackball.setQuaternion(r);
	}
}

void GLFWApp::DrawEntity(const Entity* entity)
{
	if (!entity)
		return;
	const auto& bn = dynamic_cast<const BodyNode*>(entity);
	if(bn)
	{
		DrawBodyNode(bn);
		return;
	}
	const auto& sf = dynamic_cast<const ShapeFrame*>(entity);
	if(sf)
	{
		DrawShapeFrame(sf);
		return;
	}
}

void GLFWApp::DrawBodyNode(const BodyNode* bn)
{	
	if(!bn)
		return;

	glPushMatrix();
	Eigen::Affine3d tmp = bn->getRelativeTransform();
	glMultMatrixd(tmp.data());

	auto sns = bn->getShapeNodesWith<VisualAspect>();
	for(const auto& sn : sns)
		DrawShapeFrame(sn);

	for(const auto& et : bn->getChildEntities())
		DrawEntity(et);

	glPopMatrix();
}

void GLFWApp::DrawSkeleton(const SkeletonPtr& skel)
{
	DrawBodyNode(skel->getRootBodyNode());
}

void GLFWApp::DrawShapeFrame(const ShapeFrame* sf)
{
	if(!sf)
		return;

	const auto& va = sf->getVisualAspect();

	if(!va || va->isHidden())
		return;

	glPushMatrix();
	Eigen::Affine3d tmp = sf->getRelativeTransform();
	glMultMatrixd(tmp.data());

	DrawShape(sf->getShape().get(),va->getRGBA());

	glPopMatrix();
}

void GLFWApp::DrawShape(const Shape* shape, const Eigen::Vector4d& color)
{
	if(!shape)
		return;

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
    glColor4d(color[0], color[1], color[2], color[3]);
	if(mDrawOBJ == false)
	{
        glColor4dv(color.data());
		if (shape->is<SphereShape>())
		{
			const auto* sphere = dynamic_cast<const SphereShape*>(shape);
			GUI::DrawSphere(sphere->getRadius());
		}
		else if (shape->is<BoxShape>())
		{
			const auto* box = dynamic_cast<const BoxShape*>(shape);
			GUI::DrawCube(box->getSize());
		}
		else if (shape->is<CapsuleShape>())
		{
			const auto* capsule = dynamic_cast<const CapsuleShape*>(shape);
			GUI::DrawCapsule(capsule->getRadius(), capsule->getHeight());
		}	
        else if (shape->is<CylinderShape>())
		{
			const auto* cylinder = dynamic_cast<const CylinderShape*>(shape);
			GUI::DrawCylinder(cylinder->getRadius(), cylinder->getHeight());
		}
	}
	else
	{
		if (shape->is<MeshShape>())
		{
			const auto& mesh = dynamic_cast<const MeshShape*>(shape);
            float y = mEnv->GetGround()->getBodyNode(0)->getTransform().translation()[1] + dynamic_cast<const BoxShape*>(mEnv->GetGround()->getBodyNode(0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
            mShapeRenderer.renderMesh(mesh, false, y, Eigen::Vector4d(0.6,0.6,0.6,1.0));
            // DrawShadow(mesh->getScale(), mesh->getMesh(), y);
        }
	}
	
	glDisable(GL_COLOR_MATERIAL);
}
void GLFWApp::DrawMuscles(const std::vector<Muscle*>& muscles)
{
	int count = 0;
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	
	for (auto muscle : muscles)
	{
        muscle->Update();
        
        double a = muscle->activation;
        bool s = muscle->selected;
        
        Eigen::Vector4d color;
        if(s)
            color = Eigen::Vector4d(1.0,0.2,0.2,a);
        else 
            color = Eigen::Vector4d(0.4+(2.0*a),0.4,0.4,1.0);
        glColor4dv(color.data());
        mShapeRenderer.renderMuscle(muscle);
    }
	glEnable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
}

void GLFWApp::DrawGround(double y)
{
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glDisable(GL_LIGHTING);
	double width = 0.005;
	int count = 0;
	glBegin(GL_QUADS);
	for(double x = -100.0;x<100.01;x+=1.0)
	{
		for(double z = -100.0;z<100.01;z+=1.0)
		{
			if(count%2==0)
				glColor3f(216.0/255.0,211.0/255.0,204.0/255.0);			
			else
				glColor3f(216.0/255.0-0.1,211.0/255.0-0.1,204.0/255.0-0.1);
			count++;
			glVertex3f(x,y,z);
			glVertex3f(x+1.0,y,z);
			glVertex3f(x+1.0,y,z+1.0);
			glVertex3f(x,y,z+1.0);
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);

}

void
GLFWApp::
DrawAiMesh(const struct aiScene *sc, const struct aiNode* nd,const Eigen::Affine3d& M,double y)
{
	unsigned int i;
    unsigned int n = 0, t;
    Eigen::Vector3d v;
    Eigen::Vector3d dir(0.4,0,-0.4);
    glColor3f(0.3,0.3,0.3);
    
    // update transform

    // draw all meshes assigned to this node
    for (; n < nd->mNumMeshes; ++n) {
        const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

        for (t = 0; t < mesh->mNumFaces; ++t) {
            const struct aiFace* face = &mesh->mFaces[t];
            GLenum face_mode;

            switch(face->mNumIndices) {
                case 1: face_mode = GL_POINTS; break;
                case 2: face_mode = GL_LINES; break;
                case 3: face_mode = GL_TRIANGLES; break;
                default: face_mode = GL_POLYGON; break;
            }
            glBegin(face_mode);
        	for (i = 0; i < face->mNumIndices; i++)
        	{
        		int index = face->mIndices[i];

        		v[0] = (&mesh->mVertices[index].x)[0];
        		v[1] = (&mesh->mVertices[index].x)[1];
        		v[2] = (&mesh->mVertices[index].x)[2];
        		v = M*v;
        		double h = v[1]-y;
        		
        		v += h*dir;
        		
        		v[1] = y+0.001;
        		glVertex3f(v[0],v[1],v[2]);
        	}
            glEnd();
        }

    }
    // draw all children
    for (n = 0; n < nd->mNumChildren; ++n) {
        DrawAiMesh(sc, nd->mChildren[n],M,y);
    }

}


void GLFWApp::DrawShadow(const Eigen::Vector3d& scale, const aiScene* mesh,double y) 
{
	glDisable(GL_LIGHTING);
	glPushMatrix();
	glScalef(scale[0],scale[1],scale[2]);
	GLfloat matrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	Eigen::Matrix3d A;
	Eigen::Vector3d b;
	A<<matrix[0],matrix[4],matrix[8],
	matrix[1],matrix[5],matrix[9],
	matrix[2],matrix[6],matrix[10];
	b<<matrix[12],matrix[13],matrix[14];

	Eigen::Affine3d M;
	M.linear() = A;
	M.translation() = b;
	M = (mViewMatrix.inverse()) * M;

	glPushMatrix();
	glLoadIdentity();
	glMultMatrixd(mViewMatrix.data());
	DrawAiMesh(mesh,mesh->mRootNode,M,y);
	glPopMatrix();
	glPopMatrix();
	glEnable(GL_LIGHTING);
}