#ifndef MSS_GLFWAPP_H
#define MSS_GLFWAPP_H
#include "dart/gui/Trackball.hpp"
#include "ShapeRenderer.h"
struct GLFWwindow;
namespace MASS {
class Environment;
class Muscle;
class GLFWApp
{
public:
	GLFWApp(int argc, char** argv);
	~GLFWApp();
    void startLoop();

    void setEnv(Environment* env, int argc, char** argv);
private:
    void update();
    void keyboardPress(int key, int scancode, int action, int mods);
    void mouseMove(double xpos, double ypos);
    void mousePress(int button, int action, int mods);
    void mouseScroll(double xoffset, double yoffset);
	void initGL();
	void initFog();
	void initLights();
	void draw();
	void SetFocusing();
	void Reset();

    GLFWwindow* window;
	Environment* mEnv;
	std::vector<Environment*> mEnvs;

	bool mFocus;
	bool mSimulating;
	bool mDrawOBJ;
	bool mDrawShadow;
	bool mPhysics;

	Eigen::Affine3d mViewMatrix;
	dart::gui::Trackball mTrackball;
	Eigen::Vector3d mTrans;
	Eigen::Vector3d mEye;
	Eigen::Vector3d mUp;
	
	float mZoom;
	float mPersp;
	float mMouseX, mMouseY;
	bool mMouseDown, mMouseDrag, mCapture = false;
	bool mRotate = false, mTranslate = false, mZooming = false;
	double width, height;
	double viewportWidth, imguiWidth;
	ShapeRenderer mShapeRenderer;

	int mResolution;
	int mSimCount;

	// Drawing Tool 
	void DrawEntity(const dart::dynamics::Entity* entity);
	void DrawBodyNode(const dart::dynamics::BodyNode* bn);
	void DrawSkeleton(const dart::dynamics::SkeletonPtr& skel);
	void DrawShapeFrame(const dart::dynamics::ShapeFrame* shapeFrame);
	void DrawShape(const dart::dynamics::Shape* shape,const Eigen::Vector4d& color);
	void DrawMuscles(const std::vector<Muscle*>& muscles);
	void DrawGround(double y);

	void DrawPhase(double phase, double global_phase);
	void DrawShadow(const Eigen::Vector3d& scale, const aiScene* mesh, double y);
	void DrawAiMesh(const struct aiScene *sc, const struct aiNode* nd,const Eigen::Affine3d& M,double y);

	int mCameraMoving;

	int mFramerate;
};
};

#endif
