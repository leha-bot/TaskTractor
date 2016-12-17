/** @file Ui.cpp - contains UI for Simulation. *Stub*.
 *
 */
#ifdef _MSC_VER
#pragma comment(lib, "opengl32.lib")
#include "SimulationInterface.h"
#include "GLFW/glfw3.h"
#include <memory>

class Playable {
public:
	virtual void play() = 0;
	virtual void stop() = 0;
};

struct GLRect {
	GLdouble left, right, top, bottom;
};

struct GLPoint {
	GLdouble x, y;
}; 

class GLCameraView2D {
	GLRect rt;
public:
	GLCameraView2D(GLdouble left, GLdouble right,
		GLdouble top, GLdouble bottom)
		:rt({ left, right, top, bottom }) {}

	void moveBy(GLdouble x, GLdouble y)
	{
		rt.left += x;
		rt.right += x;
		rt.top += y;
		rt.bottom += y;
	}

	bool pointInView(GLdouble x, GLdouble y)
	{
		return rt.left >= x && rt.right <= x &&
			rt.top >= y && rt.bottom <= y;
	}

	GLdouble width()
	{
		return rt.right - rt.left;
	}

	GLdouble height()
	{
		return rt.bottom - rt.top;
	}
};

/** @note Incomplete. It works on Windows and should works on Linux, but not 
 *        tested and therefore disabled on non-MSVC-compilers.
 *
 */
class OpenGlSimulationUiGlfw : public SimulationUi {
	GLFWwindow *wnd;
	GLCameraView2D camera;
	GLPoint curPosition, lastPosition;
	std::shared_ptr<SimulationLightBuilder> builder;
	SimPtr simulationController;
	enum key {
		key_accelerate,
		key_deccelerate,
		key_toggle,
		key_pause,
		key_turnleft,
		key_turnright,
		key_quit,
		keys_count
	};
	bool keys_pressed[keys_count];
	bool paused;
	

public:
	OpenGlSimulationUiGlfw(std::shared_ptr<SimulationLightBuilder> &builder)
		: wnd(nullptr), camera(0, 0, 400, 400),
		curPosition(), lastPosition(),
		builder(builder),
		keys_pressed()
	{
		glfwInit();

	}
	
	static key keyFromGlfwKeycode(int key)
	{
		switch (key)
		{
		case GLFW_KEY_W:
			return key_accelerate;
		case GLFW_KEY_S:
			return key_deccelerate;
		case GLFW_KEY_A:
			return key_turnleft;
		case GLFW_KEY_D:
			return key_turnright;
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE:
			return key_quit;
		case GLFW_KEY_E:
			return key_toggle;
		case GLFW_KEY_SPACE:
			return key_pause;
		}
	}

	/* @brief Handles keys press.
 	 *  @param[in] window The window that received the event.
	 *  @param[in] key The [keyboard key](@ref keys) that was pressed or released.
	 *  @param[in] scancode The system-specific scancode of the key.
	 *  @param[in] action `GLFW_PRESS`, `GLFW_RELEASE` or `GLFW_REPEAT`.
	 *  @param[in] mods Bit field describing which [modifier keys](@ref mods) were
	 */
	static void keyPress(GLFWwindow *window, int key, int scancode, int action, int mods)
	{
		OpenGlSimulationUiGlfw *sim = (OpenGlSimulationUiGlfw*) glfwGetWindowUserPointer(window);
		assert(sim != nullptr);
		if (action == GLFW_RELEASE) {

		}
	}

	void reshape(int w, int h)
	{
		glViewport(0, 0, w, h);
		glOrtho(0, 400.0, 400.0, 0, 1, -1);
	}

	void draw()
	{

	}

	virtual void loop() override
	{
		SimObserverPtr ptr = this->shared_from_this();
		builder->addObserver(ptr);
		this->simulationController = builder->build();

		// throw std::logic_error("The method or operation is not implemented.");
		wnd = glfwCreateWindow(400, 400, "Simulation", nullptr, nullptr);
		GLFWkeyfun();
		glfwSetKeyCallback(this->wnd, keyPress);
		glfwSetWindowUserPointer(wnd, this);
		while (!glfwWindowShouldClose(wnd)) {
			glClear(GL_COLOR_BUFFER_BIT);
			draw();
			glfwSwapBuffers(wnd);
			glfwPollEvents();
		}

	}


	virtual void notify(const BicyclePtr &sim, const char *tag, const Time &simTime, const Time &delta) override
	{
		BicycleInterface::UnitPoint pt;
		sim->getFrontWheelCoords(pt);
		this->curPosition.x = pt.x;
		this->curPosition.y = pt.y;

		// throw std::logic_error("The method or operation is not implemented.");
	}

};

std::shared_ptr<SimulationUi> getGlfwUI(std::shared_ptr<SimulationLightBuilder> &ptr)
{
	return std::make_shared<OpenGlSimulationUiGlfw>(ptr);
}

#endif