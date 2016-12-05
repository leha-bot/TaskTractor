/* @file cmdline.cpp Contains command line parse logic.
*
*/
#include <string>

/** @brief Contains an interface for command line parameter handler.
*
*/
class ProgramParameterInterface {
public:
	virtual const std::string &name() const = 0;
	virtual const std::string &value() const = 0;

	/** @note You should use a hook in derived classes if you need a param
	* of non-string type.
	*/
	virtual void setValue(const std::string &val) = 0;
	virtual ~ProgramParameterInterface() {}
};

class ConsoleParameterInterface : public ProgramParameterInterface {
public:
	virtual const std::string &description() const = 0;
	virtual void commandLineParamHandler(const std::string &param) = 0;
};

class ConsoleVoidParameter : public ConsoleParameterInterface {
	std::string m_name;
	std::string m_description;
public:
	ConsoleVoidParameter() {}

	virtual const std::string & description() const override
	{
		return m_description;
	}


	virtual void commandLineParamHandler(const std::string &param) = 0;

	virtual const std::string & name() const override
	{
		return m_name;
	}


	virtual const std::string & value() const
	{
		static const std::string s = "";
		return s;
	}

	virtual void setValue(const std::string &val) override
	{
	}

};

class ProgramVoidParameter : public ProgramParameterInterface {
	std::string m_name;
public:
	ProgramVoidParameter(const std::string &name)
	{
		m_name = name;
	}
};
