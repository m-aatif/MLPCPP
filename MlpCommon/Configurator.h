#pragma once

#include <boost/json.hpp>
#include <iostream>
#include <iomanip>
#include "Geometry.h"

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <fstream>

using namespace geon;

namespace json = boost::json;

namespace afarcloud
{
	class Algorithm
	{
	public:
		Algorithm()
		{
			id = -1;
			name = "";
		}

		int32_t id;
		std::string	name;
	};
	typedef std::map<std::string, int32_t> algorithmMap;

	class TaskAlgorithm
	{
	public:
		TaskAlgorithm()
		{
			name = "";
			algorithmName = "";
			log = false;
			params_name.clear();
			params_value.clear();
			parameters.clear();
		}

		std::string	name;
		std::string	algorithmName;
		bool log;
		std::vector<std::string> params_name;
		std::vector<double> params_value;
		std::map<std::string, double> parameters;
	};
	typedef std::map<std::string, TaskAlgorithm> taskAlgorithmMap;

	class HostParams
	{
	public:
		HostParams()
		{
			app = "";
			name = "";
			port = -1;
		}

		std::string	app;
		std::string	name;
		int32_t	port;
	};
	typedef std::map<std::string, HostParams> hostParamsMap;	//	app->HostParams

	class Configurator
	{
	public:
		Configurator(std::string filename);

		static void checkConfigurationFolders();
		static std::string getLogFolder();

		bool	loadAlgorithmConfiguration(algorithmMap &algoMap, taskAlgorithmMap &taskAlgoMap);
		bool	loadHostsConfiguration(hostParamsMap &hostMap);
		bool	isMissionLogInEnabled();
		bool	isMissionLogOutEnabled();

	private:
		json::value m_jv;
		json::value	parseJsonConfigFile(char const* filename);

		static std::string getSpecialFolder(int csidl);
		static std::string getConfigFolder();

		std::string	dequoteString(std::string orig_string);

		void	configAlgorithms(algorithmMap &algoMap, taskAlgorithmMap &taskAlgoMap, json::value const& jv, std::string key_name);
		void	addAlgorithm(Algorithm &a, json::value const& jv, std::string key_name);
		void	addTaskAlgorithm(TaskAlgorithm &ta, json::value const& jv, std::string key_name);

		void	configHosts(hostParamsMap &hostMap, json::value const& jv, std::string key_name);
		void	addHost(HostParams &h, json::value const& jv, std::string key_name);

		void	findFlag(std::string name, json::value const& jv, std::string key_name, bool &value);
	};
}
