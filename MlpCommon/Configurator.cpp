#include <shlobj_core.h>
#include <boost/filesystem.hpp>
#include "Configurator.h"

namespace fs = boost::filesystem;

namespace afarcloud
{
	Configurator::Configurator(std::string filename)
	{
		std::string fullPath = getConfigFolder();
		fullPath += "\\";
		fullPath += filename;

		m_jv = parseJsonConfigFile(fullPath.c_str());
	}

	std::string Configurator::getSpecialFolder(int csidl)
	{
		std::string path = "";
		TCHAR szPath[MAX_PATH];

		if (S_OK == SHGetFolderPath(NULL,
			csidl,
			NULL,
			0,
			szPath))
		{
			std::wstring wPath(szPath);
			path = std::string(wPath.begin(), wPath.end());
		}
		return path;
	}

	std::string Configurator::getConfigFolder()
	{
		std::string baseUserDocPath = getSpecialFolder(CSIDL_PERSONAL);
		return baseUserDocPath + "\\MLP\\Cfg";
	}

	std::string Configurator::getLogFolder()
	{
		std::string baseUserDocPath = getSpecialFolder(CSIDL_PERSONAL);
		return baseUserDocPath + "\\MLP\\Log";
	}

	void Configurator::checkConfigurationFolders()
	{
		std::string baseUserDocPath = getSpecialFolder(CSIDL_PERSONAL);

		std::string logPath = baseUserDocPath + "\\MLP\\Log";
		fs::create_directories(logPath);
		std::string cfgPath = baseUserDocPath + "\\MLP\\Cfg";
		fs::create_directories(cfgPath);
	}

	json::value	Configurator::parseJsonConfigFile(char const* filename)
	{
		json::stream_parser p;
		json::error_code ec;
		std::fstream fx;
		fx.open(filename, std::ios::in);
		std::string line;
		while (fx)
		{
			getline(fx, line);
			p.write(line, ec);
		}
		if (ec)
			return nullptr;
		p.finish(ec);
		if (ec)
			return nullptr;
		return p.release();
	}

	std::string	Configurator::dequoteString(std::string orig_string)
	{
		return orig_string.substr(1, orig_string.length() - 2);
	}

	bool	Configurator::loadAlgorithmConfiguration(algorithmMap &algoMap, taskAlgorithmMap &taskAlgoMap)
	{
		algoMap.clear();
		taskAlgoMap.clear();

		try
		{
			configAlgorithms(algoMap, taskAlgoMap, m_jv, "");
			return true;
		}
		catch (std::exception const& e)
		{
			std::cerr <<
				"Caught exception: "
				<< e.what() << std::endl;
			return false;
		}
	}

	bool Configurator::loadHostsConfiguration(hostParamsMap &hostMap)
	{
		hostMap.clear();

		try
		{
			configHosts(hostMap, m_jv, "");
			return true;
		}
		catch (std::exception const& e)
		{
			std::cerr <<
				"Caught exception: "
				<< e.what() << std::endl;
			return false;
		}
	}

	void Configurator::addAlgorithm(Algorithm &a, json::value const& jv, std::string key_name)
	{
		std::string s;
		switch (jv.kind())
		{
		case json::kind::object:
		{
			auto const& obj = jv.get_object();
			if (!obj.empty())
			{
				auto it = obj.begin();
				for (;;)
				{
					s = dequoteString(json::serialize(it->key()));
					addAlgorithm(a, it->value(), s);
					if (++it == obj.end())
						break;
				}
			}
			break;
		}

		case json::kind::int64:
			if (key_name.compare("id") == 0)
				a.id = jv.get_int64();
			break;

		case json::kind::double_:
			break;

		case json::kind::string:
		{
			s = dequoteString(json::serialize(jv.get_string()));
			if (key_name.compare("name") == 0)
				a.name = s;
			break;
		}

		}
	}

	void	Configurator::addTaskAlgorithm(TaskAlgorithm &ta, json::value const& jv, std::string key_name)
	{
		std::string s;
		double d;
		switch (jv.kind())
		{
		case json::kind::object:
		{
			auto const& obj = jv.get_object();
			if (!obj.empty())
			{
				auto it = obj.begin();
				for (;;)
				{
					s = dequoteString(json::serialize(it->key()));
					addTaskAlgorithm(ta, it->value(), s);
					/*******************************************/
					if (ta.params_value.size() == ta.params_name.size())
					{
						for (size_t p = 0; p < ta.params_name.size(); p++)
							ta.parameters[ta.params_name[p]] = ta.params_value[p];
					}
					/*******************************************/
					if (++it == obj.end())
						break;
				}
			}
			break;
		}

		case json::kind::int64:
			break;

		case json::kind::double_:
			break;

		case json::kind::bool_:
			if (key_name.compare("logTask") == 0)
				ta.log = jv.get_bool();
			break;

		case json::kind::string:
		{
			s = dequoteString(json::serialize(jv.get_string()));
			if (key_name.compare("name") == 0)
				ta.name = s;
			if (key_name.compare("algorithm") == 0)
				ta.algorithmName = s;
			break;
		}

		case json::kind::array:
		{
			auto const& arr = jv.get_array();
			if (!arr.empty())
			{
				auto it = arr.begin();
				for (;;)
				{
					if (it->kind() == json::kind::double_)
					{
						d = it->as_double();
						ta.params_value.push_back(d);
					}
					else if (it->kind() == json::kind::string)
					{
						s = dequoteString(json::serialize(it->get_string()));
						ta.params_name.push_back(s);
					}

					if (++it == arr.end())
						break;
				}
			}
			break;
		}

		}
	}

	void	Configurator::configAlgorithms(algorithmMap &algoMap, taskAlgorithmMap &taskAlgoMap, json::value const& jv, std::string key_name)
	{
		std::string s;
		switch (jv.kind())
		{
		case json::kind::object:
		{
			auto const& obj = jv.get_object();
			if (!obj.empty())
			{
				auto it = obj.begin();
				for (;;)
				{
					s = dequoteString(json::serialize(it->key()));
					configAlgorithms(algoMap, taskAlgoMap, it->value(), s);
					if (++it == obj.end())
						break;
				}
			}
			break;
		}

		case json::kind::array:
		{
			auto const& arr = jv.get_array();
			if (!arr.empty())
			{
				if (key_name.compare("algorithms") == 0)
				{
					auto it = arr.begin();
					for (;;)
					{
						Algorithm a;
						addAlgorithm(a, *it, "");
						algoMap[a.name] = a.id;
						if (++it == arr.end())
							break;
					}
				}
				else if (key_name.compare("tasks") == 0)
				{
					auto it = arr.begin();
					for (;;)
					{
						TaskAlgorithm ta;
						addTaskAlgorithm(ta, *it, "");
						taskAlgoMap[ta.name] = ta;
						if (++it == arr.end())
							break;
					}
				}
			}
			break;
		}

		case json::kind::string:
		{
			s = dequoteString(json::serialize(jv.get_string()));
			break;
		}

		case json::kind::uint64:
			break;

		case json::kind::int64:
			break;

		case json::kind::double_:
			break;

		case json::kind::bool_:
			break;

		case json::kind::null:
			break;
		}
	}

	void Configurator::addHost(HostParams &h, json::value const& jv, std::string key_name)
	{
		std::string s;
		switch (jv.kind())
		{
		case json::kind::object:
		{
			auto const& obj = jv.get_object();
			if (!obj.empty())
			{
				auto it = obj.begin();
				for (;;)
				{
					s = dequoteString(json::serialize(it->key()));
					addHost(h, it->value(), s);
					if (++it == obj.end())
						break;
				}
			}
			break;
		}

		case json::kind::int64:
			if (key_name.compare("port") == 0)
				h.port = jv.get_int64();
			break;

		case json::kind::double_:
			break;

		case json::kind::string:
		{
			s = dequoteString(json::serialize(jv.get_string()));
			if (key_name.compare("app") == 0)
				h.app = s;
			else if (key_name.compare("name") == 0)
				h.name = s;
			break;
		}

		}
	}

	void	Configurator::configHosts(hostParamsMap &hostMap, json::value const& jv, std::string key_name)
	{
		std::string s;
		switch (jv.kind())
		{
		case json::kind::object:
		{
			auto const& obj = jv.get_object();
			if (!obj.empty())
			{
				auto it = obj.begin();
				for (;;)
				{
					s = dequoteString(json::serialize(it->key()));
					configHosts(hostMap, it->value(), s);
					if (++it == obj.end())
						break;
				}
			}
			break;
		}

		case json::kind::array:
		{
			auto const& arr = jv.get_array();
			if (!arr.empty())
			{
				if (key_name.compare("hosts") == 0)
				{
					auto it = arr.begin();
					for (;;)
					{
						HostParams h;
						addHost(h, *it, "");
						hostMap[h.app] = h;
						if (++it == arr.end())
							break;
					}
				}
			}
			break;
		}

		case json::kind::string:
		{
			s = dequoteString(json::serialize(jv.get_string()));
			break;
		}

		case json::kind::uint64:
			break;

		case json::kind::int64:
			break;

		case json::kind::double_:
			break;

		case json::kind::bool_:
			break;

		case json::kind::null:
			break;
		}
	}

	void Configurator::findFlag(std::string name, json::value const& jv, std::string key_name, bool &value)
	{
		std::string s;
		switch (jv.kind())
		{
		case json::kind::object:
		{
			auto const& obj = jv.get_object();
			if (!obj.empty())
			{
				auto it = obj.begin();
				for (;;)
				{
					s = dequoteString(json::serialize(it->key()));
					findFlag(name, it->value(), s, value);
					if (++it == obj.end())
						break;
				}
			}
			break;
		}

		case json::kind::array:
			break;

		case json::kind::string:
			break;

		case json::kind::uint64:
			break;

		case json::kind::int64:
			break;

		case json::kind::double_:
			break;

		case json::kind::bool_:
			if (key_name.compare(name) == 0)
				value = jv.get_bool();
			break;

		case json::kind::null:
			break;
		}
	}

	bool Configurator::isMissionLogInEnabled()
	{
		bool ret = false;
		findFlag("logMissionIn", m_jv, "", ret);
		return ret;
	}

	bool Configurator::isMissionLogOutEnabled()
	{
		bool ret = false;
		findFlag("logMissionOut", m_jv, "", ret);
		return ret;
	}

}
