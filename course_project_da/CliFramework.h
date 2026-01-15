#pragma once

#include <string>
#include <CLI\CLI.hpp>

namespace cli {
	class OptionContainer {
		template<typename T> friend class Option;
		friend class Flag;
	public:
		OptionContainer(const char* description, const char* name);
	protected:
		void Finalize(int argc, char** argv);
		int Exit(const CLI::ParseError& e);
	private:
		CLI::App app;
	};

	template<class Derived>
	class OptionBase : public OptionContainer {
	public:
		OptionBase()
			:
			OptionContainer{ Derived::description, Derived::name }
		{}
		static const Derived& Get() {
			return Get_();
		}
		static std::optional<int> Init(int argc, char** argv) {
			auto& opts = Get_();
			try {
				opts.Finalize(argc, argv);
				return {};
			}
			catch (const CLI::ParseError& e) {
				return opts.Exit(e);
			}
		}
	private:
		static Derived& Get_() {
			static Derived opts;
			return opts;
		}
	};

	template<typename T>
	class Option {
	public:
		Option(OptionContainer* pParent, std::string name, const T& defaultVal, std::string description)
			:
			data(defaultVal)
		{
			pOption = pParent->app.add_option(std::move(name), data, std::move(description));
		}
		const T& operator*() const {
			return data;
		}
		operator bool() const {
			return static_cast<bool>(*pOption);
		}
		bool operator!() const {
			return !(static_cast<bool>(*this));
		}
	private:
		T data;
		CLI::Option* pOption = nullptr;
	};

	class Flag {
	public:
		Flag(OptionContainer* pParent, std::string name, std::string description);
		bool operator*() const;
		operator bool() const;
		bool operator!() const;
	private:
		bool data = false;
		CLI::Option* pOption = nullptr;
	};
}